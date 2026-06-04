/**
 * @brief 最简自瞄程序 — 单相机 + YOLO + PnP 解算
 *
 * 硬件：1×海康工业相机
 * 功能：检测装甲板 → PnP 解算角度 → 串口发送指令
 */

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/my_comm_manager.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/utils/exiter.hpp"
#include "tools/utils/logger.hpp"
#include "tools/math/math_tools.hpp"

#include <list>

// ================= [1. 全局配置与参数] =================
// 相机内参及畸变系数
static const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 2471.7042207556383, 0, 628.41554289522833,
                               0, 2496.6212069725448, 548.65184574727596,
                               0, 0, 1);

static const cv::Mat distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.1206891596296123, 0.62989064176675036,
     0.0042871080957956357, -0.00049692884030847102, 0);

// 物理尺寸定义 (单位：米)
static const double LIGHTBAR_LENGTH = 0.056;
static const double ARMOR_WIDTH = 0.135;

// 安装偏移量 (相机相对于云台中心的物理位置)
static const double offset_x = 0.0;
static const double offset_y = 0.05;
static const double offset_z = 0.03;

// PnP 3D 模型模板
static const std::vector<cv::Point3f> object_points{
  {-ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0},
  { ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0},
  { ARMOR_WIDTH / 2,  LIGHTBAR_LENGTH / 2, 0},
  {-ARMOR_WIDTH / 2,  LIGHTBAR_LENGTH / 2, 0}
};

// ================= [2. 工具函数] =================

// 结构体：存储目标在云台坐标系下的位姿
struct TargetPos
{
  double x, y, z;
  double yaw, pitch;
};

// 将相机识别到的像素点转换为云台角度
TargetPos solve_pose(const std::vector<cv::Point2f> & image_points)
{
  cv::Mat rvec, tvec;
  cv::solvePnP(object_points, image_points, camera_matrix, distort_coeffs, rvec, tvec);

  double x_g = tvec.at<double>(0) + offset_x;
  double y_g = tvec.at<double>(1) + offset_y;
  double z_g = tvec.at<double>(2) + offset_z;

  double yaw = std::atan2(x_g, z_g) * 180.0 / M_PI;
  double horizontal_dist = std::sqrt(x_g * x_g + z_g * z_g);
  double pitch = std::atan2(-y_g, horizontal_dist) * 180.0 / M_PI;

  return {x_g, y_g, z_g, yaw, pitch};
}

// ================= [3. 主程序逻辑] =================

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");

  io::Camera camera(config_path);
  auto_aim::YOLO yolo("configs/sentry.yaml", false);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  io::CommManager comm("/dev/ttyUSB0", 115200);

  while (!exiter.exit()) {

    // 读 IMU 姿态
    if (comm.receivePacket()) {
      Eigen::Quaterniond q = comm.getQuaternion();
      (void)q;  // 当前未使用，后续可传入 solver
    }

    // 从相机读取一帧图像
    camera.read(img, timestamp);
    auto armors = yolo.detect(img);
    TargetPos res{0, 0, 0, 0, 0};

    if (!armors.empty()) {
      res = solve_pose(armors.front().points);
      double real_dist = std::sqrt(res.x * res.x + res.y * res.y + res.z * res.z);
      bool should_shoot = (real_dist < 0.5);
      comm.sendPacket(res.yaw, res.pitch, 1, should_shoot);
    } else {
      comm.sendPacket(0, 0, 0, false);
    }

    // 调试显示
    if (!display) continue;

    for (const auto & armor : armors) {
      std::string color_str = auto_aim::COLORS[armor.color];
      std::string name_str = auto_aim::ARMOR_NAMES[armor.name];

      for (int i = 0; i < 4; i++) {
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
      }

      std::string label = fmt::format("{} - {}", color_str, name_str);
      cv::putText(img, label, armor.box.tl(),
                  cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 0, 255), 2);

      std::string dist_label =
        fmt::format("{:.2f} - {:.2f} - {:.2f}", res.x, res.y, res.z);
      cv::putText(img, dist_label, armor.box.br(),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    }

    cv::Mat display_img;
    cv::resize(img, display_img, cv::Size(640, 480));
    cv::imshow("Zero Auto Aim & YOLO Debug", display_img);
    if (cv::waitKey(1) == 27) break;
  }

  return 0;
}
