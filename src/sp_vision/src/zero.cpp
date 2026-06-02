#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"

#include <opencv2/opencv.hpp>

#include "tools/utils/exiter.hpp"
#include "tools/utils/logger.hpp"
#include "tools/math/math_tools.hpp"

#include "tools/utils/crc.hpp"
//#include "io/serial/include/serial/serial.h"
#include "io/my_comm_manager.hpp"

#include <list>


// ================= [1. 全局配置与参数] =================
// 相机内参及畸变系数
static const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) <<  2471.7042207556383, 0, 628.41554289522833,
                                0, 2496.6212069725448, 548.65184574727596,
                                0, 0, 1);
// 畸变系数
static const cv::Mat distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.1206891596296123, 0.62989064176675036, 0.0042871080957956357, -0.00049692884030847102, 0);

// 物理尺寸定义 (单位：米)
static const double LIGHTBAR_LENGTH = 0.056; // 灯条长度    单位：米
static const double ARMOR_WIDTH = 0.135;     // 装甲板宽度  单位：米

// 安装偏移量 (相机相对于云台中心的物理位置)
const double offset_x = 0.0; 
const double offset_y = 0.05; // 相机与枪管纵向距离
const double offset_z = 0.03; // 相机与云台轴心水平距离

  // PnP 3D 模型模板
static const std::vector<cv::Point3f> object_points {
    { -ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0 },  // 点 1
    {  ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2, 0 },  // 点 2
    {  ARMOR_WIDTH / 2,  LIGHTBAR_LENGTH / 2, 0 },  // 点 3
    { -ARMOR_WIDTH / 2,  LIGHTBAR_LENGTH / 2, 0 }   // 点 4
};

// ================= [2. 工具函数封装] =================

// 结构体：存储最终发给电控的角度
struct TargetPos {
    double x, y, z;      // 相对云台的三维坐标
    double yaw, pitch;   // 偏转角度
};

//将相机识别到的像素点转换为角度
TargetPos solve_pose(const std::vector<cv::Point2f>& image_points) {

    cv::Mat rvec, tvec;
    cv::solvePnP(object_points, image_points, camera_matrix, distort_coeffs, rvec, tvec);

    // 1. 坐标系转换 (相机 -> 云台)
    double x_g = tvec.at<double>(0) + offset_x;
    double y_g = tvec.at<double>(1) + offset_y;
    double z_g = tvec.at<double>(2) + offset_z;

    // 2. 角度解算 (弧度转角度)
    double yaw = std::atan2(x_g, z_g) * 180.0 / M_PI;
    double horizontal_dist = std::sqrt(x_g * x_g + z_g * z_g);
    double pitch = std::atan2(-y_g, horizontal_dist) * 180.0 / M_PI;

    return {x_g, y_g, z_g, yaw, pitch};
}

//cmake --build build -j8 && ./build/camera_test -d


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
  auto last_stamp = std::chrono::steady_clock::now();

  io::CommManager comm("/dev/ttyUSB0", 115200);

  while (!exiter.exit()) {

    if (comm.receivePacket()) { 
        // 获取解析后的 Eigen 四元数
        Eigen::Quaterniond q = comm.getQuaternion();

        // 2. 打印四元数数值
        // 使用 fmt 库（框架自带）可以非常整齐地输出
        //tools::logger()->info("收到姿态数据 -> qx: {:.4f}, qy: {:.4f}, qz: {:.4f}, qw: {:.4f}", 
                               //q.x(), q.y(), q.z(), q.w());
        
        // 3. 将姿态注入解算器（这是后续自瞄准的核心）
        //solver.set_R_gimbal2world(q);
    }

    // 从相机驱动缓冲区读取最新的一帧原始图像
    camera.read(img, timestamp); 
    auto armors = yolo.detect(img);
    TargetPos res{0, 0, 0, 0, 0};

    if (!armors.empty()) {
            // 填充数据
            res = solve_pose(armors.front().points);
            double real_dist = std::sqrt(res.x * res.x + res.y * res.y + res.z * res.z);
            bool should_shoot = (real_dist < 0.5);
            comm.sendPacket(res.yaw, res.pitch, 1, should_shoot);
            //printf("[LOG] Yaw:%6.2f | Pitch:%6.2f | State: %d | Shoot: %d\n", 
                                        //res.yaw, res.pitch, 1, should_shoot);
            //comm.sendDebugChar('a');投入
        } 
        else {
            // 没发现目标，发个空包
            comm.sendPacket(0, 0, 0, false);
            //comm.sendDebugChar('a');
        }

    for (const auto & armor : armors) {

        std::string color_str = auto_aim::COLORS[armor.color];
        std::string name_str = auto_aim::ARMOR_NAMES[armor.name];

        // 画 4 个角点的连线 (绿色)
        for (int i = 0; i < 4; i++) {
            cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }

        // 在左上角写信息 (粉色)
        std::string label = fmt::format("{} - {}", color_str, name_str);
        cv::putText(img, label, armor.box.tl(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 0, 255), 2);

        // 在右 上角写信息 (粉色)        
        std::string dist_label = fmt::format("{:.2f} - {:.2f} - {:.2f}", res.x, res.y, res.z);
        cv::putText(img, dist_label, armor.box.br(), // 画在装甲板右下角
                          cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

    }
//诉讼诉讼rewf
    if (!display) continue;
    cv::Mat display_img;
    cv::resize(img, display_img, cv::Size(640, 480));
    cv::imshow("Sentry Auto Aim & YOLO Debug", display_img);
    if (cv::waitKey(1) == 27) break;
  }
}
