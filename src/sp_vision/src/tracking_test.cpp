/**
 * @brief 装甲板跟踪 + 瞄准调试入口
 *
 * 链路：相机 → YOLO检测 → Tracker(EKF) → Aimer → Shooter
 * 调试画面：检测框(蓝) / EKF重投影(绿点) / 瞄准点(红=有效，蓝=无效)
 * 日志：帧率 + 跟踪状态 + 射击许可 + 目标距离
 */

#include <fmt/core.h>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/my_comm_manager.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/utils/exiter.hpp"
#include "tools/utils/img_tools.hpp"
#include "tools/utils/logger.hpp"
#include "tools/math/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/sentry.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");

  tools::Exiter exiter;

  io::Camera camera(config_path);
  io::CommManager comm("/dev/ttyUSB0", 115200);
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_t = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    auto dt = tools::delta_time(timestamp, last_t);
    last_t = timestamp;

    // IMU 姿态
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (comm.receivePacket()) q = comm.getQuaternion();
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 检测 + 跟踪
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, timestamp);

    // 瞄准 + 射击
    io::Command command{false, false, 0, 0};
    double distance = 0;
    if (!targets.empty()) {
      command = aimer.aim(targets, timestamp, 22);
      // 计算最近目标距离
      auto & t = targets.front();
      for (auto & xyza : t.armor_xyza_list()) {
        auto d = xyza.head(3).norm();
        if (d > distance) distance = d;
      }
    }
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    // 日志输出
    // tools::logger()->info("[{}] FPS:{:.1f} shoot:{} dist:{:.2f}m",
    //                       tracker.state(), 1 / dt,
    //                       command.shoot ? "YES" : "no",
    //                       distance);

    if (!display) continue;

    // 检测框（蓝）+ 敌方信息（颜色+兵种）
    for (const auto & armor : armors) {
      for (int i = 0; i < 4; i++)
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
      auto label = fmt::format(
        "{} {}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name]);
      tools::draw_text(img, label, armor.box.tl(), {255, 0, 0});
    }

    // 跟踪状态
    tools::draw_text(img, fmt::format("[{}] {:.1f}fps", tracker.state(), 1 / dt),
                     {10, 30}, {0, 255, 255});

    // EKF 重投影（绿点）+ 瞄准点（红=有效 / 蓝=无效）
    if (!targets.empty()) {
      auto target = targets.front();
      for (const auto & xyza : target.armor_xyza_list()) {
        auto pts = solver.reproject_armor(
          xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, pts, {0, 255, 0});
      }
      auto aim_point = aimer.debug_aim_point;
      auto aim_pts = solver.reproject_armor(
        aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
      tools::draw_points(img, aim_pts, aim_point.valid ? cv::Scalar(0, 0, 255)
                                                       : cv::Scalar(255, 0, 0));
    }

    cv::Mat display_img;
    cv::resize(img, display_img, cv::Size(640, 480));
    cv::imshow("tracking_test", display_img);
    if (cv::waitKey(1) == 27) break;
  }

  return 0;
}
