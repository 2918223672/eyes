/**
 * @brief 哨兵自瞄 — 多线程版
 *
 * 架构：采集线程 + 主线程，两线程流水线并行
 * 硬件：1×海康工业相机
 * 调试画面：YOLO 检测框（绿）/ EKF 重投影（绿点）/ 瞄准点（红=有效，蓝=无效）
 */

#include <fmt/core.h>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/my_comm_manager.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/utils/exiter.hpp"
#include "tools/utils/img_tools.hpp"
#include "tools/utils/logger.hpp"
#include "tools/math/math_tools.hpp"
#include "tools/plotter.hpp"

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
  tools::Plotter plotter;

  // ===== 硬件初始化 =====
  io::Camera camera(config_path);
  io::CommManager comm("/dev/ttyUSB0", 115200);

  // ===== 算法模块初始化 =====
  auto_aim::multithread::MultiThreadDetector detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  // ===== 采集线程：只采图，丢给 MultiThreadDetector =====
  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);  // 丢进 OpenVINO 异步队列，立刻返回
    }
  });

  auto last_t = std::chrono::steady_clock::now();

  // ===== 主循环：处理推理结果 =====
  while (!exiter.exit()) {

    // 1. 取 OpenVINO 异步推理结果
    auto [img, armors, t] = detector.debug_pop();
    auto dt = tools::delta_time(t, last_t);
    last_t = t;

    // 2. 读 IMU 姿态
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (comm.receivePacket()) {
      q = comm.getQuaternion();
    }
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 3. 多目标跟踪 + 瞄准
    auto targets = tracker.track(armors, t);
    io::Command command{false, false, 0, 0};

    if (!targets.empty()) {
      command = aimer.aim(targets, t, 22);
    }

    // 4. 射击许可
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    // 5. 发送指令
    comm.sendPacket(command.yaw, command.pitch, command.control, command.shoot);

    // ===== 调试显示 =====
    if (!display) continue;

    for (const auto & armor : armors) {
      for (int i = 0; i < 4; i++)
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }

    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30});
    tools::logger()->info("[{}] FPS: {:.1f}", tracker.state(), 1 / dt);

    if (!targets.empty()) {
      auto target = targets.front();
      for (const auto & xyza : target.armor_xyza_list()) {
        auto pts = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, pts, {0, 255, 0});
      }
      auto aim_point = aimer.debug_aim_point;
      auto aim_pts = solver.reproject_armor(
        aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, aim_pts, {0, 0, 255});
      else
        tools::draw_points(img, aim_pts, {255, 0, 0});
    }

    cv::Mat display_img;
    cv::resize(img, display_img, cv::Size(640, 480));
    cv::imshow("mt_sentry", display_img);
    if (cv::waitKey(1) == 27) {
      exiter.exit();
      break;
    }
  }

  detect_thread.join();
  return 0;
}
