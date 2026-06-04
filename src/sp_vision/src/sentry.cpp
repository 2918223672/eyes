/**
 * @brief 哨兵自瞄 + 调试画面
 *
 * 硬件：1×海康工业相机（主视角）
 * 调试画面：YOLO 检测框（绿）/ EKF 重投影（绿点）/ 瞄准点（红=有效，蓝=无效）
 */

#include <fmt/core.h>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/my_comm_manager.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
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
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  // 读取敌方颜色，用于装甲板过滤
  auto yaml = YAML::LoadFile(config_path);
  auto enemy_color =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_t = std::chrono::steady_clock::now();

  // ===== 主循环 =====
  while (!exiter.exit()) {

    // 1. 采图 + 帧率
    camera.read(img, timestamp);
    auto dt = tools::delta_time(timestamp, last_t);
    last_t = timestamp;

    // 2. 读 IMU 姿态
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (comm.receivePacket()) {
      q = comm.getQuaternion();
    }
    solver.set_R_gimbal2world(q);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 3. 检测 + 过滤 + 排序
    auto armors = yolo.detect(img);

    // 过滤：敌方颜色 + 排除前哨站和5号
    armors.remove_if(
      [&](const auto_aim::Armor & a) { return a.color != enemy_color; });
    armors.remove_if(
      [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });
    armors.remove_if(
      [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });

    // 按到图像中心的距离排序
    armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
      cv::Point2f img_center(1440 / 2, 1080 / 2);
      return cv::norm(a.center - img_center) < cv::norm(b.center - img_center);
    });

    // 4. 多目标跟踪
    auto targets = tracker.track(armors, timestamp);

    // 5. 瞄准决策
    io::Command command{false, false, 0, 0};

    if (tracker.state() != "lost") {
      command = aimer.aim(targets, timestamp, 22);
    }

    // 6. 射击许可
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    // 7. 发送指令
    comm.sendPacket(command.yaw, command.pitch, command.control, command.shoot);

    // ===== 调试显示 =====
    if (!display) continue;

    // YOLO 检测框（绿）
    for (const auto & armor : armors) {
      for (int i = 0; i < 4; i++)
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }

    // 跟踪状态（左上角） lost/deciding/tracking + 帧率 丢失/决策/跟踪中
    tools::draw_text(img, fmt::format("[{}] ", tracker.state()), {10, 30});
    tools::logger()->info("[{}] FPS: {:.1f}", tracker.state(), 1 / dt);

    // EKF 重投影（绿点）+ 瞄准点（红/蓝）
    if (!targets.empty()) {
      auto target = targets.front();

      // 跟踪器认为装甲板在哪 → 绿点
      for (const auto & xyza : target.armor_xyza_list()) {
        auto pts = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, pts, {0, 255, 0});
      }

      // Aimer 选的瞄准点 → 红（有效）/ 蓝（无效）
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
    cv::imshow("sentry", display_img);
    if (cv::waitKey(1) == 27) break;
  }

  return 0;
}
