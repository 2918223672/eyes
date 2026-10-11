/**
 * @brief 人工测量式手眼参数调整 + 自瞄识别（不依赖标定板）
 *
 * 思路来源：QD_Vision2026 的 dynamic_camera_tf 方案。
 * 手眼变换的 6 个自由度中：
 *   - 平移 3 个：尺子量或查图纸即可，差 1cm 带来的角度误差约 0.1 度，可忽略
 *   - 旋转 3 个：主体是坐标轴约定（相机 z 前 → 云台 x 前），是定义好的常量；
 *                真正的未知量只有 1~2 度的装配误差
 * 所以不用标定板，改为：保留已有 R_camera2gimbal 作基底，在其上叠加一个
 * 欧拉角偏置量（configs 里的 camera2gimbal_offset），本程序提供按键实时调整。
 *
 * 判断标准：绿点（程序认为装甲板在哪）与画面里真实装甲板是否重合。
 * 调好后按 p 打印，把数值抄回 configs/sentry.yaml 的 camera2gimbal_offset。
 *
 * 按键：
 *   w/s  pitch -/+     （上下歪，影响最大）
 *   a/d  yaw   -/+     （左右歪）
 *   q/e  roll  -/+     （滚转，一般不用动）
 *   1/2  步长 /10, *10 （默认 0.001 rad ≈ 0.057 度）
 *   r    归零
 *   p    打印当前值（yaml 格式，可直接抄）
 *   ESC  退出
 */

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/my_comm_manager.hpp"
#include "io/my_protocol.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/math/math_tools.hpp"
#include "tools/utils/exiter.hpp"
#include "tools/utils/img_tools.hpp"
#include "tools/utils/logger.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/sentry.yaml | yaml配置文件路径 }"
  "{no-serial      |                     | 不开串口，云台角全零（仅看重投影）}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");
  auto no_serial = cli.has("no-serial");

  tools::Exiter exiter;

  // ===== 初始化 =====
  io::Camera camera(config_path);
  std::unique_ptr<io::CommManager> comm;
  if (!no_serial) comm = std::make_unique<io::CommManager>("/dev/ttyUSB0", 921600);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);

  auto yaml = YAML::LoadFile(config_path);
  auto enemy_color =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;

  // 从配置读初值，后续按键在此基础上增减
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();  // [yaw, pitch, roll]
  if (yaml["camera2gimbal_offset"]) {
    auto off = yaml["camera2gimbal_offset"];
    offset = Eigen::Vector3d(
      off["yaw"].as<double>(0.0), off["pitch"].as<double>(0.0), off["roll"].as<double>(0.0));
  }

  // Solver 构造时已把配置里的 offset 叠加进去了，这里记下基底，
  // 之后每次按键都从基底重新叠加，避免反复相乘累积误差
  const Eigen::Matrix3d R_base = tools::rotation_matrix(offset).transpose() * solver.R_camera2gimbal();

  double step = 0.001;  // rad
  bool dirty = false;

  tools::logger()->info("[qd_vision] 按键: w/s=pitch a/d=yaw q/e=roll 1/2=步长 r=归零 p=打印 ESC=退出");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_t = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    auto dt = tools::delta_time(timestamp, last_t);
    last_t = timestamp;

    // 应用当前 offset
    if (dirty) {
      solver.set_R_camera2gimbal(tools::rotation_matrix(offset) * R_base);
      dirty = false;
    }

    // 云台姿态
    Eigen::Vector3d gimbal_pos(0, 0, 0);
    if (comm) {
      io::SeaskyRx rx;
      if (comm->receive(rx)) gimbal_pos = Eigen::Vector3d(rx.yaw, rx.pitch, rx.roll);
    }
    Eigen::Quaterniond q = Eigen::AngleAxisd(gimbal_pos[0], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(gimbal_pos[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(gimbal_pos[2], Eigen::Vector3d::UnitX());
    solver.set_R_gimbal2world(q);

    // 检测 + 过滤
    auto armors = yolo.detect(img);
    armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color; });
    armors.remove_if(
      [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });
    armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });

    auto targets = tracker.track(armors, timestamp);

    // ===== 显示 =====
    // 黄框：YOLO 原始检测（真值参考）
    for (const auto & armor : armors) {
      for (int i = 0; i < 4; i++)
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 255), 2);
    }

    // 绿点：解算+跟踪后重投影回来的位置。与黄框重合 = 参数对了
    if (!targets.empty()) {
      const auto & target = targets.front();
      for (const auto & xyza : target.armor_xyza_list()) {
        auto pts = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, pts, {0, 255, 0});
      }
    }

    // 参数面板
    int y = 30;
    tools::draw_text(
      img, fmt::format("yaw  {:+.5f} rad ({:+.3f} deg)", offset[0], offset[0] * 57.2958), {10, y});
    tools::draw_text(
      img, fmt::format("pitch{:+.5f} rad ({:+.3f} deg)", offset[1], offset[1] * 57.2958),
      {10, y += 35});
    tools::draw_text(
      img, fmt::format("roll {:+.5f} rad ({:+.3f} deg)", offset[2], offset[2] * 57.2958),
      {10, y += 35});
    tools::draw_text(img, fmt::format("step {:.5f} rad", step), {10, y += 35});
    tools::draw_text(
      img, fmt::format("[{}] armors={} fps={:.1f}", tracker.state(), armors.size(), 1 / dt),
      {10, y += 35});
    if (!armors.empty()) {
      solver.solve(armors.front());  // 保证 ypd_in_world 已填充（tracker 可能未解算全部）
      tools::draw_text(
        img, fmt::format("dist {:.2f} m", armors.front().ypd_in_world[2]), {10, y += 35});
    }

    cv::Mat display_img;
    cv::resize(img, display_img, cv::Size(1280, 960));
    cv::imshow("qd_vision", display_img);

    // ===== 按键 =====
    int key = cv::waitKey(1);
    if (key == 27) break;
    switch (key) {
      case 'w': offset[1] -= step; dirty = true; break;
      case 's': offset[1] += step; dirty = true; break;
      case 'a': offset[0] -= step; dirty = true; break;
      case 'd': offset[0] += step; dirty = true; break;
      case 'q': offset[2] -= step; dirty = true; break;
      case 'e': offset[2] += step; dirty = true; break;
      case '1': step = std::max(step / 10, 1e-5); break;
      case '2': step = std::min(step * 10, 1e-1); break;
      case 'r': offset.setZero(); dirty = true; break;
      case 'p':
        tools::logger()->info(
          "camera2gimbal_offset: {{roll: {:.6f}, pitch: {:.6f}, yaw: {:.6f}}}", offset[2],
          offset[1], offset[0]);
        break;
      default: break;
    }
  }

  tools::logger()->info(
    "最终值 → camera2gimbal_offset: {{roll: {:.6f}, pitch: {:.6f}, yaw: {:.6f}}}", offset[2],
    offset[1], offset[0]);
  return 0;
}
