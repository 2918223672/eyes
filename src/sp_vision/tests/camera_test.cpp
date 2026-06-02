#include <opencv2/opencv.hpp>

#include <list>

#include "io/camera.hpp"
#include "io/my_comm_manager.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/utils/exiter.hpp"
#include "tools/utils/logger.hpp"
#include "tools/math/math_tools.hpp"

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
  auto_aim::Solver solver("configs/sentry.yaml");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  io::CommManager comm("/dev/ttyUSB0", 115200);

  while (!exiter.exit()) {
    if (comm.receivePacket()) {
      Eigen::Quaterniond q = comm.getQuaternion();
      solver.set_R_gimbal2world(q);
    }

    camera.read(img, timestamp);
    auto armors = yolo.detect(img);

    if (!armors.empty()) {
      auto & armor = armors.front();
      solver.solve(armor);

      double yaw = armor.ypr_in_world[0] * 180.0 / CV_PI;
      double pitch = armor.ypr_in_world[1] * 180.0 / CV_PI;
      double real_dist = armor.ypd_in_world[2];
      bool should_shoot = (real_dist < 0.5);
      comm.sendPacket(yaw, pitch, 1, should_shoot);
    } else {
      comm.sendPacket(0, 0, 0, false);
    }

    for (const auto & armor : armors) {
      std::string color_str = auto_aim::COLORS[armor.color];
      std::string name_str = auto_aim::ARMOR_NAMES[armor.name];

      for (int i = 0; i < 4; i++) {
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
      }

      std::string label = fmt::format("{} - {}", color_str, name_str);
      cv::putText(
        img, label, armor.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 0, 255), 2);

      std::string dist_label = fmt::format(
        "{:.2f} - {:.2f} - {:.2f}", armor.xyz_in_world[0], armor.xyz_in_world[1],
        armor.xyz_in_world[2]);
      cv::putText(
        img, dist_label, armor.box.br(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    }

    if (!display) continue;
    cv::Mat display_img;
    cv::resize(img, display_img, cv::Size(640, 480));
    cv::imshow("Sentry Auto Aim & YOLO Debug", display_img);
    if (cv::waitKey(1) == 27) break;
  }

  return 0;
}
