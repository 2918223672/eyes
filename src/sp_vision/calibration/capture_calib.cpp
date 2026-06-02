#include <fmt/core.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/utils/img_tools.hpp"
#include "tools/utils/logger.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{config-path p   | configs/camera.yaml      | 相机配置文件      }"
  "{output-folder o | assets/img_with_q        | 输出文件夹        }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");
  auto output_folder = cli.get<std::string>("output-folder");

  std::filesystem::create_directories(output_folder);

  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  cv::Size pattern_size(9, 6);
  bool detected = false;
  std::vector<cv::Point2f> corners;

  int count = 0;
  int frame_count = 0;
  auto last_log = std::chrono::steady_clock::now();

  tools::logger()->info("Starting capture loop, press 's' to detect+save, 'q' to quit");

  while (true) {
    camera.read(img, timestamp);

    frame_count++;
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 1) {
      // tools::logger()->info("Live: {} frames, {} saved", frame_count, count);
      last_log = now;
    }

    cv::Mat display;
    cv::resize(img, display, {}, 0.5, 0.5);

    if (detected) {
      std::vector<cv::Point2f> scaled_corners;
      for (auto & p : corners) scaled_corners.push_back(p * 0.5);
      cv::drawChessboardCorners(display, pattern_size, scaled_corners, true);
    }

    // tools::draw_text(display,
    //                  fmt::format("Frame: {} | Saved: {} | s=save q=quit", frame_count, count),
    //                  {10, 30}, {0, 255, 0});

    if (!detected)
      tools::draw_text(display, "Aim chessboard and press 's'", {10, 60}, {0, 0, 255});

    cv::imshow("Calibration capture", display);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key != 's') continue;

    // 按 s 时才跑棋盘格检测（在原图上）
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    detected = cv::findChessboardCorners(gray, pattern_size, corners);
    if (detected) {
      cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }

    if (!detected) {
      tools::logger()->warn("Detection failed, try a different angle/distance");
      continue;
    }

    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    cv::imwrite(img_path, img);
    tools::logger()->info("[{}] Saved {}", count, img_path);
  }

  tools::logger()->info("Captured {} images", count);
  return 0;
}
