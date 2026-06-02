#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tools/utils/exiter.hpp"
#include "tools/utils/logger.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@camera-config  | configs/camera.yaml   | 相机配置文件路径 }"
  "{@detector-config| configs/detector.yaml | 检测器配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto camera_config = cli.get<std::string>("@camera-config");
  auto detector_config = cli.get<std::string>("@detector-config");

  tools::Exiter exiter;
  io::Camera camera(camera_config);
  auto_aim::Detector detector(detector_config);

  tools::logger()->info("Sentry started. Press 'q' to quit.");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int frame_count = 0;

  while (!exiter.exit()) {
    camera.read(img, timestamp);

    if (img.empty()) continue;

    auto armors = detector.detect(img, frame_count++);

    tools::logger()->debug("Frame {}: {} armors detected", frame_count, armors.size());

    if (cv::waitKey(1) == 27) break;
  }

  tools::logger()->info("Sentry stopped.");
  return 0;
}
