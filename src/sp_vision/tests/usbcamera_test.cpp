/**
 * @brief 测试 USB 相机能否正常打开和采集图像
 *
 * 使用场景：验证 USB 相机硬件和 V4L 驱动，确认帧率、分辨率正常
 * 依赖硬件：USB 相机
 */

#include "io/usbcamera/usbcamera.hpp"

#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/utils/exiter.hpp"
#include "tools/utils/logger.hpp"
#include "tools/math/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{name n         |        video0          | 端口名称 }"
  "{@config-path   | configs/sentry.yaml    | 位置参数，yaml配置文件路径 }"
  "{d display      |                        | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  tools::Exiter exiter;

  auto config_path = cli.get<std::string>(0);
  auto device_name = cli.get<std::string>("name");
  auto display = cli.has("display");

  io::USBCamera usbcam(device_name, config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    usbcam.read(img, timestamp);

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);
    std::this_thread::sleep_for(10ms);

    if (!display) continue;
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }
}
