/**
 * @brief 测试 CAN 通信板（CBoard）是否正常工作
 *
 * 使用场景：赛前确认 CAN 通信板链路通畅，能读到云台 IMU 四元数和弹速数据
 * 依赖硬件：CAN 通信板、云台 IMU
 */

#include "io/cboard.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/utils/exiter.hpp"
#include "tools/utils/logger.hpp"
#include "tools/math/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                       | 输出命令行参数说明}"
  "{@config-path   | configs/standard.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;

  io::CBoard cboard(config_path);

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0], eulers[1], eulers[2]);
    tools::logger()->info("bullet speed {:.2f} m/s", cboard.bullet_speed);
  }

  return 0;
}
