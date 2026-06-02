/**
 * @brief 测试 DM 系列 IMU 串口驱动是否正常读数
 *
 * 使用场景：哨兵底盘 IMU 通信验证，确认四元数数据正确输出
 * 依赖硬件：DM 系列 IMU 模块
 */

#include <chrono>
#include <thread>

#include "io/dm_imu/dm_imu.hpp"
#include "tools/utils/exiter.hpp"
#include "tools/utils/logger.hpp"
#include "tools/math/math_tools.hpp"

using namespace std::chrono_literals;

int main()
{
  tools::Exiter exiter;
  io::DM_IMU imu;

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = imu.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0], eulers[1], eulers[2]);
  }

  return 0;
}
