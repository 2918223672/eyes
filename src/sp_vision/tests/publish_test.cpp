/**
 * @brief ROS2 发布测试（向 auto_aim_target_pos 话题发送模拟数据）
 *
 * 使用场景：验证 ROS2 publisher 链路，确认导航端能收到视觉端发出的目标位置
 * 依赖：ROS2 环境（无需硬件）
 */

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/utils/exiter.hpp"
#include "tools/utils/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  double i = 0;
  while (!exiter.exit()) {
    Eigen::Vector4d data{i, i + 1, 1, auto_aim::ArmorName::sentry + 1};
    ros2.publish(data);
    i++;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (i > 1000) break;
  }
  return 0;
}
