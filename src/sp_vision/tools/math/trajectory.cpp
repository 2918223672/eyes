#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.81;  // 重力加速度，单位：m/s^2

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);//由水平距离和竖直高度算抬头角
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);//由水平距离和竖直高度算抬头角
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));//由水平距离和水平速度算飞行时间
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

}  // namespace tools