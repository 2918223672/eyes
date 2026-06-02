#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
enum ShootMode { left_shoot, right_shoot, both_shoot };
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;
};

}  // namespace io

#endif  // IO__COMMAND_HPP