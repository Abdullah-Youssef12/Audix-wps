#include "mecanum.hpp"

namespace app {

WheelTargets inverseKinematics(const BodyCommand& command, const RobotGeometry& geometry) {
    const float k = geometry.wheel_base_half_m + geometry.track_width_half_m;
    const float inv_r = 1.0f / geometry.wheel_radius_m;

    WheelTargets targets;
    targets.front_left = inv_r * (command.linear_x + command.linear_y - k * command.angular_z);
    targets.front_right = inv_r * (command.linear_x - command.linear_y + k * command.angular_z);
    targets.back_left = inv_r * (command.linear_x - command.linear_y - k * command.angular_z);
    targets.back_right = inv_r * (command.linear_x + command.linear_y + k * command.angular_z);
    return targets;
}

}  // namespace app
