#pragma once

#include "config.hpp"

namespace app {

struct BodyCommand {
    float linear_x = 0.0f;
    float linear_y = 0.0f;
    float angular_z = 0.0f;
};

struct WheelTargets {
    float front_left = 0.0f;
    float front_right = 0.0f;
    float back_left = 0.0f;
    float back_right = 0.0f;
};

WheelTargets inverseKinematics(const BodyCommand& command, const RobotGeometry& geometry);

}  // namespace app
