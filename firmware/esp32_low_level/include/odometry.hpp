#pragma once

#include "mecanum.hpp"
#include "shared_state.hpp"

namespace app {

class OdometryIntegrator {
public:
    Pose2D update(const WheelSpeeds& wheel_speeds, const RobotGeometry& geometry, float dt_seconds);
    const Pose2D& pose() const;

private:
    Pose2D pose_;
};

}  // namespace app
