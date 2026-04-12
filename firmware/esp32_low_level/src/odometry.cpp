#include <cmath>

#include "odometry.hpp"

namespace app {

Pose2D OdometryIntegrator::update(const WheelSpeeds& wheel_speeds, const RobotGeometry& geometry, float dt_seconds) {
    // Raw wheel odometry source for Pi-side EKF fusion.
    const float vx = (geometry.wheel_radius_m / 4.0f) * (
        wheel_speeds.front_left + wheel_speeds.front_right + wheel_speeds.back_left + wheel_speeds.back_right);
    const float vy = (geometry.wheel_radius_m / 4.0f) * (
        -wheel_speeds.front_left + wheel_speeds.front_right + wheel_speeds.back_left - wheel_speeds.back_right);
    const float wz = (geometry.wheel_radius_m / (4.0f * (geometry.wheel_base_half_m + geometry.track_width_half_m))) * (
        -wheel_speeds.front_left + wheel_speeds.front_right - wheel_speeds.back_left + wheel_speeds.back_right);

    pose_.yaw += wz * dt_seconds;
    pose_.x += (vx * std::cos(pose_.yaw) - vy * std::sin(pose_.yaw)) * dt_seconds;
    pose_.y += (vx * std::sin(pose_.yaw) + vy * std::cos(pose_.yaw)) * dt_seconds;
    return pose_;
}

const Pose2D& OdometryIntegrator::pose() const {
    return pose_;
}

}  // namespace app
