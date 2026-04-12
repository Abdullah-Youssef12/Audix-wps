#include <algorithm>

#include "pid.hpp"

namespace app {

void PidController::configure(float kp, float ki, float kd, float output_min, float output_max) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    output_min_ = output_min;
    output_max_ = output_max;
}

void PidController::setIntegralClamp(float integral_min, float integral_max) {
    integral_min_ = integral_min;
    integral_max_ = integral_max;
}

void PidController::reset() {
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    has_previous_error_ = false;
}

float PidController::update(float target, float measured, float dt_seconds, bool enable_integral) {
    const float error = target - measured;
    if (enable_integral && dt_seconds > 0.0f) {
        integral_ += error * dt_seconds;
        integral_ = std::clamp(integral_, integral_min_, integral_max_);
    }

    float derivative = 0.0f;
    if (has_previous_error_ && dt_seconds > 0.0f) {
        derivative = (error - previous_error_) / dt_seconds;
    }

    previous_error_ = error;
    has_previous_error_ = true;

    // This is the per-wheel PID output used by the motion-control task. In the
    // final firmware, instantiate four controllers and reset their integrators
    // whenever motion is disabled or command timeout safety trips.
    const float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return std::clamp(output, output_min_, output_max_);
}

}  // namespace app
