#pragma once

#include "shared_state.hpp"

namespace app {

struct MotorOutputs {
    float front_left_pwm = 0.0f;
    float front_right_pwm = 0.0f;
    float back_left_pwm = 0.0f;
    float back_right_pwm = 0.0f;
};

void initializeMotorDriver();
void applyMotorOutputs(const MotorOutputs& outputs);
void stopAllMotors();

}  // namespace app
