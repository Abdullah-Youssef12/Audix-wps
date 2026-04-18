#pragma once

#include "shared_state.hpp"

namespace app {

void initMotorDriver();
void writeMotorOutputs(const float pwm_output[kWheelCount]);
void stopAllMotors();

}  // namespace app
