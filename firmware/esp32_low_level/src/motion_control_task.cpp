#include "mecanum.hpp"
#include "motor_driver.hpp"
#include "pid.hpp"
#include "safety.hpp"
#include "shared_state.hpp"

namespace app {

void runMotionControlTask(void*) {
    // TODO: Run the fixed-period low-level control loop from the flowcharts:
    // command timeout check, enable gating, mecanum inverse kinematics,
    // four wheel-speed PID updates, output clamp, and PWM application.
}

}  // namespace app
