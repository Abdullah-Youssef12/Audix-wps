#include "mecanum.hpp"
#include "motor_driver.hpp"
#include "pid.hpp"
#include "safety.hpp"
#include "shared_state.hpp"

namespace app {

void runMotionControlTask(void*) {
    // Placeholder for the fixed-period motion loop:
    // command timeout check, mecanum targets, per-wheel PID, PWM output.
}

}  // namespace app
