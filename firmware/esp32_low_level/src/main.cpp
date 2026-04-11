#include <Arduino.h>

#include "motor_driver.hpp"
#include "shared_state.hpp"

namespace app {

namespace {

SharedState g_shared_state;

}  // namespace

SharedState& sharedState() {
    return g_shared_state;
}

}  // namespace app

void setup() {
    app::initializeMotorDriver();
}

void loop() {
    delay(1000);
}
