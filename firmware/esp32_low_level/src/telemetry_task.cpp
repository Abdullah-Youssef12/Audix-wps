#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "config.hpp"
#include "microros_transport.hpp"

namespace app {

void telemetryTask(void*) {
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        publishTelemetry();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

}  // namespace app
