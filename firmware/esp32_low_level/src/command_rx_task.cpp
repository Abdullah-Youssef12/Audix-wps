#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "microros_transport.hpp"

namespace app {

void commandRxTask(void*) {
    for (;;) {
        microrosSpinSome(5UL);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

}  // namespace app
