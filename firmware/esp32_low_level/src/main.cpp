#include <Arduino.h>
#include <cstdint>

#include "config.hpp"
#include "encoder_isr.hpp"
#include "imu_driver.hpp"
#include "microros_transport.hpp"
#include "motor_driver.hpp"
#include "odometry.hpp"
#include "shared_state.hpp"

void setup() {
    Serial.begin(app::SERIAL_BAUD);
    const std::uint32_t serial_wait_start = millis();
    while (!Serial && (millis() - serial_wait_start) < 2000U) {
        delay(10);
    }

    app::initSharedState();

    pinMode(app::LIMIT_SWITCH_PIN, app::LIMIT_SWITCH_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
    app::initEncoders();
    app::resetEncoders();
    app::initMotorDriver();
    app::odometryTracker().reset();
    app::imuDriver().begin();
    app::initMicroRosTransport();

    xTaskCreatePinnedToCore(
        app::commandRxTask,
        "command_rx",
        app::COMMAND_TASK_STACK_BYTES,
        nullptr,
        app::COMMAND_TASK_PRIORITY,
        nullptr,
        0);

    xTaskCreatePinnedToCore(
        app::motionControlTask,
        "motion_control",
        app::MOTION_TASK_STACK_BYTES,
        nullptr,
        app::MOTION_TASK_PRIORITY,
        nullptr,
        1);

    xTaskCreatePinnedToCore(
        app::sensorUpdateTask,
        "sensor_update",
        app::SENSOR_TASK_STACK_BYTES,
        nullptr,
        app::SENSOR_TASK_PRIORITY,
        nullptr,
        0);

    xTaskCreatePinnedToCore(
        app::telemetryTask,
        "telemetry",
        app::TELEMETRY_TASK_STACK_BYTES,
        nullptr,
        app::TELEMETRY_TASK_PRIORITY,
        nullptr,
        0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
