#pragma once

#include <cstdint>

#ifdef ARDUINO
#include <freertos/FreeRTOS.h>
#else
#include <FreeRTOS.h>
#endif

namespace app {

// -- Math constants -------------------------------------------------
constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravityMps2 = 9.80665f;

// -- Serial ---------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200U;

// -- I2C / IMU ------------------------------------------------------
constexpr int IMU_SDA_PIN = 21;
constexpr int IMU_SCL_PIN = 22;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000U;
constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68U;
constexpr uint32_t IMU_GYRO_BIAS_SAMPLES = 500U;

// -- Limit switch ---------------------------------------------------
constexpr int LIMIT_SWITCH_PIN = 23;
constexpr bool LIMIT_SWITCH_ACTIVE_LOW = true;

// -- Encoder GPIO pins ----------------------------------------------
// Ordered: FL, FR, RL, RR
constexpr int ENCODER_A_PINS[4] = {34, 36, 16, 18};
constexpr int ENCODER_B_PINS[4] = {35, 39, 17, 19};

// Encoder CPR (counts per revolution of motor output shaft)
// 11 pulses per rev at motor shaft, gear ratio 34 (JGA25-370 ~34:1)
// x4 quadrature -> 11 * 34 * 4 = 1496 counts/rev
constexpr float ENCODER_COUNTS_PER_REV = 1496.0f;

inline float encoderCountToRad(int32_t counts) {
    return static_cast<float>(counts) * (2.0f * kPi / ENCODER_COUNTS_PER_REV);
}

inline float wrapAngleRad(float angle) {
    while (angle > kPi) {
        angle -= 2.0f * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0f * kPi;
    }
    return angle;
}

// -- Motor driver GPIO pins ----------------------------------------
// One direction pin per motor. PWM on separate pin.
constexpr int MOTOR_DIR_PINS[4] = {25, 27, 12, 2};
constexpr int MOTOR_PWM_PINS[4] = {32, 33, 15, 0};
// +1 or -1 to flip physical wiring polarity per wheel
constexpr int MOTOR_POLARITY[4] = {1, -1, 1, -1};

// -- PWM ------------------------------------------------------------
constexpr uint32_t PWM_FREQUENCY = 2000U;
constexpr uint32_t PWM_RESOLUTION_BITS = 10U;
constexpr float PWM_MAX_F = 1023.0f;  // 2^10 - 1
constexpr float MOTOR_DEADBAND = 40.0f;  // counts below which motor stalls

// -- Mecanum kinematics ---------------------------------------------
constexpr float WHEEL_RADIUS_M = 0.0485f;
constexpr float WHEELBASE_HALF_M = 0.090f;  // half of 180 mm wheelbase
constexpr float TRACK_HALF_WIDTH_M = 0.147f;  // half of 294 mm track width

// -- PID gains ------------------------------------------------------
constexpr float PID_KP = 1.2f;
constexpr float PID_KI = 0.8f;
constexpr float PID_KD = 0.05f;
constexpr float PID_OUTPUT_MAX = 1.0f;
constexpr float PID_INTEGRAL_MAX = 50.0f;
constexpr float WHEEL_STOP_EPSILON_RAD_S = 0.05f;

// -- IMU-odometry blending -----------------------------------------
constexpr float IMU_YAW_BLEND_ALPHA = 0.05f;  // 0=encoder only, 1=IMU only
constexpr float IMU_YAW_BLEND_LIMIT_RAD = 0.1f;  // max correction per cycle

// -- Timing ---------------------------------------------------------
constexpr uint32_t CONTROL_PERIOD_MS = 10U;
constexpr uint32_t SENSOR_PERIOD_MS = 5U;
constexpr uint32_t TELEMETRY_PERIOD_MS = 50U;
constexpr uint32_t CMD_TIMEOUT_MS = 500U;

// -- Task config ----------------------------------------------------
constexpr uint32_t COMMAND_TASK_STACK_BYTES = 8192U;
constexpr uint32_t MOTION_TASK_STACK_BYTES = 4096U;
constexpr uint32_t SENSOR_TASK_STACK_BYTES = 4096U;
constexpr uint32_t TELEMETRY_TASK_STACK_BYTES = 4096U;

constexpr UBaseType_t COMMAND_TASK_PRIORITY = 5U;
constexpr UBaseType_t MOTION_TASK_PRIORITY = 4U;
constexpr UBaseType_t SENSOR_TASK_PRIORITY = 3U;
constexpr UBaseType_t TELEMETRY_TASK_PRIORITY = 2U;

}  // namespace app
