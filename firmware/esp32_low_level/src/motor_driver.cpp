#include "motor_driver.hpp"

#include <algorithm>
#include <cmath>

#include <Arduino.h>
#include <driver/ledc.h>

namespace app {

namespace {

constexpr ledc_channel_t kLedsChannels[kWheelCount] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
};

void writeSingleMotor(WheelIndex wheel, float pwm_command) {
    const float polarity_adjusted = pwm_command * static_cast<float>(MOTOR_POLARITY[wheel]);
    const float clamped = std::clamp(polarity_adjusted, -PWM_MAX_F, PWM_MAX_F);
    float magnitude = std::fabs(clamped);

    if (magnitude > 0.0f && magnitude < MOTOR_DEADBAND) {
        magnitude = MOTOR_DEADBAND;
    }
    if (magnitude < 1.0f) {
        magnitude = 0.0f;
    }

    digitalWrite(MOTOR_DIR_PINS[wheel], clamped >= 0.0f ? HIGH : LOW);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, kLedsChannels[wheel], static_cast<std::uint32_t>(magnitude));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, kLedsChannels[wheel]);
}

}  // namespace

void initMotorDriver() {
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_config.duty_resolution = static_cast<ledc_timer_bit_t>(PWM_RESOLUTION_BITS);
    timer_config.timer_num = LEDC_TIMER_0;
    timer_config.freq_hz = PWM_FREQUENCY;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_config);

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        pinMode(MOTOR_DIR_PINS[wheel], OUTPUT);
        digitalWrite(MOTOR_DIR_PINS[wheel], LOW);

        ledc_channel_config_t channel_config = {};
        channel_config.gpio_num = MOTOR_PWM_PINS[wheel];
        channel_config.speed_mode = LEDC_HIGH_SPEED_MODE;
        channel_config.channel = kLedsChannels[wheel];
        channel_config.intr_type = LEDC_INTR_DISABLE;
        channel_config.timer_sel = LEDC_TIMER_0;
        channel_config.duty = 0;
        channel_config.hpoint = 0;
        ledc_channel_config(&channel_config);
    }
}

void writeMotorOutputs(const float pwm_output[kWheelCount]) {
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        writeSingleMotor(static_cast<WheelIndex>(wheel), pwm_output[wheel]);
    }
}

void stopAllMotors() {
    const float zero_pwm[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    writeMotorOutputs(zero_pwm);
}

}  // namespace app
