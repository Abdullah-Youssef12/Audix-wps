#include <algorithm>
#include <cmath>

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "config.hpp"
#include "mecanum.hpp"
#include "motor_driver.hpp"
#include "pid.hpp"
#include "safety.hpp"
#include "shared_state.hpp"

namespace app {

void motionControlTask(void*) {
    PidController wheel_pid[kWheelCount];
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        wheel_pid[wheel].setGains(PID_KP, PID_KI, PID_KD);
        wheel_pid[wheel].setOutputLimits(-PID_OUTPUT_MAX, PID_OUTPUT_MAX);
        wheel_pid[wheel].setIntegralLimits(-PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
        wheel_pid[wheel].reset();
    }

    MotionSafety motion_safety;
    bool previous_motion_allowed = false;
    TickType_t last_wake_time = xTaskGetTickCount();
    const float dt_seconds = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;

    for (;;) {
        const CommandState command_state = getCommandState();
        const WheelState wheel_state = getWheelState();
        const bool motion_allowed = motion_safety.motionAllowed(command_state, millis());

        if (motion_allowed && !previous_motion_allowed) {
            for (auto& controller : wheel_pid) {
                controller.reset();
            }
        }

        ChassisVelocity chassis_command;
        chassis_command.vx = motion_allowed ? command_state.cmd_vx : 0.0f;
        chassis_command.vy = motion_allowed ? command_state.cmd_vy : 0.0f;
        chassis_command.wz = motion_allowed ? command_state.cmd_wz : 0.0f;

        float target_wheel_rad_s[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        inverseKinematics(chassis_command, target_wheel_rad_s);
        setWheelTargets(target_wheel_rad_s);

        float pwm_output[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            pwm_output[wheel] = wheel_pid[wheel].update(
                target_wheel_rad_s[wheel],
                wheel_state.measured_w_rad_s[wheel],
                dt_seconds,
                motion_allowed);

            if (!motion_allowed && std::fabs(wheel_state.measured_w_rad_s[wheel]) < WHEEL_STOP_EPSILON_RAD_S) {
                wheel_pid[wheel].reset();
                pwm_output[wheel] = 0.0f;
            }

            pwm_output[wheel] = std::clamp(pwm_output[wheel], -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
        }

        writeMotorOutputs(pwm_output);
        setPwmOutputs(pwm_output);

        previous_motion_allowed = motion_allowed;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

}  // namespace app
