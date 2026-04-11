#pragma once

#include <cstdint>

namespace app {

struct CommandState {
    float linear_x = 0.0f;
    float linear_y = 0.0f;
    float angular_z = 0.0f;
    bool enabled = false;
    std::uint32_t last_command_ms = 0;
};

struct WheelSpeeds {
    float front_left = 0.0f;
    float front_right = 0.0f;
    float back_left = 0.0f;
    float back_right = 0.0f;
};

struct SensorState {
    WheelSpeeds measured_wheels;
    bool ir_front = false;
    bool ir_front_left = false;
    bool ir_front_right = false;
    bool ir_left = false;
    bool ir_right = false;
    bool ir_back = false;
    bool limit_switch = false;
    float imu_yaw_rate = 0.0f;
};

struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
};

struct SharedState {
    CommandState command;
    SensorState sensors;
    Pose2D odom;
};

SharedState& sharedState();

void runCommandRxTask(void*);
void runMotionControlTask(void*);
void runSensorUpdateTask(void*);
void runTelemetryTask(void*);

}  // namespace app
