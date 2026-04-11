#pragma once

#include <cstdint>

namespace app {

struct RobotGeometry {
    float wheel_radius_m = 0.0485f;
    float wheel_base_half_m = 0.09f;
    float track_width_half_m = 0.1574f;
};

struct TaskPeriodsMs {
    static constexpr std::uint32_t command_rx = 20;
    static constexpr std::uint32_t motion_control = 10;
    static constexpr std::uint32_t sensor_update = 10;
    static constexpr std::uint32_t telemetry = 50;
};

struct SafetyConfig {
    std::uint32_t command_timeout_ms = 300;
    float max_pwm = 255.0f;
};

inline const RobotGeometry kRobotGeometry{};
inline const SafetyConfig kSafetyConfig{};

}  // namespace app
