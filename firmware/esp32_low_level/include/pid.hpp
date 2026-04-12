#pragma once

namespace app {

class PidController {
public:
    // Configure one wheel-speed PID controller. The final firmware should
    // instantiate four of these, one per drive wheel.
    void configure(float kp, float ki, float kd, float output_min, float output_max);
    void setIntegralClamp(float integral_min, float integral_max);
    void reset();
    float update(float target, float measured, float dt_seconds, bool enable_integral = true);

private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float output_min_ = -255.0f;
    float output_max_ = 255.0f;
    float integral_min_ = -100.0f;
    float integral_max_ = 100.0f;
    float integral_ = 0.0f;
    float previous_error_ = 0.0f;
    bool has_previous_error_ = false;
};

}  // namespace app
