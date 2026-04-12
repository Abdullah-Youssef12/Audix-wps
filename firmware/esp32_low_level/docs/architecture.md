# ESP32 Low-Level Architecture

This directory is a scaffold for the future ESP32 firmware that will own low-level control.

## Intended Task Split

- command receive task
- motion control task
- sensor update task
- telemetry task
- minimal encoder ISR handlers

## Ownership Boundary

This firmware owns:
- encoder reading
- IMU reading
- limit switch reading
- mecanum inverse kinematics
- four wheel-speed PID controllers
- raw odometry generation
- PWM and direction output
- command-timeout safety
- telemetry publishing

This firmware does not own:
- EKF
- `arena_roamer.py`
- IR scan conversion
- RViz or other ROS-side debugging tools

## Module Boundaries

- `pid.cpp`: per-wheel PID controller logic
- `mecanum.cpp`: chassis command to wheel target conversion
- `odometry.cpp`: wheel-speed integration into raw odometry
- `motor_driver.cpp`: PWM and direction output abstraction
- `safety.cpp`: command timeout and enable gating
- `microros_transport.cpp`: transport and executor integration points

This is intentionally a skeleton, not a finished firmware implementation.
