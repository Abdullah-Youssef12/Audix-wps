# Message Contract

This file captures message expectations at the Pi and ESP boundary.

## Principle

Use standard ROS message types wherever possible so the Pi-side stack can remain simple.

This file and [interface/pi_esp_topics.md](interface/pi_esp_topics.md) are the source of truth for Pi to ESP interface expectations.

## Current Standard Messages

### Pi to ESP32
- `/cmd_vel` uses `geometry_msgs/msg/Twist`
- `/robot_enable` uses `std_msgs/msg/Bool`

### ESP32 to Pi
- `/odom` uses `nav_msgs/msg/Odometry`
- `/imu` uses `sensor_msgs/msg/Imu`
- `/ir_front_digital` uses `std_msgs/msg/Bool`
- `/ir_front_left_digital` uses `std_msgs/msg/Bool`
- `/ir_front_right_digital` uses `std_msgs/msg/Bool`
- `/ir_left_digital` uses `std_msgs/msg/Bool`
- `/ir_right_digital` uses `std_msgs/msg/Bool`
- `/ir_back_digital` uses `std_msgs/msg/Bool`
- `/limit_switch` uses `std_msgs/msg/Bool`

## Semantic Expectations

### `/cmd_vel`
- expresses chassis-level motion intent only
- does not encode wheel-level commands
- is consumed by the ESP32 low-level controller

### `/robot_enable`
- `true` means motion is permitted
- `false` means motion must be stopped or held safe

### `/odom`
- is the raw odometry source from wheel encoders and low-level integration
- is fused with IMU on the Pi by EKF

### `/odometry/filtered`
- is produced on the Pi by `robot_localization`
- is the only odometry topic the protected `arena_roamer.py` behavior should depend on

### `/imu`
- should be published in standard ROS IMU form
- should be suitable for direct EKF consumption without a custom translator if possible

### Digital IR topics
- `true` means blocked
- `false` means clear
- these are consumed by `ir_digital_bridge.py`, which preserves the existing scan interface for `arena_roamer.py`

## Future Custom Messages

The following are intentionally deferred until needed:
- wheel speed telemetry message
- motor status or fault message
- richer actuator or lift feedback message

Until then, prefer primitive standard types over premature custom messages.