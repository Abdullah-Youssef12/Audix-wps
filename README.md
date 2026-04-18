# Audix-wps
Audix final ROS 2/micro-ROS workspace for Pi-to-ESP32 integration. Includes ESP32 low-level firmware, micro-ROS transport, motor PID, encoder odometry, IMU and limit switch feedback, Pi hardware launch files, IR/stepper bench tools, and validation docs for real robot bring-up.

```markdown
# Audix ROS 2 / micro-ROS Workspace

Final ROS 2 and micro-ROS workspace for the Audix robot. This repository contains the Raspberry Pi ROS 2 runtime, ESP32 low-level micro-ROS firmware, hardware bring-up tools, and validation procedures for the real robot.

## Purpose

This workspace is the final Pi-to-ESP32 integration path.

The Raspberry Pi runs ROS 2 Jazzy, owns the high-level robot runtime, and hosts the `micro_ros_agent`.

The ESP32 runs the low-level base controller firmware and communicates with the Pi through micro-ROS.

## Repository Layout

```text
microROS/
├── firmware/
│   └── esp32_low_level/
├── src/
│   └── audix_pkg/
├── interface/
├── docs/
└── audix_ubuntu.code-workspace
```

## ESP32 Low-Level Firmware

Final ESP32 firmware lives here:

```text
microROS/firmware/esp32_low_level
```

The ESP32 owns:

- DC motor drivers
- wheel encoders
- IMU
- base limit switch
- low-level PID/motion control
- RTOS tasks
- micro-ROS client

The ESP32 subscribes to:

```text
/cmd_vel
/robot_enable
```

The ESP32 publishes:

```text
/odom
/imu
/limit_switch
```

## Raspberry Pi ROS 2 Runtime

Pi-side ROS package lives here:

```text
microROS/src/audix_pkg
```

The Pi owns:

- ROS 2 Jazzy runtime
- `micro_ros_agent`
- launch files
- IR sensors
- Pi-owned stepper/scissor-lift bench path
- hardware validation scripts
- future GUI bridge
- future vision node integration
- future joystick teleop

## Public Pi-to-ESP Contract

The core Pi-to-ESP topic contract is:

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Pi -> ESP32 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Base velocity command |
| Pi -> ESP32 | `/robot_enable` | `std_msgs/msg/Bool` | Enable/disable motion |
| ESP32 -> Pi | `/odom` | `nav_msgs/msg/Odometry` | Encoder-derived odometry |
| ESP32 -> Pi | `/imu` | `sensor_msgs/msg/Imu` | IMU feedback |
| ESP32 -> Pi | `/limit_switch` | `std_msgs/msg/Bool` | Base limit switch state |

Do not change this contract unless the Pi and ESP32 sides are updated together.

## Pi-Owned Hardware

The Pi owns the six IR sensors:

```text
/ir_front_digital
/ir_front_left_digital
/ir_front_right_digital
/ir_left_digital
/ir_right_digital
/ir_back_digital
```

The Pi also owns the first real scissor-lift bench path:

- DRV8825 stepper driver
- lift limit switch
- jog up/down
- homing
- later lift-level control

## ESP32 Firmware Build

From Ubuntu/WSL or Pi:

```bash
cd ~/audix/microROS/firmware/esp32_low_level
pio run -e esp32dev
```

Upload when the ESP32 is connected:

```bash
pio run -e esp32dev -t upload
```

Optional serial monitor:

```bash
pio device monitor -b 115200
```

## Build The Pi Workspace

```bash
cd ~/audix/microROS
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys="micro_ros_agent"
colcon build --symlink-install
```

Source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/audix/microROS/install/setup.bash
```

## Build micro-ROS Agent On The Pi

```bash
mkdir -p ~/micro_ros_agent_ws/src
cd ~/micro_ros_agent_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

cd ~/micro_ros_agent_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Source it:

```bash
source ~/micro_ros_agent_ws/install/setup.bash
```

## Run Real Pi Hardware Stack

```bash
cd ~/audix/microROS
source /opt/ros/jazzy/setup.bash
source ~/micro_ros_agent_ws/install/setup.bash
source ~/audix/microROS/install/setup.bash

ros2 launch audix pi_hardware.launch.py serial_device:=/dev/serial0 serial_baud:=115200 use_rviz:=false
```

## Verify micro-ROS Link

In another Pi terminal:

```bash
cd ~/audix/microROS
source /opt/ros/jazzy/setup.bash
source ~/micro_ros_agent_ws/install/setup.bash
source ~/audix/microROS/install/setup.bash

ros2 topic info /cmd_vel
ros2 topic info /robot_enable
ros2 topic echo /odom --once
ros2 topic echo /imu --once
ros2 topic echo /limit_switch --once
```

Expected:

```text
/cmd_vel subscription count: 1
/robot_enable subscription count: 1
/odom publishes data
/imu publishes data
/limit_switch publishes data
```

## Base Motion Test

Always put the robot on blocks first.

Enable motion:

```bash
ros2 topic pub /robot_enable std_msgs/msg/Bool "{data: true}" --once
```

Forward test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

Strafe test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.10, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

Yaw test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.40}}" -r 10
```

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
ros2 topic pub /robot_enable std_msgs/msg/Bool "{data: false}" --once
```

## IMU Test

```bash
ros2 topic hz /imu
ros2 topic echo /imu
```

Expected:

- acceleration changes when tilted or moved
- angular velocity spikes during rotation
- yaw/orientation changes when rotating around Z
- topic publishes at stable rate

## Encoder / Odometry Test

```bash
ros2 topic hz /odom
ros2 topic echo /odom
```

Expected:

- forward command changes `odom.twist.twist.linear.x`
- strafe command changes `odom.twist.twist.linear.y`
- yaw command changes `odom.twist.twist.angular.z`

## Limit Switch Test

The ESP32 base limit switch publishes:

```text
/limit_switch
```

Test:

```bash
ros2 topic echo /limit_switch
```

Expected:

```text
released -> false
pressed  -> true
```

## RTOS / Timing Test

Use ROS topic rates as the external health check:

```bash
ros2 topic hz /odom
ros2 topic hz /imu
ros2 topic hz /limit_switch
```

Expected:

- stable topic rates
- no stalls
- no deadlocks
- robot stops on command timeout
- robot stops on disable
- robot stops if micro-ROS agent connection is lost

## Pi Manual Bench Tools

This workspace includes Pi-side bench tools for real hardware validation:

- `base_pid_bench.py`
- `pi_stepper_bench.py`
- `realtime_watch.py`
- `pi_ir_gpio_publisher.py`

Launch manual bench tools with:

```bash
ros2 launch audix pi_manual_bench.launch.py
```

## IR Sensor Test

Run the Pi hardware stack, then inspect digital IR topics:

```bash
ros2 topic echo /ir_front_digital
ros2 topic echo /ir_front_left_digital
ros2 topic echo /ir_front_right_digital
ros2 topic echo /ir_left_digital
ros2 topic echo /ir_right_digital
ros2 topic echo /ir_back_digital
```

Trigger one physical sensor at a time and confirm the correct topic changes.

## Stepper / Scissor Lift Bench

The first lift bench path is Pi-owned, not ESP32-owned.

Expected commands are handled through the Pi bench utility:

- jog up
- jog down
- stop
- home
- read limit switch

Use this before integrating lift levels into the full mission stack.

## Debug Playbook

If `/cmd_vel` shows `Subscription count: 0`:

- ESP32 is not connected to ROS
- check ESP32 power
- check `micro_ros_agent`
- check UART wiring
- check common ground
- check baud rate `115200`

Run the agent alone for debug:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial0 -b 115200 -v6
```

If `/odom` does not update:

- check encoder wiring
- check encoder polarity
- check counts-per-revolution
- check motor driver wiring
- check wheel order

If `/imu` does not update:

- check I2C wiring
- check IMU power
- check address
- check sensor orientation

If motors do not move:

- confirm `/robot_enable` is true
- confirm `/cmd_vel` is publishing
- confirm motor power rail
- confirm driver wiring
- confirm timeout is not stopping motion

If movement direction is wrong:

- test one wheel at a time
- fix motor polarity
- fix encoder sign
- fix wheel order
- verify mecanum wheel orientation

## Safety

Before real motion:

- put robot on blocks
- use low speeds first
- verify stop command
- verify disable command
- verify timeout stop
- verify agent-loss stop
- verify limit switch behavior

Never tune PID before confirming:

- encoder counts change correctly
- motor direction is correct
- wheel order is correct
- odometry updates
- stop/disable works

## Related Windows Sandbox

The separate Windows-only ESP32 sandbox lives outside this repo:

```text
C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test
```

That sandbox is for standalone ESP32 testing from VS Code and should not be confused with this final Pi/micro-ROS workspace.
```
