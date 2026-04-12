# System Architecture

This document defines the intended split between the Raspberry Pi high-level ROS stack and the ESP32 low-level real-time controller.

## Design Goal

Preserve the current `arena_roamer.py` behavior while moving real-time control, sensing, and actuation to the ESP32.

## Source Of Truth Files

### Main Pi hardware path

- [src/audix_pkg/launch/pi_hardware.launch.py](src/audix_pkg/launch/pi_hardware.launch.py)
- [src/audix_pkg/scripts/arena_roamer.py](src/audix_pkg/scripts/arena_roamer.py)
- [src/audix_pkg/scripts/ir_digital_bridge.py](src/audix_pkg/scripts/ir_digital_bridge.py)
- [src/audix_pkg/scripts/start_stop_node.py](src/audix_pkg/scripts/start_stop_node.py)
- [src/audix_pkg/config/hardware/ekf.yaml](src/audix_pkg/config/hardware/ekf.yaml)

### Main mock hardware path

- [src/audix_pkg/launch/pi_hardware_mock.launch.py](src/audix_pkg/launch/pi_hardware_mock.launch.py)
- [src/audix_pkg/scripts/mock_imu_publisher.py](src/audix_pkg/scripts/mock_imu_publisher.py)
- [src/audix_pkg/scripts/mock_odom_publisher.py](src/audix_pkg/scripts/mock_odom_publisher.py)
- [src/audix_pkg/scripts/mock_ir_digital_publisher.py](src/audix_pkg/scripts/mock_ir_digital_publisher.py)
- [src/audix_pkg/scripts/mock_robot_enable_publisher.py](src/audix_pkg/scripts/mock_robot_enable_publisher.py)
- [src/audix_pkg/scripts/mock_limit_switch_publisher.py](src/audix_pkg/scripts/mock_limit_switch_publisher.py)

### Main simulation path

- [src/audix_pkg/launch/full_mission.launch.py](src/audix_pkg/launch/full_mission.launch.py)
- [src/audix_pkg/launch/scissor_gazebo.launch.py](src/audix_pkg/launch/scissor_gazebo.launch.py)
- [src/audix_pkg/scripts/arena_obstacle_manager.py](src/audix_pkg/scripts/arena_obstacle_manager.py)
- [src/audix_pkg/scripts/arena_spawn_panel.py](src/audix_pkg/scripts/arena_spawn_panel.py)

### Legacy or reference paths

These remain in the repository for reference, comparison, or transitional use, but they are not the target final Raspberry Pi plus ESP32 path:

- [src/audix_pkg/launch/hardware.launch.py](src/audix_pkg/launch/hardware.launch.py)
- [src/audix_pkg/scripts/mission_controller.py](src/audix_pkg/scripts/mission_controller.py)
- [src/audix_pkg/scripts/mecanum_kinematics.py](src/audix_pkg/scripts/mecanum_kinematics.py)
- [src/audix_pkg/scripts/warehouse_robot.py](src/audix_pkg/scripts/warehouse_robot.py)
- [src/audix_pkg/scripts/waypoints_control.py](src/audix_pkg/scripts/waypoints_control.py)
- [src/audix_pkg/scripts/waypoints_final.py](src/audix_pkg/scripts/waypoints_final.py)

## Ownership Split

### Raspberry Pi

The Raspberry Pi owns high-level autonomy and ROS orchestration.

Responsibilities:
- ROS 2 runtime
- `arena_roamer.py`
- `robot_localization` EKF
- IR topic adaptation via `ir_digital_bridge.py`
- high-level enable and supervision tools
- RViz and debugging tools
- launch orchestration for hardware mode

Pi inputs:
- wheel odometry from ESP32
- IMU data from ESP32
- digital IR states from ESP32
- optional limit switch and status telemetry from ESP32

Pi outputs:
- `/cmd_vel`
- `/robot_enable`
- optional future lift command transport

### ESP32

The ESP32 owns low-level control and hardware-facing timing-sensitive work.

Responsibilities:
- encoder acquisition
- IMU acquisition
- wheel odometry generation
- mecanum inverse kinematics
- per-wheel PID control
- PWM and motor direction output
- command timeout and low-level safety handling
- telemetry publishing through micro-ROS

ESP32 outputs to Pi:
- odometry source
- IMU source
- digital IR states
- optional limit switch state
- optional wheel speed and fault telemetry

ESP32 inputs from Pi:
- `/cmd_vel`
- `/robot_enable`

## Core Rule

The migration must preserve current behavior in [src/audix_pkg/scripts/arena_roamer.py](src/audix_pkg/scripts/arena_roamer.py).

That means:
- do not move detection or reroute logic to the ESP32
- do not rewrite motion selection to fit low-level firmware
- do not redesign the scan-based obstacle interface

Instead:
- keep `arena_roamer.py` as the decision-making layer
- make hardware publishers and adapters satisfy its existing interface

## Runtime Flow

### Hardware mode

1. ESP32 publishes raw odometry, IMU, and digital IR states.
2. `ir_digital_bridge.py` converts digital IR booleans into the six scan topics expected by `arena_roamer.py`.
3. `robot_localization` fuses odometry and IMU into `/odometry/filtered`.
4. `arena_roamer.py` consumes filtered state plus the six scan topics.
5. `arena_roamer.py` publishes `/cmd_vel` and lift slider commands.
6. ESP32 consumes `/cmd_vel` and drives the base.

The hardware path must not launch Gazebo, obstacle spawning tools, or `mecanum_kinematics.py`.

### Simulation mode

Simulation remains separate and continues to provide the same effective inputs to `arena_roamer.py` through Gazebo and ROS bridges.

Simulation-only helpers such as the obstacle manager and spawn panel should remain available for testing, but they are not part of the real robot path.

## Launch Separation

Two execution paths should stay distinct:
- simulation launch path for Gazebo-based testing
- hardware launch path for Pi plus ESP32 execution

The hardware launch path should not start:
- Gazebo
- obstacle spawning tools
- sim-only controller bridges
- wheel kinematics nodes that belong on the ESP32

## Current Sprint Output

Sprint 1 establishes:
- a protected contract for `arena_roamer.py`
- a Pi-to-ESP topic contract
- a dedicated Pi hardware launch skeleton
- a hardware-specific EKF config

Sprint 2 establishes:
- a mock hardware launch that mirrors the future ESP topic contract
- a firmware scaffold under [firmware/esp32_low_level](firmware/esp32_low_level)
- explicit separation between main paths and legacy or reference paths