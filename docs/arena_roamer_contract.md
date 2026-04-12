# arena_roamer Contract

This document freezes the external contract for the current high-level navigation node at [src/audix_pkg/scripts/arena_roamer.py](src/audix_pkg/scripts/arena_roamer.py).

## Purpose

`arena_roamer.py` is the active high-level navigation and obstacle-response brain for the Audix robot stack.

Within the final Raspberry Pi plus ESP32 architecture, this file remains Pi-owned and is part of the main hardware path.

Its responsibilities are:
- consume filtered robot state and obstacle sensor inputs
- select motion in response to waypoints and nearby obstacles
- preserve the current detection, avoidance, reroute, and motion-selection behavior
- publish chassis motion commands and lift triggers

This file should be treated as a protected behavior module.

## Protected Behavior

The following behavior areas are intentionally preserved and should not be redesigned as part of hardware integration:
- IR sensor interpretation
- obstacle detection thresholds and sequencing behavior
- blocked-side memory and motion scoring
- escape and repulsion behavior
- reroute state progression
- waypoint stop, scan, and resume behavior

Allowed changes:
- interface adaptation
- ROS remapping or launch wiring
- bug fixes that preserve intended semantics
- logging or observability improvements that do not alter decisions

Disallowed changes during hardware migration:
- replacing the obstacle logic with a new planner
- rewriting reroute behavior for convenience
- changing topic semantics inside the node unless absolutely required
- moving low-level control logic into this node

## Required Inputs

The node currently expects these ROS topics:

### State input
- `/odometry/filtered` as `nav_msgs/msg/Odometry`

### Obstacle input
- `/ir_front/scan` as `sensor_msgs/msg/LaserScan`
- `/ir_front_left/scan` as `sensor_msgs/msg/LaserScan`
- `/ir_front_right/scan` as `sensor_msgs/msg/LaserScan`
- `/ir_left/scan` as `sensor_msgs/msg/LaserScan`
- `/ir_right/scan` as `sensor_msgs/msg/LaserScan`
- `/ir_back/scan` as `sensor_msgs/msg/LaserScan`

### Control and override input
- `/robot_enable` as `std_msgs/msg/Bool`
- `/avoid_cmd_vel` as `geometry_msgs/msg/Twist` for short-lived override input

## Published Outputs

- `/cmd_vel` as `geometry_msgs/msg/Twist`
- `/scissor_lift/slider` as `std_msgs/msg/Float64`
- `/debug/planned_path` as `nav_msgs/msg/Path`
- `/debug/robot_path` as `nav_msgs/msg/Path`
- `/debug/targets` as `visualization_msgs/msg/MarkerArray`
- `/debug/state` as `std_msgs/msg/String`

## Parameters It Depends On

Primary configuration comes from [src/audix_pkg/config/mission_params.yaml](src/audix_pkg/config/mission_params.yaml) and related launch-time parameter files.

Important parameter groups:
- waypoint route data
- motion limits
- obstacle detection and clearance thresholds
- reroute timing and bias values
- startup gating and timeout values
- body-frame sign conventions

## Main Path Status

Current main hardware path files:
- [src/audix_pkg/launch/pi_hardware.launch.py](src/audix_pkg/launch/pi_hardware.launch.py)
- [src/audix_pkg/scripts/arena_roamer.py](src/audix_pkg/scripts/arena_roamer.py)
- [src/audix_pkg/scripts/ir_digital_bridge.py](src/audix_pkg/scripts/ir_digital_bridge.py)
- [src/audix_pkg/config/hardware/ekf.yaml](src/audix_pkg/config/hardware/ekf.yaml)

Related but non-main files kept as legacy or reference:
- [src/audix_pkg/scripts/mission_controller.py](src/audix_pkg/scripts/mission_controller.py)
- [src/audix_pkg/scripts/mecanum_kinematics.py](src/audix_pkg/scripts/mecanum_kinematics.py)
- [src/audix_pkg/launch/hardware.launch.py](src/audix_pkg/launch/hardware.launch.py)

## Hardware Integration Rule

For real hardware, adapt the rest of the system to this contract:
- feed odometry to the EKF
- feed IMU to the EKF
- convert raw digital IR signals into the six scan topics expected here
- preserve `/cmd_vel`, `/robot_enable`, and `/scissor_lift/slider` semantics

The preferred strategy is:

wrap `arena_roamer.py`, do not redesign it