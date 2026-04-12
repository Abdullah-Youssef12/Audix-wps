# Mock Hardware

This folder documents the fake-ESP workflow for testing the Pi-side hardware stack before the real robot exists.

## Purpose

The goal is to run the future hardware launch structure while replacing the ESP32 with ROS mock publishers.

## Launch Entry

Use [src/audix_pkg/launch/pi_hardware_mock.launch.py](src/audix_pkg/launch/pi_hardware_mock.launch.py).

Example:

```bash
ros2 launch audix pi_hardware_mock.launch.py use_rviz:=true ir_scenario:=all_clear
```

That launch starts:
- the Pi-side hardware stack from [src/audix_pkg/launch/pi_hardware.launch.py](src/audix_pkg/launch/pi_hardware.launch.py) with the micro-ROS agent disabled
- mock odometry publisher
- mock IMU publisher
- mock digital IR publisher
- mock robot-enable publisher
- optional mock limit switch publisher

## Mock Nodes

The executable mock nodes live in [src/audix_pkg/scripts](src/audix_pkg/scripts) so they can be launched as normal ROS package executables:
- [src/audix_pkg/scripts/mock_imu_publisher.py](src/audix_pkg/scripts/mock_imu_publisher.py)
- [src/audix_pkg/scripts/mock_odom_publisher.py](src/audix_pkg/scripts/mock_odom_publisher.py)
- [src/audix_pkg/scripts/mock_ir_digital_publisher.py](src/audix_pkg/scripts/mock_ir_digital_publisher.py)
- [src/audix_pkg/scripts/mock_limit_switch_publisher.py](src/audix_pkg/scripts/mock_limit_switch_publisher.py)
- [src/audix_pkg/scripts/mock_robot_enable_publisher.py](src/audix_pkg/scripts/mock_robot_enable_publisher.py)

What each one simulates:
- `mock_imu_publisher.py`: future ESP IMU topic
- `mock_odom_publisher.py`: future ESP raw wheel odometry topic
- `mock_ir_digital_publisher.py`: future ESP digital IR topics
- `mock_robot_enable_publisher.py`: Pi-side enable topic needed by `arena_roamer.py`
- `mock_limit_switch_publisher.py`: optional digital input for future hardware supervision

## Initial Use

Recommended first use:
- leave odometry static
- leave IMU static
- keep robot enable true
- set `ir_scenario:=all_clear`

Then test obstacle response by changing `ir_scenario`.

## Scenario Launch Arguments

Primary scenario controls:
- `ir_scenario`: named scenario for the mock digital IR publisher
- `scenario_loop`: repeat the selected scenario continuously
- `scenario_start_delay`: hold all sensors clear before the scenario begins

Backward-compatible controls still exist:
- `ir_mode:=static` with `blocked_sensors:=front,left`
- `ir_mode:=manual` with `blocked_sensors:=front_left`
- `ir_mode:=cycle_front`

## Available Scenarios

- `all_clear`: keep all digital IR topics clear
- `front_blocked`: hold only the front IR blocked
- `front_left_blocked`: hold only the front-left IR blocked
- `front_right_blocked`: hold only the front-right IR blocked
- `left_blocked`: hold only the left IR blocked
- `right_blocked`: hold only the right IR blocked
- `transient_front_blocked`: briefly block the front IR after an initial clear period
- `persistent_front_blocked`: hold the front IR blocked after an initial clear period
- `reroute_left_case`: block the front plus right-side sensors to create a left-favoring reroute case
- `reroute_right_case`: block the front plus left-side sensors to create a right-favoring reroute case
- `obstacle_appears_after_motion`: start clear, then introduce a front obstacle after a motion window
- `obstacle_reappears_during_rejoin`: obstacle clears briefly and then reappears during the later phase of the scenario
- `side_clearance_delayed_left`: left-side blockage clears in stages to test delayed side-release behavior
- `side_clearance_delayed_right`: right-side blockage clears in stages to test delayed side-release behavior

## Example Commands

All clear baseline:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=all_clear
```

Short obstacle pulse in front:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=transient_front_blocked
```

Persistent front obstacle:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=persistent_front_blocked
```

Repeat a reroute case continuously:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=reroute_left_case scenario_loop:=true
```

Obstacle appears after motion begins:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=obstacle_appears_after_motion linear_x:=0.10
```

Obstacle returns during the later phase:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=obstacle_reappears_during_rejoin scenario_start_delay:=1.5
```

Delayed left-side clearance:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=side_clearance_delayed_left
```

Delayed right-side clearance:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=side_clearance_delayed_right
```

## What Each Scenario Is Intended To Exercise

- `all_clear`: nominal route-following without obstacle interference
- `front_blocked`: immediate obstacle detection directly ahead
- `front_left_blocked`: front-left obstacle detection and side-awareness
- `front_right_blocked`: front-right obstacle detection and side-awareness
- `left_blocked`: lateral obstacle handling on the left side
- `right_blocked`: lateral obstacle handling on the right side
- `transient_front_blocked`: detection and recovery after a short-lived obstruction
- `persistent_front_blocked`: sustained front obstruction handling
- `reroute_left_case`: a repeatable case where left-side bypass should be more attractive than right-side bypass
- `reroute_right_case`: a repeatable case where right-side bypass should be more attractive than left-side bypass
- `obstacle_appears_after_motion`: response when an obstacle appears after an initial clear run-up
- `obstacle_reappears_during_rejoin`: response when the path briefly clears and then becomes blocked again
- `side_clearance_delayed_left`: observe how the stack behaves when left-side clearance improves only gradually
- `side_clearance_delayed_right`: observe how the stack behaves when right-side clearance improves only gradually

## Observing Behavior

RViz:
- watch the robot path and debug markers from the existing Pi stack

Topic echo:

```bash
ros2 topic echo /debug/state
```

```bash
ros2 topic echo /odometry/filtered
```

```bash
ros2 topic echo /ir_front/scan --once
```

```bash
ros2 topic echo /ir_front_left/scan --once
```

```bash
ros2 topic echo /ir_front_right/scan --once
```

```bash
ros2 topic echo /cmd_vel
```

You can also observe `/odometry/filtered` to confirm the Pi stack is staying active while the mock obstacles change.

## Lightweight Monitor Helper

You can run the helper monitor in a second terminal during any scenario:

```bash
python3 tools/mock_hardware/mock_validation_monitor.py
```

It prints a compact summary of:
- `/debug/state`
- `/cmd_vel`
- `/odometry/filtered`
- currently blocked IR scan topics