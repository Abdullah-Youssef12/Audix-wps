# Mock Hardware

This folder documents the fake-ESP workflow for testing the Pi-side hardware stack before the real robot exists.

## Purpose

The goal is to run the future hardware launch structure while replacing the ESP32 with ROS mock publishers.

## Launch Entry

Use [src/audix_pkg/launch/pi_hardware_mock.launch.py](src/audix_pkg/launch/pi_hardware_mock.launch.py).

That launch starts:
- the Pi-side hardware stack from [src/audix_pkg/launch/pi_hardware.launch.py](src/audix_pkg/launch/pi_hardware.launch.py) with the micro-ROS agent disabled
- mock odometry publisher
- mock IMU publisher
- mock digital IR publisher
- mock robot-enable publisher

## Mock Nodes

The executable mock nodes live in [src/audix_pkg/scripts](src/audix_pkg/scripts) so they can be launched as normal ROS package executables:
- [src/audix_pkg/scripts/mock_imu_publisher.py](src/audix_pkg/scripts/mock_imu_publisher.py)
- [src/audix_pkg/scripts/mock_odom_publisher.py](src/audix_pkg/scripts/mock_odom_publisher.py)
- [src/audix_pkg/scripts/mock_ir_digital_publisher.py](src/audix_pkg/scripts/mock_ir_digital_publisher.py)
- [src/audix_pkg/scripts/mock_limit_switch_publisher.py](src/audix_pkg/scripts/mock_limit_switch_publisher.py)
- [src/audix_pkg/scripts/mock_robot_enable_publisher.py](src/audix_pkg/scripts/mock_robot_enable_publisher.py)

## Initial Use

Recommended first use:
- leave odometry static
- leave IMU static
- keep robot enable true
- set IR mode to `clear`

Then test obstacle response by switching IR mode to `manual` or `cycle_front`.