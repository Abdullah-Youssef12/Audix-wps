# Audix Pi + ESP32 End-to-End Bring-Up

This runbook explains where each part of the system runs, how to connect the Pi and ESP32, and which commands to use in bench mode and on the final robot.

## 1. What runs where

### ESP32

Runs the low-level motor and sensor firmware. It is built and flashed from the PlatformIO project:

- [C:\Users\TiBa\OneDrive\Spring 26\Design 2\audix_wps\microROS\firmware\esp32_low_level](</C:/Users/TiBa/OneDrive/Spring%2026/Design%202/audix_wps/microROS/firmware/esp32_low_level>)

The ESP32 subscribes to:

- `/cmd_vel`
- `/robot_enable`

The ESP32 publishes:

- `/odom`
- `/imu`
- `/limit_switch`

### Raspberry Pi / Ubuntu ROS workspace

Runs the ROS 2 Jazzy graph from:

- [C:\Users\TiBa\OneDrive\Spring 26\Design 2\audix_wps\microROS](</C:/Users/TiBa/OneDrive/Spring%2026/Design%202/audix_wps/microROS>)

The Pi side owns:

- `micro_ros_agent`
- `arena_roamer.py`
- `start_stop_node.py`
- `ir_digital_bridge.py`
- the new real-hardware `pi_ir_gpio_publisher.py`
- EKF and RViz

## 2. Bench mode versus final robot mode

### Bench mode

Use the Ubuntu laptop first.

- flash the ESP32 over its normal USB cable
- connect a separate USB-to-TTL adapter to ESP32 `GPIO16/GPIO17`
- run `micro_ros_agent` against the adapter device such as `/dev/ttyUSB0`

### Final robot mode

Move the ROS workspace onto the Pi and keep the topic contract unchanged.

- Pi hardware UART talks to the ESP32
- `serial_device` becomes `/dev/serial0`
- the rest of the launch flow stays the same

## 3. Physical connections

### Bench bring-up wiring

- USB cable from Ubuntu host to ESP32 dev board
- USB-to-TTL adapter `TX` -> ESP32 `GPIO16`
- USB-to-TTL adapter `RX` -> ESP32 `GPIO17`
- USB-to-TTL adapter `GND` -> ESP32 `GND`

The USB-to-TTL adapter is the micro-ROS transport link. The ESP32 programming USB is still used for flashing.

### Final robot wiring

- Pi UART `TX` -> ESP32 `GPIO16`
- Pi UART `RX` -> ESP32 `GPIO17`
- Pi `GND` -> ESP32 `GND`

Use 3.3 V logic only.

## 4. Pi serial configuration for final robot mode

Free the Pi hardware UART for user-space access:

```bash
sudo systemctl stop serial-getty@serial0.service
sudo systemctl disable serial-getty@serial0.service
```

Ensure UART is enabled in the Pi firmware config. On Ubuntu for Raspberry Pi this is typically:

```bash
sudo editor /boot/firmware/config.txt
```

Add or confirm:

```text
enable_uart=1
```

Then reboot and verify:

```bash
ls -l /dev/serial0
```

## 5. Bench bring-up commands

### On the Ubuntu laptop, in the ESP32 project terminal

```bash
cd ~/audix/microROS/firmware/esp32_low_level
pio run -e esp32dev
pio test -e native
pio run -e esp32dev -t upload
```

### On the Ubuntu laptop, in the ROS workspace terminal

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch audix pi_hardware.launch.py serial_device:=/dev/ttyUSB0 serial_baud:=115200 use_pi_ir_gpio:=false
```

That single launch runs:

- `micro_ros_agent`
- `ir_digital_bridge.py`
- EKF
- `arena_roamer.py`
- `start_stop_node.py`
- RViz

The real `pi_ir_gpio_publisher.py` is disabled in laptop bench mode because it requires Raspberry Pi GPIO hardware.

## 6. Final robot commands on the Pi

From the Pi terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/audix/microROS/install/setup.bash
ros2 launch audix pi_hardware.launch.py serial_device:=/dev/serial0 serial_baud:=115200
```

In final robot mode, `use_pi_ir_gpio` stays at its default `true`, so the Pi publishes the six real IR digital topics.

## 7. Validation checks

Verify the ESP32-side topics:

```bash
ros2 topic echo /odom --once
ros2 topic echo /imu --once
ros2 topic echo /limit_switch --once
```

Verify the Pi IR publisher and bridge:

```bash
ros2 topic echo /ir_front_digital --once
ros2 topic echo /ir_front/scan --once
```

Verify the robot-enable channel:

```bash
ros2 topic echo /robot_enable --once
```

Emergency stop:

```bash
ros2 topic pub /robot_enable std_msgs/msg/Bool "{data: false}" --once
```

## 8. Before using the final control PCB harness

Update the Pi GPIO mapping file to match your real GST harness:

- [C:\Users\TiBa\OneDrive\Spring 26\Design 2\audix_wps\microROS\src\audix_pkg\config\hardware\pi_ir_gpio.yaml](</C:/Users/TiBa/OneDrive/Spring%2026/Design%202/audix_wps/microROS/src/audix_pkg/config/hardware/pi_ir_gpio.yaml>)

The default BCM pins in that file are placeholders for bring-up only.

## 9. Manual bench mode

For manual PID, RTOS, and lift-stepper validation, use the dedicated manual bench launch instead of the full mission launch:

```bash
ros2 launch audix pi_manual_bench.launch.py serial_device:=/dev/serial0 serial_baud:=115200
```

That launch keeps `arena_roamer.py` and `start_stop_node.py` off, so you can drive `/cmd_vel` and `/robot_enable` manually without a second publisher fighting your tests.

Detailed operator commands for:

- separate base PID tests
- real-time topic watching
- Pi-owned stepper + limit switch tests

are in:

- [manual_bench_validation.md](./manual_bench_validation.md)

## 10. Windows-only standalone ESP testing

For a disposable Windows / PlatformIO sandbox that does not depend on the Pi or
ROS, use:

- [C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test](</C:/Users/TiBa/Documents/PlatformIO/Projects/audix_esp32_windows_test>)

Open that folder directly in VS Code if you want standalone serial bench
commands for IMU, encoders, single-wheel tests, PID X/Y/yaw tests, and RTOS
timing output.
