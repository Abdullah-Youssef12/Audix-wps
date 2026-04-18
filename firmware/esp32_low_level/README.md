# Audix ESP32 Final Firmware

This is the canonical ESP32 firmware for the final Pi-connected Audix submission.

Use this folder for:

- Ubuntu / WSL editing when validating the real Pi path
- building and flashing from PlatformIO CLI on Linux
- the final Pi-to-ESP32 runtime contract with micro-ROS over serial

Do not use this folder as the Windows-only sandbox. The standalone Windows bench
folder lives separately at:

- `C:\Users\TiBa\Documents\PlatformIO\Projects\audix_esp32_windows_test`

## Runtime contract

Pi to ESP32:

- `/cmd_vel`
- `/robot_enable`

ESP32 to Pi:

- `/odom`
- `/imu`
- `/limit_switch`

## Ubuntu / Pi build flow

On Ubuntu / WSL / the Pi, build this firmware from a real Linux path such as
`~/audix/microROS/firmware/esp32_low_level`, not from a `/mnt/c/...` path with
spaces.

The first build creates a project-local Python virtual environment under
`.pio/platformio-python-venv` so `micro_ros_platformio` can install its helper
packages without touching the system Python.

Then run:

```bash
cd ~/audix/microROS/firmware/esp32_low_level
pio run -e esp32dev
pio run -e esp32dev -t upload
```

Or use the helper script:

```bash
cd ~/audix/microROS/firmware/esp32_low_level
./tools/build_native_wsl.sh
```

## Operator path on the Pi

After flashing and wiring Pi UART to ESP32 `GPIO16/GPIO17`, launch from the ROS
workspace root:

```bash
cd ~/audix/microROS
source /opt/ros/jazzy/setup.bash
source ~/micro_ros_agent_ws/install/setup.bash
source install/setup.bash
ros2 launch audix pi_manual_bench.launch.py serial_device:=/dev/serial0 serial_baud:=115200
```

Use the existing Pi-side bench tools for:

- base PID validation
- stepper + homing validation
- RTOS / rate monitoring

Those remain in the ROS workspace and are not duplicated inside this firmware
folder.
