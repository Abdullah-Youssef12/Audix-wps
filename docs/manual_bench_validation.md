# Audix Manual Bench Validation

This runbook is for the first real Pi-to-ESP32 hardware session when you want to drive the base manually, verify feedback, and test the lift stepper on the Pi without starting `arena_roamer.py`.

## 1. Start the manual bench stack on the Pi

```bash
cd ~/audix/microROS
source /opt/ros/jazzy/setup.bash
source ~/micro_ros_agent_ws/install/setup.bash
source ~/audix/microROS/install/setup.bash
ros2 launch audix pi_manual_bench.launch.py serial_device:=/dev/serial0 serial_baud:=115200
```

This launch:

- starts `micro_ros_agent`
- starts the real Pi IR GPIO publisher
- starts `ir_digital_bridge.py`
- starts EKF
- does **not** start `arena_roamer.py`
- does **not** start `start_stop_node.py`

That keeps `/cmd_vel` and `/robot_enable` under your manual control.

## 2. Prove the ESP32 is connected

In a second Pi terminal:

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

- `/cmd_vel` -> `Subscription count: 1`
- `/robot_enable` -> `Subscription count: 1`
- `/odom`, `/imu`, `/limit_switch` each print one message

## 3. Enable and stop the base manually

Enable:

```bash
ros2 topic pub /robot_enable std_msgs/msg/Bool "{data: true}" --once
```

Disable:

```bash
ros2 topic pub /robot_enable std_msgs/msg/Bool "{data: false}" --once
```

Emergency stop command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## 4. Run separate base PID tests

Forward:

```bash
ros2 run audix base_pid_bench.py -- --mode forward --linear-speed 0.10 --duration-sec 2.0 --enable-robot --disable-at-end
```

Strafe:

```bash
ros2 run audix base_pid_bench.py -- --mode strafe --linear-speed 0.10 --duration-sec 2.0 --enable-robot --disable-at-end
```

Rotate:

```bash
ros2 run audix base_pid_bench.py -- --mode rotate --angular-speed 0.40 --duration-sec 2.0 --enable-robot --disable-at-end
```

What the script reports:

- expected travel or expected yaw
- actual body-frame X travel
- actual body-frame Y travel
- IMU yaw change
- odom yaw change
- average and peak reported `vx`, `vy`, and `wz`

Use these runs twice:

- first with the robot on blocks
- then on the floor with tape marks and a ruler

## 5. Watch real-time topic behavior

Run:

```bash
ros2 run audix realtime_watch.py -- --watch-seconds 30
```

It reports live status for:

- `/odom`
- `/imu`
- `/limit_switch`

The status text flags:

- `OK` when the topic rate and freshness look healthy
- `SLOW` when the rate is below the expected floor
- `GAP` when the largest inter-message gap is too large
- `STALE` when the topic has stopped updating recently

Use this while:

- the robot is idle
- the robot is moving
- you intentionally stop `/cmd_vel`
- you disable the robot
- you stop `micro_ros_agent`

## 6. Test the Pi-owned stepper + limit switch

Before the first stepper run, update:

- `src/audix_pkg/config/hardware/pi_stepper_bench.yaml`

Then use the following commands.

Watch only the lift limit switch:

```bash
ros2 run audix pi_stepper_bench.py --ros-args --params-file ~/audix/microROS/src/audix_pkg/config/hardware/pi_stepper_bench.yaml -- --command status
```

Jog up for 2 seconds:

```bash
ros2 run audix pi_stepper_bench.py --ros-args --params-file ~/audix/microROS/src/audix_pkg/config/hardware/pi_stepper_bench.yaml -- --command jog_up --duration-sec 2.0
```

Jog down for 2 seconds:

```bash
ros2 run audix pi_stepper_bench.py --ros-args --params-file ~/audix/microROS/src/audix_pkg/config/hardware/pi_stepper_bench.yaml -- --command jog_down --duration-sec 2.0
```

Home toward the configured limit switch:

```bash
ros2 run audix pi_stepper_bench.py --ros-args --params-file ~/audix/microROS/src/audix_pkg/config/hardware/pi_stepper_bench.yaml -- --command home
```

Disable the stepper driver explicitly:

```bash
ros2 run audix pi_stepper_bench.py --ros-args --params-file ~/audix/microROS/src/audix_pkg/config/hardware/pi_stepper_bench.yaml -- --command stop
```

The stepper bench tool publishes:

- `/debug/stepper_bench/status`
- `/debug/stepper_bench/limit_switch`

You can inspect them with:

```bash
ros2 topic echo /debug/stepper_bench/status
ros2 topic echo /debug/stepper_bench/limit_switch
```

## 7. Quick operator debug rules

- If `/cmd_vel` shows `Subscription count: 0`, the ESP32 is not connected through `micro_ros_agent`.
- If `/cmd_vel` shows `Subscription count: 1` but the base does not move, debug the ESP32 motor path and `robot_enable`.
- If the base PID script reports motion in the wrong axis, debug wheel order, motor polarity, or encoder sign.
- If the rotation test changes IMU yaw but not odometry yaw, debug odometry integration.
- If the stepper homes in the wrong direction, fix the direction polarity in `pi_stepper_bench.yaml`.
- If the stepper never stops on the switch, fix the switch pin, pull-up, or active-low setting before any further homing.
