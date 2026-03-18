# Audix ROS2 Jazzy Workspace

This repository contains the Audix warehouse robot simulation stack on ROS 2 Jazzy and Gazebo Harmonic.

## Current Canonical Stack
- Package name: `audix` (source folder: `src/audix_pkg`)
- Main launch: `midterm.launch.py`
- Gazebo world used by midterm launch: `world/debug_empty.sdf`
- Robot launch base: `scissor_gazebo.launch.py`
- Localization: `robot_localization` EKF (`config/ekf.yaml`)
- Mission node: `scripts/mission_controller.py`

## Build
Run from repository root:

```bash
cd /home/hiddenlegend07/Audix_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select audix
source install/setup.bash
```

## Reliable Launch (Recommended)
This command launches Gazebo + RViz + navigation stack and starts mission automatically.

```bash
cd /home/hiddenlegend07/Audix_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch audix midterm.launch.py use_rviz:=true use_slider_gui:=false run_navigation:=true run_cardinal_test:=false auto_start:=true auto_send_goal:=true
```

## Manual Mission Start Mode
Use this when you want the simulation to launch and wait in IDLE until you trigger it.

```bash
cd /home/hiddenlegend07/Audix_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch audix midterm.launch.py use_rviz:=true use_slider_gui:=false run_navigation:=true run_cardinal_test:=false auto_start:=false auto_send_goal:=false
```

Then trigger mission manually from another terminal:

```bash
cd /home/hiddenlegend07/Audix_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub /robot_enable std_msgs/msg/Bool "{data: true}" --once
ros2 service call /send_mission std_srvs/srv/Trigger "{}"
```

## Stop Everything
Use this whenever stale processes cause inconsistent behavior.

```bash
pkill -9 -f "ros2|gz sim|gzserver|gzclient|rviz2|parameter_bridge|static_transform_publisher|robot_state_publisher|controller_manager|spawner|joint_state_publisher|scissor_lift_mapper|odom_tf_broadcaster|mission_controller|start_stop_node|goal_sender_node|cardinal_motion_debug|waypoints|audix" || true
```

## Quick Health Checks

```bash
source /opt/ros/jazzy/setup.bash
source /home/hiddenlegend07/Audix_ws/install/setup.bash
ros2 topic info /clock
ros2 topic echo /joint_states --once
ros2 topic echo /cmd_vel --once
```

Expected behavior:
- `/clock` has at least 1 publisher while sim is running.
- `/joint_states` publishes robot joints after spawn.
- `/cmd_vel` stays zero in IDLE and becomes non-zero after mission starts.

## Known Gotchas
- `midterm.launch.py` defaults: `run_navigation:=false`, `auto_start:=false`, `auto_send_goal:=false`.
  - If you launch with defaults, robot will not autonomously move.
- If mission stays in IDLE, you must publish `/robot_enable` and call `/send_mission`.
- Multiple leftover launch processes can break timing and TF behavior. Use the kill-all command before relaunching.
- On the current main branch, keep `run_cardinal_test:=false` unless you intentionally want the cardinal debug run.

## Important Paths
- `src/audix_pkg/launch/midterm.launch.py`
- `src/audix_pkg/launch/scissor_gazebo.launch.py`
- `src/audix_pkg/scripts/mission_controller.py`
- `src/audix_pkg/config/mission_params.yaml`
- `src/audix_pkg/config/ekf.yaml`
- `src/audix_pkg/world/debug_empty.sdf`

## Team Notes
- Keep waypoints and thresholds in `mission_params.yaml` (do not hardcode in Python).
- Keep `use_sim_time: True` for simulation nodes.
- Do not modify files under `meshes/`.
