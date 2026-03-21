#!/usr/bin/env python3
"""
warehouse_robot.py

Monolithic fusion launcher that runs both `ArenaRoamer` and
`MissionController` in the same process and arbitrates `/cmd_vel` so
the mission controller has priority when a mission is active.

This loader-based approach keeps the original classes untouched and
avoids duplicating large amounts of code.
"""
import importlib.machinery
import importlib.util
import os
import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String


def _load_module(module_name, path):
    loader = importlib.machinery.SourceFileLoader(module_name, path)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


def main():
    rclpy.init()

    # Resolve script paths relative to workspace root. Adjust if needed.
    ws = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    arena_path = os.path.join(ws, 'src', 'audix_pkg', 'scripts', 'arena_roamer.py')
    mission_path = os.path.join(ws, 'src', 'audix_pkg', 'scripts', 'mission_controller.py')

    if not os.path.exists(arena_path) or not os.path.exists(mission_path):
        print('ERROR: expected scripts not found:', arena_path, mission_path)
        return 1

    arena_mod = _load_module('arena_roamer_mod', arena_path)
    mission_mod = _load_module('mission_controller_mod', mission_path)

    ArenaRoamer = getattr(arena_mod, 'ArenaRoamer')
    MissionController = getattr(mission_mod, 'MissionController')

    # Create a small arbiter node which will host the shared publishers.
    arbiter = Node('warehouse_arbiter')
    cmd_pub = arbiter.create_publisher(Twist, '/cmd_vel', 10)
    planned_path_pub = arbiter.create_publisher(Path, '/debug/planned_path', 10)
    robot_trail_pub = arbiter.create_publisher(Path, '/debug/robot_path', 10)
    marker_pub = arbiter.create_publisher(MarkerArray, '/debug/targets', 10)
    state_pub = arbiter.create_publisher(String, '/debug/state', 10)

    # Instantiate both nodes (they create their own timers/subscriptions)
    roamer = ArenaRoamer()
    mission = MissionController()

    # Replace their publishers with the arbiter's publishers so we have a
    # single ROS writer for each debug topic and /cmd_vel.
    try:
        roamer.cmd_pub = cmd_pub
        roamer.path_pub = planned_path_pub
        roamer.trail_pub = robot_trail_pub
        roamer.marker_pub = marker_pub
        roamer.state_pub = state_pub
    except Exception:
        pass

    try:
        mission.cmd_pub = cmd_pub
        mission.planned_path_pub = planned_path_pub
        mission.robot_trail_pub = robot_trail_pub
        mission.marker_pub = marker_pub
        mission.state_pub = state_pub
        mission.lift_pub = mission.lift_pub if hasattr(mission, 'lift_pub') else None
    except Exception:
        pass

    # Monkeypatch roamer._publish_cmd so it yields to mission when mission is active.
    if hasattr(roamer, '_publish_cmd') and hasattr(mission, 'mission_loaded'):
        roamer._publish_cmd_original = roamer._publish_cmd

        def _roamer_publish_cmd_guard(vx=0.0, vy=0.0, wz=0.0):
            # If mission is loaded and enabled, mission takes priority.
            if getattr(mission, 'mission_loaded', False) and getattr(mission, 'enabled', False):
                return
            return roamer._publish_cmd_original(vx, vy, wz)

        roamer._publish_cmd = _roamer_publish_cmd_guard

    # Ensure mission uses arbiter publisher (mission usually has higher-level control)
    if hasattr(mission, '_publish_cmd'):
        mission._publish_cmd_original = mission._publish_cmd

        def _mission_publish_cmd_proxy(vx=0.0, vy=0.0, wz=0.0):
            return mission._publish_cmd_original(vx, vy, wz)

        mission._publish_cmd = _mission_publish_cmd_proxy

    # Spin both nodes using a MultiThreadedExecutor so each node's timers/callbacks run concurrently.
    executor = MultiThreadedExecutor()
    executor.add_node(arbiter)
    executor.add_node(roamer)
    executor.add_node(mission)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(roamer)
        executor.remove_node(mission)
        executor.remove_node(arbiter)
        roamer.destroy_node()
        mission.destroy_node()
        arbiter.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
