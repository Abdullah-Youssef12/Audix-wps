#!/usr/bin/env python3
"""
Unified mission controller for Audix warehouse robot.

State machine:
    IDLE -> ROTATE_TO_TARGET -> MOVE_FORWARD -> ROTATE_TO_SCAN
    -> LIFTING_UP -> SCANNING -> ROTATE_BACK_TO_PATH -> LIFTING_DOWN
    -> (next waypoint or COMPLETE)

Interrupt states: OBSTACLE_HALT, OBSTACLE_REROUTE

Subscribes to 6 individual IR sensor topics, IMU, filtered odom, enable flag.
Publishes /cmd_vel (Twist) and high-level scissor lift slider commands.
Receives mission goals via /send_mission service.
"""

import math
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _euler_from_quat(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(t2)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class State(Enum):
    IDLE = auto()
    ROTATE_TO_TARGET = auto()
    MOVE_FORWARD = auto()
    ROTATE_TO_SCAN = auto()
    LIFTING_UP = auto()
    SCANNING = auto()
    LIFTING_DOWN = auto()
    ROTATE_BACK_TO_PATH = auto()
    OBSTACLE_HALT = auto()
    OBSTACLE_REROUTE = auto()
    COMPLETE = auto()


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- Declare ALL parameters ---
        self.declare_parameter('waypoints', [0.0])
        self.declare_parameter('obstacle_speed_full', 0.30)
        self.declare_parameter('obstacle_speed_slow', 0.18)
        self.declare_parameter('obstacle_speed_creep', 0.08)
        self.declare_parameter('obstacle_detect_distance', 0.25)
        self.declare_parameter('obstacle_danger_distance', 0.15)
        self.declare_parameter('obstacle_side_min', 0.15)
        self.declare_parameter('front_blocked_threshold', 3)
        self.declare_parameter('clear_threshold', 5)
        self.declare_parameter('reroute_strafe_speed', 0.16)
        self.declare_parameter('reroute_creep_speed', 0.08)
        self.declare_parameter('reroute_lateral_distance', 0.42)
        self.declare_parameter('reroute_advance_distance', 0.45)
        self.declare_parameter('reroute_return_lookahead', 0.18)
        self.declare_parameter('reroute_return_tolerance', 0.03)
        self.declare_parameter('reroute_rejoin_extra_distance', 0.30)
        self.declare_parameter('reroute_side_clearance', 0.20)
        self.declare_parameter('reroute_side_clear_cycles', 6)
        self.declare_parameter('obstacle_halt_timeout', 2.5)
        self.declare_parameter('reroute_timeout', 12.0)
        self.declare_parameter('obstacle_sensor_warmup_sec', 2.0)
        self.declare_parameter('lift_dwell_time', 3.0)
        self.declare_parameter('lift_position_tolerance', 0.003)
        self.declare_parameter('lift_motion_time', 2.0)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('scan_angle_tolerance', 0.08)
        self.declare_parameter('scan_rotation_kp_scale', 1.6)
        self.declare_parameter('scan_rotation_min_speed', 0.12)
        self.declare_parameter('scan_rotation_max_speed', 1.2)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('scan_turn_degrees', 90.0)
        self.declare_parameter('scan_turn_direction', 'left')
        self.declare_parameter('scripted_skid_scan_mode', False)
        self.declare_parameter('skid_lateral_distance', 0.6)
        self.declare_parameter('end_goal_distance', 4.0)
        self.declare_parameter('scan_use_lift', False)
        self.declare_parameter('straight_line_only', True)
        self.declare_parameter('straight_line_distance', 3.0)
        self.declare_parameter('enable_obstacle_avoidance', False)
        self.declare_parameter('enable_stop_scan', False)
        self.declare_parameter('status_log_to_terminal', True)
        self.declare_parameter('status_log_period_sec', 0.5)
        self.declare_parameter('debug_visualization', True)
        self.declare_parameter('debug_path_point_spacing', 0.03)
        self.declare_parameter('debug_path_max_points', 2500)
        self.declare_parameter('robot_center_offset_x', -0.16)
        self.declare_parameter('robot_center_offset_y', -0.15)
        self.declare_parameter('straight_line_lift_height', 0.0)

        # Load parameters
        self.obstacle_speed_full = float(self.get_parameter('obstacle_speed_full').value)
        self.obstacle_speed_slow = float(self.get_parameter('obstacle_speed_slow').value)
        self.obstacle_speed_creep = float(self.get_parameter('obstacle_speed_creep').value)
        self.obstacle_detect_distance = float(self.get_parameter('obstacle_detect_distance').value)
        self.obstacle_danger_distance = float(self.get_parameter('obstacle_danger_distance').value)
        self.obstacle_side_min = float(self.get_parameter('obstacle_side_min').value)
        self.front_blocked_threshold = int(self.get_parameter('front_blocked_threshold').value)
        self.clear_threshold = int(self.get_parameter('clear_threshold').value)
        self.reroute_strafe_speed = float(self.get_parameter('reroute_strafe_speed').value)
        self.reroute_creep_speed = float(self.get_parameter('reroute_creep_speed').value)
        self.reroute_lateral_distance = float(self.get_parameter('reroute_lateral_distance').value)
        self.reroute_advance_distance = float(self.get_parameter('reroute_advance_distance').value)
        self.reroute_return_lookahead = float(self.get_parameter('reroute_return_lookahead').value)
        self.reroute_return_tolerance = float(self.get_parameter('reroute_return_tolerance').value)
        self.reroute_rejoin_extra_distance = float(self.get_parameter('reroute_rejoin_extra_distance').value)
        self.reroute_side_clearance = float(self.get_parameter('reroute_side_clearance').value)
        self.reroute_side_clear_cycles = int(self.get_parameter('reroute_side_clear_cycles').value)
        self.obstacle_halt_timeout = float(self.get_parameter('obstacle_halt_timeout').value)
        self.reroute_timeout = float(self.get_parameter('reroute_timeout').value)
        self.obstacle_warmup = self.get_parameter('obstacle_sensor_warmup_sec').value
        self.lift_dwell = self.get_parameter('lift_dwell_time').value
        self.lift_tol = self.get_parameter('lift_position_tolerance').value
        self.lift_motion_time = float(self.get_parameter('lift_motion_time').value)
        self.lin_kp = self.get_parameter('linear_kp').value
        self.ang_kp = self.get_parameter('angular_kp').value
        self.scan_ang_tol = float(self.get_parameter('scan_angle_tolerance').value)
        self.scan_rotation_kp_scale = float(self.get_parameter('scan_rotation_kp_scale').value)
        self.scan_rotation_min_speed = float(self.get_parameter('scan_rotation_min_speed').value)
        self.scan_rotation_max_speed = float(self.get_parameter('scan_rotation_max_speed').value)
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.ang_tol = self.get_parameter('angle_tolerance').value
        self.scan_turn_rad = math.radians(self.get_parameter('scan_turn_degrees').value)
        self.scan_turn_direction = str(self.get_parameter('scan_turn_direction').value).lower()
        self.scripted_skid_scan_mode = bool(self.get_parameter('scripted_skid_scan_mode').value)
        self.skid_lateral_distance = float(self.get_parameter('skid_lateral_distance').value)
        self.end_goal_distance = float(self.get_parameter('end_goal_distance').value)
        self.scan_use_lift = bool(self.get_parameter('scan_use_lift').value)
        self.straight_line_only = bool(self.get_parameter('straight_line_only').value)
        self.straight_line_distance = float(self.get_parameter('straight_line_distance').value)
        self.enable_obstacle_avoidance = bool(self.get_parameter('enable_obstacle_avoidance').value)
        self.enable_stop_scan = bool(self.get_parameter('enable_stop_scan').value)
        self.status_log_to_terminal = bool(self.get_parameter('status_log_to_terminal').value)
        self.status_log_period = float(self.get_parameter('status_log_period_sec').value)
        self.debug_viz = self.get_parameter('debug_visualization').value
        self.debug_path_spacing = self.get_parameter('debug_path_point_spacing').value
        self.debug_path_max_points = self.get_parameter('debug_path_max_points').value
        self.robot_center_offset_x = float(self.get_parameter('robot_center_offset_x').value)
        self.robot_center_offset_y = float(self.get_parameter('robot_center_offset_y').value)
        self.straight_line_lift_height = float(self.get_parameter('straight_line_lift_height').value)

        # Parse waypoints from YAML: flat list [x,y,yaw,h, x,y,yaw,h, ...]
        wp_flat = self.get_parameter('waypoints').value
        self.waypoints = []
        if wp_flat and len(wp_flat) >= 4:
            # Could be list-of-lists or flat
            if isinstance(wp_flat[0], (list, tuple)):
                for wp in wp_flat:
                    self.waypoints.append((float(wp[0]), float(wp[1]),
                                           float(wp[2]), float(wp[3])))
            else:
                for i in range(0, len(wp_flat), 4):
                    self.waypoints.append((float(wp_flat[i]), float(wp_flat[i+1]),
                                           float(wp_flat[i+2]), float(wp_flat[i+3])))

        # --- State ---
        self.state = State.IDLE
        self.enabled = False
        self.mission_loaded = False
        self.current_wp_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.center_x = 0.0
        self.center_y = 0.0

        # IR sensor readings
        self.ir = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'back': float('inf'),
        }

        # Obstacle tracking
        self.front_blocked_count = 0
        self.clear_count = 0
        self.pre_obstacle_state = None
        self.reroute_waypoints = []
        self.reroute_step = 0
        self.reroute_direction = 0.0
        self.reroute_start_x = 0.0
        self.reroute_start_y = 0.0
        self.reroute_start_progress = 0.0
        self.reroute_rejoin_min_progress = 0.0
        self.reroute_side_clear_count = 0
        self.reroute_entry_time = None
        self.path_line_start = None
        self.path_line_end = None
        self.node_start_time = self.get_clock().now()

        # Lift state
        self.current_lift_pos = 0.0
        self.target_lift_pos = 0.0
        self.scan_start_time = None
        self.scan_heading_target = 0.0
        self.resume_heading_target = 0.0
        self.scan_waypoint_indices = set()

        # Debug visualization state
        self.robot_path_points = []
        self.last_path_point = None

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(
            Float64,
            '/scissor_lift/slider',
            10
        )
        self.planned_path_pub = self.create_publisher(
            Path,
            '/debug/planned_path',
            10,
        )
        self.robot_trail_pub = self.create_publisher(
            Path,
            '/debug/robot_path',
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/debug/targets', 10)
        self.state_pub = self.create_publisher(String, '/debug/state', 10)

        # --- Subscribers ---
        ir_topics = {
            'front': '/ir_front/scan',
            'front_left': '/ir_front_left/scan',
            'front_right': '/ir_front_right/scan',
            'left': '/ir_left/scan',
            'right': '/ir_right/scan',
            'back': '/ir_back/scan',
        }
        for key, topic in ir_topics.items():
            self.create_subscription(
                LaserScan, topic,
                lambda msg, k=key: self._ir_cb(k, msg), 10
            )

        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.create_subscription(Bool, '/robot_enable', self._enable_cb, 10)

        # --- Service ---
        self.create_service(Trigger, '/send_mission', self._mission_srv_cb)

        # --- Control timer at 20 Hz ---
        self.create_timer(0.05, self._control_loop)
        if self.status_log_to_terminal:
            self.create_timer(self.status_log_period, self._status_log_cb)
        if self.debug_viz:
            self.create_timer(0.2, self._publish_debug_viz)

        self.get_logger().info(
            f'Mission controller ready. {len(self.waypoints)} waypoints loaded.'
        )

    # ================================================================
    # Callbacks
    # ================================================================
    def _ir_cb(self, key: str, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        # Mask readings outside [range_min, range_max) — Gazebo returns range_max for "no hit"
        rmin = msg.range_min if msg.range_min > 0 else 0.01
        rmax = msg.range_max if msg.range_max > 0 else float('inf')
        ranges[(ranges < rmin) | (ranges >= rmax) | np.isnan(ranges)] = np.inf
        valid = ranges[np.isfinite(ranges)]
        self.ir[key] = float(np.min(valid)) if len(valid) > 0 else float('inf')

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = _euler_from_quat(q.x, q.y, q.z, q.w)
        self.yaw = yaw
        self.center_x, self.center_y = self._center_xy(self.x, self.y, self.yaw)

    def _imu_cb(self, msg: Imu):
        pass  # EKF handles IMU fusion; kept for potential direct use

    def _enable_cb(self, msg: Bool):
        prev = self.enabled
        self.enabled = msg.data
        if self.enabled and not prev:
            self.get_logger().info('Robot ENABLED — start signal received')
        elif not self.enabled and prev:
            self.get_logger().warn('Robot DISABLED — emergency stop')
            self._publish_stop()
            self.state = State.IDLE

    def _mission_srv_cb(self, request, response):
        if self.scripted_skid_scan_mode or self.straight_line_only or self.waypoints:
            self.mission_loaded = True
            response.success = True
            if self.scripted_skid_scan_mode:
                response.message = 'Mission loaded: scripted skid-scan mode'
            elif self.straight_line_only:
                response.message = 'Mission loaded: straight-line debug mode'
            else:
                response.message = f'Mission loaded: {len(self.waypoints)} waypoints'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'No waypoints in parameters'
            self.get_logger().error(response.message)
        return response

    # ================================================================
    # Helpers
    # ================================================================
    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = max(-self.max_lin, min(self.max_lin, vx))
        cmd.linear.y = max(-self.max_lin, min(self.max_lin, vy))
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, wz))
        self.cmd_pub.publish(cmd)

    def _center_xy(self, base_x, base_y, yaw):
        center_x = (
            base_x
            + math.cos(yaw) * self.robot_center_offset_x
            - math.sin(yaw) * self.robot_center_offset_y
        )
        center_y = (
            base_y
            + math.sin(yaw) * self.robot_center_offset_x
            + math.cos(yaw) * self.robot_center_offset_y
        )
        return center_x, center_y

    def _closest_front_obstacle(self):
        return min(self.ir['front'], self.ir['front_left'], self.ir['front_right'])

    def _obstacle_detection_active(self):
        elapsed = (self.get_clock().now() - self.node_start_time).nanoseconds / 1e9
        return elapsed >= self.obstacle_warmup

    def _front_obstacle_detected(self):
        if not self._obstacle_detection_active():
            return False
        return self._closest_front_obstacle() < self.obstacle_detect_distance

    def _reset_obstacle_counters(self):
        self.front_blocked_count = 0
        self.clear_count = 0

    def _update_obstacle_counters(self):
        if not self._obstacle_detection_active():
            self._reset_obstacle_counters()
            return

        if self._front_obstacle_detected():
            self.front_blocked_count += 1
            self.clear_count = 0
        else:
            self.clear_count += 1
            self.front_blocked_count = 0

    def _max_allowed_speed(self):
        closest = self._closest_front_obstacle()
        if closest <= self.obstacle_danger_distance:
            return self.obstacle_speed_creep
        if closest <= self.obstacle_detect_distance:
            return self.obstacle_speed_slow
        return self.obstacle_speed_full

    def _apply_side_collision_prevention(self, vx, vy):
        if vy > 0.0 and min(self.ir['left'], self.ir['front_left']) < self.obstacle_side_min:
            vy = 0.0
        if vy < 0.0 and min(self.ir['right'], self.ir['front_right']) < self.obstacle_side_min:
            vy = 0.0
        if vx < 0.0 and self.ir['back'] < self.obstacle_side_min:
            vx = 0.0
        return vx, vy

    def _publish_motion(self, vx=0.0, vy=0.0, wz=0.0, apply_speed_zone=True):
        wz = max(-self.max_ang, min(self.max_ang, wz))
        planar_speed = math.hypot(vx, vy)
        if apply_speed_zone and planar_speed > 1e-6:
            speed_cap = min(self._max_allowed_speed(), self.max_lin)
            if planar_speed > speed_cap:
                scale = speed_cap / planar_speed
                vx *= scale
                vy *= scale

        if abs(wz) > 1e-6:
            # Convert desired robot-center motion into base-frame motion so the
            # center of the chassis follows the path and rotation happens about center.
            vx += wz * self.robot_center_offset_y
            vy -= wz * self.robot_center_offset_x

        vx, vy = self._apply_side_collision_prevention(vx, vy)

        self._publish_cmd(vx=vx, vy=vy, wz=wz)

    def _publish_lift(self, slider_value):
        """Publish a normalized slider command so scissor_lift_mapper applies the agreed joint mapping."""
        msg = Float64()
        msg.data = max(0.0, min(1.0, slider_value))
        self.current_lift_pos = msg.data
        self.lift_pub.publish(msg)

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _append_robot_path(self):
        if self.last_path_point is None:
            self.last_path_point = (self.center_x, self.center_y)
            self.robot_path_points.append(self._make_pose(self.center_x, self.center_y, self.yaw))
            return
        dx = self.center_x - self.last_path_point[0]
        dy = self.center_y - self.last_path_point[1]
        if math.hypot(dx, dy) < self.debug_path_spacing:
            return
        self.last_path_point = (self.center_x, self.center_y)
        self.robot_path_points.append(self._make_pose(self.center_x, self.center_y, self.yaw))
        if len(self.robot_path_points) > self.debug_path_max_points:
            self.robot_path_points = self.robot_path_points[-self.debug_path_max_points:]

    def _build_planned_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'

        poses = [self._make_pose(self.center_x, self.center_y, self.yaw)]
        if self.state == State.OBSTACLE_REROUTE and self.reroute_waypoints:
            for i in range(len(self.reroute_waypoints)):
                wx, wy = self.reroute_waypoints[i]
                poses.append(self._make_pose(wx, wy, 0.0))

        for i in range(self.current_wp_idx, len(self.waypoints)):
            wx, wy, wyaw, _ = self.waypoints[i]
            poses.append(self._make_pose(wx, wy, wyaw))

        path.poses = poses
        return path

    def _build_target_markers(self):
        now = self.get_clock().now().to_msg()
        markers = []

        clear = Marker()
        clear.header.frame_id = 'odom'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.append(clear)

        for i, wp in enumerate(self.waypoints):
            wx, wy, _, _ = wp

            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = now
            marker.ns = 'mission_targets'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.14
            marker.scale.y = 0.14
            marker.scale.z = 0.14

            if i < self.current_wp_idx:
                marker.color.r = 0.3
                marker.color.g = 0.3
                marker.color.b = 0.3
                marker.color.a = 0.9
            elif i == self.current_wp_idx and self.state != State.COMPLETE:
                marker.color.r = 1.0
                marker.color.g = 0.85
                marker.color.b = 0.1
                marker.color.a = 1.0
            else:
                marker.color.r = 0.1
                marker.color.g = 0.75
                marker.color.b = 1.0
                marker.color.a = 0.9

            markers.append(marker)

            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = now
            label.ns = 'mission_labels'
            label.id = 100 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = wx
            label.pose.position.y = wy
            label.pose.position.z = 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'WP{i}'
            markers.append(label)

        if self.state == State.OBSTACLE_REROUTE:
            for i in range(len(self.reroute_waypoints)):
                rx, ry = self.reroute_waypoints[i]
                reroute = Marker()
                reroute.header.frame_id = 'odom'
                reroute.header.stamp = now
                reroute.ns = 'reroute_targets'
                reroute.id = 200 + i
                reroute.type = Marker.CUBE
                reroute.action = Marker.ADD
                reroute.pose.position.x = rx
                reroute.pose.position.y = ry
                reroute.pose.orientation.w = 1.0
                reroute.scale.x = 0.12
                reroute.scale.y = 0.12
                reroute.scale.z = 0.12
                reroute.color.r = 1.0
                reroute.color.g = 0.2
                reroute.color.b = 0.9
                reroute.color.a = 1.0
                markers.append(reroute)

        state_marker = Marker()
        state_marker.header.frame_id = 'odom'
        state_marker.header.stamp = now
        state_marker.ns = 'mission_state'
        state_marker.id = 900
        state_marker.type = Marker.TEXT_VIEW_FACING
        state_marker.action = Marker.ADD
        state_marker.pose.position.x = self.center_x
        state_marker.pose.position.y = self.center_y
        state_marker.pose.position.z = 0.45
        state_marker.pose.orientation.w = 1.0
        state_marker.scale.z = 0.10
        state_marker.color.r = 0.95
        state_marker.color.g = 0.95
        state_marker.color.b = 0.95
        state_marker.color.a = 1.0
        state_marker.text = (
            f'{self.state.name} | wp={self.current_wp_idx} '
            f'| f={self.ir["front"]:.2f} fl={self.ir["front_left"]:.2f} '
            f'fr={self.ir["front_right"]:.2f}'
        )
        markers.append(state_marker)

        arr = MarkerArray()
        arr.markers = markers
        return arr

    def _publish_debug_viz(self):
        if not self.debug_viz:
            return

        trail = Path()
        trail.header.stamp = self.get_clock().now().to_msg()
        trail.header.frame_id = 'odom'

        self._append_robot_path()
        trail.poses = self.robot_path_points
        self.robot_trail_pub.publish(trail)

        self.planned_path_pub.publish(self._build_planned_path())
        self.marker_pub.publish(self._build_target_markers())

        s = String()
        s.data = (
            f'state={self.state.name}, wp={self.current_wp_idx}, '
            f'front={self.ir["front"]:.3f}, front_left={self.ir["front_left"]:.3f}, '
            f'front_right={self.ir["front_right"]:.3f}, cx={self.center_x:.3f}, cy={self.center_y:.3f}'
        )
        self.state_pub.publish(s)

    def _height_to_slider(self, height_m):
        """Convert desired platform height to the normalized slider used by scissor_lift_mapper."""
        max_height = 0.3383
        clamped_height = max(0.0, min(max_height, height_m))
        return clamped_height / max_height if max_height > 1e-6 else 0.0

    def _scan_direction_sign(self):
        return -1.0 if self.scan_turn_direction == 'right' else 1.0

    def _compute_resume_heading(self, tx, ty):
        if self.current_wp_idx + 1 < len(self.waypoints):
            nx, ny, _, _ = self.waypoints[self.current_wp_idx + 1]
            return math.atan2(ny - ty, nx - tx)
        if self.current_wp_idx > 0:
            px, py, _, _ = self.waypoints[self.current_wp_idx - 1]
            return math.atan2(ty - py, tx - px)
        return self.yaw

    def _start_skid_scan_mission(self):
        base_yaw = self.yaw
        self.resume_heading_target = base_yaw
        fx = math.cos(base_yaw)
        fy = math.sin(base_yaw)
        rx = math.cos(base_yaw - math.pi / 2.0)
        ry = math.sin(base_yaw - math.pi / 2.0)

        lateral_distance = self.skid_lateral_distance
        x0, y0 = self.center_x, self.center_y
        p1 = (x0 + lateral_distance * rx, y0 + lateral_distance * ry)
        p2 = (p1[0] - 2.0 * lateral_distance * rx, p1[1] - 2.0 * lateral_distance * ry)
        p3 = (p2[0] + lateral_distance * rx, p2[1] + lateral_distance * ry)
        p4 = (
            p3[0] + self.end_goal_distance * fx,
            p3[1] + self.end_goal_distance * fy,
        )

        self.waypoints = [
            (p1[0], p1[1], base_yaw, 0.0),
            (p2[0], p2[1], base_yaw, 0.0),
            (p3[0], p3[1], base_yaw, 0.0),
            (p4[0], p4[1], base_yaw, 0.0),
        ]
        self.scan_waypoint_indices = {0, 1}
        self.current_wp_idx = 0
        self.path_line_start = (self.center_x, self.center_y)
        self.path_line_end = self.waypoints[0][:2]
        self.state = State.MOVE_FORWARD
        self.get_logger().info(
            'Mission started! Skid sequence generated: RIGHT -> LEFT -> CENTER -> GOAL'
        )

    def _start_straight_line_mission(self):
        tx = self.center_x - self.straight_line_distance * math.cos(self.yaw)
        ty = self.center_y - self.straight_line_distance * math.sin(self.yaw)
        self.waypoints = [(tx, ty, self.yaw, self.straight_line_lift_height)]
        self.scan_waypoint_indices = {0} if self.enable_stop_scan else set()
        self.current_wp_idx = 0
        self.resume_heading_target = self.yaw
        self.path_line_start = (self.center_x, self.center_y)
        self.path_line_end = (tx, ty)
        self.state = State.MOVE_FORWARD
        self.get_logger().info(
            'Mission started! Centered straight-line target set to '
            f'({tx:.2f}, {ty:.2f})'
        )

    def _status_log_cb(self):
        self.get_logger().info(
            'state=%s wp=%d pos=(%.2f, %.2f) yaw=%.2f | '
            'ir[f=%.2f fl=%.2f fr=%.2f l=%.2f r=%.2f b=%.2f]' % (
                self.state.name,
                self.current_wp_idx,
                self.center_x,
                self.center_y,
                self.yaw,
                self.ir['front'],
                self.ir['front_left'],
                self.ir['front_right'],
                self.ir['left'],
                self.ir['right'],
                self.ir['back'],
            )
        )

    def _reset_reroute_state(self):
        self.reroute_waypoints = []
        self.reroute_step = 0
        self.reroute_direction = 0.0
        self.reroute_start_x = self.center_x
        self.reroute_start_y = self.center_y
        self.reroute_start_progress = self._path_progress(self.center_x, self.center_y)
        self.reroute_rejoin_min_progress = self.reroute_start_progress
        self.reroute_side_clear_count = 0
        self.reroute_entry_time = None

    def _point_to_line_distance(self, px, py):
        if self.path_line_start is None or self.path_line_end is None:
            return 0.0

        _, lateral = self._path_unit_vectors()
        dx = px - self.path_line_start[0]
        dy = py - self.path_line_start[1]
        return dx * lateral[0] + dy * lateral[1]

    def _path_unit_vectors(self):
        if self.path_line_start is None or self.path_line_end is None:
            return (math.cos(self.yaw), math.sin(self.yaw)), (-math.sin(self.yaw), math.cos(self.yaw))

        dx = self.path_line_end[0] - self.path_line_start[0]
        dy = self.path_line_end[1] - self.path_line_start[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (math.cos(self.yaw), math.sin(self.yaw)), (-math.sin(self.yaw), math.cos(self.yaw))

        tangent = (dx / norm, dy / norm)
        lateral = (-tangent[1], tangent[0])
        return tangent, lateral

    def _path_progress(self, px, py):
        if self.path_line_start is None:
            return 0.0
        tangent, _ = self._path_unit_vectors()
        dx = px - self.path_line_start[0]
        dy = py - self.path_line_start[1]
        return dx * tangent[0] + dy * tangent[1]

    def _point_on_path(self, progress, lateral_offset=0.0):
        if self.path_line_start is None:
            return self.center_x, self.center_y
        tangent, lateral = self._path_unit_vectors()
        return (
            self.path_line_start[0] + progress * tangent[0] + lateral_offset * lateral[0],
            self.path_line_start[1] + progress * tangent[1] + lateral_offset * lateral[1],
        )

    def _front_arc_clear(self):
        return min(self.ir['front'], self.ir['front_left'], self.ir['front_right']) > self.obstacle_detect_distance

    def _reroute_obstacle_side_distance(self):
        if self.reroute_direction > 0.0:
            return min(self.ir['right'], self.ir['front_right'])
        return min(self.ir['left'], self.ir['front_left'])

    def _update_reroute_side_clear_count(self):
        if self._reroute_obstacle_side_distance() >= self.reroute_side_clearance:
            self.reroute_side_clear_count += 1
        else:
            self.reroute_side_clear_count = 0

    def _reroute_ready_to_rejoin(self, current_progress):
        self._update_reroute_side_clear_count()
        return (
            current_progress >= self.reroute_rejoin_min_progress
            and self.reroute_side_clear_count >= self.reroute_side_clear_cycles
        )

    def _drive_toward_point(self, target_x, target_y, max_speed, heading_target=None, apply_speed_zone=False):
        dx = target_x - self.center_x
        dy = target_y - self.center_y
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        body_x = dx * cos_y + dy * sin_y
        body_y = -dx * sin_y + dy * cos_y
        norm = math.hypot(body_x, body_y)
        if norm < 1e-6:
            self._publish_motion(wz=0.0, apply_speed_zone=apply_speed_zone)
            return

        vx = max_speed * body_x / norm
        vy = max_speed * body_y / norm
        wz = 0.0
        if heading_target is not None:
            heading_error = self._normalize(heading_target - self.yaw)
            wz = self.ang_kp * heading_error * 0.25
        self._publish_motion(vx=vx, vy=vy, wz=wz, apply_speed_zone=apply_speed_zone)

    def _drive_world_vector(self, world_vx, world_vy, heading_target=None, apply_speed_zone=False):
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        body_x = world_vx * cos_y + world_vy * sin_y
        body_y = -world_vx * sin_y + world_vy * cos_y
        wz = 0.0
        if heading_target is not None:
            heading_error = self._normalize(heading_target - self.yaw)
            wz = self.ang_kp * heading_error * 0.25
        self._publish_motion(vx=body_x, vy=body_y, wz=wz, apply_speed_zone=apply_speed_zone)

    def _choose_reroute_direction(self):
        left_clearance = min(self.ir['left'], self.ir['front_left'])
        right_clearance = min(self.ir['right'], self.ir['front_right'])
        if left_clearance > right_clearance:
            return 1.0
        if right_clearance > left_clearance:
            return -1.0
        return 1.0

    def _enter_obstacle_halt(self):
        self.pre_obstacle_state = self.state
        self.state = State.OBSTACLE_HALT
        self.reroute_entry_time = self.get_clock().now()
        self.clear_count = 0
        self._publish_stop()
        self.get_logger().warn('Obstacle ahead — entering halt state')

    def _enter_reroute(self):
        self.state = State.OBSTACLE_REROUTE
        self.reroute_step = 1
        self.reroute_direction = self._choose_reroute_direction()
        self.reroute_start_x = self.center_x
        self.reroute_start_y = self.center_y
        self.reroute_start_progress = self._path_progress(self.center_x, self.center_y)
        self.reroute_rejoin_min_progress = (
            self.reroute_start_progress
            + self.reroute_advance_distance
            + self.reroute_rejoin_extra_distance
        )
        self.reroute_side_clear_count = 0
        self.reroute_entry_time = self.get_clock().now()

        lateral_offset = self.reroute_direction * self.reroute_lateral_distance
        x1, y1 = self._point_on_path(self.reroute_start_progress, lateral_offset)
        x2, y2 = self._point_on_path(
            self.reroute_start_progress + self.reroute_advance_distance,
            lateral_offset,
        )
        x3, y3 = self._point_on_path(
            self.reroute_start_progress + self.reroute_advance_distance + self.reroute_return_lookahead,
            0.0,
        )
        self.reroute_waypoints = [(x1, y1), (x2, y2), (x3, y3)]
        self._reset_obstacle_counters()
        self.get_logger().warn(
            'Obstacle remained blocked — rerouting to the %s' %
            ('left' if self.reroute_direction > 0.0 else 'right')
        )

    def _reroute_timed_out(self):
        if self.reroute_entry_time is None:
            return False
        elapsed = (self.get_clock().now() - self.reroute_entry_time).nanoseconds / 1e9
        return elapsed >= self.reroute_timeout

    def _halt_timed_out(self):
        if self.reroute_entry_time is None:
            return False
        elapsed = (self.get_clock().now() - self.reroute_entry_time).nanoseconds / 1e9
        return elapsed >= self.obstacle_halt_timeout

    def _rotation_command(self, yaw_error, speed_scale=1.0, max_speed=None, min_speed=0.0):
        limit = self.max_ang if max_speed is None else min(self.max_ang, max_speed)
        command = max(-limit, min(limit, self.ang_kp * speed_scale * yaw_error))
        if min_speed > 0.0 and abs(yaw_error) > 1e-6:
            magnitude = max(min_speed, abs(command))
            command = math.copysign(min(limit, magnitude), yaw_error)
        return command

    def _handle_reroute(self):
        if self._reroute_timed_out():
            self._publish_stop()
            self.state = State.COMPLETE
            self.get_logger().error('Reroute timed out — stopping mission for safety')
            return

        signed_lateral_distance = self._point_to_line_distance(self.center_x, self.center_y)
        lateral_distance = self.reroute_direction * signed_lateral_distance
        current_progress = self._path_progress(self.center_x, self.center_y)
        target_lateral_offset = self.reroute_direction * self.reroute_lateral_distance
        tangent, lateral = self._path_unit_vectors()

        if self.reroute_step == 1:
            if lateral_distance >= self.reroute_lateral_distance - self.reroute_return_tolerance:
                self.reroute_step = 2
                return
            lateral_error = target_lateral_offset - signed_lateral_distance
            lateral_speed = max(-self.reroute_strafe_speed, min(self.reroute_strafe_speed, 1.5 * lateral_error))
            self._drive_world_vector(
                lateral[0] * lateral_speed,
                lateral[1] * lateral_speed,
                self.resume_heading_target,
                apply_speed_zone=False,
            )
            return

        if self.reroute_step == 2:
            lateral_error = target_lateral_offset - signed_lateral_distance
            lateral_correction = max(-0.06, min(0.06, 1.2 * lateral_error))
            if (
                self._front_arc_clear()
                and self._reroute_ready_to_rejoin(current_progress)
            ):
                self.reroute_step = 3
                return
            self._drive_world_vector(
                tangent[0] * self.reroute_creep_speed + lateral[0] * lateral_correction,
                tangent[1] * self.reroute_creep_speed + lateral[1] * lateral_correction,
                self.resume_heading_target,
                apply_speed_zone=False,
            )
            return

        if self.reroute_step == 3:
            if abs(signed_lateral_distance) <= self.reroute_return_tolerance and self._front_arc_clear():
                self.reroute_step = 4
                return
            if not self._front_arc_clear() or not self._reroute_ready_to_rejoin(current_progress):
                self.reroute_step = 2
                return
            lateral_correction = max(-self.reroute_strafe_speed, min(self.reroute_strafe_speed, -1.5 * signed_lateral_distance))
            self._drive_world_vector(
                tangent[0] * max(self.reroute_creep_speed, 0.12) + lateral[0] * lateral_correction,
                tangent[1] * max(self.reroute_creep_speed, 0.12) + lateral[1] * lateral_correction,
                self.resume_heading_target,
                apply_speed_zone=False,
            )
            return

        self._reset_reroute_state()
        self.state = self.pre_obstacle_state or State.MOVE_FORWARD
        self.get_logger().info('Reroute complete — resuming waypoint tracking')

    # ================================================================
    # Main control loop
    # ================================================================
    def _control_loop(self):
        # Emergency stop check
        if not self.enabled and self.state != State.IDLE:
            self._publish_stop()
            self.state = State.IDLE
            return

        # ---- IDLE ----
        if self.state == State.IDLE:
            self._publish_stop()
            if self.enabled and self.mission_loaded:
                if self.scripted_skid_scan_mode:
                    self._start_skid_scan_mission()
                elif self.straight_line_only:
                    self._start_straight_line_mission()
                else:
                    self.scan_waypoint_indices = set(range(len(self.waypoints)))
                    self.current_wp_idx = 0
                    if self.waypoints:
                        self.path_line_start = (self.center_x, self.center_y)
                        self.path_line_end = self.waypoints[0][:2]
                    self.state = State.ROTATE_TO_TARGET
                    self.get_logger().info('Mission started! Heading to waypoint 0')
            return

        # Get current target
        if self.current_wp_idx < len(self.waypoints):
            tx, ty, tyaw, tlift = self.waypoints[self.current_wp_idx]
        else:
            self.state = State.COMPLETE
            self._publish_stop()
            self.get_logger().info('ALL WAYPOINTS COMPLETE — mission done!')
            return

        dx = tx - self.center_x
        dy = ty - self.center_y
        dist = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        angle_error = self._normalize(angle_to_target - self.yaw)
        scan_yaw_error = self._normalize(self.scan_heading_target - self.yaw)
        resume_yaw_error = self._normalize(self.resume_heading_target - self.yaw)

        # ---- Check obstacles during movement states ----
        obstacle_sensitive_states = (
            State.MOVE_FORWARD,
            State.ROTATE_TO_TARGET,
            State.ROTATE_TO_SCAN,
            State.ROTATE_BACK_TO_PATH,
        )
        if self.enable_obstacle_avoidance and self.state in obstacle_sensitive_states:
            self._update_obstacle_counters()
            if self.front_blocked_count >= self.front_blocked_threshold:
                self._enter_obstacle_halt()
                return
        elif self.state not in (State.OBSTACLE_HALT, State.OBSTACLE_REROUTE):
            self._reset_obstacle_counters()

        # ---- OBSTACLE_HALT ----
        if self.state == State.OBSTACLE_HALT:
            self._publish_stop()
            self._update_obstacle_counters()
            if self.clear_count >= self.clear_threshold:
                self._reset_obstacle_counters()
                self.state = self.pre_obstacle_state or State.MOVE_FORWARD
                self.reroute_entry_time = None
                self.get_logger().info('Obstacle cleared — resuming mission')
                return
            if self._halt_timed_out():
                self._enter_reroute()
            return

        # ---- OBSTACLE_REROUTE ----
        if self.state == State.OBSTACLE_REROUTE:
            self._handle_reroute()
            return

        # ---- ROTATE_TO_TARGET ----
        if self.state == State.ROTATE_TO_TARGET:
            if abs(angle_error) > self.ang_tol:
                self._publish_motion(wz=self._rotation_command(angle_error))
            else:
                self.state = State.MOVE_FORWARD
                self.path_line_start = (self.center_x, self.center_y)
                self.path_line_end = (tx, ty)
                self.get_logger().info(
                    f'Aligned to waypoint {self.current_wp_idx} — moving forward'
                )
            return

        # ---- MOVE_FORWARD ----
        if self.state == State.MOVE_FORWARD:
            if dist > self.pos_tol:
                # Holonomic tracking: move toward waypoint in robot frame.
                cos_y = math.cos(self.yaw)
                sin_y = math.sin(self.yaw)
                body_x = dx * cos_y + dy * sin_y
                body_y = -dx * sin_y + dy * cos_y
                norm = math.hypot(body_x, body_y)
                speed = min(self.lin_kp * dist, self.max_lin)
                if norm > 1e-6:
                    vx = speed * body_x / norm
                    vy = speed * body_y / norm
                else:
                    vx = 0.0
                    vy = 0.0
                hold_heading_error = self._normalize(self.resume_heading_target - self.yaw)
                self._publish_motion(
                    vx=vx,
                    vy=vy,
                    wz=self.ang_kp * hold_heading_error * 0.25,
                )
            else:
                if not self.enable_stop_scan:
                    self._publish_stop()
                    self.current_wp_idx += 1
                    if self.current_wp_idx < len(self.waypoints):
                        self.path_line_start = (self.center_x, self.center_y)
                        self.path_line_end = self.waypoints[self.current_wp_idx][:2]
                        self.state = State.MOVE_FORWARD
                    else:
                        self.state = State.COMPLETE
                        self.get_logger().info('Straight-line run complete')
                    return
                if self.current_wp_idx not in self.scan_waypoint_indices:
                    self._publish_stop()
                    self.current_wp_idx += 1
                    if self.current_wp_idx < len(self.waypoints):
                        self.path_line_start = (self.center_x, self.center_y)
                        self.path_line_end = self.waypoints[self.current_wp_idx][:2]
                        self.state = State.MOVE_FORWARD
                    else:
                        self.state = State.COMPLETE
                        self.get_logger().info('Skid sequence complete')
                    return
                self.resume_heading_target = self._compute_resume_heading(tx, ty)
                self.scan_heading_target = self._normalize(
                    self.resume_heading_target +
                    self._scan_direction_sign() * self.scan_turn_rad
                )
                self.state = State.ROTATE_TO_SCAN
                self.get_logger().info('At stop point — rotating 90deg toward shelf')
            return

        # ---- ROTATE_TO_SCAN ----
        if self.state == State.ROTATE_TO_SCAN:
            if abs(scan_yaw_error) > self.scan_ang_tol:
                self._publish_motion(
                    wz=self._rotation_command(
                        scan_yaw_error,
                        speed_scale=self.scan_rotation_kp_scale,
                        min_speed=self.scan_rotation_min_speed,
                        max_speed=self.scan_rotation_max_speed,
                    )
                )
            else:
                self._publish_stop()
                if self.scan_use_lift and tlift > 0.0:
                    self.target_lift_pos = self._height_to_slider(tlift)
                    self.state = State.LIFTING_UP
                    self.get_logger().info(
                        f'Waypoint {self.current_wp_idx} reached — rotating done, lifting with slider={self.target_lift_pos:.2f}'
                    )
                else:
                    self.scan_start_time = None
                    self.state = State.SCANNING
                    self.get_logger().info('Waypoint reached — rotating done, scanning in place')
            return

        # ---- LIFTING_UP ----
        if self.state == State.LIFTING_UP:
            self._publish_stop()
            self._publish_lift(self.target_lift_pos)
            if self.scan_start_time is None:
                self.scan_start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed > self.lift_motion_time:
                self.scan_start_time = None
                self.state = State.SCANNING
                self.get_logger().info('Lift at height — scanning shelf')
            return

        # ---- SCANNING ----
        if self.state == State.SCANNING:
            self._publish_stop()
            if self.scan_start_time is None:
                self.scan_start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed >= self.lift_dwell:
                self.scan_start_time = None
                self.state = State.ROTATE_BACK_TO_PATH
                if self.scan_use_lift and self.target_lift_pos > 0.0:
                    self.get_logger().info('Scan complete — rotating back to path heading with lift raised')
                else:
                    self.get_logger().info('Scan complete — rotating back to path heading')
            return

        # ---- LIFTING_DOWN ----
        if self.state == State.LIFTING_DOWN:
            self._publish_stop()
            self._publish_lift(0.0)  # fully retracted
            if self.scan_start_time is None:
                self.scan_start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed > self.lift_motion_time:
                self.scan_start_time = None
                self.target_lift_pos = 0.0
                self.current_wp_idx += 1
                if self.current_wp_idx < len(self.waypoints):
                    self.path_line_start = (self.center_x, self.center_y)
                    self.path_line_end = self.waypoints[self.current_wp_idx][:2]
                    self.state = State.MOVE_FORWARD
                    self.get_logger().info(
                        f'Lift retracted — moving to waypoint {self.current_wp_idx}'
                    )
                else:
                    self.state = State.COMPLETE
                    self.get_logger().info('ALL WAYPOINTS COMPLETE!')
            return

        # ---- ROTATE_BACK_TO_PATH ----
        if self.state == State.ROTATE_BACK_TO_PATH:
            if self.scan_use_lift and self.target_lift_pos > 0.0:
                self._publish_lift(self.target_lift_pos)
            if abs(resume_yaw_error) > self.scan_ang_tol:
                self._publish_motion(
                    wz=self._rotation_command(
                        resume_yaw_error,
                        speed_scale=self.scan_rotation_kp_scale,
                        min_speed=self.scan_rotation_min_speed,
                        max_speed=self.scan_rotation_max_speed,
                    )
                )
            else:
                self._publish_stop()
                if self.scan_use_lift and self.target_lift_pos > 0.0:
                    self.scan_start_time = None
                    self.state = State.LIFTING_DOWN
                    self.get_logger().info('Resumed heading — lowering lift')
                else:
                    self.current_wp_idx += 1
                    if self.current_wp_idx < len(self.waypoints):
                        self.path_line_start = (self.center_x, self.center_y)
                        self.path_line_end = self.waypoints[self.current_wp_idx][:2]
                        self.state = State.MOVE_FORWARD
                        self.get_logger().info(
                            f'Resumed heading — moving to waypoint {self.current_wp_idx}'
                        )
                    else:
                        self.state = State.COMPLETE
                        self.get_logger().info('ALL WAYPOINTS COMPLETE!')
            return

        # ---- COMPLETE ----
        if self.state == State.COMPLETE:
            self._publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
