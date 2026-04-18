#!/usr/bin/env python3
"""
Manual base PID bench utility.

Runs one simple chassis command profile at a time and prints a concise summary so
the operator can compare commanded X/Y/yaw motion against odometry and IMU
feedback while watching the robot.
"""

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class PoseSnapshot:
    x: float
    y: float
    yaw: float


@dataclass
class FeedbackStats:
    linear_x: List[float]
    linear_y: List[float]
    angular_z: List[float]


class BasePidBench(Node):
    def __init__(self, cli_args: argparse.Namespace):
        super().__init__('base_pid_bench')
        self.args = cli_args

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/robot_enable', 10)
        self.status_pub = self.create_publisher(String, '/debug/pid_bench/status', 10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)

        self.latest_odom: Optional[Odometry] = None
        self.latest_imu: Optional[Imu] = None
        self.last_odom_time: Optional[float] = None
        self.last_imu_time: Optional[float] = None

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.last_odom_time = time.monotonic()

    def _imu_cb(self, msg: Imu) -> None:
        self.latest_imu = msg
        self.last_imu_time = time.monotonic()

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def wait_for_feedback(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.latest_odom is not None) and (self.latest_imu is not None):
                return True
        return False

    def publish_enable(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.enable_pub.publish(msg)

    def publish_twist(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def latest_pose(self) -> PoseSnapshot:
        odom = self.latest_odom
        imu = self.latest_imu
        if (odom is None) or (imu is None):
            raise RuntimeError('BasePidBench requires /odom and /imu before it can run.')

        odom_yaw = quaternion_to_yaw(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        imu_yaw = quaternion_to_yaw(
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
        )
        imu_quat_norm = math.sqrt(
            (imu.orientation.x * imu.orientation.x)
            + (imu.orientation.y * imu.orientation.y)
            + (imu.orientation.z * imu.orientation.z)
            + (imu.orientation.w * imu.orientation.w)
        )
        # Use IMU yaw when the quaternion looks valid; otherwise fall back to odom yaw.
        yaw = imu_yaw if imu_quat_norm > 0.5 else odom_yaw
        return PoseSnapshot(
            x=float(odom.pose.pose.position.x),
            y=float(odom.pose.pose.position.y),
            yaw=yaw,
        )

    def spin_and_sample(self, deadline: float, linear_x: float, linear_y: float, angular_z: float) -> FeedbackStats:
        stats = FeedbackStats(linear_x=[], linear_y=[], angular_z=[])
        publish_period = 1.0 / max(1.0, self.args.publish_rate_hz)
        next_publish = time.monotonic()

        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.publish_twist(linear_x, linear_y, angular_z)
                if self.args.enable_robot:
                    self.publish_enable(True)
                next_publish = now + publish_period

            rclpy.spin_once(self, timeout_sec=0.02)

            if self.latest_odom is not None:
                stats.linear_x.append(float(self.latest_odom.twist.twist.linear.x))
                stats.linear_y.append(float(self.latest_odom.twist.twist.linear.y))
                stats.angular_z.append(float(self.latest_odom.twist.twist.angular.z))

        return stats

    def body_frame_displacement(self, start_pose: PoseSnapshot, end_pose: PoseSnapshot) -> tuple[float, float]:
        dx_world = end_pose.x - start_pose.x
        dy_world = end_pose.y - start_pose.y
        cos_yaw = math.cos(start_pose.yaw)
        sin_yaw = math.sin(start_pose.yaw)
        body_x = cos_yaw * dx_world + sin_yaw * dy_world
        body_y = -sin_yaw * dx_world + cos_yaw * dy_world
        return body_x, body_y

    def summarize(self, start_pose: PoseSnapshot, end_pose: PoseSnapshot, stats: FeedbackStats) -> str:
        actual_body_x, actual_body_y = self.body_frame_displacement(start_pose, end_pose)
        odom_yaw_delta = normalize_angle(
            quaternion_to_yaw(
                self.latest_odom.pose.pose.orientation.x,
                self.latest_odom.pose.pose.orientation.y,
                self.latest_odom.pose.pose.orientation.z,
                self.latest_odom.pose.pose.orientation.w,
            ) - start_pose.yaw
        ) if self.latest_odom is not None else 0.0
        imu_yaw_delta = normalize_angle(end_pose.yaw - start_pose.yaw)

        mean_vx = statistics.fmean(stats.linear_x) if stats.linear_x else 0.0
        mean_vy = statistics.fmean(stats.linear_y) if stats.linear_y else 0.0
        mean_wz = statistics.fmean(stats.angular_z) if stats.angular_z else 0.0
        max_abs_vx = max((abs(value) for value in stats.linear_x), default=0.0)
        max_abs_vy = max((abs(value) for value in stats.linear_y), default=0.0)
        max_abs_wz = max((abs(value) for value in stats.angular_z), default=0.0)

        if self.args.mode == 'forward':
            expected = self.args.linear_speed * self.args.duration_sec
            actual = actual_body_x
            label = 'body_x'
        elif self.args.mode == 'strafe':
            expected = self.args.linear_speed * self.args.duration_sec
            actual = actual_body_y
            label = 'body_y'
        elif self.args.mode == 'rotate':
            expected = self.args.angular_speed * self.args.duration_sec
            actual = imu_yaw_delta
            label = 'yaw'
        else:
            expected = 0.0
            actual = 0.0
            label = 'none'

        return (
            f'mode={self.args.mode}, expected_{label}={expected:.3f}, actual_{label}={actual:.3f}, '
            f'body_x={actual_body_x:.3f}, body_y={actual_body_y:.3f}, '
            f'imu_yaw_delta={imu_yaw_delta:.3f}, odom_yaw_delta={odom_yaw_delta:.3f}, '
            f'mean_twist=({mean_vx:.3f}, {mean_vy:.3f}, {mean_wz:.3f}), '
            f'max_abs_twist=({max_abs_vx:.3f}, {max_abs_vy:.3f}, {max_abs_wz:.3f})'
        )

    def run(self) -> int:
        if not self.wait_for_feedback(self.args.feedback_timeout_sec):
            self.publish_status('Timed out waiting for /odom and /imu.')
            return 1

        start_pose = self.latest_pose()
        self.publish_status(
            f'Starting {self.args.mode} bench test: duration={self.args.duration_sec:.2f}s, '
            f'linear_speed={self.args.linear_speed:.3f}, angular_speed={self.args.angular_speed:.3f}, '
            f'enable_robot={self.args.enable_robot}'
        )

        if self.args.enable_robot:
            for _ in range(5):
                self.publish_enable(True)
                rclpy.spin_once(self, timeout_sec=0.05)

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        if self.args.mode == 'forward':
            linear_x = self.args.linear_speed
        elif self.args.mode == 'strafe':
            linear_y = self.args.linear_speed
        elif self.args.mode == 'rotate':
            angular_z = self.args.angular_speed

        stats = self.spin_and_sample(
            time.monotonic() + self.args.duration_sec,
            linear_x,
            linear_y,
            angular_z,
        )

        stop_deadline = time.monotonic() + self.args.stop_publish_sec
        self.publish_status('Publishing stop command.')
        while rclpy.ok() and time.monotonic() < stop_deadline:
            self.publish_twist(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

        settle_deadline = time.monotonic() + self.args.settle_sec
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.args.disable_at_end:
            self.publish_status('Disabling robot at end of bench test.')
            for _ in range(3):
                self.publish_enable(False)
                rclpy.spin_once(self, timeout_sec=0.05)

        end_pose = self.latest_pose()
        summary = self.summarize(start_pose, end_pose, stats)
        self.publish_status(summary)
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run one base PID bench profile.')
    parser.add_argument('--mode', choices=['forward', 'strafe', 'rotate', 'stop'], required=True)
    parser.add_argument('--linear-speed', type=float, default=0.10)
    parser.add_argument('--angular-speed', type=float, default=0.40)
    parser.add_argument('--duration-sec', type=float, default=2.0)
    parser.add_argument('--settle-sec', type=float, default=0.5)
    parser.add_argument('--stop-publish-sec', type=float, default=0.5)
    parser.add_argument('--publish-rate-hz', type=float, default=20.0)
    parser.add_argument('--feedback-timeout-sec', type=float, default=8.0)
    parser.add_argument('--enable-robot', action='store_true')
    parser.add_argument('--disable-at-end', action='store_true')
    return parser


def main(args=None) -> int:
    cli_args = build_parser().parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])
    rclpy.init(args=args)
    node = BasePidBench(cli_args)
    try:
        return node.run()
    except KeyboardInterrupt:
        node.publish_status('Base PID bench interrupted by operator.')
        return 130
    finally:
        node.publish_twist(0.0, 0.0, 0.0)
        if cli_args.disable_at_end:
            node.publish_enable(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
