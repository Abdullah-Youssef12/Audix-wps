#!/usr/bin/env python3
"""
Real-time watch utility for the Pi-side bench setup.

Observes the ESP32-fed topics and prints live rate/gap/staleness information so
the operator can spot missed deadlines, stalled publishers, or degraded link
quality while the robot is running.
"""

import argparse
import collections
import sys
import time
from dataclasses import dataclass, field
from typing import Deque, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


@dataclass
class TopicStats:
    arrivals: Deque[float] = field(default_factory=collections.deque)
    last_time: Optional[float] = None
    total_messages: int = 0

    def note(self, timestamp: float, window_sec: float) -> None:
        self.total_messages += 1
        self.last_time = timestamp
        self.arrivals.append(timestamp)
        while self.arrivals and (timestamp - self.arrivals[0]) > window_sec:
            self.arrivals.popleft()

    def rate_hz(self) -> float:
        if len(self.arrivals) < 2:
            return 0.0
        duration = self.arrivals[-1] - self.arrivals[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.arrivals) - 1) / duration

    def max_gap(self) -> float:
        if len(self.arrivals) < 2:
            return 0.0
        previous = None
        max_gap = 0.0
        for sample in self.arrivals:
            if previous is not None:
                max_gap = max(max_gap, sample - previous)
            previous = sample
        return max_gap

    def stale_age(self, now: float) -> float:
        if self.last_time is None:
            return float('inf')
        return now - self.last_time


class RealtimeWatch(Node):
    def __init__(self, cli_args: argparse.Namespace):
        super().__init__('realtime_watch')
        self.args = cli_args
        self.status_pub = self.create_publisher(String, '/debug/realtime_watch/status', 10)
        self.stats = {
            '/odom': TopicStats(),
            '/imu': TopicStats(),
            '/limit_switch': TopicStats(),
        }

        self.create_subscription(Odometry, '/odom', self._make_cb('/odom'), 10)
        self.create_subscription(Imu, '/imu', self._make_cb('/imu'), 10)
        self.create_subscription(Bool, '/limit_switch', self._make_cb('/limit_switch'), 10)

        self.report_period = max(0.2, float(self.args.report_period_sec))
        self.window_sec = max(1.0, float(self.args.window_sec))
        self.min_rates = {
            '/odom': float(self.args.odom_min_hz),
            '/imu': float(self.args.imu_min_hz),
            '/limit_switch': float(self.args.limit_min_hz),
        }
        self.max_gap_warn = float(self.args.max_gap_warn_sec)
        self.stale_warn = float(self.args.stale_warn_sec)

    def _make_cb(self, topic_name: str):
        def callback(_msg) -> None:
            self.stats[topic_name].note(time.monotonic(), self.window_sec)
        return callback

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def format_line(self, topic_name: str, now: float) -> str:
        topic_stats = self.stats[topic_name]
        rate = topic_stats.rate_hz()
        max_gap = topic_stats.max_gap()
        stale_age = topic_stats.stale_age(now)

        status = 'OK'
        if stale_age > self.stale_warn:
            status = 'STALE'
        elif max_gap > self.max_gap_warn:
            status = 'GAP'
        elif rate < self.min_rates[topic_name]:
            status = 'SLOW'

        return (
            f'{topic_name}: status={status}, rate_hz={rate:.2f}, '
            f'max_gap_sec={max_gap:.3f}, stale_age_sec={stale_age:.3f}, '
            f'total={topic_stats.total_messages}'
        )

    def run(self) -> int:
        end_time = None
        if self.args.watch_seconds > 0.0:
            end_time = time.monotonic() + float(self.args.watch_seconds)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.report_period)
            now = time.monotonic()
            lines = [self.format_line(topic_name, now) for topic_name in ('/odom', '/imu', '/limit_switch')]
            self.publish_status(' | '.join(lines))
            if (end_time is not None) and (now >= end_time):
                break

        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Observe live topic rates and staleness.')
    parser.add_argument('--watch-seconds', type=float, default=30.0)
    parser.add_argument('--window-sec', type=float, default=5.0)
    parser.add_argument('--report-period-sec', type=float, default=1.0)
    parser.add_argument('--odom-min-hz', type=float, default=15.0)
    parser.add_argument('--imu-min-hz', type=float, default=15.0)
    parser.add_argument('--limit-min-hz', type=float, default=10.0)
    parser.add_argument('--max-gap-warn-sec', type=float, default=0.20)
    parser.add_argument('--stale-warn-sec', type=float, default=0.30)
    return parser


def main(args=None) -> int:
    cli_args = build_parser().parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])
    rclpy.init(args=args)
    node = RealtimeWatch(cli_args)
    try:
        return node.run()
    except KeyboardInterrupt:
        node.publish_status('Realtime watch interrupted by operator.')
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
