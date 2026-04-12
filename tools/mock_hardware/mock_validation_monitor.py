#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


SENSOR_TOPICS = {
    'front': '/ir_front/scan',
    'front_left': '/ir_front_left/scan',
    'front_right': '/ir_front_right/scan',
    'left': '/ir_left/scan',
    'right': '/ir_right/scan',
    'back': '/ir_back/scan',
}


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MockValidationMonitor(Node):
    def __init__(self):
        super().__init__('mock_validation_monitor')

        self.state_name = 'UNKNOWN'
        self.cmd = (0.0, 0.0, 0.0)
        self.pose = (0.0, 0.0, 0.0)
        self.blocked = {name: False for name in SENSOR_TOPICS}

        self.create_subscription(String, '/debug/state', self._state_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        for sensor_name, topic in SENSOR_TOPICS.items():
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, name=sensor_name: self._scan_cb(name, msg),
                10,
            )

        self.create_timer(0.5, self._print_summary)

    def _state_cb(self, msg: String):
        new_state = msg.data.strip() or 'UNKNOWN'
        if new_state != self.state_name:
            self.state_name = new_state
            self.get_logger().info(f'state -> {self.state_name}')

    def _cmd_cb(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _odom_cb(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _scan_cb(self, sensor_name: str, msg: LaserScan):
        blocked = False
        for sample in msg.ranges:
            if math.isfinite(sample) and sample <= msg.range_max:
                blocked = True
                break
        self.blocked[sensor_name] = blocked

    def _print_summary(self):
        blocked_names = [name for name, blocked in self.blocked.items() if blocked]
        blocked_text = ','.join(blocked_names) if blocked_names else 'none'
        self.get_logger().info(
            'summary '
            f'state={self.state_name} '
            f'cmd=({self.cmd[0]:+.2f},{self.cmd[1]:+.2f},{self.cmd[2]:+.2f}) '
            f'odom=({self.pose[0]:+.2f},{self.pose[1]:+.2f},{self.pose[2]:+.2f}) '
            f'blocked={blocked_text}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockValidationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()