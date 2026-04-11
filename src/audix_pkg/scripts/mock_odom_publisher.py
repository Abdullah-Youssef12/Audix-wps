#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MockOdomPublisher(Node):
    def __init__(self):
        super().__init__('mock_odom_publisher')

        self.declare_parameter('topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('linear_x', 0.0)
        self.declare_parameter('linear_y', 0.0)
        self.declare_parameter('angular_z', 0.0)

        self.topic = str(self.get_parameter('topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.linear_x = float(self.get_parameter('linear_x').value)
        self.linear_y = float(self.get_parameter('linear_y').value)
        self.angular_z = float(self.get_parameter('angular_z').value)

        self.publisher = self.create_publisher(Odometry, self.topic, 10)
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

    def _publish(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt < 0.0:
            dt = 0.0
        self.last_time = now

        self.yaw += self.angular_z * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += (self.linear_x * math.cos(self.yaw) - self.linear_y * math.sin(self.yaw)) * dt
        self.y += (self.linear_x * math.sin(self.yaw) + self.linear_y * math.cos(self.yaw)) * dt

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.linear_x
        msg.twist.twist.linear.y = self.linear_y
        msg.twist.twist.angular.z = self.angular_z

        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.02
        msg.twist.covariance[35] = 0.05
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockOdomPublisher()
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