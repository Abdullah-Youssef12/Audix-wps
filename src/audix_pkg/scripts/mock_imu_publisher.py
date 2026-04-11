#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MockImuPublisher(Node):
    def __init__(self):
        super().__init__('mock_imu_publisher')

        self.declare_parameter('topic', '/imu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('yaw_rate_rad_s', 0.0)
        self.declare_parameter('linear_accel_x', 0.0)
        self.declare_parameter('linear_accel_y', 0.0)

        self.topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.yaw_rate = float(self.get_parameter('yaw_rate_rad_s').value)
        self.linear_accel_x = float(self.get_parameter('linear_accel_x').value)
        self.linear_accel_y = float(self.get_parameter('linear_accel_y').value)

        self.publisher = self.create_publisher(Imu, self.topic, 10)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

    def _publish(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self.start_time
        yaw = self.yaw_rate * elapsed
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = self.yaw_rate
        msg.linear_acceleration.x = self.linear_accel_x
        msg.linear_acceleration.y = self.linear_accel_y

        msg.orientation_covariance[0] = 0.05
        msg.orientation_covariance[4] = 0.05
        msg.orientation_covariance[8] = 0.05
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockImuPublisher()
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