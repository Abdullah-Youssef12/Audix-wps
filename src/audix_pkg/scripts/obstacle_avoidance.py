#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Map six IR topics (as published by the robot URDF) to sensor names
        self.sensor_topics = {
            'front': '/ir_front/scan',
            'front_left': '/ir_front_left/scan',
            'front_right': '/ir_front_right/scan',
            'left': '/ir_left/scan',
            'right': '/ir_right/scan',
            'back': '/ir_back/scan',
        }

        # Boolean blocked state per sensor (True == blocked)
        self.ir_blocked = {name: False for name in self.sensor_topics}

        # Physical sensor bounds (FC-51 in URDF)
        self.ir_min = 0.05
        self.ir_max = 0.30

        # Subscribe to each sensor topic and bind the sensor name
        for name, topic in self.sensor_topics.items():
            self.create_subscription(LaserScan, topic, lambda msg, n=name: self.scan_callback(n, msg), 10)

        # Publish Twist on /cmd_vel to match this repo's controller expectations
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Obstacle Avoidance Node Started (binary IR mode)')

    def scan_callback(self, sensor_name: str, msg: LaserScan):
        # Convert ranges and test for any valid reading inside physical sensor bounds
        ranges = np.array(msg.ranges)
        if ranges.size == 0:
            return

        finite_mask = np.isfinite(ranges)
        in_range_mask = (ranges >= self.ir_min) & (ranges <= self.ir_max)
        blocked = bool(np.any(finite_mask & in_range_mask))

        self.ir_blocked[sensor_name] = blocked

        # Simple binary policy: if any front-facing sensor reports blocked, rotate in place;
        # otherwise drive forward. This uses `front`, `front_left`, `front_right` sensors.
        front_blocked = self.ir_blocked['front'] or self.ir_blocked['front_left'] or self.ir_blocked['front_right']

        cmd = Twist()
        if front_blocked:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().debug('Front blocked -> Turning')
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            self.get_logger().debug('Path clear -> Moving forward')

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
