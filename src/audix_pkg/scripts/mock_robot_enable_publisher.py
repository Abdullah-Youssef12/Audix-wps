#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class MockRobotEnablePublisher(Node):
    def __init__(self):
        super().__init__('mock_robot_enable_publisher')

        self.declare_parameter('topic', '/robot_enable')
        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.topic = str(self.get_parameter('topic').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.publish_rate_hz = max(0.5, float(self.get_parameter('publish_rate_hz').value))

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(Bool, self.topic, qos)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

    def _publish(self):
        msg = Bool()
        msg.data = self.enabled
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotEnablePublisher()
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