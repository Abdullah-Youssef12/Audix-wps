#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MockLimitSwitchPublisher(Node):
    def __init__(self):
        super().__init__('mock_limit_switch_publisher')

        self.declare_parameter('topic', '/limit_switch')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('pressed', False)

        self.topic = str(self.get_parameter('topic').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.pressed = bool(self.get_parameter('pressed').value)

        self.publisher = self.create_publisher(Bool, self.topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

    def _publish(self):
        msg = Bool()
        msg.data = self.pressed
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockLimitSwitchPublisher()
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