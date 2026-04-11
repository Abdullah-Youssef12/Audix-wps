#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


SENSOR_TOPICS = {
    'front': '/ir_front_digital',
    'front_left': '/ir_front_left_digital',
    'front_right': '/ir_front_right_digital',
    'left': '/ir_left_digital',
    'right': '/ir_right_digital',
    'back': '/ir_back_digital',
}


class MockIrDigitalPublisher(Node):
    def __init__(self):
        super().__init__('mock_ir_digital_publisher')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('mode', 'clear')
        self.declare_parameter('blocked_sensors', '')
        self.declare_parameter('cycle_period_sec', 6.0)

        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        blocked_text = str(self.get_parameter('blocked_sensors').value).strip().lower()
        self.blocked_sensors = {name.strip() for name in blocked_text.split(',') if name.strip()}
        self.cycle_period_sec = max(0.5, float(self.get_parameter('cycle_period_sec').value))

        self.publishers = {
            key: self.create_publisher(Bool, topic, 10)
            for key, topic in SENSOR_TOPICS.items()
        }
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

    def _blocked_state(self, sensor_name, now_sec):
        if self.mode == 'clear':
            return False
        if self.mode == 'all_blocked':
            return True
        if self.mode == 'manual':
            return sensor_name in self.blocked_sensors
        if self.mode == 'cycle_front':
            phase = (now_sec - self.start_time) / self.cycle_period_sec
            return sensor_name == 'front' and math.sin(phase * 2.0 * math.pi) >= 0.0
        return False

    def _publish(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for sensor_name, publisher in self.publishers.items():
            msg = Bool()
            msg.data = self._blocked_state(sensor_name, now_sec)
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockIrDigitalPublisher()
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