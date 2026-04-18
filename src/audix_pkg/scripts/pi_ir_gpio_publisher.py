#!/usr/bin/env python3
"""
Pi IR GPIO Publisher

Reads six Raspberry Pi GPIO-backed digital IR sensors and republishes them as
the stable Bool topics consumed by ir_digital_bridge.py.
"""

from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from gpiozero import DigitalInputDevice
except ImportError as exc:  # pragma: no cover - exercised on the Pi target
    DigitalInputDevice = None
    GPIOZERO_IMPORT_ERROR = exc
else:
    GPIOZERO_IMPORT_ERROR = None


SENSOR_TOPICS = {
    'front': '/ir_front_digital',
    'front_left': '/ir_front_left_digital',
    'front_right': '/ir_front_right_digital',
    'left': '/ir_left_digital',
    'right': '/ir_right_digital',
    'back': '/ir_back_digital',
}


DEFAULT_GPIO_PINS = {
    'front': 5,
    'front_left': 6,
    'front_right': 13,
    'left': 19,
    'right': 26,
    'back': 21,
}


class PiIrGpioPublisher(Node):
    def __init__(self):
        super().__init__('pi_ir_gpio_publisher')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('debounce_ms', 10.0)
        self.declare_parameter('blocked_when_low', True)
        self.declare_parameter('pull_up', False)
        self.declare_parameter('log_state_changes', True)

        for sensor_name, default_pin in DEFAULT_GPIO_PINS.items():
            self.declare_parameter(f'{sensor_name}_gpio', default_pin)

        if DigitalInputDevice is None:
            raise RuntimeError(
                'gpiozero is required for pi_ir_gpio_publisher. '
                'Install python3-gpiozero and python3-lgpio.'
            ) from GPIOZERO_IMPORT_ERROR

        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.debounce_sec = max(0.0, float(self.get_parameter('debounce_ms').value) / 1000.0)
        self.blocked_when_low = bool(self.get_parameter('blocked_when_low').value)
        self.pull_up = bool(self.get_parameter('pull_up').value)
        self.log_state_changes = bool(self.get_parameter('log_state_changes').value)

        self.pin_map: Dict[str, int] = {
            sensor_name: int(self.get_parameter(f'{sensor_name}_gpio').value)
            for sensor_name in SENSOR_TOPICS
        }

        self._validate_pin_map()

        self.publishers = {
            sensor_name: self.create_publisher(Bool, topic_name, 10)
            for sensor_name, topic_name in SENSOR_TOPICS.items()
        }
        self.devices = {
            sensor_name: DigitalInputDevice(
                pin=pin_number,
                pull_up=self.pull_up,
                bounce_time=self.debounce_sec,
            )
            for sensor_name, pin_number in self.pin_map.items()
        }
        self.last_states = {
            sensor_name: self._read_blocked(sensor_name)
            for sensor_name in SENSOR_TOPICS
        }

        self.create_timer(1.0 / self.publish_rate_hz, self._publish_all)

        self.get_logger().info(
            'Pi IR GPIO publisher ready on BCM pins '
            f'{self.pin_map} (blocked_when_low={self.blocked_when_low}, pull_up={self.pull_up}).'
        )

    def _validate_pin_map(self) -> None:
        pin_values = list(self.pin_map.values())
        if any(pin_value < 0 for pin_value in pin_values):
            raise ValueError(f'IR GPIO pins must be non-negative BCM numbers. Got {self.pin_map}.')
        if len(set(pin_values)) != len(pin_values):
            raise ValueError(f'IR GPIO pins must be unique. Got {self.pin_map}.')

    def _read_blocked(self, sensor_name: str) -> bool:
        raw_high = bool(self.devices[sensor_name].value)
        return (not raw_high) if self.blocked_when_low else raw_high

    def _publish_all(self) -> None:
        for sensor_name, publisher in self.publishers.items():
            blocked = self._read_blocked(sensor_name)
            msg = Bool()
            msg.data = blocked
            publisher.publish(msg)

            if self.log_state_changes and blocked != self.last_states[sensor_name]:
                state_text = 'blocked' if blocked else 'clear'
                self.get_logger().info(f'{sensor_name} -> {state_text}')
                self.last_states[sensor_name] = blocked


def main(args=None):
    rclpy.init(args=args)
    node = PiIrGpioPublisher()
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
