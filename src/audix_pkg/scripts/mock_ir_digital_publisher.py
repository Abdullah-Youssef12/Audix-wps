#!/usr/bin/env python3

import math
from typing import Dict, List, Set, Tuple

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


ScenarioStep = Tuple[float, Set[str]]


SCENARIOS: Dict[str, List[ScenarioStep]] = {
    'all_clear': [
        (1.0, set()),
    ],
    'front_blocked': [
        (1.0, {'front'}),
    ],
    'front_left_blocked': [
        (1.0, {'front_left'}),
    ],
    'front_right_blocked': [
        (1.0, {'front_right'}),
    ],
    'left_blocked': [
        (1.0, {'left'}),
    ],
    'right_blocked': [
        (1.0, {'right'}),
    ],
    'transient_front_blocked': [
        (1.5, set()),
        (1.2, {'front'}),
        (2.0, set()),
    ],
    'persistent_front_blocked': [
        (1.5, set()),
        (30.0, {'front'}),
    ],
    'reroute_left_case': [
        (1.5, set()),
        (6.0, {'front', 'front_right', 'right'}),
        (2.0, set()),
    ],
    'reroute_right_case': [
        (1.5, set()),
        (6.0, {'front', 'front_left', 'left'}),
        (2.0, set()),
    ],
    'obstacle_appears_after_motion': [
        (3.0, set()),
        (4.0, {'front'}),
        (3.0, set()),
    ],
    'obstacle_reappears_during_rejoin': [
        (2.5, set()),
        (4.0, {'front', 'front_right', 'right'}),
        (1.5, set()),
        (2.0, {'front'}),
        (3.0, set()),
    ],
    'side_clearance_delayed_left': [
        (1.5, set()),
        (2.5, {'front', 'front_left', 'left'}),
        (2.0, {'front', 'left'}),
        (2.0, {'front'}),
        (2.0, set()),
    ],
    'side_clearance_delayed_right': [
        (1.5, set()),
        (2.5, {'front', 'front_right', 'right'}),
        (2.0, {'front', 'right'}),
        (2.0, {'front'}),
        (2.0, set()),
    ],
}


class MockIrDigitalPublisher(Node):
    def __init__(self):
        super().__init__('mock_ir_digital_publisher')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('mode', 'clear')
        self.declare_parameter('blocked_sensors', '')
        self.declare_parameter('cycle_period_sec', 6.0)
        self.declare_parameter('ir_scenario', 'all_clear')
        self.declare_parameter('scenario_loop', False)
        self.declare_parameter('scenario_start_delay', 0.0)

        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        blocked_text = str(self.get_parameter('blocked_sensors').value).strip().lower()
        self.blocked_sensors = {name.strip() for name in blocked_text.split(',') if name.strip()}
        self.cycle_period_sec = max(0.5, float(self.get_parameter('cycle_period_sec').value))
        self.ir_scenario = str(self.get_parameter('ir_scenario').value).strip().lower()
        self.scenario_loop = bool(self.get_parameter('scenario_loop').value)
        self.scenario_start_delay = max(0.0, float(self.get_parameter('scenario_start_delay').value))

        if self.ir_scenario not in SCENARIOS:
            self.get_logger().warn(
                f'Unknown ir_scenario={self.ir_scenario}. Falling back to all_clear.'
            )
            self.ir_scenario = 'all_clear'

        self.scenario_steps = SCENARIOS[self.ir_scenario]
        self.scenario_total_duration = sum(duration for duration, _ in self.scenario_steps)

        self.publishers = {
            key: self.create_publisher(Bool, topic, 10)
            for key, topic in SENSOR_TOPICS.items()
        }
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

        self.get_logger().info(
            'Mock IR publisher ready: '
            f'mode={self.mode}, ir_scenario={self.ir_scenario}, '
            f'scenario_loop={self.scenario_loop}, scenario_start_delay={self.scenario_start_delay:.1f}s'
        )

    def _scenario_blocked_sensors(self, now_sec: float) -> Set[str]:
        elapsed = now_sec - self.start_time - self.scenario_start_delay
        if elapsed < 0.0:
            return set()

        if self.scenario_total_duration <= 0.0:
            return set()

        if self.scenario_loop:
            elapsed = elapsed % self.scenario_total_duration
        elif elapsed >= self.scenario_total_duration:
            return set(self.scenario_steps[-1][1])

        cursor = 0.0
        for duration, blocked in self.scenario_steps:
            cursor += duration
            if elapsed <= cursor:
                return set(blocked)

        return set(self.scenario_steps[-1][1])

    def _static_blocked_sensors(self) -> Set[str]:
        if self.ir_scenario in SCENARIOS and self.ir_scenario != 'all_clear' and not self.blocked_sensors:
            return set(self.scenario_steps[-1][1])
        return set(self.blocked_sensors)

    def _blocked_state(self, sensor_name, now_sec):
        if self.mode == 'clear':
            return False
        if self.mode == 'all_blocked':
            return True
        if self.mode == 'manual':
            return sensor_name in self.blocked_sensors
        if self.mode == 'static':
            return sensor_name in self._static_blocked_sensors()
        if self.mode == 'scripted':
            return sensor_name in self._scenario_blocked_sensors(now_sec)
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