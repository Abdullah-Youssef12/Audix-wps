#!/usr/bin/env python3
"""
Manual Pi GPIO stepper bench utility.

This script is intentionally separate from the main mission stack so the lift
and its limit switch can be validated in isolation before any higher-level
automation is layered on top.
"""

import argparse
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool, String

try:
    from gpiozero import DigitalInputDevice, OutputDevice
except ImportError as exc:  # pragma: no cover - exercised on the Pi target
    DigitalInputDevice = None
    OutputDevice = None
    GPIOZERO_IMPORT_ERROR = exc
else:
    GPIOZERO_IMPORT_ERROR = None


class PiStepperBench(Node):
    def __init__(self, cli_args: argparse.Namespace):
        super().__init__('pi_stepper_bench')
        self.cli_args = cli_args

        self.declare_parameter('step_gpio', 20)
        self.declare_parameter('dir_gpio', 16)
        self.declare_parameter('enable_gpio', 21)
        self.declare_parameter('enable_active_low', True)
        self.declare_parameter('up_direction_high', True)
        self.declare_parameter('limit_switch_gpio', 26)
        self.declare_parameter('limit_switch_active_low', True)
        self.declare_parameter('limit_switch_pull_up', True)
        self.declare_parameter('limit_switch_debounce_ms', 5.0)
        self.declare_parameter('default_rate_hz', 400.0)
        self.declare_parameter('home_rate_hz', 250.0)
        self.declare_parameter('default_home_timeout_sec', 12.0)
        self.declare_parameter('home_direction', 'down')
        self.declare_parameter('disable_on_exit', True)
        self.declare_parameter('log_interval_sec', 1.0)
        self.declare_parameter('status_topic', '/debug/stepper_bench/status')
        self.declare_parameter('limit_switch_topic', '/debug/stepper_bench/limit_switch')

        if (DigitalInputDevice is None) or (OutputDevice is None):
            raise RuntimeError(
                'gpiozero is required for pi_stepper_bench. Install python3-gpiozero and python3-lgpio.'
            ) from GPIOZERO_IMPORT_ERROR

        self.step_gpio = int(self.get_parameter('step_gpio').value)
        self.dir_gpio = int(self.get_parameter('dir_gpio').value)
        self.enable_gpio = int(self.get_parameter('enable_gpio').value)
        self.enable_active_low = bool(self.get_parameter('enable_active_low').value)
        self.up_direction_high = bool(self.get_parameter('up_direction_high').value)
        self.limit_switch_gpio = int(self.get_parameter('limit_switch_gpio').value)
        self.limit_switch_active_low = bool(self.get_parameter('limit_switch_active_low').value)
        self.limit_switch_pull_up = bool(self.get_parameter('limit_switch_pull_up').value)
        self.limit_switch_debounce_sec = max(
            0.0, float(self.get_parameter('limit_switch_debounce_ms').value) / 1000.0
        )
        self.default_rate_hz = max(1.0, float(self.get_parameter('default_rate_hz').value))
        self.home_rate_hz = max(1.0, float(self.get_parameter('home_rate_hz').value))
        self.default_home_timeout_sec = max(0.5, float(self.get_parameter('default_home_timeout_sec').value))
        self.home_direction = str(self.get_parameter('home_direction').value).strip().lower()
        self.disable_on_exit = bool(self.get_parameter('disable_on_exit').value)
        self.log_interval_sec = max(0.1, float(self.get_parameter('log_interval_sec').value))

        status_topic = str(self.get_parameter('status_topic').value)
        limit_switch_topic = str(self.get_parameter('limit_switch_topic').value)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.limit_pub = self.create_publisher(Bool, limit_switch_topic, 10)

        self.step_pin = OutputDevice(self.step_gpio, initial_value=False)
        self.dir_pin = OutputDevice(self.dir_gpio, initial_value=False)
        self.enable_pin: Optional[OutputDevice]
        if self.enable_gpio >= 0:
            self.enable_pin = OutputDevice(
                self.enable_gpio,
                active_high=not self.enable_active_low,
                initial_value=False,
            )
        else:
            self.enable_pin = None

        self.limit_switch = DigitalInputDevice(
            self.limit_switch_gpio,
            pull_up=self.limit_switch_pull_up,
            bounce_time=self.limit_switch_debounce_sec,
        )
        self.driver_enabled = False
        self.last_limit_state = self.limit_pressed()

        self.publish_status(
            f'Pi stepper bench ready on step={self.step_gpio}, dir={self.dir_gpio}, '
            f'enable={self.enable_gpio}, limit={self.limit_switch_gpio}, '
            f'home_direction={self.home_direction}'
        )

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_limit_state(self) -> None:
        pressed = self.limit_pressed()
        msg = Bool()
        msg.data = pressed
        self.limit_pub.publish(msg)
        if pressed != self.last_limit_state:
            state_text = 'PRESSED' if pressed else 'released'
            self.publish_status(f'Lift limit switch -> {state_text}')
            self.last_limit_state = pressed

    def limit_pressed(self) -> bool:
        raw_high = bool(self.limit_switch.value)
        return (not raw_high) if self.limit_switch_active_low else raw_high

    def set_enabled(self, enabled: bool) -> None:
        if self.enable_pin is not None:
            if enabled:
                self.enable_pin.on()
            else:
                self.enable_pin.off()
        self.driver_enabled = enabled

    def set_direction(self, direction_up: bool) -> None:
        direction_high = self.up_direction_high if direction_up else (not self.up_direction_high)
        if direction_high:
            self.dir_pin.on()
        else:
            self.dir_pin.off()

    def pulse_once(self, rate_hz: float) -> None:
        half_period_sec = 0.5 / max(1.0, rate_hz)
        self.step_pin.on()
        time.sleep(half_period_sec)
        self.step_pin.off()
        time.sleep(half_period_sec)

    def status_loop(self) -> int:
        self.publish_status('Status mode: watching the lift limit switch. Press Ctrl+C to stop.')
        while rclpy.ok():
            self.publish_limit_state()
            time.sleep(1.0 / max(1.0, self.cli_args.status_rate_hz))
        return 0

    def run_motion(
        self,
        *,
        direction_up: bool,
        rate_hz: float,
        duration_sec: Optional[float],
        max_steps: Optional[int],
        respect_limit: bool,
    ) -> int:
        self.set_direction(direction_up)
        self.set_enabled(True)

        start_time = time.monotonic()
        last_log = start_time
        steps_taken = 0

        direction_text = 'up' if direction_up else 'down'
        self.publish_status(
            f'Starting stepper motion: direction={direction_text}, rate_hz={rate_hz:.1f}, '
            f'duration_sec={duration_sec}, steps={max_steps}, respect_limit={respect_limit}'
        )

        while rclpy.ok():
            now = time.monotonic()
            self.publish_limit_state()

            if respect_limit and self.limit_pressed():
                self.publish_status('Limit switch active, stopping motion.')
                break

            if (duration_sec is not None) and ((now - start_time) >= duration_sec):
                self.publish_status('Jog duration reached, stopping motion.')
                break

            if (max_steps is not None) and (steps_taken >= max_steps):
                self.publish_status('Requested step count reached, stopping motion.')
                break

            self.pulse_once(rate_hz)
            steps_taken += 1

            if (now - last_log) >= self.log_interval_sec:
                self.publish_status(
                    f'Jogging {direction_text}: steps_taken={steps_taken}, '
                    f'elapsed_sec={now - start_time:.2f}, limit_pressed={self.limit_pressed()}'
                )
                last_log = now

        self.set_enabled(False)
        self.publish_status(
            f'Stepper motion finished: direction={direction_text}, steps_taken={steps_taken}, '
            f'limit_pressed={self.limit_pressed()}'
        )
        return 0

    def run_home(self) -> int:
        direction_up = self.home_direction == 'up'
        timeout_sec = self.cli_args.timeout_sec or self.default_home_timeout_sec
        rate_hz = self.cli_args.rate_hz or self.home_rate_hz
        direction_text = 'up' if direction_up else 'down'

        if self.limit_pressed():
            self.publish_status('Home requested, but the lift limit switch is already pressed.')
            return 0

        self.set_direction(direction_up)
        self.set_enabled(True)
        self.publish_status(
            f'Starting home sequence: direction={direction_text}, rate_hz={rate_hz:.1f}, timeout_sec={timeout_sec:.1f}'
        )

        start_time = time.monotonic()
        last_log = start_time
        steps_taken = 0

        while rclpy.ok():
            now = time.monotonic()
            self.publish_limit_state()

            if self.limit_pressed():
                self.publish_status(
                    f'Home switch triggered after {steps_taken} steps and {now - start_time:.2f}s.'
                )
                self.set_enabled(False)
                return 0

            if (now - start_time) >= timeout_sec:
                self.publish_status('Home timeout exceeded before the limit switch triggered.')
                self.set_enabled(False)
                return 1

            self.pulse_once(rate_hz)
            steps_taken += 1

            if (now - last_log) >= self.log_interval_sec:
                self.publish_status(
                    f'Homing {direction_text}: steps_taken={steps_taken}, '
                    f'elapsed_sec={now - start_time:.2f}'
                )
                last_log = now

        self.set_enabled(False)
        return 130

    def run(self) -> int:
        command = self.cli_args.command
        self.publish_limit_state()

        if command == 'status':
            return self.status_loop()

        if command == 'stop':
            self.set_enabled(False)
            self.publish_status('Stepper driver disabled.')
            return 0

        if command in ('jog_up', 'jog_down'):
            return self.run_motion(
                direction_up=(command == 'jog_up'),
                rate_hz=self.cli_args.rate_hz or self.default_rate_hz,
                duration_sec=self.cli_args.duration_sec,
                max_steps=self.cli_args.steps,
                respect_limit=not self.cli_args.ignore_limit,
            )

        if command == 'home':
            return self.run_home()

        self.publish_status(f'Unsupported command: {command}')
        return 1

    def cleanup(self) -> None:
        if self.disable_on_exit or (not self.cli_args.keep_enabled):
            self.set_enabled(False)
        self.step_pin.off()
        self.dir_pin.off()
        if self.enable_pin is not None:
            self.enable_pin.close()
        self.limit_switch.close()
        self.step_pin.close()
        self.dir_pin.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Direct GPIO stepper bench tool.')
    parser.add_argument('--command', choices=['status', 'jog_up', 'jog_down', 'stop', 'home'], required=True)
    parser.add_argument('--duration-sec', type=float, default=None)
    parser.add_argument('--steps', type=int, default=None)
    parser.add_argument('--rate-hz', type=float, default=None)
    parser.add_argument('--timeout-sec', type=float, default=None)
    parser.add_argument('--status-rate-hz', type=float, default=5.0)
    parser.add_argument('--ignore-limit', action='store_true')
    parser.add_argument('--keep-enabled', action='store_true')
    return parser


def main(args=None) -> int:
    cli_args = build_parser().parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])
    rclpy.init(args=args)
    node = PiStepperBench(cli_args)
    try:
        return node.run()
    except KeyboardInterrupt:
        node.publish_status('Stepper bench interrupted by operator.')
        return 130
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
