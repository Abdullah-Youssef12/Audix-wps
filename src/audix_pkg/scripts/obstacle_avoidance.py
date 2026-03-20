#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Map six IR topics to sensor names
        self.sensor_topics = {
            'front': '/ir_front/scan',
            'front_left': '/ir_front_left/scan',
            'front_right': '/ir_front_right/scan',
            'left': '/ir_left/scan',
            'right': '/ir_right/scan',
            'back': '/ir_back/scan',
        }

        # Debounce counters and blocked state
        self.declare_parameter('debounce_count', 3)
        self.debounce_count = max(1, int(self.get_parameter('debounce_count').value))
        self._counters = {name: 0 for name in self.sensor_topics}
        self.ir_blocked = {name: False for name in self.sensor_topics}

        # Sensor physical bounds
        self.ir_min = 0.05
        self.ir_max = 0.30

        # Movement parameters (suggested avoidance speeds)
        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('lateral_speed', 0.15)
        self.declare_parameter('evade_forward_speed', 0.15)
        self.declare_parameter('evade_lateral_speed', 0.20)
        self.declare_parameter('ramp_step_linear', 0.05)
        self.declare_parameter('ramp_step_lateral', 0.05)
        self.declare_parameter('clearance_time_sec', 0.8)
        self.declare_parameter('control_rate', 10.0)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.lateral_speed = float(self.get_parameter('lateral_speed').value)
        self.evade_forward_speed = float(self.get_parameter('evade_forward_speed').value)
        self.evade_lateral_speed = float(self.get_parameter('evade_lateral_speed').value)
        self.ramp_step_linear = float(self.get_parameter('ramp_step_linear').value)
        self.ramp_step_lateral = float(self.get_parameter('ramp_step_lateral').value)
        self.clearance_time_sec = float(self.get_parameter('clearance_time_sec').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        # Publish suggestions to /avoid_cmd_vel (roamer will optionally accept)
        self.publisher = self.create_publisher(Twist, '/avoid_cmd_vel', 10)

        # Subscribe sensors
        for name, topic in self.sensor_topics.items():
            self.create_subscription(LaserScan, topic, lambda msg, n=name: self.scan_callback(n, msg), 10)

        # Optional odom subscription kept for future enhancement
        # from nav_msgs.msg import Odometry
        # self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)


        # Internal command smoothing state
        self.current_cmd = Twist()
        self.target_cmd = Twist()
        self.blocked_side = None

        # State machine variables
        self.state = 'NOMINAL'  # NOMINAL, FRONT_BLOCKED, EVADE_RIGHT, EVADE_LEFT
        self.clearance_required_ticks = max(1, int(self.clearance_time_sec * self.control_rate))
        self.clearance_ticks_remaining = 0

        # Control loop
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Node Started (smooth /avoid_cmd_vel)')

    def scan_callback(self, sensor_name: str, msg: LaserScan):
        ranges = np.array(msg.ranges)
        if ranges.size == 0:
            return
        finite_mask = np.isfinite(ranges)
        in_range_mask = (ranges >= self.ir_min) & (ranges <= self.ir_max)
        sample_blocked = bool(np.any(finite_mask & in_range_mask))

        # update debounce counter
        if sample_blocked:
            self._counters[sensor_name] = min(self.debounce_count, self._counters[sensor_name] + 1)
        else:
            self._counters[sensor_name] = max(0, self._counters[sensor_name] - 1)

        # effective blocked only when counter reaches threshold
        prev = self.ir_blocked[sensor_name]
        self.ir_blocked[sensor_name] = (self._counters[sensor_name] >= self.debounce_count)
        # record when a side just cleared
        if prev and not self.ir_blocked[sensor_name]:
            # if left/right cleared, start post-clearance hold
            if sensor_name in ('left', 'front_left', 'front_right', 'right'):
                self.last_clear_time = time.time()

    def _ramp_toward(self, current, target, step):
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def control_loop(self):
        # Decide target command based on debounced binary sensors using a commitment
        tcmd = Twist()

        front = self.ir_blocked['front']
        fl = self.ir_blocked['front_left']
        fr = self.ir_blocked['front_right']
        left = self.ir_blocked['left']
        right = self.ir_blocked['right']

        # State transitions and outputs
        if self.state == 'NOMINAL':
            if not front and not fl and not fr:
                # stay nominal: go forward
                tcmd.linear.x = self.forward_speed
                tcmd.linear.y = 0.0
                tcmd.angular.z = 0.0
            else:
                if front:
                    # immediate reaction to wall ahead
                    self.state = 'FRONT_BLOCKED'
                elif fl:
                    # left corner clipping -> evade right
                    self.state = 'EVADE_RIGHT'
                    self.clearance_ticks_remaining = self.clearance_required_ticks
                elif fr:
                    self.state = 'EVADE_LEFT'
                    self.clearance_ticks_remaining = self.clearance_required_ticks

        elif self.state == 'FRONT_BLOCKED':
            # drop forward velocity and prefer sliding to clear
            tcmd.linear.x = 0.0
            tcmd.angular.z = 0.0
            # prefer side that is clear
            if not right:
                tcmd.linear.y = -abs(self.lateral_speed)
            elif not left:
                tcmd.linear.y = abs(self.lateral_speed)
            else:
                # trapped: spin as last resort
                tcmd.linear.y = 0.0
                tcmd.angular.z = 0.6

            # exit to NOMINAL immediately when front clears
            if not front:
                self.state = 'NOMINAL'

        elif self.state == 'EVADE_RIGHT':
            # while front_left is blocked, maintain evade vector and reset timer
            if fl:
                tcmd.linear.x = self.evade_forward_speed
                tcmd.linear.y = -abs(self.evade_lateral_speed)
                tcmd.angular.z = 0.0
                self.clearance_ticks_remaining = self.clearance_required_ticks
            else:
                # count down clearance ticks; if interrupted, reset
                if self.clearance_ticks_remaining > 0:
                    self.clearance_ticks_remaining -= 1
                if self.clearance_ticks_remaining <= 0:
                    self.state = 'NOMINAL'
                else:
                    # continue sliding until timer expires
                    tcmd.linear.x = self.evade_forward_speed
                    tcmd.linear.y = -abs(self.evade_lateral_speed)
                    tcmd.angular.z = 0.0

        elif self.state == 'EVADE_LEFT':
            if fr:
                tcmd.linear.x = self.evade_forward_speed
                tcmd.linear.y = abs(self.evade_lateral_speed)
                tcmd.angular.z = 0.0
                self.clearance_ticks_remaining = self.clearance_required_ticks
            else:
                if self.clearance_ticks_remaining > 0:
                    self.clearance_ticks_remaining -= 1
                if self.clearance_ticks_remaining <= 0:
                    self.state = 'NOMINAL'
                else:
                    tcmd.linear.x = self.evade_forward_speed
                    tcmd.linear.y = abs(self.evade_lateral_speed)
                    tcmd.angular.z = 0.0

        # ramp toward target for smoothness
        cur_x = self.current_cmd.linear.x
        cur_y = self.current_cmd.linear.y
        cur_w = self.current_cmd.angular.z

        step_lin = self.ramp_step_linear
        step_lat = self.ramp_step_lateral
        step_ang = 0.08

        self.current_cmd.linear.x = self._ramp_toward(cur_x, tcmd.linear.x, step_lin)
        self.current_cmd.linear.y = self._ramp_toward(cur_y, tcmd.linear.y, step_lat)
        self.current_cmd.angular.z = self._ramp_toward(cur_w, tcmd.angular.z, step_ang)

        out = Twist()
        out.linear.x = float(self.current_cmd.linear.x)
        out.linear.y = float(self.current_cmd.linear.y)
        out.angular.z = float(self.current_cmd.angular.z)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
