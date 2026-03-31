#!/usr/bin/env python3
"""
Test Node Template — Sensor Investigation
COMPSYS732

Use this template as the starting point for your autonomous test nodes.
It combines the publisher pattern from Task 5 (to command motion) with
the subscriber pattern from Task 4 (to read sensor data).

Fill in the sections marked TODO to implement your specific test.

To run:
    ros2 run tb4_sensor_reader test_node
    (after adding 'test_node = tb4_sensor_reader.test_node:main' to setup.py)
"""

import rclpy
import math
from pathlib import Path
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import InterfaceButtons

# ── TODO: Set your robot namespace ─────────────────────────────────────────────
NAMESPACE = '/T11'           # Change to your robot e.g. /T10

# ── TODO: Define your motion parameters ────────────────────────────────────────
FORWARD_SPEED = 0.1          # m/s  — linear velocity when driving forward
TURN_SPEED    = 0.5          # rad/s — angular velocity when turning
FORWARD_DURATION = 5.0       # seconds to drive for each straight-line test
NUM_RUNS = 5                 # number of straight-line runs to perform
# Example durations for known distances/angles:
#   Drive 1.0 m at 0.2 m/s  → duration = 1.0 / 0.2 = 5.0 seconds
#   Turn 90°  at 0.5 rad/s  → duration = (pi/2) / 0.5 = 3.14 seconds
#   Turn 360° at 0.5 rad/s  → duration = (2*pi) / 0.5 = 12.57 seconds


class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist,
            f'{NAMESPACE}/cmd_vel',
            10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{NAMESPACE}/odom',
            self.odom_callback,
            10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'{NAMESPACE}/scan',
            self.scan_callback,
            10)
        self.button_sub = self.create_subscription(
            InterfaceButtons,
            '/hmi/buttons',
            self.buttons_callback,
            10)

        # ── State variables ──────────────────────────────────────────────────
        # These store the latest sensor readings so any callback can use them
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0   # degrees
        self.nearest_obstacle = float('inf')   # metres

        # ── TODO: Add your own state variables ───────────────────────────────
        # Example: track which phase of your test sequence you are in
        self.phase     = 0
        self.phase_start_time = None
        self.test_done = False
        self.current_run = 1
        self.last_button2_pressed = False
        self.pending_next_run = False
        self.waiting_for_button_logged = False
        self.log_counter = 0
        self.leg_start_x = 0.0
        self.leg_start_y = 0.0
        self.turn_duration = math.pi / TURN_SPEED   # 180° turn
        self.log_dir = Path.home() / 'straight_line_logs'
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # ── Control loop timer (runs every 0.1 seconds = 10 Hz) ─────────────
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Test node started')

    # ── Odometry callback ────────────────────────────────────────────────────

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y

        # Convert quaternion to yaw in degrees
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    # ── LiDAR callback ───────────────────────────────────────────────────────

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        self.nearest_obstacle = min(valid) if valid else float('inf')

    # ── Helper: publish a velocity command ───────────────────────────────────

    def drive(self, linear, angular):
        """Publish a Twist command. Call with (0, 0) to stop."""
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    # ── Control loop ─────────────────────────────────────────────────────────

    def control_loop(self):
        """
        Called at 10 Hz. Implement your test sequence here using self.phase
        to track which step of the sequence you are in.

        Pattern:
            Phase 0 → do action A for N seconds → advance to phase 1
            Phase 1 → do action B for N seconds → advance to phase 2
            ...
            Final phase → stop, log results, set self.test_done = True
        """

        if self.test_done:
            self.stop()
            return

        now = self.get_clock().now().nanoseconds / 1e9   # current time in seconds

        # ── Straight-line run sequence (trigger next run from HMI button) ────

        # Phase 0 — Drive forward (single run)
        if self.phase == 0:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.leg_start_x = self.current_x
                self.leg_start_y = self.current_y
                self.get_logger().info(
                    f'Run {self.current_run}/{NUM_RUNS} | Driving forward')
            elapsed = now - self.phase_start_time
            if elapsed < FORWARD_DURATION:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.log_leg_result(elapsed)
                self.phase += 1
                self.phase_start_time = None

        # Phase 1 — Wait for HMI button 2 to trigger next run
        elif self.phase == 1:
            self.stop()
            if self.current_run >= NUM_RUNS:
                self.get_logger().info('Test sequence complete — stopping')
                self.test_done = True
            elif self.pending_next_run:
                self.pending_next_run = False
                self.waiting_for_button_logged = False
                self.phase = 2
                self.phase_start_time = None
            elif not self.waiting_for_button_logged:
                self.get_logger().info(
                    'Waiting for HMI button create3_2 on /hmi/buttons')
                self.waiting_for_button_logged = True

        # Phase 2 — Turn around 180 degrees
        elif self.phase == 2:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.get_logger().info(
                    f'Run {self.current_run + 1}/{NUM_RUNS} | Turning 180 degrees')
            elapsed = now - self.phase_start_time
            if elapsed < self.turn_duration:
                self.drive(0.0, TURN_SPEED)
            else:
                self.stop()
                self.current_run += 1
                self.phase = 0
                self.get_logger().info(
                    f'Starting run {self.current_run}/{NUM_RUNS}')

    def buttons_callback(self, msg):
        button_state = None
        if hasattr(msg, 'create3_2'):
            button_state = getattr(msg, 'create3_2')
        elif hasattr(msg, 'button_2'):
            button_state = getattr(msg, 'button_2')
        elif hasattr(msg, 'button2'):
            button_state = getattr(msg, 'button2')

        if button_state is None:
            return

        is_pressed = (
            bool(button_state.is_pressed)
            if hasattr(button_state, 'is_pressed')
            else bool(button_state)
        )
        if is_pressed and not self.last_button2_pressed and self.phase == 1:
            self.pending_next_run = True
            self.get_logger().info('Received create3_2 press: scheduling turn + next run')
        self.last_button2_pressed = is_pressed

    def log_leg_result(self, elapsed):
        distance = math.hypot(
            self.current_x - self.leg_start_x,
            self.current_y - self.leg_start_y)
        self.log_counter += 1
        ros_time_ns = self.get_clock().now().nanoseconds
        log_file = self.log_dir / (
            f'run_{self.current_run}_{ros_time_ns}_{self.log_counter}.log')
        with log_file.open('w', encoding='utf-8') as handle:
            handle.write(f'run={self.current_run}\n')
            handle.write(f'duration_s={elapsed:.3f}\n')
            handle.write(f'start_x={self.leg_start_x:.4f}\n')
            handle.write(f'start_y={self.leg_start_y:.4f}\n')
            handle.write(f'end_x={self.current_x:.4f}\n')
            handle.write(f'end_y={self.current_y:.4f}\n')
            handle.write(f'distance_m={distance:.4f}\n')
            handle.write(f'final_yaw_deg={self.current_yaw:.2f}\n')
            handle.write(f'nearest_obstacle_m={self.nearest_obstacle:.4f}\n')
        self.get_logger().info(
            f'Run {self.current_run} complete | '
            f'Distance: {distance:.4f} m | Log: {log_file}')


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
