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

# ── TODO: Set your robot namespace ─────────────────────────────────────────────
NAMESPACE = '/T11'           # Change to your robot e.g. /T10

# ── TODO: Define your motion parameters ────────────────────────────────────────
FORWARD_SPEED = 0.1          # m/s  — linear velocity when driving forward
TURN_SPEED    = 0.5          # rad/s — angular velocity when turning
FORWARD_DURATION = 5.0       # seconds to drive for each straight-line test
NUM_RUNS = 5                 # each run performs 2 straight-line tests
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

        # ── Straight-line distance sequence (5 runs, 2 legs per run) ─────────

        # Phase 0 — Drive forward (leg 1)
        if self.phase == 0:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.leg_start_x = self.current_x
                self.leg_start_y = self.current_y
                self.get_logger().info(
                    f'Run {self.current_run}/{NUM_RUNS} | Leg 1: Driving forward')
            elapsed = now - self.phase_start_time
            if elapsed < FORWARD_DURATION:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.log_leg_result(1, elapsed)
                self.phase += 1
                self.phase_start_time = None

        # Phase 1 — Turn around 180 degrees
        elif self.phase == 1:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.get_logger().info(
                    f'Run {self.current_run}/{NUM_RUNS} | Turning 180 degrees')
            elapsed = now - self.phase_start_time
            if elapsed < self.turn_duration:
                self.drive(0.0, TURN_SPEED)
            else:
                self.stop()
                self.phase += 1
                self.phase_start_time = None

        # Phase 2 — Drive forward (leg 2)
        elif self.phase == 2:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.leg_start_x = self.current_x
                self.leg_start_y = self.current_y
                self.get_logger().info(
                    f'Run {self.current_run}/{NUM_RUNS} | Leg 2: Driving forward')
            elapsed = now - self.phase_start_time
            if elapsed < FORWARD_DURATION:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.log_leg_result(2, elapsed)
                self.phase += 1
                self.phase_start_time = None

        # Phase 3 — Move to next run or finish
        elif self.phase == 3:
            if self.current_run < NUM_RUNS:
                self.current_run += 1
                self.phase = 0
                self.get_logger().info(
                    f'Starting run {self.current_run}/{NUM_RUNS}')
            else:
                self.stop()
                self.get_logger().info('Test sequence complete — stopping')
                self.test_done = True

    def log_leg_result(self, leg_number, elapsed):
        distance = math.hypot(
            self.current_x - self.leg_start_x,
            self.current_y - self.leg_start_y)
        self.log_counter += 1
        ros_time_ns = self.get_clock().now().nanoseconds
        log_file = self.log_dir / (
            f'run_{self.current_run}_leg_{leg_number}_{ros_time_ns}_{self.log_counter}.log')
        with log_file.open('w', encoding='utf-8') as handle:
            handle.write(f'run={self.current_run}\n')
            handle.write(f'leg={leg_number}\n')
            handle.write(f'duration_s={elapsed:.3f}\n')
            handle.write(f'start_x={self.leg_start_x:.4f}\n')
            handle.write(f'start_y={self.leg_start_y:.4f}\n')
            handle.write(f'end_x={self.current_x:.4f}\n')
            handle.write(f'end_y={self.current_y:.4f}\n')
            handle.write(f'distance_m={distance:.4f}\n')
            handle.write(f'final_yaw_deg={self.current_yaw:.2f}\n')
            handle.write(f'nearest_obstacle_m={self.nearest_obstacle:.4f}\n')
        self.get_logger().info(
            f'Run {self.current_run} Leg {leg_number} complete | '
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
