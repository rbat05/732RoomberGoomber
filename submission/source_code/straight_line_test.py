#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ── TODO: Set your robot namespace ─────────────────────────────────────────────
NAMESPACE = '/T23'           # Change to your robot e.g. /T10

# ── TODO: Define your motion parameters ────────────────────────────────────────
FORWARD_SPEED = 0.3          # m/s - linear velocity when driving forward
TURN_SPEED    = 0.5          # rad/s - angular velocity when turning
TIME          = 16.667        # seconds - duration to drive forward in phase 0

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
        self.start_x = None
        self.start_y = None

        self.last_x = None
        self.last_y = None
        self.stable_count = 0   # for stop detection


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
        if self.test_done:
            self.stop()
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Phase 0 - Drive forward
        if self.phase == 0:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.start_x = self.current_x # Record starting position for displacement calculation
                self.start_y = self.current_y
                self.get_logger().info('Phase 0: Driving forward')

            elapsed = now - self.phase_start_time

            if elapsed < TIME:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.get_logger().info('Phase 0 complete - waiting for robot to fully stop')
                self.phase += 1
                self.phase_start_time = None

        # Phase 1 - Test complete
        elif self.phase == 1:
            # Detect if robot has stopped (position not changing)
            if self.last_x is None:
                self.last_x = self.current_x
                self.last_y = self.current_y
                return

            dx = abs(self.current_x - self.last_x)
            dy = abs(self.current_y - self.last_y)

            threshold = 0.001  # 1 mm tolerance for considering the robot as "stopped"

            if dx < threshold and dy < threshold:
                self.stable_count += 1
            else:
                self.stable_count = 0

            self.last_x = self.current_x
            self.last_y = self.current_y

            # Require stability over multiple cycles (~0.5s)
            if self.stable_count > 5:
                total_dx = self.current_x - self.start_x
                total_dy = self.current_y - self.start_y
                displacement = math.sqrt(total_dx**2 + total_dy**2) # euclidean distance from start to final position

                # Print required information
                self.get_logger().info('Robot has stopped')
                self.get_logger().info(
                    f'Final position x: {self.current_x:.4f} m  y: {self.current_y:.4f} m')
                self.get_logger().info(
                    f'Displacement: {displacement:.4f} m') # odometry-based displacement from start to final position, what we jot down during the test

                self.test_done = True

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
