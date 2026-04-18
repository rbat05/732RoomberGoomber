#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T23'           # Change to your robot e.g. /T10

# ── Motion parameters ──────────────────────────────────────────────────────────
SIDE_LENGTH   = 0.5          # metres - length of each square side
FORWARD_SPEED = 0.3          # m/s - linear velocity when driving straight
TURN_SPEED    = 0.5          # rad/s - angular velocity when turning left

DRIVE_TIME = SIDE_LENGTH / FORWARD_SPEED         # seconds to cover one side
TURN_TIME  = (math.pi / 2.0) / TURN_SPEED        # seconds for a 90° left turn


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

        # ── Sensor state ─────────────────────────────────────────────────────
        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_yaw      = 0.0   # degrees
        self.nearest_obstacle = float('inf')  # metres

        # ── Sequence state ───────────────────────────────────────────────────
        # Phases 0, 2, 4, 6 -> drive forward one side
        # Phases 1, 3, 5, 7 -> turn left 90°
        # Phase 8            -> done
        self.phase            = 0
        self.phase_start_time = None
        self.test_done        = False

        # ── Control loop (10 Hz) ─────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Square test node started - side={SIDE_LENGTH} m  '
            f'drive={DRIVE_TIME:.2f}s  turn={TURN_TIME:.2f}s')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        self.nearest_obstacle = min(valid) if valid else float('inf')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    def _start_phase(self, label):
        """Record the start time for a new phase and log it."""
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(label)

    # ── Control loop ─────────────────────────────────────────────────────────

    def control_loop(self):
        if self.test_done:
            self.stop()
            return

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = (now - self.phase_start_time) if self.phase_start_time is not None else 0.0

        # ── Drive phases (0, 2, 4, 6) ────────────────────────────────────────
        if self.phase in (0, 2, 4, 6):
            side_num = self.phase // 2 + 1 # 1-based just for nice logging

            if self.phase_start_time is None:
                self._start_phase(f'Phase {self.phase}: driving side {side_num}')

            if elapsed < DRIVE_TIME:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.get_logger().info(
                    f'Phase {self.phase} complete | '
                    f'x={self.current_x:.4f}  y={self.current_y:.4f}  '
                    f'yaw={self.current_yaw:.2f} deg')
                self.phase += 1
                self.phase_start_time = None

        # ── Turn phases (1, 3, 5, 7) ─────────────────────────────────────────
        elif self.phase in (1, 3, 5, 7):
            turn_num = (self.phase + 1) // 2 # 1-based label just for nice logging

            if self.phase_start_time is None:
                self._start_phase(f'Phase {self.phase}: turning left (corner {turn_num})')

            if elapsed < TURN_TIME:
                self.drive(0.0, TURN_SPEED)
            else:
                self.stop()
                self.get_logger().info(
                    f'Phase {self.phase} complete | '
                    f'yaw={self.current_yaw:.2f} deg')
                self.phase += 1
                self.phase_start_time = None

        # ── Done ──────────────────────────────────────────────────────────────
        elif self.phase == 8:
            self.stop()
            closing_error = math.sqrt(self.current_x ** 2 + self.current_y ** 2)
            self.get_logger().info('═' * 50)
            self.get_logger().info('Square path complete')
            # not required, but just to see
            self.get_logger().info(
                f'Final pose  x={self.current_x:.4f} m  '
                f'y={self.current_y:.4f} m  '
                f'yaw={self.current_yaw:.2f} deg')
            self.get_logger().info(f'Closing error: {closing_error:.4f} m')
            self.get_logger().info('═' * 50)
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