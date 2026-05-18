#!/usr/bin/env python3
"""
C-Shape Odometry Test — Closed Loop
COMPSYS732

Drives the robot in a C-shape path using odometry feedback for each leg and turn.
Used to measure odometry drift/error against known ground truth.

To run:
    ros2 run tb4_sensor_reader c_shape_test
    (after adding 'c_shape_test = tb4_sensor_reader.c_shape_test:main' to setup.py)
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T26'

# ── Motion parameters ──────────────────────────────────────────────────────────
FORWARD_SPEED = 0.2          # m/s
TURN_SPEED    = 0.4          # rad/s

# ── Tolerance thresholds ───────────────────────────────────────────────────────
DIST_TOLERANCE  = 0.02       # metres — how close to target distance before stopping
ANGLE_TOLERANCE = 1.0        # degrees — how close to target angle before stopping

# ── C-shape dimensions (metres) ────────────────────────────────────────────────
LEG_1_LENGTH = 3.5           # First straight — along bottom of C
LEG_2_LENGTH = 4.15           # Second straight — across middle of C
LEG_3_LENGTH = 3.5           # Third straight — along top of C


class CShapeTest(Node):

    def __init__(self):
        super().__init__('c_shape_test')

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

        # ── State variables ──────────────────────────────────────────────────
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0   # degrees

        self.phase        = 0
        self.phase_origin_x   = 0.0   # x at start of current leg
        self.phase_origin_y   = 0.0   # y at start of current leg
        self.target_yaw       = 0.0   # degrees — target heading for current turn
        self.phase_initialised = False
        self.test_done        = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('C-shape closed-loop test node started')

    # ── Odometry callback ────────────────────────────────────────────────────

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    # ── Helpers ──────────────────────────────────────────────────────────────

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    def distance_travelled(self):
        """Euclidean distance from the start of the current leg."""
        dx = self.current_x - self.phase_origin_x
        dy = self.current_y - self.phase_origin_y
        return math.sqrt(dx**2 + dy**2)

    def angle_diff(self, target, current):
        """Shortest signed difference between two angles in degrees."""
        diff = (target - current + 180) % 360 - 180
        return diff

    def begin_phase(self, label):
        self.get_logger().info(label)
        self.phase_origin_x = self.current_x
        self.phase_origin_y = self.current_y
        self.phase_initialised = True

    # ── Control loop ─────────────────────────────────────────────────────────

    def control_loop(self):
        """
        C-shape sequence:
            Phase 0 — Drive forward (Leg 1)
            Phase 1 — Turn left 90°
            Phase 2 — Drive forward (Leg 2)
            Phase 3 — Turn left 90°
            Phase 4 — Drive forward (Leg 3)
            Phase 5 — Stop, done
        """

        if self.test_done:
            self.stop()
            return

        # ── Phase 0: Leg 1 ───────────────────────────────────────────────────
        if self.phase == 0:
            if not self.phase_initialised:
                self.begin_phase(f'Phase 0: Leg 1 — driving {LEG_1_LENGTH} m forward')
            if self.distance_travelled() < LEG_1_LENGTH - DIST_TOLERANCE:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.get_logger().info(
                    f'Leg 1 complete | '
                    f'x={self.current_x:.4f} y={self.current_y:.4f}')
                self.phase += 1
                self.phase_initialised = False

        # ── Phase 1: Turn left 90° ───────────────────────────────────────────
        elif self.phase == 1:
            if not self.phase_initialised:
                self.target_yaw = self.current_yaw + 90.0
                # Normalise to [-180, 180]
                self.target_yaw = (self.target_yaw + 180) % 360 - 180
                self.begin_phase(f'Phase 1: Turning left 90° → target yaw {self.target_yaw:.1f}°')
            if abs(self.angle_diff(self.target_yaw, self.current_yaw)) > ANGLE_TOLERANCE:
                self.drive(0.0, TURN_SPEED)
            else:
                self.stop()
                self.get_logger().info(
                    f'Turn 1 complete | yaw={self.current_yaw:.2f}°')
                self.phase += 1
                self.phase_initialised = False

        # ── Phase 2: Leg 2 ───────────────────────────────────────────────────
        elif self.phase == 2:
            if not self.phase_initialised:
                self.begin_phase(f'Phase 2: Leg 2 — driving {LEG_2_LENGTH} m forward')
            if self.distance_travelled() < LEG_2_LENGTH - DIST_TOLERANCE:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.get_logger().info(
                    f'Leg 2 complete | '
                    f'x={self.current_x:.4f} y={self.current_y:.4f}')
                self.phase += 1
                self.phase_initialised = False

        # ── Phase 3: Turn left 90° ───────────────────────────────────────────
        elif self.phase == 3:
            if not self.phase_initialised:
                self.target_yaw = self.current_yaw + 90.0
                self.target_yaw = (self.target_yaw + 180) % 360 - 180
                self.begin_phase(f'Phase 3: Turning left 90° → target yaw {self.target_yaw:.1f}°')
            if abs(self.angle_diff(self.target_yaw, self.current_yaw)) > ANGLE_TOLERANCE:
                self.drive(0.0, TURN_SPEED)
            else:
                self.stop()
                self.get_logger().info(
                    f'Turn 2 complete | yaw={self.current_yaw:.2f}°')
                self.phase += 1
                self.phase_initialised = False

        # ── Phase 4: Leg 3 ───────────────────────────────────────────────────
        elif self.phase == 4:
            if not self.phase_initialised:
                self.begin_phase(f'Phase 4: Leg 3 — driving {LEG_3_LENGTH} m forward')
            if self.distance_travelled() < LEG_3_LENGTH - DIST_TOLERANCE:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.get_logger().info(
                    f'Leg 3 complete | '
                    f'x={self.current_x:.4f} y={self.current_y:.4f}')
                self.phase += 1
                self.phase_initialised = False

        # ── Phase 5: Done ────────────────────────────────────────────────────
        elif self.phase == 5:
            self.stop()
            self.get_logger().info('C-shape complete — stopped')
            self.get_logger().info(
                f'Final pose: x={self.current_x:.4f} m  y={self.current_y:.4f} m  '
                f'yaw={self.current_yaw:.2f} deg')
            self.test_done = True


def main(args=None):
    rclpy.init(args=args)
    node = CShapeTest()
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