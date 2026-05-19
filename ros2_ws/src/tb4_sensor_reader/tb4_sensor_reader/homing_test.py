#!/usr/bin/env python3
"""
Homing / Axis Alignment Debug Script — COMPSYS732
===================================================
Run this BEFORE the autonomous search to verify that the robot's odometry
frame matches the PGM map coordinate frame.

Sequence
--------
  Phase 1 — ALIGN_X  : rotate until the robot faces +X  (yaw ≈ 0°)
  Phase 2 — HOLD_X   : hold for HOLD_DURATION seconds so you can observe
  Phase 3 — ALIGN_Y  : rotate until the robot faces +Y  (yaw ≈ 90°)
  Phase 4 — HOLD_Y   : hold for HOLD_DURATION seconds so you can observe
  Phase 5 — DONE     : stop and print summary

Expected result
---------------
  • During ALIGN_X the robot's nose should point along the PGM map's +X axis.
  • During ALIGN_Y the robot's nose should point along the PGM map's +Y axis.

If the robot points the wrong way, adjust LIDAR_OFFSET_DEG in
autonomous_search.py (or swap your map's origin/resolution).

To run:
    ~/ros2_venv/bin/python3 homing_debug.py
"""

import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T6'

# ── Rotation speed ─────────────────────────────────────────────────────────────
TURN_SPEED      = 0.3        # rad/s — slow so alignment is accurate
HEADING_TOL     = 0.03       # rad (~1.7°) — tight tolerance for a static check

# ── How long to pause at each axis so you can inspect the robot ───────────────
HOLD_DURATION   = 3.0        # seconds

# ── Target headings (map frame) ───────────────────────────────────────────────
TARGET_X_YAW    = 0.0                  # +X axis → yaw = 0 rad
TARGET_Y_YAW    = math.pi / 2.0       # +Y axis → yaw = π/2 rad


class HomingDebug(Node):

    def __init__(self):
        super().__init__('homing_debug')

        self.cmd_pub = self.create_publisher(Twist, f'{NAMESPACE}/cmd_vel', 10)
        self.create_subscription(Odometry, f'{NAMESPACE}/odom',
                                 self._odom_cb, 10)

        self.current_yaw = None          # None until first odom message arrives
        self.state       = 'WAIT_ODOM'
        self.hold_start  = None

        self.timer = self.create_timer(0.05, self._loop)   # 20 Hz
        self.get_logger().info('Homing debug node started — waiting for odometry…')

    # ── Odometry ──────────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _rotate_toward(self, target_yaw):
        """
        Publish a rotation command toward target_yaw.
        Returns True when within HEADING_TOL.
        """
        err = self._angle_diff(target_yaw, self.current_yaw)
        if abs(err) <= HEADING_TOL:
            self._stop()
            return True
        # Scale speed down as we approach to avoid overshoot
        scale = min(1.0, abs(err) / 0.5)
        angular = math.copysign(TURN_SPEED * scale, err)
        msg = Twist()
        msg.angular.z = angular
        self.cmd_pub.publish(msg)
        return False

    @staticmethod
    def _angle_diff(target, current):
        d = target - current
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):

        # ── WAIT_ODOM ────────────────────────────────────────────────────────
        if self.state == 'WAIT_ODOM':
            if self.current_yaw is not None:
                self.get_logger().info(
                    f'Odometry received — initial yaw = '
                    f'{math.degrees(self.current_yaw):.1f}°')
                self.get_logger().info(
                    'Phase 1: Rotating to face +X axis (yaw = 0°)…')
                self.state = 'ALIGN_X'
            return

        # ── ALIGN_X ──────────────────────────────────────────────────────────
        if self.state == 'ALIGN_X':
            aligned = self._rotate_toward(TARGET_X_YAW)
            self.get_logger().info(
                f'[ALIGN_X] yaw={math.degrees(self.current_yaw):+.1f}°  '
                f'err={math.degrees(self._angle_diff(TARGET_X_YAW, self.current_yaw)):+.1f}°',
                throttle_duration_sec=0.5)
            if aligned:
                self.get_logger().info(
                    f'✓ Aligned to +X  yaw={math.degrees(self.current_yaw):.2f}°  '
                    f'— holding for {HOLD_DURATION:.0f} s')
                self.hold_start = time.time()
                self.state = 'HOLD_X'
            return

        # ── HOLD_X ───────────────────────────────────────────────────────────
        if self.state == 'HOLD_X':
            self._stop()
            elapsed = time.time() - self.hold_start
            remaining = HOLD_DURATION - elapsed
            self.get_logger().info(
                f'[HOLD_X] yaw={math.degrees(self.current_yaw):+.1f}°  '
                f'holding… {remaining:.1f} s remaining',
                throttle_duration_sec=0.5)
            if elapsed >= HOLD_DURATION:
                self.get_logger().info(
                    'Phase 2: Rotating to face +Y axis (yaw = 90°)…')
                self.state = 'ALIGN_Y'
            return

        # ── ALIGN_Y ──────────────────────────────────────────────────────────
        if self.state == 'ALIGN_Y':
            aligned = self._rotate_toward(TARGET_Y_YAW)
            self.get_logger().info(
                f'[ALIGN_Y] yaw={math.degrees(self.current_yaw):+.1f}°  '
                f'err={math.degrees(self._angle_diff(TARGET_Y_YAW, self.current_yaw)):+.1f}°',
                throttle_duration_sec=0.5)
            if aligned:
                self.get_logger().info(
                    f'✓ Aligned to +Y  yaw={math.degrees(self.current_yaw):.2f}°  '
                    f'— holding for {HOLD_DURATION:.0f} s')
                self.hold_start = time.time()
                self.state = 'HOLD_Y'
            return

        # ── HOLD_Y ───────────────────────────────────────────────────────────
        if self.state == 'HOLD_Y':
            self._stop()
            elapsed = time.time() - self.hold_start
            remaining = HOLD_DURATION - elapsed
            self.get_logger().info(
                f'[HOLD_Y] yaw={math.degrees(self.current_yaw):+.1f}°  '
                f'holding… {remaining:.1f} s remaining',
                throttle_duration_sec=0.5)
            if elapsed >= HOLD_DURATION:
                self.state = 'DONE'
            return

        # ── DONE ─────────────────────────────────────────────────────────────
        if self.state == 'DONE':
            self._stop()
            self.get_logger().info('=' * 55)
            self.get_logger().info('HOMING DEBUG COMPLETE')
            self.get_logger().info(
                f'  Final yaw : {math.degrees(self.current_yaw):.2f}°')
            self.get_logger().info('')
            self.get_logger().info('  Interpret results:')
            self.get_logger().info(
                '  • Robot nose pointed DOWN the arena  when yaw=0°  → +X OK')
            self.get_logger().info(
                '  • Robot nose pointed LEFT of arena   when yaw=90° → +Y OK')
            self.get_logger().info(
                '  If directions are wrong, the map origin/orientation')
            self.get_logger().info(
                '  does not match odometry — adjust LIDAR_OFFSET_DEG')
            self.get_logger().info(
                '  or re-export the map with the correct origin.')
            self.get_logger().info('=' * 55)
            # Destroy the timer so the node idles without spamming logs
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = HomingDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()