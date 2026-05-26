#!/usr/bin/env python3
"""
LiDAR Direction Tester — COMPSYS732
Prints front/left/right readings for every offset candidate so you can
hold your hand in each direction and immediately see which reading responds.

Run:
    ~/ros2_venv/bin/python3 -m tb4_sensor_reader.lidar_test
    (or however you run nodes in your setup)
"""

import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

NAMESPACE = '/T7'

# All candidates to test simultaneously
OFFSET_CANDIDATES_DEG = [-180, -90, -45, 0, 45, 90, 180]

class LidarTester(Node):

    def __init__(self):
        super().__init__('lidar_tester')
        self.create_subscription(
            LaserScan, f'{NAMESPACE}/scan', self.scan_callback, 10)
        self._count = 0
        self.get_logger().info('LiDAR tester running — hold your hand in front, then left, then right.')
        self.get_logger().info(f'Testing offsets: {OFFSET_CANDIDATES_DEG} degrees')
        self.get_logger().info('=' * 80)

    def scan_callback(self, msg):
        self._count += 1
        if self._count % 10 != 0:   # print every 10th scan (~1 Hz)
            return

        inc = msg.angle_increment
        n   = len(msg.ranges)

        def arc_min(center_i, half_deg=20):
            hw = int(round(math.radians(half_deg) / inc))
            lo = max(0, center_i - hw)
            hi = min(n - 1, center_i + hw)
            vals = [r for r in msg.ranges[lo:hi+1]
                    if msg.range_min < r < msg.range_max
                    and not math.isnan(r) and not math.isinf(r)]
            return min(vals) if vals else float('inf')

        self.get_logger().info(
            f'angle_min={math.degrees(msg.angle_min):.1f}°  '
            f'angle_max={math.degrees(msg.angle_max):.1f}°  '
            f'n={n}  inc={math.degrees(inc):.2f}°/step')

        lines = []
        for off_deg in OFFSET_CANDIDATES_DEG:
            off_rad  = math.radians(off_deg)
            front_i  = int(round((off_rad       - msg.angle_min) / inc))
            left_i   = int(round((off_rad + math.radians(90)  - msg.angle_min) / inc))
            right_i  = int(round((off_rad - math.radians(90)  - msg.angle_min) / inc))

            front = arc_min(front_i)
            left  = arc_min(left_i)
            right = arc_min(right_i)

            lines.append(
                f'  offset={off_deg:+4d}°  '
                f'FRONT={front:5.2f}m  '
                f'LEFT={left:5.2f}m  '
                f'RIGHT={right:5.2f}m'
            )

        self.get_logger().info('┌─ Offset candidates ────────────────────────────────────┐')
        for l in lines:
            self.get_logger().info(l)
        self.get_logger().info('└────────────────────────────────────────────────────────┘')
        self.get_logger().info(
            '^ Hold hand in FRONT → FRONT column should be smallest. '
            'Matching row = correct offset.')
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    node = LidarTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()