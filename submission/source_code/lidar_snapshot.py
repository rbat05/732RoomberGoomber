#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T21'           # Change to your robot e.g. /T10

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'{NAMESPACE}/scan',
            self.scan_callback,
            10)

        self.get_logger().info('LiDAR scanner node started - waiting for scans...')

    def scan_callback(self, msg):
        # ── 1. Total number of beams ─────────────────────────────────────────
        total_beams = len(msg.ranges)

        # ── 2. Minimum valid range ───────────────────────────────────────────
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min <= r <= msg.range_max
        ]
        min_range = min(valid_ranges) if valid_ranges else float('inf')

        # ── 3. Range straight ahead ──────────────────────────────────────────
        ahead_index = 270  # 0° is right, 90° is back, 180° is left, 270° is ahead from our testing
        ahead_range = msg.ranges[ahead_index]

        if not (msg.range_min <= ahead_range <= msg.range_max):
            ahead_range_str = 'invalid (nan/inf or out of range)'
        else:
            ahead_range_str = f'{ahead_range:.4f} m'

        # ── Print once per scan ──────────────────────────────────────────────
        print(
            f'[SCAN] '
            f'beams={total_beams} | '
            f'min_valid={min_range:.4f} m | '
            f'ahead[{ahead_index}]={ahead_range_str}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()