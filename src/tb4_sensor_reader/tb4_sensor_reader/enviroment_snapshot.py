#!/usr/bin/env python3
"""
Environment Snapshot Node
COMPSYS732

Subscribes to LiDAR scan and prints:
- Total number of beams
- Minimum valid range
- Range straight ahead

Runs continuously until Ctrl+C.
"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ── Set your robot namespace ────────────────────────────────────────────────
NAMESPACE = '/T12'   # ← change this


class EnvironmentSnapshot(Node):

    def __init__(self):
        super().__init__('environment_snapshot')

        # ── Subscriber ──────────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'{NAMESPACE}/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Environment snapshot node started')

    # ── LiDAR callback ──────────────────────────────────────────────────────
    def scan_callback(self, msg):
        # Total number of beams
        total_beams = len(msg.ranges)

        # Valid ranges only
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        # Minimum valid range
        min_range = min(valid_ranges) if valid_ranges else float('inf')
        max_range = max(valid_ranges) if valid_ranges else float('inf')

        target_angle = -math.pi / 2   # +90 degrees (left)

        front_index = int((target_angle - msg.angle_min) / msg.angle_increment)

        # Clamp index to valid bounds
        front_index = max(0, min(front_index, total_beams - 1))

        front_range = msg.ranges[front_index]

        # Print once per scan
        self.get_logger().info(
            f'Beams: {total_beams} | '
            f'Min range: {min_range:.3f} m | '
            f'Max range: {max_range:.3f} m | '
            f'Front range: {front_range:.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentSnapshot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()