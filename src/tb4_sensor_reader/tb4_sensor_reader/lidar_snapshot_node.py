#!/usr/bin/env python3
"""
LiDAR Snapshot Node — Test B2
COMPSYS732

Prints per scan:
- Total beams
- Minimum valid range
- Forward-facing range (corrected for TB4 offset)
"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Update robot number
NAMESPACE = '/T14'


class LidarSnapshot(Node):

    def __init__(self):
        super().__init__('lidar_snapshot')

        self.subscription = self.create_subscription(
            LaserScan,
            f'{NAMESPACE}/scan',
            self.scan_callback,
            10)

        self.get_logger().info('LiDAR Snapshot Node started')

    def scan_callback(self, msg):

        # Total beams
        total_beams = len(msg.ranges)

        # Minimum valid range
        valid_ranges = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]

        if valid_ranges:
            min_range = min(valid_ranges)
        else:
            min_range = float('nan')

        # Forward range (FIXED OFFSET)
        n = total_beams

        # Based on our observation:
        # midpoint = right → shift +90° to get forward
        forward_idx = (n // 2 + n // 4) % n

        forward_value = msg.ranges[forward_idx]

        if not (math.isfinite(forward_value) and msg.range_min <= forward_value <= msg.range_max):
            forward_value = float('nan')

        # Print results 
        print(f"Beams: {total_beams} | Min: {min_range:.3f} m | Forward: {forward_value:.3f} m")


def main(args=None):
    rclpy.init(args=args)
    node = LidarSnapshot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
