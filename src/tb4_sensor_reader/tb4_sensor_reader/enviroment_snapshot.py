#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ── Set your robot namespace ─────────────────────────────────────────────
NAMESPACE = '/T2'   # change to your robot


class EnvironmentSnapshot(Node):

    def __init__(self):
        super().__init__('environment_snapshot')

        self.subscription = self.create_subscription(
            LaserScan,
            f'{NAMESPACE}/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Environment snapshot node running...')

    def scan_callback(self, msg):
        ranges = msg.ranges
        n_beams = len(ranges)

        # ── Minimum valid range ─────────────────────────────────────────
        valid_ranges = [
            r for r in ranges
            if msg.range_min <= r <= msg.range_max
        ]
        min_range = min(valid_ranges) if valid_ranges else float('inf')

        # ── Forward-facing beam (TurtleBot 4 orientation) ───────────────
        forward_idx = (n_beams // 2) - (n_beams // 4)
        forward_idx = forward_idx % n_beams
        forward_range = ranges[forward_idx]

        # ── Print once per scan ─────────────────────────────────────────
        print(
            f"Beams: {n_beams} | "
            f"Min: {min_range:.3f} m | "
            f"Forward: {forward_range:.3f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentSnapshot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()