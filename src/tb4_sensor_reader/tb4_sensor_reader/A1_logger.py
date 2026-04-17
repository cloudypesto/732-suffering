#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import csv
import os

NAMESPACE = '/T12'
CSV_FILE = "a1_results.csv"


class A1OdomSubscriber(Node):

    def __init__(self):
        super().__init__('a1_odom_subscriber')

        self.subscription = self.create_subscription(
            Odometry,
            f'{NAMESPACE}/odom',
            self.odom_callback,
            10
        )

        self.start_x = None
        self.start_y = None
        self.last_x = 0.0
        self.last_y = 0.0

        self.get_logger().info("A1 Subscriber started")

        # Create CSV with header if missing
        if not os.path.exists(CSV_FILE):
            with open(CSV_FILE, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "start_x", "start_y",
                    "final_x", "final_y",
                    "displacement"
                ])

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Store start position once
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(
                f"Start position: x={x:.3f}, y={y:.3f}")
            return

        # Continuously update last known position
        self.last_x = x
        self.last_y = y

    def destroy_node(self):
        # Compute odometry displacement
        dx = self.last_x - self.start_x
        dy = self.last_y - self.start_y
        displacement = math.sqrt(dx**2 + dy**2)

        self.get_logger().info("----- FINAL RESULTS -----")
        self.get_logger().info(
            f"Final position: x={self.last_x:.3f}, y={self.last_y:.3f}")
        self.get_logger().info(
            f"Displacement (odom): {displacement:.3f} m")

        # Append to CSV
        with open(CSV_FILE, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.start_x, self.start_y,
                self.last_x, self.last_y,
                displacement
            ])

        self.get_logger().info(f"Saved to {CSV_FILE}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = A1OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
