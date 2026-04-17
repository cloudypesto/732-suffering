#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import csv
import os
import time

NAMESPACE = '/T12'
CSV_FILE = "a1_results.csv"

TRIAL = 1
EXPECTED_DISTANCE = 1.0


class A1OdomSubscriber(Node):

    def __init__(self):
        super().__init__('a1_odom_subscriber')

        self.subscription = self.create_subscription(
            Odometry,
            f'{NAMESPACE}/odom',
            self.odom_callback,
            10
        )

        # --- state ---
        self.start_x = None
        self.start_y = None
        self.last_x = None
        self.last_y = None

        self.started = False
        self.finished = False

        self.get_logger().info("A1 Odom Logger started (waiting for motion...)")

        # Create CSV if needed
        if not os.path.exists(CSV_FILE):
            with open(CSV_FILE, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "trial",
                    "expected_m",
                    "start_x", "start_y",
                    "final_x", "final_y",
                    "displacement_m"
                ])

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # wait until robot actually starts moving node-wise
        if not self.started:
            self.start_x = x
            self.start_y = y
            self.last_x = x
            self.last_y = y
            self.started = True

            self.get_logger().info(
                f"START captured: x={x:.3f}, y={y:.3f}"
            )
            return

        # continuously update last known pose
        self.last_x = x
        self.last_y = y

    def finalise_and_save(self):
        if self.finished or self.start_x is None or self.last_x is None:
            return

        dx = self.last_x - self.start_x
        dy = self.last_y - self.start_y
        displacement = math.sqrt(dx**2 + dy**2)

        self.get_logger().info("----- FINAL RESULTS -----")
        self.get_logger().info(
            f"Final x={self.last_x:.3f}, y={self.last_y:.3f}"
        )
        self.get_logger().info(
            f"Displacement: {displacement:.3f} m"
        )

        with open(CSV_FILE, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                TRIAL,
                EXPECTED_DISTANCE,
                self.start_x, self.start_y,
                self.last_x, self.last_y,
                displacement
            ])

        self.get_logger().info(f"Saved to {CSV_FILE}")
        self.finished = True

    def destroy_node(self):
        # ensure final write always happens
        self.finalise_and_save()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = A1OdomSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.finalise_and_save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()