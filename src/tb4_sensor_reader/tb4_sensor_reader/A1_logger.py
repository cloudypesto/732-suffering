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
RUN_DURATION = 15.0  # seconds (adjust if needed)


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
        self.last_x = None
        self.last_y = None

        self.start_time = None
        self.finished = False

        self.get_logger().info("A1 Logger started (waiting for motion)")

        if not os.path.exists(CSV_FILE):
            with open(CSV_FILE, 'w', newline='') as f:
                writer = csv.writer(f)
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

        # capture start on first message
        if self.start_time is None:
            self.start_time = time.time()
            self.start_x = x
            self.start_y = y
            self.last_x = x
            self.last_y = y

            self.get_logger().info(
                f"START captured: x={x:.3f}, y={y:.3f}"
            )
            return

        # update continuously
        self.last_x = x
        self.last_y = y

        # TIME-BASED STOP CONDITION
        if not self.finished and (time.time() - self.start_time > RUN_DURATION):
            self.finalise_and_save()
            rclpy.shutdown()

    def finalise_and_save(self):
        if self.finished:
            return

        dx = self.last_x - self.start_x
        dy = self.last_y - self.start_y
        displacement = math.sqrt(dx**2 + dy**2)

        self.get_logger().info("----- FINAL RESULTS -----")
        self.get_logger().info(
            f"Start: ({self.start_x:.3f}, {self.start_y:.3f})"
        )
        self.get_logger().info(
            f"Final: ({self.last_x:.3f}, {self.last_y:.3f})"
        )
        self.get_logger().info(
            f"Displacement: {displacement:.3f} m"
        )

        with open(CSV_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                TRIAL,
                EXPECTED_DISTANCE,
                self.start_x, self.start_y,
                self.last_x, self.last_y,
                displacement
            ])

        self.get_logger().info(f"Saved to {CSV_FILE}")
        self.finished = True


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