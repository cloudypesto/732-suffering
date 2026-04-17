#!/usr/bin/env python3
"""
A1 Odometry Logger (Clean + Consistent Frame)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import csv
import os
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class A1OdomLogger(Node):

    def __init__(self, namespace, target, duration, trial):
        super().__init__('a1_odom_logger')

        self.namespace = namespace.rstrip('/')
        self.target = target
        self.duration = duration
        self.trial = trial

        self.data = []
        self.start_time = None
        self.recording = True

        topic = f"{self.namespace}/odom"
        self.subscription = self.create_subscription(
            Odometry, topic, self.odom_callback, 10
        )

        print(f"\nA1 Logger running on {topic}")
        print("Waiting for odometry...\n")

        self.start_time = time.time()

    def odom_callback(self, msg):
        if not self.recording:
            return

        elapsed = time.time() - self.start_time

        if elapsed > self.duration:
            self.recording = False
            return

        pos = msg.pose.pose.position

        self.data.append((elapsed, pos.x, pos.y))

    # -----------------------------
    # FIXED: consistent frame usage
    # -----------------------------
    def compute_displacement(self):
        if len(self.data) < 2:
            return 0.0

        x0 = self.data[0][1]
        y0 = self.data[0][2]

        fx = self.data[-1][1] - x0
        fy = self.data[-1][2] - y0

        return math.sqrt(fx**2 + fy**2)

    def save_csv(self, path):
        displacement = self.compute_displacement()

        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["trial", "displacement_m"])
            writer.writerow([self.trial, round(displacement, 4)])

        print(f"Final displacement saved → {path}")

    def plot(self, path):
        if not self.data:
            return

        # normalise trajectory
        x0 = self.data[0][1]
        y0 = self.data[0][2]

        xs = [p[1] - x0 for p in self.data]
        ys = [p[2] - y0 for p in self.data]

        plt.figure(figsize=(6, 6))

        plt.plot(xs, ys, linewidth=2, label="Actual trajectory")
        plt.scatter(xs[0], ys[0], label="Start")
        plt.scatter(xs[-1], ys[-1], label="End")

        # ideal path (same frame now)
        plt.plot([0, self.target], [0, 0], '--', label="Ideal path")

        plt.title("A1 Odometry Test (Normalised Frame)")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        plt.savefig(path, dpi=150)
        plt.close()

        print(f"Plot saved → {path}")


def main():
    rclpy.init()

    namespace = "/T12"
    target = 1.0
    duration = 15
    trial = 1

    node = A1OdomLogger(namespace, target, duration, trial)

    try:
        while rclpy.ok() and node.recording:
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass

    finally:
        base = os.path.expanduser("~")

        csv_path = os.path.join(base, f"a1_trial_{trial}.csv")
        plot_path = os.path.join(base, f"a1_trial_{trial}.png")

        node.save_csv(csv_path)
        node.plot(plot_path)

        node.destroy_node()
        rclpy.shutdown()

        print("\nDone")
        print("CSV:", csv_path)
        print("Plot:", plot_path)


if __name__ == "__main__":
    main()