#!/usr/bin/env python3
"""
A1 Logger — Linear Odometry Test (Clean Version)
"""

import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class A1Logger(Node):

    def __init__(self, namespace, target, duration, trial):
        super().__init__('a1_logger')

        self.namespace = namespace.rstrip('/')
        self.target = target
        self.duration = duration
        self.trial = trial

        self.data = []
        self.start_time = time.time()
        self.recording = True

        topic = f"{self.namespace}/odom"

        self.subscription = self.create_subscription(
            Odometry, topic, self.odom_callback, 10
        )

        print(f"\nA1 Logger started")
        print(f"Trial: {trial}")
        print(f"Topic: {topic}")
        print(f"Target: {target} m")
        print(f"Duration: {duration} s\n")

    def odom_callback(self, msg):

        if not self.recording:
            return

        elapsed = time.time() - self.start_time

        if elapsed > self.duration:
            self.recording = False
            return

        pos = msg.pose.pose.position
        self.data.append((elapsed, pos.x, pos.y))

    # ----------------------------
    # METRICS
    # ----------------------------
    def compute_displacement(self):

        if len(self.data) < 2:
            return 0.0

        x0 = self.data[0][1]
        y0 = self.data[0][2]

        fx = self.data[-1][1] - x0
        fy = self.data[-1][2] - y0

        return math.sqrt(fx**2 + fy**2)

    def compute_error(self):
        return self.compute_displacement() - self.target

    def compute_error_pct(self):
        return abs(self.compute_error()) / self.target * 100.0

    # ----------------------------
    # CSV (APPEND ALL TRIALS)
    # ----------------------------
    def save_csv(self, path):

        disp = self.compute_displacement()
        file_exists = os.path.isfile(path)

        with open(path, 'a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(["trial", "displacement_m", "target_m", "error_m", "error_pct"])

            writer.writerow([
                self.trial,
                round(disp, 4),
                self.target,
                round(self.compute_error(), 4),
                round(self.compute_error_pct(), 2)
            ])

        print(f"CSV updated → {path}")

    # ----------------------------
    # PLOT
    # ----------------------------
    def plot(self, path):

        if not self.data:
            return

        x0 = self.data[0][1]
        y0 = self.data[0][2]

        xs = [p[1] - x0 for p in self.data]
        ys = [p[2] - y0 for p in self.data]

        plt.figure(figsize=(6, 6))

        plt.plot(xs, ys, linewidth=2, label="Actual trajectory")
        plt.scatter(xs[0], ys[0], label="Start")
        plt.scatter(xs[-1], ys[-1], label="End")

        plt.plot([0, self.target], [0, 0], '--', label="Ideal path")

        plt.title(f"A1 Odometry Test (Trial {self.trial})")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        plt.savefig(path, dpi=150)
        plt.close()

        print(f"Plot saved → {path}")


# ----------------------------
# MAIN
# ----------------------------
def main():

    parser = argparse.ArgumentParser(description="A1 Logger")
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--target', type=float, default=1.0)
    parser.add_argument('--duration', type=int, default=15)
    parser.add_argument('--trial', type=int, default=1)

    args = parser.parse_args()

    rclpy.init()

    node = A1Logger(
        args.namespace,
        args.target,
        args.duration,
        args.trial
    )

    end_time = time.time() + args.duration + 3

    try:
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass

    finally:
        node.recording = False

        disp = node.compute_displacement()
        print(f"\nTrial {node.trial} Displacement: {disp:.4f} m\n")

        base = os.path.expanduser("~")

        csv_path = os.path.join(base, "a1_displacement.csv")
        plot_path = os.path.join(base, f"a1_plot_trial{node.trial}.png")

        node.save_csv(csv_path)
        node.plot(plot_path)

        node.destroy_node()
        rclpy.shutdown()

        print("\nDONE")
        print("CSV:", csv_path)
        print("Plot:", plot_path)


if __name__ == "__main__":
    main()