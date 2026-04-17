#!/usr/bin/env python3
"""
A1 Odometry Logger + Plotter (TurtleBot 4)
Linear displacement test only
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


class A1OdomLogger(Node):

    def __init__(self, namespace, target, duration):
        super().__init__('a1_odom_logger')

        self.namespace = namespace.rstrip('/')
        self.target = target
        self.duration = duration

        self.data = []
        self.start_time = None
        self.recording = True

        self.start_x = None
        self.start_y = None

        topic = f"{self.namespace}/odom"
        self.subscription = self.create_subscription(
            Odometry, topic, self.odom_callback, 10
        )

        print(f"\n A1 Logger running on: {topic}")
        print(" Waiting for robot motion...\n")

        self.start_time = time.time()

    def odom_callback(self, msg):
        if not self.recording:
            return

        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self.recording = False
            return

        pos = msg.pose.pose.position

        # capture start once
        if self.start_x is None:
            self.start_x = pos.x
            self.start_y = pos.y

        self.data.append((elapsed, pos.x, pos.y))

    def compute_results(self):
        if not self.data:
            return None

        _, sx, sy = self.data[0]
        _, fx, fy = self.data[-1]

        displacement = math.sqrt((fx - sx)**2 + (fy - sy)**2)
        error = displacement - self.target
        error_pct = abs(error) / self.target * 100 if self.target > 0 else 0

        return {
            "target_m": self.target,
            "displacement_m": round(displacement, 4),
            "error_m": round(error, 4),
            "error_pct": round(error_pct, 2),
            "samples": len(self.data)
        }

    def save_csv(self, path):
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["time", "x", "y"])
            writer.writerows(self.data)

        print(f" Raw data saved: {path}")

    def save_stats(self, stats, path):
        file_exists = os.path.exists(path)

        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=stats.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(stats)

        print(f" Stats saved: {path}")

    def plot(self, path):
        if not self.data:
            return

        t = [d[0] for d in self.data]
        x = [d[1] for d in self.data]
        y = [d[2] for d in self.data]

        plt.figure(figsize=(7, 6))

        plt.plot(x, y, linewidth=2)
        plt.scatter(x[0], y[0], label="Start")
        plt.scatter(x[-1], y[-1], label="End")

        # ideal line
        plt.plot([0, self.target], [0, 0], '--', label="Ideal path")

        plt.title("A1 Odometry Linear Displacement")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        plt.savefig(path, dpi=150)
        plt.close()

        print(f" Plot saved: {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--target', type=float, default=1.0)
    parser.add_argument('--duration', type=int, default=15)
    parser.add_argument('--trial', type=int, default=1)
    args = parser.parse_args()

    base = os.path.expanduser("~")

    csv_path = os.path.join(base, f"a1_odom_trial_{args.trial}.csv")
    plot_path = os.path.join(base, f"a1_odom_trial_{args.trial}.png")
    stats_path = os.path.join(base, "a1_odom_stats.csv")

    rclpy.init()
    node = A1OdomLogger(args.namespace, args.target, args.duration)

    try:
        while rclpy.ok() and node.recording:
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        pass

    finally:
        stats = node.compute_results()

        print("\n--- A1 RESULTS ---")
        for k, v in stats.items():
            print(f"{k}: {v}")

        node.save_csv(csv_path)
        node.save_stats(stats, stats_path)
        node.plot(plot_path)

        node.destroy_node()
        rclpy.shutdown()

        print("\nDone.")
        print(f"Plot: {plot_path}")
        print(f"CSV: {csv_path}")
        print(f"Stats: {stats_path}")


if __name__ == "__main__":
    main()