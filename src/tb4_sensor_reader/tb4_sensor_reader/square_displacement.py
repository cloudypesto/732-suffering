#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


NAMESPACE = '/T23'

FORWARD_SPEED = 0.2
TURN_SPEED = 0.5

SIDE_LENGTH = 1.0
TURN_ANGLE = 90.0

FORWARD_DURATION = SIDE_LENGTH / FORWARD_SPEED
TURN_DURATION = math.radians(TURN_ANGLE) / TURN_SPEED


class SquareNode(Node):

    def __init__(self):
        super().__init__('square_node')

        # QoS FIX (critical for Create robots)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            f'{NAMESPACE}/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'{NAMESPACE}/odom',
            self.odom_cb,
            qos
        )

        self.odom_received = False

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.phase = 0
        self.phase_start = None

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("Square node started (waiting for odom...)")

    # ───────── ODOM ─────────
    def odom_cb(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.degrees(math.atan2(siny, cosy))

        if not self.odom_received:
            self.odom_received = True
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
            self.get_logger().info("✔ ODOM RECEIVED")

        self.x = x - self.start_x
        self.y = y - self.start_y

        rel = yaw - self.start_yaw
        self.yaw = (rel + 180) % 360 - 180

    # ───────── DRIVE ─────────
    def drive(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    # ───────── LOOP ─────────
    def loop(self):

        if not self.odom_received:
            self.get_logger().info_throttle(2.0, "Waiting for odom...")
            self.stop()
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.phase < 8:

            if self.phase_start is None:
                self.phase_start = now
                self.get_logger().info(f"Phase {self.phase}")

            t = now - self.phase_start

            if self.phase % 2 == 0:
                if t < FORWARD_DURATION:
                    self.drive(FORWARD_SPEED, 0.0)
                else:
                    self.stop()
                    self.phase += 1
                    self.phase_start = None
            else:
                if t < TURN_DURATION:
                    self.drive(0.0, -TURN_SPEED)
                else:
                    self.stop()
                    self.phase += 1
                    self.phase_start = None

        else:
            err = math.sqrt(self.x**2 + self.y**2)
            self.get_logger().info(f"Closing error: {err:.4f}")
            self.stop()
            raise SystemExit


def main():
    rclpy.init()
    node = SquareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()