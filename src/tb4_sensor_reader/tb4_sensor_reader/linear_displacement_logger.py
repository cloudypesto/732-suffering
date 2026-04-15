#!/usr/bin/env python3
"""
Test Node — Linear Displacement Accuracy (Test A1)
COMPSYS732

Drives the robot straight for a fixed distance, resets start pose to (0,0),
resets heading to 0° along +X, and checks displacement.
Pass/fail is based on 5% distance tolerance.
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Robot namespace and motion parameters 
NAMESPACE = '/T12'           # e.g. /T10
FORWARD_SPEED = 0.2          # m/s
TARGET_DISTANCE = 1.0        # metres
DRIVE_DURATION = TARGET_DISTANCE / FORWARD_SPEED   # seconds to travel target distance


class TestNode(Node):

    def __init__(self):
        super().__init__('linear_displacement_test')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, f'{NAMESPACE}/cmd_vel', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{NAMESPACE}/odom',
            self.odom_callback,
            10)

        # State variables
        self.start_x = None
        self.start_y = None
        self.local_x = 0.0
        self.local_y = 0.0
        self.current_yaw = 0.0       # degrees relative to start
        self.initial_yaw = None
        self.test_done = False

        self.phase = 0
        self.phase_start_time = None

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Linear Displacement Test Node started')

    # Odometry callback 
    def odom_callback(self, msg):
        # Raw position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Raw yaw in degrees
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        # Set starting reference (position + heading)
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.initial_yaw = raw_yaw
            self.get_logger().info('Reference frame set: (0,0) and heading = 0°')

        # Local position (origin at start)
        self.local_x = x - self.start_x
        self.local_y = y - self.start_y

        # Relative heading (0° = forward)
        rel_yaw = raw_yaw - self.initial_yaw
        rel_yaw = (rel_yaw + 180) % 360 - 180   # normalize to [-180,180]
        self.current_yaw = rel_yaw

    # Helper: publish velocity 
    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    # Control loop 
    def control_loop(self):
        if self.test_done:
            self.stop()
            return

        now = self.get_clock().now().nanoseconds / 1e9  # seconds

        # Phase 0 - Drive forward for DRIVE_DURATION seconds
        if self.phase == 0:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.get_logger().info(
                    f'Phase 0: Driving forward for {DRIVE_DURATION:.2f} s | Forward = +X')

            elapsed = now - self.phase_start_time
            if elapsed < DRIVE_DURATION:
                self.drive(FORWARD_SPEED, 0.0)
                # Print current relative heading for monitorings
                # self.get_logger().info(f'Current heading (relative to start): {self.current_yaw:.2f} deg')
            else:
                self.stop()
                self.get_logger().info(
                    f'Phase 0 complete | Final position x={self.local_x:.4f} y={self.local_y:.4f} '
                    f'| Heading={self.current_yaw:.2f} deg')
                self.phase += 1
                self.phase_start_time = None

        # Phase 1 - Evaluate displacement and check pass/fail
        elif self.phase == 1:
            dx = self.local_x
            dy = self.local_y
            displacement = math.sqrt(dx**2 + dy**2)

            # Print in assignment-required format using get_logger().info()
            self.get_logger().info(f"{self.local_x:.6f}, {self.local_y:.6f}")  # final x, y
            self.get_logger().info(f"{displacement:.6f}")                      # displacement

            self.test_done = True


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()