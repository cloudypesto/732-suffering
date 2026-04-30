#!/usr/bin/env python3
"""
Test Node — Square Displacement (Closed-Loop Drift, Test A2)
COMPSYS732

Drives the robot in a square and measures odometry drift.
Outputs:
- Closing error (m)
- Final heading error (deg)
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Parameters 
NAMESPACE = '/T12'   # Update robot numeber

FORWARD_SPEED = 0.2      # m/s
TURN_SPEED = 0.5         # rad/s

SIDE_LENGTH = 1.0        # metres
TURN_ANGLE = 90.0        # degrees

FORWARD_DURATION = SIDE_LENGTH / FORWARD_SPEED
TURN_DURATION = math.radians(TURN_ANGLE) / TURN_SPEED


class SquareDisplacement(Node):

    def __init__(self):
        super().__init__('square_displacement')

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, f'{NAMESPACE}/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{NAMESPACE}/odom',
            self.odom_callback,
            10)

        # State variables
        self.start_x = None
        self.start_y = None
        self.initial_yaw = None

        self.local_x = 0.0
        self.local_y = 0.0
        self.current_yaw = 0.0

        self.phase = 0
        self.phase_start_time = None
        self.test_done = False

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Square Displacement Test Node started')

    # Odometry callback 
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.degrees(math.atan2(siny, cosy))

        # Set reference frame
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.initial_yaw = yaw
            self.get_logger().info('Reference frame set: (0,0), heading = 0°')

        # Local position
        self.local_x = x - self.start_x
        self.local_y = y - self.start_y

        # Relative heading
        rel = yaw - self.initial_yaw
        rel = (rel + 180) % 360 - 180
        self.current_yaw = rel

    # Motion helpers 
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

        # Wait for odometry
        if self.start_x is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Phases 0–7: square motion
        if self.phase < 8:

            if self.phase_start_time is None:
                self.phase_start_time = now

                if self.phase % 2 == 0:
                    self.get_logger().info(f'Phase {self.phase}: Forward')
                else:
                    self.get_logger().info(f'Phase {self.phase}: Turn')

            elapsed = now - self.phase_start_time

            # Even phases → forward
            if self.phase % 2 == 0:
                if elapsed < FORWARD_DURATION:
                    self.drive(FORWARD_SPEED, 0.0)
                else:
                    self.stop()
                    self.phase += 1
                    self.phase_start_time = None

            # Odd phases → turn
            else:
                if elapsed < TURN_DURATION:
                    self.drive(0.0, -TURN_SPEED)
                else:
                    self.stop()
                    self.phase += 1
                    self.phase_start_time = None

        # Phase 8: Evaluation 
        elif self.phase == 8:

            dx = self.local_x
            dy = self.local_y

            closing_error = math.sqrt(dx**2 + dy**2)
            heading_error = self.current_yaw

            # REQUIRED OUTPUT FORMAT
            self.get_logger().info(f"{closing_error:.6f}")
            self.get_logger().info(f"{heading_error:.6f}")

            self.test_done = True


def main(args=None):
    rclpy.init(args=args)
    node = SquareDisplacement()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()