#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

NAMESPACE = '/T12'
FORWARD_SPEED = 0.2
TARGET_DISTANCE = 1.0
DRIVE_DURATION = TARGET_DISTANCE / FORWARD_SPEED


class A1TestNode(Node):

    def __init__(self):
        super().__init__('a1_test_node')

        self.cmd_pub = self.create_publisher(
            Twist, f'{NAMESPACE}/cmd_vel', 10)

        self.phase = 0
        self.phase_start_time = None
        self.test_done = False

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("A1 Test Node started")

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    def control_loop(self):
        if self.test_done:
            self.stop()
            return

        now = self.get_clock().now().nanoseconds / 1e9

    
        # PHASE 0 - Stabilisation
        if self.phase == 0:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.get_logger().info("Stabilising for 2 seconds...")

            if now - self.phase_start_time < 2.0:
                self.stop()
            else:
                self.phase = 1
                self.phase_start_time = None


        # PHASE 1 - Drive forward
        elif self.phase == 1:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.get_logger().info(
                    f"Driving forward for {DRIVE_DURATION:.2f} seconds")

            elapsed = now - self.phase_start_time

            if elapsed < DRIVE_DURATION:
                self.drive(FORWARD_SPEED, 0.0)
            else:
                self.stop()
                self.get_logger().info("Motion complete")
                self.phase = 2
                self.phase_start_time = None


        # PHASE 2 - Final pause for odom to settle
        elif self.phase == 2:
            if self.phase_start_time is None:
                self.phase_start_time = now
                self.get_logger().info("Pausing before ending test...")

            if now - self.phase_start_time < 2.0:
                self.stop()
            else:
                self.get_logger().info(
                    "A1 test complete — check subscriber output")
                self.test_done = True


def main(args=None):
    rclpy.init(args=args)
    node = A1TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
