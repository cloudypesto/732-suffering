import rclpy, cv2, math, csv, os, time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry

NAMESPACE      = '/T21'
FORWARD_SPEED  = 0.15
TURN_SPEED     = 0.5
AVOID_DISTANCE = 0.2
FRONT_ARC_DEG  = 60

RED_LOW1,  RED_HIGH1  = np.array([0,   160, 60]), np.array([10,  255, 255])
RED_LOW2,  RED_HIGH2  = np.array([140, 160, 60]), np.array([180, 255, 255])

DETECTION_THRESHOLD = 500    # red is visible
PHOTO_THRESHOLD     = 6000   # cube is close enough to photograph


class DetectAndStop(Node):

    def __init__(self):
        super().__init__('detect_and_stop')
        self.pub = self.create_publisher(Twist, f'{NAMESPACE}/cmd_vel', 10)
        self.create_subscription(LaserScan, f'{NAMESPACE}/scan', self.scan_callback, 10)
        self.create_subscription(
            CompressedImage,
            f'{NAMESPACE}/oakd/rgb/image_raw/compressed',
            self.image_callback, 10)
        self.create_subscription(Odometry, f'{NAMESPACE}/odom', self.odom_callback, 10)

        # Shared state
        self.nearest_front  = float('inf')
        self.nearest_left   = float('inf')
        self.nearest_right  = float('inf')
        self.cube_detected  = False
        self.current_x      = 0.0
        self.current_y      = 0.0
        self.state          = 'SEARCHING'
        self.photo_taken    = False
        self.detection_count = 0
        self.last_save_time = 0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Detect-and-stop node started — SEARCHING')

    def scan_callback(self, msg):
        inc    = msg.angle_increment
        arc_r  = math.radians(FRONT_ARC_DEG)
        side_r = math.radians(90)
        front_i = int(round(-msg.angle_min / inc))
        half_a  = int(round(arc_r  / inc))
        side_a  = int(round(side_r / inc))
        n       = len(msg.ranges)

        def arc_min(lo, hi):
            lo = max(0, lo); hi = min(n - 1, hi)
            vals = [r for r in msg.ranges[lo:hi+1]
                    if msg.range_min < r < msg.range_max]
            return min(vals) if vals else float('inf')

        self.nearest_front = arc_min(front_i - half_a, front_i + half_a)
        self.nearest_left  = arc_min(front_i,          front_i + side_a)
        self.nearest_right = arc_min(front_i - side_a, front_i)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def image_callback(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, RED_LOW1, RED_HIGH1) | cv2.inRange(hsv, RED_LOW2, RED_HIGH2)

        # Clean up speckle noise
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        pixels = int(cv2.countNonZero(mask))

        # Build overlay
        overlay = img.copy()
        overlay[mask > 0] = [0, 0, 255]
        blended = cv2.addWeighted(overlay, 0.6, img, 0.4, 0)

        # Pixel count — colour indicates state
        if pixels > PHOTO_THRESHOLD:
            colour = (0, 255, 0)    # green — close enough
        elif pixels > DETECTION_THRESHOLD:
            colour = (0, 165, 255)  # orange — detected, moving closer
        else:
            colour = (0, 255, 0)    # green — nothing yet

        cv2.putText(blended, f"Red pixels: {pixels}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)

        if pixels > PHOTO_THRESHOLD:
            cv2.putText(blended, "DETECTED", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            self.cube_detected = True
            # Take photo once, with 2s cooldown
            if not self.photo_taken and time.time() - self.last_save_time > 2.0:
                self.save_detection(blended, pixels)
                self.last_save_time = time.time()
                self.photo_taken = True
        elif pixels > DETECTION_THRESHOLD:
            cv2.putText(blended, "MOVING CLOSER", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 3)

        cv2.imshow('Camera — Red Detection', blended)
        cv2.waitKey(1)

    def save_detection(self, img, pixels):
        self.detection_count += 1
        snap_path = os.path.expanduser(f'~/detection_{self.detection_count}.png')
        csv_path  = os.path.expanduser('~/detections.csv')

        cv2.imwrite(snap_path, img)

        write_header = not os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['detection', 'pixels', 'snapshot', 'time'])
            if write_header:
                writer.writeheader()
            writer.writerow({
                'detection': self.detection_count,
                'pixels':    pixels,
                'snapshot':  snap_path,
                'time':      round(time.time(), 2),
            })

        self.get_logger().info(
            f'Photo saved: {snap_path} - {pixels} red pixels')

    def stop(self):
        self.pub.publish(Twist())

    def control_loop(self):
        if self.state == 'DONE':
            return

        if self.state == 'SEARCHING':
            if self.cube_detected:
                self.state = 'DETECTED'
                self.stop()
                self.get_logger().info('RED CUBE DETECTED - stopping')
                self.get_logger().info(
                    f'Detected position: x={self.current_x:.3f} m  y={self.current_y:.3f} m')
                return

            cmd = Twist()
            if self.nearest_front > AVOID_DISTANCE:
                cmd.linear.x  = FORWARD_SPEED
                cmd.angular.z = 0.0
            else:
                self.get_logger().info("OW")
                cmd.linear.x  = 0.0
                cmd.angular.z = (TURN_SPEED if
                    self.nearest_left >= self.nearest_right
                    else -TURN_SPEED)
            self.pub.publish(cmd)

        elif self.state == 'DETECTED':
            self.stop()
            self.state = 'DONE'


def main(args=None):
    rclpy.init(args=args)
    node = DetectAndStop()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()