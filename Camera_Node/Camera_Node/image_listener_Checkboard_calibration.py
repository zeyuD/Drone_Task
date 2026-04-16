import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageMetadataSubscriber(Node):
    def __init__(self):
        super().__init__('image_metadata_subscriber')
        self.bridge = CvBridge()
        self.topic_name = '/image_raw'

        # --- SETTINGS ---
        # Number of inner corners on your checkerboard (Width, Height)
        self.CHECKERBOARD = (9, 6)
        self.SQUARE_SIZE = 20  # Size of a square in millimeters (example)
        # Resolution to calibrate
        self.WIDTH, self.HEIGHT = 640, 480
        # Termination criteria for refining corner detection
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objpoints = [] # 3d points in real world space
        self.imgpoints = [] # 2d points in image plane

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10) # History depth
        
        self.get_logger().info(f'Monitoring metadata on {self.topic_name}...')

    def listener_callback(self, msg):
        # Extract and print metadata fields
        # # Note: 'seq' is no longer a part of the Header in ROS 2.
        # self.get_logger().info('--- New Frame Metadata ---')
        # self.get_logger().info(f"Frame ID: {msg.header.frame_id}")
        # self.get_logger().info(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        # self.get_logger().info(f"Resolution: {msg.width}x{msg.height}")
        # self.get_logger().info(f"Encoding: {msg.encoding}")
        # self.get_logger().info(f"Step (row length in bytes): {msg.step}")
        # self.get_logger().info(f"Is Bigendian: {bool(msg.is_bigendian)}")
        # self.get_logger().info('--------------------------')
        received_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... based on CHECKERBOARD size
        objp = np.zeros((self.CHECKERBOARD[0] * self.CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.CHECKERBOARD[0], 0:self.CHECKERBOARD[1]].T.reshape(-1, 2) * self.SQUARE_SIZE  # Scale by square size

        gray = cv2.cvtColor(received_image, cv2.COLOR_BGR2GRAY)
        # Find chessboard corners
        ret_corners, corners = cv2.findChessboardCorners(gray, self.CHECKERBOARD, None)
        if ret_corners:
            # Refine corners and draw them for visual feedback
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            cv2.drawChessboardCorners(received_image, self.CHECKERBOARD, corners2, ret_corners)

        cv2.imshow('Calibration - Logitech Brio 500', received_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and ret_corners:
            self.objpoints.append(objp)
            self.imgpoints.append(corners2)
            print(f"Frame captured! Total: {len(self.imgpoints)}")
        elif key == ord('c') and len(self.objpoints) > 0:
            # --- CALCULATION ---
            if len(self.imgpoints) > 0:
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, (self.WIDTH, self.HEIGHT), None, None)

                print("\n--- Calibration Results ---")
                print(f"Focal Length x (fx): {mtx[0,0]:.2f} px")
                print(f"Focal Length y (fy): {mtx[1,1]:.2f} px")
                print(f"Principal Point (cx, cy): ({mtx[0,2]:.2f}, {mtx[1,2]:.2f})")
                print("\nFull Camera Matrix:\n", mtx)
                print("\nDistortion Coefficients:\n", dist)
            else:
                print("No frames were captured for calibration.")


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageMetadataSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
