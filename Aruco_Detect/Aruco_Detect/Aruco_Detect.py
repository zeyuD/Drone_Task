
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time
import serial.tools.list_ports as ser2
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ArucoNode(Node):
    def __init__(self):
        super().__init__('Aruco_Detect')
        
        self.bridge = CvBridge()
        self.input_topic_name = '/image_raw'
        self.output_topic_name = '/aruco_detections'

        # Camera Calibration settings (example values, adjust as needed)
        self.mtx = np.array([[444.88364382, 0, 316.76067727], [0, 447.22855927, 236.99517483], [0, 0, 1]])  # Camera matrix (example)
        self.dist = np.array([0.02181464, -0.02377532, -0.00200427, 0.00402658, -0.04997791])  # Distortion coefficients (example)
        # ArUco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.focal_length = (self.mtx[0, 0] + self.mtx[1, 1]) / 2  # Average focal length in pixels
        self.marker_length = 100  # Marker size in mm (example)

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            self.input_topic_name,
            self.listener_callback,
            10) # History depth
        self.publisher_ = self.create_publisher(String, self.output_topic_name, 10)
        
        self.get_logger().info(f'Monitoring metadata on {self.input_topic_name}...')

    def get_2d_angle(corners_array):
        # corner_array is the array of its 4 pixel points
        pts = corners_array
        
        # Calculate the vector from Top-Left (0) to Top-Right (1)
        dx = pts[1][0] - pts[0][0]
        dy = pts[1][1] - pts[0][1]
        
        # Calculate angle in radians and convert to degrees
        # dy is usually inverted in image coords (y increases downward)
        angle_rad = np.arctan2(dy, dx)
        angle_deg = np.degrees(angle_rad)
        
        return angle_deg
        

    def listener_callback(self, camera_msg):
        # Extract and print metadata fields
        # # Note: 'seq' is no longer a part of the Header in ROS 2.
        # self.get_logger().info('--- New Frame Metadata ---')
        # self.get_logger().info(f"Frame ID: {camera_msg.header.frame_id}")
        # self.get_logger().info(f"Timestamp: {camera_msg.header.stamp.sec}.{camera_msg.header.stamp.nanosec}")
        # self.get_logger().info(f"Resolution: {camera_msg.width}x{camera_msg.height}")
        # self.get_logger().info(f"Encoding: {camera_msg.encoding}")
        # self.get_logger().info(f"Step (row length in bytes): {camera_msg.step}")
        # self.get_logger().info(f"Is Bigendian: {bool(camera_msg.is_bigendian)}")
        # self.get_logger().info('--------------------------')
        received_image = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='bgr8')
        # Undistort the image using the camera matrix and distortion coefficients
        h, w = received_image.shape[:2]
        new_mtx = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))[0]
        undistorted_image = cv2.undistort(received_image, self.mtx, self.dist, None, new_mtx)

        gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

        corners_list, ids, _ = self.detector.detectMarkers(gray)

        detections = []

        if ids is not None:
            h, w = gray.shape[:2]
            cx_img, cy_img = w / 2.0, h / 2.0

            for i, corners in enumerate(corners_list):
                c = corners[0]
                angles = self.get_2d_angle(c)
                center = c.mean(axis=0)
                cx, cy = center

                dx = (cx - cx_img) / cx_img
                dy = (cy - cy_img) / cy_img

                # Estimate distance based on marker size (assuming a known marker size and camera parameters)
                # This is a very rough estimation and should be calibrated for real applications
                distance = (self.marker_length * self.focal_length) / (corners[0][1][0] - corners[0][0][0])  # Using width of the marker in pixels

                detections.append({
                    "id": int(ids[i][0]),
                    "center": [float(cx), float(cy)],
                    "offset_norm": [float(dx), float(dy)],
                    "distance": float(distance),
                    "angle": float(angles)
                })

        if detections:
            annotated_image = aruco.drawDetectedMarkers(undistorted_image.copy(), corners_list, ids)
        else:
            annotated_image = undistorted_image

        # Publish detections as JSON string
        aruco_msg = String()
        aruco_msg.data = json.dumps(detections)
        self.publisher_.publish(aruco_msg)
        self.get_logger().debug("Published ArUco detections")

        print(json.dumps(detections, indent=2))
        # Open a window to display the image (optional)
        cv2.imshow('Camera Frame', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms    

    def destroy_node(self):
        """Clean up resources before shutting down."""
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    try:
        rclpy.spin(aruco_node)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_node.destroy_node()
        rclpy.shutdown()