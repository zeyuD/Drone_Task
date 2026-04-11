
#!/usr/bin/env python3

# Import the subprocess and time modules
import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
import numpy as np
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('Object_Detection_Node')
        
        self.bridge = CvBridge()
        self.input_topic_name = '/image_raw'
        self.output_topic_name = '/object_detections'
        self.confidence_threshold = 0.4

        # Camera Calibration settings (example values, adjust as needed)
        self.mtx = np.array([[444.88364382, 0, 316.76067727], [0, 447.22855927, 236.99517483], [0, 0, 1]])  # Camera matrix (example)
        self.dist = np.array([0.02181464, -0.02377532, -0.00200427, 0.00402658, -0.04997791])  # Distortion coefficients (example)
        self.focal_length = (self.mtx[0, 0] + self.mtx[1, 1]) / 2  # Average focal length in pixels
        # YOLO model
        self.model = YOLO("yolo26m.pt")
        # self.model = YOLO("yolov8n.pt")

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            self.input_topic_name,
            self.listener_callback,
            10) # History depth
        self.publisher_ = self.create_publisher(String, self.output_topic_name, 10)
        
        self.get_logger().info(f'Monitoring metadata on {self.input_topic_name}...')

    def listener_callback(self, camera_msg):
        received_image = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='bgr8')
        # Undistort the image using the camera matrix and distortion coefficients
        h, w = received_image.shape[:2]
        new_mtx = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))[0]
        undistorted_image = cv2.undistort(received_image, self.mtx, self.dist, None, new_mtx)

        results = self.model(undistorted_image, conf=self.confidence_threshold)
        annotated_image = results[0].plot()

        detections = []

        # Publish detections as JSON string
        obj_msg = String()
        obj_msg.data = json.dumps(detections)
        self.publisher_.publish(obj_msg)
        self.get_logger().debug("Published Object detections")

        print(json.dumps(detections, indent=2))
        # Open a window to display the image (optional)
        cv2.imshow('Camera Frame', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms    

    def destroy_node(self):
        """Clean up resources before shutting down."""
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        object_detection_node.destroy_node()
        rclpy.shutdown()