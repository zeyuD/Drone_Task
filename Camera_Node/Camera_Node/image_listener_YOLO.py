import rclpy
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageMetadataSubscriber(Node):
    def __init__(self):
        super().__init__('image_metadata_subscriber')
        self.bridge = CvBridge()
        self.topic_name = '/image_raw'
        self.confidence_threshold = 0.4
        # Load YOLO model
        self.model = YOLO("yolo26m.pt")
        # self.model = YOLO("yolov8n.pt")
        # Camera Calibration settings (example values, adjust as needed)
        self.mtx = np.array([[444.88364382, 0, 316.76067727], [0, 447.22855927, 236.99517483], [0, 0, 1]])  # Camera matrix (example)
        self.dist = np.array([0.02181464, -0.02377532, -0.00200427, 0.00402658, -0.04997791])  # Distortion coefficients (example)
        
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
        # Undistort the image using the camera matrix and distortion coefficients
        h, w = received_image.shape[:2]
        new_mtx = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))[0]
        undistorted_image = cv2.undistort(received_image, self.mtx, self.dist, None, new_mtx)

        results = self.model(undistorted_image, conf=self.confidence_threshold)
        annotated_image = results[0].plot()
        # Open a window to display the image (optional)
        cv2.imshow('Camera Frame', annotated_image)
        cv2.waitKey(1)  # Display the image for 1 ms    

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
