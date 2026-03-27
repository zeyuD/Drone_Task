
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    """ROS2 Node for broadcasting camera frames to a topic."""

    def __init__(self):
        super().__init__('Camera_Node')
        
        # Create publisher for camera frames
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        
        # Initialize CV Bridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()
        
        # Open the default camera (0 = built-in camera, adjust if needed)
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera. Check if camera is available.')
            return
        
        # Set camera resolution (optional, adjust as needed)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Create timer to capture and publish frames at 30 Hz
        timer_period = 1.0 / 30.0  # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera Node initialized and broadcasting on /image_raw')

    def timer_callback(self):
        """Capture frame from camera and publish it."""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera')
            return
        
        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # Publish the image
        self.publisher_.publish(ros_image)
        self.get_logger().debug("Published frame")

    def destroy_node(self):
        """Clean up resources before shutting down."""
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()