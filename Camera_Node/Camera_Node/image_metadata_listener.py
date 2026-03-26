import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageMetadataSubscriber(Node):
    def __init__(self):
        super().__init__('image_metadata_subscriber')
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/Camera_Node/image_raw',
            self.listener_callback,
            10) # History depth
        
        self.get_logger().info('Monitoring metadata on /Camera_Node/image_raw...')

    def listener_callback(self, msg):
        # Extract and print metadata fields
        # Note: 'seq' is no longer a part of the Header in ROS 2.
        self.get_logger().info('--- New Frame Metadata ---')
        self.get_logger().info(f"Frame ID: {msg.header.frame_id}")
        self.get_logger().info(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        self.get_logger().info(f"Resolution: {msg.width}x{msg.height}")
        self.get_logger().info(f"Encoding: {msg.encoding}")
        self.get_logger().info(f"Step (row length in bytes): {msg.step}")
        self.get_logger().info(f"Is Bigendian: {bool(msg.is_bigendian)}")
        self.get_logger().info('--------------------------')
        # Open a window to display the image (optional)
        cv2.imshow('Camera Frame', self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
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
