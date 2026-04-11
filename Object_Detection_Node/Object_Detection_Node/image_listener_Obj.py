import rclpy
import cv2
import cv2.aruco as aruco
import numpy as np
import json
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# from packaging import version
# if version.parse(cv2.__version__) >= version.parse("4.7.0"):
#     dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
#     detectorParams = cv2.aruco.DetectorParameters()
#     detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
#     marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(image)
# else:
#     dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
#     detectorParams = cv2.aruco.DetectorParameters_create()
#     marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(
#         image, dictionary, parameters=detectorParams
#     )

class ImageMetadataSubscriber(Node):
    def __init__(self):
        super().__init__('image_metadata_subscriber')
        self.bridge = CvBridge()
        self.topic_name = '/image_raw'

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

        gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

        corners_list, ids, _ = self.detector.detectMarkers(gray)

        detections = []

        if ids is not None:
            h, w = gray.shape[:2]
            cx_img, cy_img = w / 2.0, h / 2.0

            for i, corners in enumerate(corners_list):
                c = corners[0]
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
                    "distance": float(distance)
                })

        if detections:
            annotated_image = aruco.drawDetectedMarkers(undistorted_image.copy(), corners_list, ids)
        else:
            annotated_image = undistorted_image
        print(json.dumps(detections, indent=2))
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
