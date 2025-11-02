#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

# ROS2 message imports
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge

# Crazyflie service and message imports
from crazyflie_interfaces.srv import Takeoff
from crazyflie_interfaces.msg import Hover


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')

        # State variables
        self.image = None
        self.dist_coeffs = None
        self.camera_matrix = None
        self.bridge = CvBridge()  # For converting between ROS and OpenCV images

        # Image subscriber        
        self.image_sub = self.create_subscription(
            Image,                      # Topic type
            '/cf_1/image',              # Topic name
            self.image_callback,        # Callback function called on new image
            10                          # Queue size
        )

        # Camera info subscriber                
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/cf_1/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher for processed image
        self.processed_image_pub = self.create_publisher(
            Image,
            '/processed_image',  # Topic name for processed image
            10
        )

        # Timer for periodic processing with a rate of 10Hz (0.1s delay)
        self.create_timer(0.1, self._run_loop)
        
    def _run_loop(self):
        """Main loop to process images."""

        # Image and Image information is grabbed and stored in callbacks.
        # See camera_info_callback and image_callback

        # Check if image and camera info is received
        if self.image is not None or self.camera_matrix is not None:
            
            #__________ TODO: Process the image here
            # You have access to self.image, self.camera_matrix, and self.dist_coeffs
            # Process them according to given tasks and publish a new image to a topic /new_image.

            # Replace the following line with actual image processing you want to perform
            processed_image = self.image

            # Publish the processed image on /processed_image topic
            self.publish_processed_image(processed_image)


    #_________________________  Message Callbacks  _________________________

    def camera_info_callback(self, msg):
        """Process camera calibration info."""

        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f"Camera matrix:\n{self.camera_matrix}")


    def image_callback(self, msg):
        """Process camera image and detect ArUco markers."""
        
        # Convert ROS image to OpenCV
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        

    #_________________________  Publishing function  _________________________

    def publish_processed_image(self, processed_image):
        """Publish the processed image to /processed_image topic."""
        try:
            # Convert OpenCV image back to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            
            # Set the timestamp
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "cf_1"
            
            # Publish the processed image
            self.processed_image_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish processed image: {e}")
        

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageProcessingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()