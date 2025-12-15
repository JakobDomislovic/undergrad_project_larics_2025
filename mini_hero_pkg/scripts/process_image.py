#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import tf_transformations

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



        # Add ArUco setup
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        #self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        self.marker_size = 0.25



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


        #Publisher for aruco location 
        self.aruco_poses_pub = self.create_publisher(
            PoseStamped,
            "/aruco_marker_pose",
            10
        )

        self.aruco_marker = self.create_publisher(
            Marker,
            "/aruco_marker",
            10
        )
        self.aruco_arrow = self.create_publisher(
            Marker,
            "/arruco_arrow",
            10
        )
        # Timer for periodic processing with a rate of 10Hz (0.1s delay)
        self.create_timer(0.1, self._run_loop)

        
    def _run_loop(self):
        """Main loop to process images."""

        # Image and Image information is grabbed and stored in callbacks.
        # See camera_info_callback and image_callback


            
        # Check if image and camera info is received
        if self.image is not None and self.camera_matrix is not None:
            

            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dictionary,
                parameters=self.aruco_parameters
            )

            if ids is None:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "invalid"
                self.aruco_poses_pub.publish(pose_stamped)
                return


            
            cv2.aruco.drawDetectedMarkers(self.image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs,
            )

            #self.get_logger().info(f"tvecs: {tvecs},\nrvecs: {rvecs}")
            processed_image = self.image

            for idx, corner in enumerate(corners):
                pose_marker = Marker()
                pose_marker.header.stamp = self.get_clock().now().to_msg()
                pose_marker.header.frame_id = "cf_1"

                pose_marker.type = Marker.ARROW

                start_point = Point()
                start_point.x = 0.0
                start_point.y = 0.0
                start_point.z = 0.0

                #rotating the original position:
                # x = z
                # y = -x
                # z = -y
                T = np.array([
                    [0, 0, 1],
                    [-1, 0, 0],
                    [0, -1, 0]
                ])
                x, y, z = T @ tvecs[idx][0]
                end_point = Point()
                end_point.x = x
                end_point.y = y
                end_point.z = z

                pose_marker.points.append(start_point)
                pose_marker.points.append(end_point)

                pose_marker.scale.x = 0.02
                pose_marker.scale.y = 0.04
                pose_marker.scale.z = 0.1

                pose_marker.color.r = 1.0
                pose_marker.color.a = 1.0


                R, _ = cv2.Rodrigues(rvecs[idx])

                R_ros = T @ R
                quaternion = tf_transformations.quaternion_from_matrix(
                    np.vstack((np.hstack((R_ros, np.array([[0], [0], [0]]))), [0, 0, 0, 1]))
                )
                pose_stamped = PoseStamped()
                #first, second, yaw = tf_transformations.euler_from_quaternion(quaternion)

                #self.get_logger().info(f"first: {first}\nsecond: {second}\nyaw: {yaw}")
                pose_stamped.pose.position = end_point
                pose_stamped.pose.orientation.x = float(quaternion[0])
                pose_stamped.pose.orientation.y = float(quaternion[1])
                pose_stamped.pose.orientation.z = float(quaternion[2])
                pose_stamped.pose.orientation.w = float(quaternion[3])

                pose_stamped.header.frame_id = "cf_1"
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
               # self.get_logger().info(f"position: {pose_stamped}")
                self.aruco_poses_pub.publish(pose_stamped)
                
                self.aruco_arrow.publish(pose_marker)


                marker = Marker()
                marker.header.frame_id = "cf_1"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                marker.pose = pose_stamped.pose
                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = 0.002

                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.a = 0.8
                self.aruco_marker.publish(marker)



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
