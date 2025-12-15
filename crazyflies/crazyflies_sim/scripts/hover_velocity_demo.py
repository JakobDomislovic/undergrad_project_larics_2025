#!/usr/bin/env python3
"""
Simple Hover Velocity Demo

This script demonstrates:
1. Takeoff and hover
2. Move around with varying velocities
3. Stop velocity commands but keep hovering
4. Land
"""

import rclpy
from rclpy.node import Node
import time
import math
from builtin_interfaces.msg import Duration
from crazyflie_interfaces.srv import Takeoff, Land
from crazyflie_interfaces.msg import Hover
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import tf_transformations


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi

    return angle

class HoverVelocityDemoNode(Node):
    def __init__(self):
        super().__init__('hover_velocity_demo')
        
        self.cf_name = 'cf_1'
        self.hover_height = 1.0
        self.takeoff = False
        
        self.get_logger().info(f"🚁 Hover Velocity Demo - Crazyflie: {self.cf_name}")
        
        # Publishers
        self.hover_pub = self.create_publisher(
            Hover,
            f'/{self.cf_name}/cmd_hover',
            10
        )
        
        # Service clients
        self.takeoff_client = self.create_client(Takeoff, f'/{self.cf_name}/takeoff')
        self.land_client = self.create_client(Land, f'/{self.cf_name}/land')
        
        # Wait for services
        while not self.takeoff_client.service_is_ready() or not self.land_client.service_is_ready():
            self.get_logger().info("⏳ Waiting for services...")
            time.sleep(0.5)
        
        self.aruco_poses = self.create_subscription(
            PoseStamped,
            "/aruco_marker_pose",
            self.move_towards_aruco,
            1
        )

        self.get_logger().info("✅ Services ready! Starting demo...")
        #self.run_demo()
        #self.run_spiral()

    def publish_hover_command(self, vx, vy, yaw_rate):
        """Publish hover command with given velocities."""
        msg = Hover()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cf_name
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.yaw_rate = float(yaw_rate)
        msg.z_distance = float(self.hover_height)
        
        self.hover_pub.publish(msg)

    def publish_rising_command(self, vx, vy, yaw_rate, z_distance):
        """Publish hover command with given velocities."""
        msg = Hover()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cf_name
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.yaw_rate = float(yaw_rate)
        msg.z_distance = float(z_distance)
        
        self.hover_pub.publish(msg)

    def run_demo(self):
        """Run the complete demo sequence."""
        try:
            # Takeoff
            self.get_logger().info("🚀 Taking off...")
            request = Takeoff.Request()
            request.height = self.hover_height
            request.duration = Duration(sec=3, nanosec=0)
            self.takeoff_client.call_async(request)
            time.sleep(4)  # Wait for takeoff
            
            # Publish hovering commands continuously
            self.get_logger().info("📍 Starting hover with velocity commands...")
            
            # Phase 1: Move forward
            self.get_logger().info("➡️  Moving forward (vx=0.5 m/s)...")
            for i in range(20):  # 2 seconds at 10 Hz
                self.publish_hover_command(0.5, 0.0, 0.0)
                time.sleep(0.1)
            
            # Phase 2: Move right
            self.get_logger().info("↗️  Moving right (vy=0.5 m/s)...")
            for i in range(20):  # 2 seconds at 10 Hz
                self.publish_hover_command(0.0, 0.5, 0.0)
                time.sleep(0.1)
            
            # Phase 3: Move backward
            self.get_logger().info("⬅️  Moving backward (vx=-0.5 m/s)...")
            for i in range(20):  # 2 seconds at 10 Hz
                self.publish_hover_command(-0.5, 0.0, 0.0)
                time.sleep(0.1)
            
            # Phase 4: Move left
            self.get_logger().info("↙️  Moving left (vy=-0.5 m/s)...")
            for i in range(20):  # 2 seconds at 10 Hz
                self.publish_hover_command(0.0, -0.5, 0.0)
                time.sleep(0.1)
            
            # Phase 5: ZERO velocities but keep hovering!
            self.get_logger().info("⏸️  Zeroing velocities but keeping hover active...")
            for i in range(30):  # 3 seconds at 10 Hz with zero velocity
                self.publish_hover_command(0.0, 0.0, 0.0)
                time.sleep(0.1)
            
            # Land
            self.get_logger().info("🛬 Landing...")
            request = Land.Request()
            request.height = 0.0
            request.duration = Duration(sec=3, nanosec=0)
            self.land_client.call_async(request)
            time.sleep(4)
            
            self.get_logger().info("✅ Demo complete!")
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            rclpy.shutdown()


    def run_spiral(self):
        """Moving and spinning the drone in a spiral"""
        try:
            z = 1.0
            # Takeoff
            self.get_logger().info("🚀 Taking off...")
            request = Takeoff.Request()
            request.height = self.hover_height
            request.duration = Duration(sec=3, nanosec=0)
            self.takeoff_client.call_async(request)
            time.sleep(4)  # Wait for takeoff
            
            # Publish hovering commands continuously
            self.get_logger().info("📍 Starting hover with velocity commands...")
            
            # Phase 1: Move forward
            self.get_logger().info("➡️  Moving forward (vx=0.5 m/s)...")
            for i in range(110):  # 2 seconds at 10 Hz
                z += 0.05
                self.publish_rising_command(0.5, 0.0, -1.0, z)
                time.sleep(0.1)
            
            # Phase 5: ZERO velocities but keep hovering!
            self.get_logger().info("⏸️  Zeroing velocities but keeping hover active...")
            for i in range(30):  # 3 seconds at 10 Hz with zero velocity
                self.publish_rising_command(0.0, 0.0, 0.0, z)
                time.sleep(0.1)
            
            # Land
            self.get_logger().info("🛬 Landing...")
            request = Land.Request()
            request.height = 0.0
            request.duration = Duration(sec=3, nanosec=0)
            self.land_client.call_async(request)
            time.sleep(4)
            
            self.get_logger().info("✅ Demo complete!")
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            rclpy.shutdown()

    def move_towards_aruco(self, pose_stamped):
        """Moving the drone towards an ar tag"""
        try:
            quaternion = [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w]
            _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)

            yaw = wrap_to_pi(yaw)
            #self.get_logger().info(f"yaw: {yaw}")
            vx = 0.0
            vy = 0.0
            rotate = 0.0

            if self.takeoff == False:
            # Takeoff
                self.get_logger().info("🚀 Taking off...")
                request = Takeoff.Request()
                request.height = self.hover_height
                request.duration = Duration(sec=3, nanosec=0)
                self.takeoff_client.call_async(request)
                time.sleep(4)  # Wait for takeoff
                
                # Publish hovering commands continuously
                self.get_logger().info("📍 Starting hover with velocity commands...")
                self.takeoff = True
            
            if pose_stamped.header.frame_id == "invalid":
                self.publish_rising_command(0.0, 0.0, 0.2, self.hover_height)
                #time.sleep(0.1)
                return

            if pose_stamped.pose.position.x < 1.35 or pose_stamped.pose.position.x > 1.65:
            # Phase 1: Move forward
                vx = 0.0
                if pose_stamped.pose.position.x < 1.35:
                    vx = -0.2
                else:
                    vx = 0.2
                #self.publish_rising_command(vx, 0.0, 0.0, self.hover_height)
                #time.sleep(0.1)

            if pose_stamped.pose.position.y < -0.07 or pose_stamped.pose.position.y > 0.07:
            # Phase 1: Move forward
                vy = 0.0
                if pose_stamped.pose.position.y < -0.07:
                    vy = -0.15
                else:
                    vy = 0.15
                #self.publish_rising_command(0.0, vy, 0.0, self.hover_height)
                #time.sleep(0.1)

            if pose_stamped.pose.position.z < -0.07 or pose_stamped.pose.position.z > 0.07:
            # Phase 1: Move forward
                if pose_stamped.pose.position.z < -0.07:
                    self.hover_height -= 0.01
                else:
                    self.hover_height += 0.01

                #self.publish_rising_command(0.0, 0.0, 0.0, self.hover_height)
                #time.sleep(0.1)

            if yaw < -1.75 or yaw > -1.35:
                if yaw < -1.75:
                    rotate = -0.1
                else:
                    rotate = 0.1

                #self.publish_rising_command(0.0, 0.0, rotate, self.hover_height)
                #time.sleep(0.1)


                    
            self.publish_rising_command(vx, vy, rotate, self.hover_height)

            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HoverVelocityDemoNode()
    rclpy.spin(node)
    rclpy.shutdown()
    # Node runs demo in __init__ and shuts down
    

if __name__ == '__main__':
    main()
