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
from builtin_interfaces.msg import Duration
from crazyflie_interfaces.srv import Takeoff, Land
from crazyflie_interfaces.msg import Hover


class HoverVelocityDemoNode(Node):
    def __init__(self):
        super().__init__('hover_velocity_demo')
        
        self.cf_name = 'cf_1'
        self.hover_height = 1.0
        
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
        
        self.get_logger().info("✅ Services ready! Starting demo...")
        self.run_demo()

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


def main(args=None):
    rclpy.init(args=args)
    node = HoverVelocityDemoNode()
    # Node runs demo in __init__ and shuts down
    

if __name__ == '__main__':
    main()
