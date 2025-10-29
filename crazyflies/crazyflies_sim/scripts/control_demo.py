#!/usr/bin/env python3
"""
Simple Crazyflie Flight Demo for Students

This script shows how to:
1. Take off to 1 meter height
2. Fly forward at constant speed for 4 seconds
3. Land

The yellow arrow shows the velocity direction!
"""

import rclpy
from rclpy.node import Node

import numpy as np

# ROS2 message imports
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

# Crazyflie service and message imports
from crazyflie_interfaces.srv import Takeoff, Land
from crazyflie_interfaces.msg import Hover




class SimpleFlightDemoNode(Node):
    def __init__(self):
        super().__init__('simple_flight_demo')
        
        # Simple configuration - no parameters needed
        self.takeoff_height = 1.0  # meters
        self.cf_name = 'cf_1'     # crazyflie name
        
        self.get_logger().info(f"Crazyflie: {self.cf_name}")
        self.get_logger().info(f"Takeoff height: {self.takeoff_height}m")
        
        # Subscriber for pose (simplified QoS)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.cf_name}/pose',
            self.pose_callback,
            10
        )
        
        # Publishers
        self.velocity_arrow_pub = self.create_publisher(
            Marker,
            '/velocity_arrow',
            10
        )
        
        self.hover_cmd_pub = self.create_publisher(
            Hover,
            f'/{self.cf_name}/cmd_hover',
            10
        )
        
        # Service clients
        self.takeoff_client = self.create_client(Takeoff, f'/{self.cf_name}/takeoff')
        self.land_client = self.create_client(Land, f'/{self.cf_name}/land')
        
        # State variables
        self.current_pose = None
        self.is_airborne = False
        self.flight_phase = 'waiting'  # waiting, takeoff, forward, landing, done
        self.flight_start_time = None
        self.phase_start_time = None
        
        # Flight parameters - you can change these!
        self.forward_velocity = 0.3  # Constant forward speed in m/s
        self.forward_duration = 10  # How long to move forward (seconds)
        
        # Control timer - this runs our main control logic
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # Start the flight sequence
        self.get_logger().info("🚁 Starting flight demo sequence...")
        self.start_flight_sequence()

    def control_loop(self):
        """Main control loop - this runs 10 times per second!
        
        This is where we decide what the drone should do:
        - Forward phase: fly forward at constant speed for 20 seconds
        - Then land!
        """
        # Don't do anything if we're still taking off or done
        if self.flight_phase == 'waiting' or self.flight_phase == 'takeoff':
            return
            
        if self.flight_phase == 'done' or self.flight_phase == 'landing':
            return
            
        if not self.is_airborne or self.current_pose is None:
            return
            
        # How long have we been in this phase?
        current_time = self.get_clock().now()
        phase_elapsed = (current_time - self.phase_start_time).nanoseconds / 1e9
        
        # FORWARD MOVEMENT PHASE
        if self.flight_phase == 'forward':
            if phase_elapsed < self.forward_duration:
                # Move forward at constant velocity!
                self.publish_velocity_command(self.forward_velocity, 0.0, 0.0)
                self.publish_velocity_arrow(self.forward_velocity, 0.0, 0.0)
                
                # Print status every second
                if phase_elapsed % 1.0 < 0.1:
                    self.get_logger().info(f"➡️  Forward: speed = {self.forward_velocity:.2f} m/s, time = {phase_elapsed:.1f}s/{self.forward_duration:.1f}s")
            else:
                # Time's up! Time to land
                self.get_logger().info("🛬 Demo complete! Landing now...")
                self.flight_phase = 'landing'
                # self.publish_velocity_command(0.0, 0.0, 0.0)  # Stop moving
                # self.publish_empty_arrow()  # Remove arrow
                self.call_land()  # Start landing

    def start_flight_sequence(self):
        """Start the flight sequence - wait for services and takeoff."""
        self.get_logger().info("⏳ Waiting for takeoff service...")
        
        # Simple waiting loop
        while not self.takeoff_client.service_is_ready():
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().info("✅ Takeoff service ready. Starting takeoff...")
        self.call_takeoff()

    # Remove the check_services function - we don't need it anymore!

    def call_takeoff(self):
        """Call the takeoff service - simple async call."""
        try:
            request = Takeoff.Request()
            request.height = self.takeoff_height
            request.duration = Duration(sec=6, nanosec=0)

            # Send the takeoff command (don't wait for it to complete)
            future = self.takeoff_client.call_async(request)
            
            self.flight_phase = 'takeoff'
            self.flight_start_time = self.get_clock().now()
            # The pose_callback will detect when takeoff is actually complete
            self.get_logger().info("🚀 Takeoff command sent! Waiting for drone to reach altitude...")
            
        except Exception as e:
            self.get_logger().error(f"Failed to call takeoff service: {e}")

    def call_land(self):
        """Call the land service - simple async call."""
        try:
            request = Land.Request()
            request.height = 0.0
            request.duration = Duration(sec=4, nanosec=0)

            # Send the landing command (don't wait for it to complete)
            future = self.land_client.call_async(request)
            
            self.flight_phase = 'done'
            self.get_logger().info("🛬 Landing command sent!")
            
        except Exception as e:
            self.get_logger().error(f"Failed to call land service: {e}")

    def pose_callback(self, msg):
        """Process Crazyflie pose."""
        self.current_pose = msg
        
        # Check if we're airborne during takeoff phase
        if self.flight_phase == 'takeoff' and not self.is_airborne:
            if msg.pose.position.z >= self.takeoff_height * 0.9:
                self.is_airborne = True
                self.get_logger().info(f"🎯 Takeoff complete! Starting forward movement...")
                self.flight_phase = 'forward'
                self.phase_start_time = self.get_clock().now()

    def publish_velocity_command(self, vx, vy, vz):
        """Send velocity commands to the drone.
        
        vx = forward/backward speed (+ = forward, - = backward)
        vy = left/right speed (+ = right, - = left) 
        vz = up/down speed (+ = up, - = down)
        """
        hover_msg = Hover()
        hover_msg.header.stamp = self.get_clock().now().to_msg()
        hover_msg.header.frame_id = self.cf_name
        
        # Set the velocities
        hover_msg.vx = vx  # Forward/backward
        hover_msg.vy = vy  # Left/right
        hover_msg.z_distance = self.takeoff_height  # Stay at takeoff height
        hover_msg.yaw_rate = 0.0  # Don't rotate
        
        # Send the command!
        self.hover_cmd_pub.publish(hover_msg)

    def publish_velocity_arrow(self, vx, vy, vz):
        """Create the yellow arrow that shows velocity direction and magnitude!
        
        This makes a cool visualization arrow that:
        - Points in the direction the drone is moving
        - Gets longer when speed is higher
        """
        marker = Marker()
        marker.header.frame_id = self.cf_name  # Use drone frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "velocity_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow starts at origin (0,0,0) in drone frame
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Calculate arrow length based on velocity magnitude
        velocity_magnitude = np.sqrt(vx**2 + vy**2 + vz**2)
        
        if velocity_magnitude > 0.01:
            # Scale arrow length with velocity magnitude
            base_length = 0.05  # Minimum arrow length for visibility
            scale_factor = velocity_magnitude * 2.0  # Scale factor
            arrow_length = base_length + min(scale_factor, 1.0)  # Max 1.0m total length
            
            # Normalize velocity direction
            vx_norm = vx / velocity_magnitude
            vy_norm = vy / velocity_magnitude
            vz_norm = vz / velocity_magnitude
            
            # Use points to define arrow direction (start -> end)
            start_point = Point()
            start_point.x = 0.0
            start_point.y = 0.0
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = vx_norm * arrow_length
            end_point.y = vy_norm * arrow_length
            end_point.z = vz_norm * arrow_length
            
            marker.points = [start_point, end_point]
            
            # Arrow styling
            marker.scale.x = 0.035  # Arrow shaft diameter
            marker.scale.y = 0.065  # Arrow head diameter
            marker.scale.z = 0.0    # Not used for ARROW with points
            
            # Color: Bright yellow arrow for constant velocity
            marker.color.r = 1.0
            marker.color.g = 1.0  # Yellow
            marker.color.b = 0.0
            marker.color.a = 0.9
            
            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            
            self.velocity_arrow_pub.publish(marker)
        else:
            self.publish_empty_arrow()

    def publish_empty_arrow(self):
        """Clear the velocity arrow visualization."""
        marker = Marker()
        marker.header.frame_id = self.cf_name
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "velocity_arrow"
        marker.id = 0
        marker.action = Marker.DELETE
        
        self.velocity_arrow_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleFlightDemoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()