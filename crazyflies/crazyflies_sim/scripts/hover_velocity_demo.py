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
import json
from std_msgs.msg import String

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < -math.pi:
        angle += 2*math.pi

    return angle

class PID:
    def __init__(self, kp, ki, kd, deadzone=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadzone = deadzone

        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def compute(self, error, dt):

        if abs(error) < self.deadzone:
            self.integral = 0.0
            derivative = 0.0
        else:
        
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt if dt > 0 or  self.prev_error is not None else 0.0

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        return output

    

class HoverVelocityDemoNode(Node):
    def __init__(self):
        super().__init__('hover_velocity_demo')
        
        self.cf_name = 'cf_1'
        self.hover_height = 0.3
        self.takeoff = False
        
        self.get_logger().info(f"🚁 Hover Velocity Demo - Crazyflie: {self.cf_name}")

        # PID controllers
        
        # PID XY
        self.declare_parameter('kp_xy', 0.32)
        self.declare_parameter('ki_xy', 0.01)
        self.declare_parameter('kd_xy', 0.05)

        # PID Z
        self.declare_parameter('kp_z', 0.32)
        self.declare_parameter('ki_z', 0.01)
        self.declare_parameter('kd_z', 0.05)

        # PID YAW
        self.declare_parameter('kp_yaw', 0.15)
        self.declare_parameter('ki_yaw', 0.01)
        self.declare_parameter('kd_yaw', 0.05)

        # XY params
        self.kp_xy = self.get_parameter('kp_xy').value
        self.ki_xy = self.get_parameter('ki_xy').value
        self.kd_xy = self.get_parameter('kd_xy').value

        # Z params
        self.kp_z = self.get_parameter('kp_z').value
        self.ki_z = self.get_parameter('ki_z').value
        self.kd_z = self.get_parameter('kd_z').value

        # Yaw params
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.ki_yaw = self.get_parameter('ki_yaw').value
        self.kd_yaw = self.get_parameter('kd_yaw').value
        

        self.last_timestamp = self.get_clock().now()

        self.x_const = 0.8
        self.y_const = 0.0
        self.z_const = 0.0
        self.yaw_const = -1.57
        self.restart_measurements = True


        self.pid_x = PID(self.kp_xy, self.ki_xy, self.kd_xy)
        self.pid_y = PID(self.kp_xy, self.ki_xy, self.kd_xy)
        self.pid_z = PID(self.kp_z, self.ki_z, self.kd_z)
        self.pid_yaw = PID(self.kp_yaw, self.ki_yaw, self.kd_yaw)
        
        
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
        self.create_subscription(String, "/pid_tuning", self.update_pid, 1)

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
        try:


            #  # XY params
            # self.kp_xy = self.get_parameter('kp_xy').value
            # self.ki_xy = self.get_parameter('ki_xy').value
            # self.kd_xy = self.get_parameter('kd_xy').value

            # # Z params
            # self.kp_z = self.get_parameter('kp_z').value
            # self.ki_z = self.get_parameter('ki_z').value
            # self.kd_z = self.get_parameter('kd_z').value

            # # Yaw params
            # self.kp_yaw = self.get_parameter('kp_yaw').value
            # self.ki_yaw = self.get_parameter('ki_yaw').value
            # self.kd_yaw = self.get_parameter('kd_yaw').value

            # Update XY
            self.pid_x.kp = self.kp_xy
            self.pid_x.ki = self.ki_xy
            self.pid_x.kd = self.kd_xy

            self.pid_y.kp = self.kp_xy
            self.pid_y.ki = self.ki_xy
            self.pid_y.kd = self.kd_xy

            # Update Z
            self.pid_z.kp = self.kp_z
            self.pid_z.ki = self.ki_z
            self.pid_z.kd = self.kd_z

            # Update Yaw
            self.pid_yaw.kp = self.kp_yaw
            self.pid_yaw.ki = self.ki_yaw
            self.pid_yaw.kd = self.kd_yaw

            quaternion = [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w]
            _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
            yaw = wrap_to_pi(yaw)

            vx = 0.0
            vy = 0.0
            yaw_rate = 0.0

            error_x = pose_stamped.pose.position.x - self.x_const
            error_y = pose_stamped.pose.position.y - self.y_const
            error_z = pose_stamped.pose.position.z - self.z_const
            error_yaw = wrap_to_pi(yaw - self.yaw_const)



            current_time = rclpy.time.Time.from_msg(pose_stamped.header.stamp)
            dt = (current_time - self.last_timestamp).nanoseconds * 1e-9
            self.last_timestamp = current_time
            
            if self.restart_measurements:
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
                self.pid_yaw.reset()
                self.restart_measurements = False
                dt = 0.0
                


            

            # --- TAKEOFF ---
            if not self.takeoff:
                self.get_logger().info("🚀 Taking off...")
                req = Takeoff.Request()
                req.height = self.hover_height
                req.duration = Duration(sec=3, nanosec=0)
                self.takeoff_client.call_async(req)
                time.sleep(4)
                self.takeoff = True
                self.restart_measurements = True
                #self.last_time = self.get_clock().now()
                
            
         
            if pose_stamped.header.frame_id == "invalid":
               
                self.publish_rising_command(0.0, 0.0, 0.3, self.hover_height)
                self.restart_measurements = True
                return



            vx = self.pid_x.compute(error_x, dt)
            vy = self.pid_y.compute(error_y, dt)
            self.hover_height += self.pid_z.compute(error_z, dt)
            yaw_rate = self.pid_yaw.compute(error_yaw, dt)

            


            self.publish_rising_command(vx, vy, yaw_rate, self.hover_height)
            
        except Exception as e:
            self.get_logger().error(f"PID error: {e}")

    
    def update_pid(self, msg):
        data = json.loads(msg.data)

        # Update numeric values
        self.kp_xy = data["kp_xy"]
        self.ki_xy = data["ki_xy"]
        self.kd_xy = data["kd_xy"]

        self.kp_z  = data["kp_z"]
        self.ki_z  = data["ki_z"]
        self.kd_z  = data["kd_z"]

        self.kp_yaw = data["kp_yaw"]
        self.ki_yaw = data["ki_yaw"]
        self.kd_yaw = data["kd_yaw"]

        # Apply directly to PID controllers
        self.pid_x.kp = self.kp_xy
        self.pid_x.ki = self.ki_xy
        self.pid_x.kd = self.kd_xy

        self.pid_y.kp = self.kp_xy
        self.pid_y.ki = self.ki_xy
        self.pid_y.kd = self.kd_xy

        self.pid_z.kp = self.kp_z
        self.pid_z.ki = self.ki_z
        self.pid_z.kd = self.kd_z

        self.pid_yaw.kp = self.kp_yaw
        self.pid_yaw.ki = self.ki_yaw
        self.pid_yaw.kd = self.kd_yaw

        self.get_logger().info(f"""  PID updated:
        XY -> Kp={self.kp_xy}, Ki={self.ki_xy}, Kd={self.kd_xy}
        Z  -> Kp={self.kp_z}, Ki={self.ki_z}, Kd={self.kd_z}
        Yaw-> Kp={self.kp_yaw}, Ki={self.ki_yaw}, Kd={self.kd_yaw}
        """)





def main(args=None):
    rclpy.init(args=args)
    node = HoverVelocityDemoNode()
    rclpy.spin(node)
    rclpy.shutdown()
    # Node runs demo in __init__ and shuts down
    

if __name__ == '__main__':
    main()
