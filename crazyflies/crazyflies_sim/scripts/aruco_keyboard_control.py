#!/usr/bin/env python3
"""ArUco Marker Keyboard Control - ROS2 Node for moving markers in Gazebo."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import sys, termios, tty, select, math, subprocess


class ArucoKeyboardControl(Node):
    def __init__(self):
        super().__init__('aruco_keyboard_control')
        self.declare_parameter('step_size', 0.075)
        self.declare_parameter('rotation_step', 0.15)
        self.declare_parameter('world_name', 'empty')
        
        self.step_size = self.get_parameter('step_size').value
        self.rotation_step = self.get_parameter('rotation_step').value
        self.world_name = self.get_parameter('world_name').value
        self.service_name = f'/world/{self.world_name}/set_pose'
        
        self.marker_id = None
        self.marker_name = None
        self.current_pose = Pose()
        self.original_pose = Pose()
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info(f"✓ Initialized for {self.service_name}")

    def prompt_for_marker(self):
        """Prompt user to enter ArUco marker ID."""
        print("\n" + "="*60)
        print("ArUco Marker Keyboard Control")
        print("="*60)
        print("\nAvailable ArUco markers: 1, 2, 3, 4, 5")
        
        while True:
            try:
                marker_input = input("Enter ArUco marker ID to control (1-5): ").strip()
                marker_id = int(marker_input)
                
                if 1 <= marker_id <= 5:
                    self.marker_id = marker_id
                    self.marker_name = f"aruco_marker_{marker_id}"
                    self.get_logger().info(f"Controlling {self.marker_name}")
                    return True
                else:
                    print("Please enter a number between 1 and 5.")
            except ValueError:
                print("Invalid input. Please enter a number.")
            except KeyboardInterrupt:
                print("\nExiting...")
                return False

    def initialize_marker_pose(self):
        """Initialize marker pose from empty.sdf spawn positions."""
        # Set position based on marker ID (from empty.sdf)
        y_pos = {1: 1.0, 2: 0.5}.get(self.marker_id, 0.0)
        self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z = 2.0, y_pos, 0.125
        
        # Set orientation (roll=1.57, pitch=0.0, yaw=-1.57 from empty.sdf)
        qx, qy, qz, qw = self.quaternion_from_euler(1.57, 0.0, -1.57)
        self.current_pose.orientation.x, self.current_pose.orientation.y = qx, qy
        self.current_pose.orientation.z, self.current_pose.orientation.w = qz, qw
        
        # Store original pose for reset
        self.original_pose = Pose()
        self.original_pose.position.x, self.original_pose.position.y, self.original_pose.position.z = 2.0, y_pos, 0.125
        self.original_pose.orientation.x, self.original_pose.orientation.y = qx, qy
        self.original_pose.orientation.z, self.original_pose.orientation.w = qz, qw
        
        self.get_logger().info(f"Initialized {self.marker_name} at ({2.0:.2f}, {y_pos:.2f}, {0.125:.2f})")

    def set_marker_pose(self, pose):
        """Send pose command to Gazebo and update internal tracking."""
        try:
            cmd = ['gz', 'service', '-s', self.service_name, '--reqtype', 'gz.msgs.Pose',
                   '--reptype', 'gz.msgs.Boolean', '--timeout', '1000',
                   '--req', f'name: "{self.marker_name}" '
                           f'position {{x: {pose.position.x} y: {pose.position.y} z: {pose.position.z}}} '
                           f'orientation {{x: {pose.orientation.x} y: {pose.orientation.y} '
                           f'z: {pose.orientation.z} w: {pose.orientation.w}}}']
            
            if subprocess.run(cmd, capture_output=True, timeout=2).returncode == 0:
                self.current_pose = pose
                return True
            return False
        except Exception as e:
            self.get_logger().error(f"Pose error: {e}")
            return False

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)

    def euler_from_quaternion(self, x, y, z, w):
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def print_controls(self):
        print("="*70)
        print("║  Movement Controls:" + " "*49 + "║")
        print("║    W / ↑  : Move forward (+X)" + " "*39 + "║")
        print("║    S / ↓  : Move backward (-X)" + " "*38 + "║")
        print("║    A / ←  : Move left (-Y)" + " "*42 + "║")
        print("║    D / →  : Move right (+Y)" + " "*41 + "║")
        print("║    R      : Move up (+Z)" + " "*44 + "║")
        print("║    F      : Move down (-Z)" + " "*42 + "║")
        print("║" + " "*69 + "║")
        print("║  Rotation Controls:" + " "*49 + "║")
        print("║    Q      : Rotate counter-clockwise (around Z)" + " "*21 + "║")
        print("║    E      : Rotate clockwise (around Z)" + " "*29 + "║")
        print("║" + " "*69 + "║")
        print("║  Other:" + " "*61 + "║")
        print("║    Space  : Reset to original position" + " "*30 + "║")
        print("║    ESC    : Exit program" + " "*44 + "║")
        print("="*70)

    def run(self):
        if not self.prompt_for_marker():
            return
        self.initialize_marker_pose()
        self.print_controls()
        
        try:
            while rclpy.ok():
                key = self.get_key()
                new_pose = Pose()
                new_pose.position.x, new_pose.position.y, new_pose.position.z = (
                    self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)
                new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w = (
                    self.current_pose.orientation.x, self.current_pose.orientation.y, 
                    self.current_pose.orientation.z, self.current_pose.orientation.w)
                
                moved, action = False, ""
                
                # Handle keys
                if key == '\x1b':
                    nk = sys.stdin.read(1)
                    if nk == '[':
                        ak = sys.stdin.read(1)
                        if ak in 'ABDC': 
                            moved = True
                            if ak == 'A': new_pose.position.x += self.step_size; action = f"+X"
                            elif ak == 'B': new_pose.position.x -= self.step_size; action = f"-X"
                            elif ak == 'D': new_pose.position.y += self.step_size; action = f"+Y"
                            elif ak == 'C': new_pose.position.y -= self.step_size; action = f"-Y"
                    else: break
                elif key.lower() == 'w': new_pose.position.x += self.step_size; moved, action = True, "+X"
                elif key.lower() == 's': new_pose.position.x -= self.step_size; moved, action = True, "-X"
                elif key.lower() == 'a': new_pose.position.y += self.step_size; moved, action = True, "+Y"
                elif key.lower() == 'd': new_pose.position.y -= self.step_size; moved, action = True, "-Y"
                elif key.lower() == 'r': new_pose.position.z += self.step_size; moved, action = True, "+Z"
                elif key.lower() == 'f': new_pose.position.z -= self.step_size; moved, action = True, "-Z"
                elif key.lower() in 'qe':
                    roll, pitch, yaw = self.euler_from_quaternion(
                        self.current_pose.orientation.x, self.current_pose.orientation.y,
                        self.current_pose.orientation.z, self.current_pose.orientation.w)
                    yaw += self.rotation_step if key.lower() == 'q' else -self.rotation_step
                    new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w = \
                        self.quaternion_from_euler(roll, pitch, yaw)
                    moved, action = True, "CCW" if key.lower() == 'q' else "CW"
                elif key == ' ':
                    new_pose = self.original_pose
                    moved, action = True, "RESET"
                    print(f"\n🔄 Reset")
                elif key == '\x03': break
                
                if moved and self.set_marker_pose(new_pose):
                    _, _, yaw = self.euler_from_quaternion(new_pose.orientation.x, new_pose.orientation.y,
                                                           new_pose.orientation.z, new_pose.orientation.w)
                    print(f"\r📍 [{new_pose.position.x:5.2f}, {new_pose.position.y:5.2f}, "
                          f"{new_pose.position.z:5.2f}] Yaw:{math.degrees(yaw):5.1f}° {action:<6}", 
                          end='', flush=True)
                
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoKeyboardControl()
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n\n Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
