#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import json


class PIDGuiNode(Node):
    def __init__(self):
        super().__init__('pid_gui')

        self.pub = self.create_publisher(String, "/pid_tuning", 10)

        self.root = tk.Tk()
        self.root.title("Crazyflie PID Tuning Panel")

        row = 0

        def field(name, default):
            nonlocal row
            tk.Label(self.root, text=name).grid(row=row, column=0, padx=10, pady=4)
            e = tk.Entry(self.root)
            e.insert(0, default)
            e.grid(row=row, column=1, padx=10, pady=4)
            row += 1
            return e

        self.kp_xy = field("Kp XY", "0.32")
        self.ki_xy = field("Ki XY", "0.01")
        self.kd_xy = field("Kd XY", "0.05")

        self.kp_z = field("Kp Z", "0.32")
        self.ki_z = field("Ki Z", "0.01")
        self.kd_z = field("Kd Z", "0.05")

        self.kp_yaw = field("Kp Yaw", "0.15")
        self.ki_yaw = field("Ki Yaw", "0.01")
        self.kd_yaw = field("Kd Yaw", "0.05")

        tk.Button(self.root, text="Send",
                  command=self.send,
                  bg="green", fg="white").grid(row=row, column=0, columnspan=2, pady=10)

        self.get_logger().info("PID GUI Loaded ✔️")

    def send(self):
        msg = {
            "kp_xy": float(self.kp_xy.get()),
            "ki_xy": float(self.ki_xy.get()),
            "kd_xy": float(self.kd_xy.get()),

            "kp_z": float(self.kp_z.get()),
            "ki_z": float(self.ki_z.get()),
            "kd_z": float(self.kd_z.get()),

            "kp_yaw": float(self.kp_yaw.get()),
            "ki_yaw": float(self.ki_yaw.get()),
            "kd_yaw": float(self.kd_yaw.get()),
        }

        ros_msg = String()
        ros_msg.data = json.dumps(msg)
        self.pub.publish(ros_msg)

        self.get_logger().info(f"Sent PID params: {msg}")

    def loop(self):
        while rclpy.ok():
            self.root.update()
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = PIDGuiNode()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()