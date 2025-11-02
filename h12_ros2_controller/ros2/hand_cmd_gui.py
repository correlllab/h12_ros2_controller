import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import time
import numpy as np
import tkinter as tk

class HandCmdGUI(Node):
    def __init__(self, dt=0.005):
        super().__init__('hand_cmd_publisher')
        self.dt = dt
        # ROS publishers
        self.left_cmd_publisher = self.create_publisher(
            Float64MultiArray, 'left_hand_cmd', 10)
        self.right_cmd_publisher = self.create_publisher(
            Float64MultiArray, 'right_hand_cmd', 10)

        # GUI setup
        self.root = tk.Tk()
        self.root.title('Hand Command Publisher')
        self.root.geometry('600x400')

        self.left_frame = tk.Frame(self.root)
        self.right_frame = tk.Frame(self.root)
        self.left_frame.pack(side=tk.LEFT, padx=10, pady=10)
        self.right_frame.pack(side=tk.RIGHT, padx=10, pady=10)

        self.setup_sliders()

    def setup_sliders(self):
        # Left hand sliders
        self.sliders_left = [
            tk.Scale(self.left_frame, label=name, from_=0.0, to=1.0, resolution=0.01,
                     orient=tk.HORIZONTAL, length=250)
            for name in ['Left Pinky', 'Left Ring', 'Left Middle',
                         'Left Index', 'Left Thumb', 'Left Angle']
        ]
        for s in self.sliders_left:
            s.pack(in_=self.left_frame, pady=5)

        # Right hand sliders
        self.sliders_right = [
            tk.Scale(self.right_frame, label=name, from_=0.0, to=1.0, resolution=0.01,
                     orient=tk.HORIZONTAL, length=250)
            for name in ['Right Pinky', 'Right Ring', 'Right Middle',
                         'Right Index', 'Right Thumb', 'Right Angle']
        ]
        for s in self.sliders_right:
            s.pack(in_=self.right_frame, pady=5)

    def spin(self):
        while rclpy.ok():
            start_time = time.time()
            self.root.update()

            # get slider states
            left_arr = [s.get() for s in self.sliders_left]
            right_arr = [s.get() for s in self.sliders_right]

            # publish to ROS topics
            left_msg = Float64MultiArray()
            left_msg.data = left_arr
            self.left_cmd_publisher.publish(left_msg)

            right_msg = Float64MultiArray()
            right_msg.data = right_arr
            self.right_cmd_publisher.publish(right_msg)

            time.sleep(max(0.0, self.dt - (time.time() - start_time)))

def main(args=None):
    rclpy.init(args=args)
    node = HandCmdGUI()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
