import time
import numpy as np
import tkinter as tk

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.channel_interface import ArmSDKPublisher

def main():
    ChannelFactoryInitialize()
    arm_sdk_publisher = ArmSDKPublisher()

    arm_sdk_publisher.kp[4] = 10.0
    arm_sdk_publisher.kd[4] = 2.0
    arm_sdk_publisher.enable_motor([4], [0.0])

    arm_sdk_publisher.start_publisher()

    root = tk.Tk()
    root.title("Arm Joint 4 Control")
    slider = tk.Scale(root, label='Joint 4 Position',
                      from_=-2.0, to=2.0,
                      resolution=0.01, orient=tk.HORIZONTAL, length=400)
    slider.pack()
    slider.set(0.0)
    root.update()

    while True:
        root.update()
        arm_sdk_publisher.q[4] = slider.get()
        time.sleep(0.01)

if __name__ == '__main__':
    main()
