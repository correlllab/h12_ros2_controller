import time
import numpy as np
import tkinter as tk

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import ArmSDKPublisher

def main():
    ChannelFactoryInitialize()
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_subscriber()
    time.sleep(1.0)
    robot_model.sync_subscriber()
    robot_model.update_kinematics()

    arm_sdk_publisher = ArmSDKPublisher()

    arm_sdk_publisher.kp[:] = 0.0
    arm_sdk_publisher.kd[:] = 3.0

    i_enabled = 4
    arm_sdk_publisher.kp[i_enabled] = 30.0

    init_q = np.zeros(15)
    init_q[0:7] = robot_model.q[13:20]
    init_q[7:12] = robot_model.q[32:29]
    arm_sdk_publisher.enable_motor([i for i in range(0, 15)], init_q)
    arm_sdk_publisher.start_publisher()

    root = tk.Tk()
    root.title("Arm Joint 4 Control")
    slider = tk.Scale(root, label='Joint 4 Position',
                      from_=-2.0, to=2.0,
                      resolution=0.01, orient=tk.HORIZONTAL, length=400)
    slider.pack()
    slider.set(init_q[i_enabled])
    root.update()

    while True:
        root.update()
        arm_sdk_publisher.q[i_enabled] = slider.get()
        time.sleep(0.01)

if __name__ == '__main__':
    main()
