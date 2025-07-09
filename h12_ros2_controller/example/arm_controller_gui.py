import time
import numpy as np
import tkinter as tk

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.arm_controller import ArmController

def main():
    ChannelFactoryInitialize()
    # example usage
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   'assets/h1_2/h1_2_sphere.urdf',
                                   'assets/h1_2/h1_2_sphere_collision.srdf',
                                   dt=0.02,
                                   vlim=1.0,
                                   wlim=2.0,
                                   dmin=0.01,
                                   visualize=True)

    root = tk.Tk()
    root.title('Arm Controller')
    root.geometry('600x400')

    # pack sliders side by side
    left_frame = tk.Frame(root)
    right_frame = tk.Frame(root)
    left_frame.pack(side=tk.LEFT, padx=10, pady=10)
    right_frame.pack(side=tk.RIGHT, padx=10, pady=10)

    # left hand sliders
    slider_lx = tk.Scale(left_frame, label="Left X",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_ly = tk.Scale(left_frame, label="Left Y",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_lz = tk.Scale(left_frame, label="Left Z",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_lr = tk.Scale(left_frame, label="Left Roll",
                         from_=-np.pi, to=np.pi, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_lp = tk.Scale(left_frame, label="Left Pitch",
                         from_=-np.pi, to=np.pi, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_lyaw = tk.Scale(left_frame, label="Left Yaw",
                           from_=-np.pi, to=np.pi, resolution=0.01,
                           orient=tk.HORIZONTAL, length=250)
    slider_lx.pack(in_=left_frame, pady=5)
    slider_ly.pack(in_=left_frame, pady=5)
    slider_lz.pack(in_=left_frame, pady=5)
    slider_lr.pack(in_=left_frame, pady=5)
    slider_lp.pack(in_=left_frame, pady=5)
    slider_lyaw.pack(in_=left_frame, pady=5)

    # right hand sliders
    slider_rx = tk.Scale(right_frame, label="Right X",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_ry = tk.Scale(right_frame, label="Right Y",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_rz = tk.Scale(right_frame, label="Right Z",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_rr = tk.Scale(right_frame, label="Right Roll",
                         from_=-np.pi, to=np.pi, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_rp = tk.Scale(right_frame, label="Right Pitch",
                         from_=-np.pi, to=np.pi, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_ryaw = tk.Scale(right_frame, label="Right Yaw",
                           from_=-np.pi, to=np.pi, resolution=0.01,
                           orient=tk.HORIZONTAL, length=250)
    slider_rx.pack(in_=right_frame, pady=5)
    slider_ry.pack(in_=right_frame, pady=5)
    slider_rz.pack(in_=right_frame, pady=5)
    slider_rr.pack(in_=right_frame, pady=5)
    slider_rp.pack(in_=right_frame, pady=5)
    slider_ryaw.pack(in_=right_frame, pady=5)

    # left hand target initialization
    left_ee_position = arm_controller.left_ee_target_pose[:3]
    slider_lx.set(left_ee_position[0])
    slider_ly.set(left_ee_position[1])
    slider_lz.set(left_ee_position[2])
    left_ee_rpy = arm_controller.left_ee_target_rpy
    slider_lr.set(left_ee_rpy[0])
    slider_lp.set(left_ee_rpy[1])
    slider_lyaw.set(left_ee_rpy[2])

    # Right hand target initialization
    right_ee_position = arm_controller.right_ee_target_pose[:3]
    slider_rx.set(right_ee_position[0])
    slider_ry.set(right_ee_position[1])
    slider_rz.set(right_ee_position[2])
    right_ee_rpy = arm_controller.right_ee_target_rpy
    slider_rr.set(right_ee_rpy[0])
    slider_rp.set(right_ee_rpy[1])
    slider_ryaw.set(right_ee_rpy[2])

    root.update()

    while True:
        start_time = time.time()
        root.update()
        # update left hand target
        lx = slider_lx.get()
        ly = slider_ly.get()
        lz = slider_lz.get()
        lr = slider_lr.get()
        lp = slider_lp.get()
        lyaw = slider_lyaw.get()
        arm_controller.left_ee_target_pose = [lx, ly, lz, lr, lp, lyaw]

        # update right hand target
        rx = slider_rx.get()
        ry = slider_ry.get()
        rz = slider_rz.get()
        rr = slider_rr.get()
        rp = slider_rp.get()
        ryaw = slider_ryaw.get()
        arm_controller.right_ee_target_pose = [rx, ry, rz, rr, rp, ryaw]

        arm_controller.control_dual_arm_step()
        # arm_controller.sim_dual_arm_step()
        time.sleep(max(0.0, arm_controller.dt - (time.time() - start_time)))

if __name__ == '__main__':
    main()
