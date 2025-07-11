import time
import numpy as np
import tkinter as tk
import pinocchio as pin

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.arm_controller import ArmController

def input_pose(side):
    home_pose = {
        'left': [0.3, 0.2, 0.1, 0.0, 0.0, 0.0],  # x, y, z, roll, pitch, yaw
        'right': [0.3, -0.2, 0.1, 0.0, 0.0, 0.0]  # x, y, z, roll, pitch, yaw
    }

    home = home_pose.get(side, None)
    if home is not None:
        print(f'{side.capitalize()} end-effector pose...')
        choice = input(f'Home position {home}? (y/n): ').lower()
        if choice == 'y':
            return home
    while True:
        input_pose = input('Enter x y z roll pitch yaw (separated by space): ')
        parts = input_pose.strip().split()

        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue
        try:
            values = [float(val) for val in parts]
            return values
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')
            continue


def main():
    ChannelFactoryInitialize()

    # Initialize arm controller
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   'assets/h1_2/h1_2_sphere.urdf',
                                   'assets/h1_2/h1_2_sphere_collision.srdf',
                                   dt=0.02,
                                   vlim=1.0,
                                   wlim=2.0,
                                   dmin=0.01,
                                   visualize=False)

    while True:
        # get target poses
        left_pose = input_pose('left')
        right_pose = input_pose('right')
        # set target poses
        arm_controller.left_ee_target_pose = left_pose
        arm_controller.right_ee_target_pose = right_pose

        start_time = time.time()
        while time.time() - start_time < 10.0:
            frame_start_time = time.time()
            arm_controller.control_dual_arm_step()

            # print errors
            left_error = np.linalg.norm(arm_controller.left_ee_error)
            right_error = np.linalg.norm(arm_controller.right_ee_error)

            print(f'Left Error: {left_error:.4f}, Right Error: {right_error:.4f}')

            # early break
            if left_error < 1e-4 and right_error < 1e-4:
                print('Target reached!')
                break

            time.sleep(max(0.0, arm_controller.dt - (time.time() - frame_start_time)))

        input('Press any key to continue...') # flush the input buffer
        cont = input('Do you want to send another goal? (y/n): ').lower()
        if cont != 'y':
            break

if __name__ == '__main__':
    main()
