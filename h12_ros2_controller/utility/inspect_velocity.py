import time
import argparse
import numpy as np
from tqdm import tqdm

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel

def save_data():
    ChannelFactoryInitialize()
    print('Initializing RobotModel...')
    robot_model = RobotModel('assets/h1_2/h1_2.urdf')
    robot_model.init_subscriber()
    robot_model.init_visualizer()

    dq_arr = []
    left_ee_name = 'left_wrist_yaw_link'
    right_ee_name = 'right_wrist_yaw_link'
    left_ee_twist_arr = []
    right_ee_twist_arr = []

    for _ in tqdm(range(3000)):
        # sync data
        robot_model.sync_subscriber()
        robot_model.update_kinematics()
        robot_model.update_visualizer()

        # record all dp and ee velocities
        dq_arr.append(robot_model.dq)
        left_ee_twist_arr.append(robot_model.get_frame_twist(left_ee_name))
        right_ee_twist_arr.append(robot_model.get_frame_twist(right_ee_name))

        time.sleep(0.01)

    dq_arr = np.array(dq_arr)
    left_ee_twist_arr = np.array(left_ee_twist_arr)
    right_ee_twist_arr = np.array(right_ee_twist_arr)
    np.save('./data/h1_2_dq.npy', dq_arr)
    np.save('./data/h1_2_left_ee_twist.npy', left_ee_twist_arr)
    np.save('./data/h1_2_right_ee_twist.npy', right_ee_twist_arr)

def print_data():
    dq_arr = np.load('./data/h1_2_dq.npy')
    left_ee_twist_arr = np.load('./data/h1_2_left_ee_twist.npy')
    right_ee_twist_arr = np.load('./data/h1_2_right_ee_twist.npy')

    print('dq shape:', dq_arr.shape)
    print('left ee twist shape:', left_ee_twist_arr.shape)
    print('right ee twist shape:', right_ee_twist_arr.shape)

    # end effector linear and angular velocities
    left_v = left_ee_twist_arr[:, :3]
    left_w = left_ee_twist_arr[:, 3:]
    right_v = right_ee_twist_arr[:, :3]
    right_w = right_ee_twist_arr[:, 3:]
    print('left ee linear velocity mean:', np.mean(np.linalg.norm(left_v, axis=1)))
    print('left ee angular velocity mean:', np.mean(np.linalg.norm(left_w, axis=1)))
    print('right ee linear velocity mean:', np.mean(np.linalg.norm(right_v, axis=1)))
    print('right ee angular velocity mean:', np.mean(np.linalg.norm(right_w, axis=1)))

    # joint velocities
    print('joint velocity mean:', np.mean(np.abs(dq_arr)))
    print('joint velocity max:', np.max(np.abs(dq_arr)))

    # different joints
    # 14-20 left arm joints
    # 33-39 right arm joints
    print('left arm joint velocity mean:', np.mean(np.abs(dq_arr[:, 13:20]), axis=0))
    print('left arm joint velocity max:', np.max(np.abs(dq_arr[:, 13:20]), axis=0))
    print('right arm joint velocity mean:', np.mean(np.abs(dq_arr[:, 32:39]), axis=0))
    print('right arm joint velocity max:', np.max(np.abs(dq_arr[:, 32:39]), axis=0))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command selector')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--save', action='store_true', help='Call the save_data() function')
    group.add_argument('--print', action='store_true', help='Call the print_data() function')

    args = parser.parse_args()

    if args.save:
        save_data()
    elif args.print:
        print_data()
