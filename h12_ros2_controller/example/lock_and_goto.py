import time
import argparse
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.arm_controller import UpperController
from h12_ros2_controller.core.robot_model import RobotModel

def save():
    ChannelFactoryInitialize()
    print('Initializing RobotModel...')
    robot_model = RobotModel('assets/h1_2/h1_2.urdf')
    robot_model.init_subscriber()
    time.sleep(3.0)
    robot_model.update_kinematics()
    print('Saving current configuration...')
    q = robot_model.state['q']
    np.save('./data/h12_configuration.npy', q)

def lock():
    ChannelFactoryInitialize()
    print('Initializing UpperController...')
    upper_controller = UpperController('assets/h1_2/h1_2.urdf',
                                       'assets/h1_2/h1_2_sphere.urdf',
                                       'assets/h1_2/h1_2_sphere_collision.srdf',
                                       dt=0.01,
                                       visualize=True)

    print('Lock robot in current configuration')
    q = upper_controller.robot_model.state['q']
    np.save('./data/h12_configuration.npy', q)

    while True:
        upper_controller.lock_configuration(q)
        time.sleep(upper_controller.dt)

def goto():
    ChannelFactoryInitialize()
    print('Initializing UpperController...')
    upper_controller = UpperController('assets/h1_2/h1_2.urdf',
                                       'assets/h1_2/h1_2_sphere.urdf',
                                       'assets/h1_2/h1_2_sphere_collision.srdf',
                                       dt=0.01,
                                       visualize=True)

    print('Goto saved configuration')
    q = np.load('./data/h12_configuration.npy')

    while True:
        upper_controller.goto_configuration(q)
        time.sleep(upper_controller.dt)

# check
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Command selector")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--save", action="store_true", help="Call the save() function")
    group.add_argument("--lock", action="store_true", help="Call the lock() function")
    group.add_argument("--goto", action="store_true", help="Call the goto() function")

    args = parser.parse_args()

    if args.lock:
        lock()
    elif args.goto:
        goto()
    elif args.save:
        save()
