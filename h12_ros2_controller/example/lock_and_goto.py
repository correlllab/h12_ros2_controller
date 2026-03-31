import time
import argparse
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.arm_controller import UpperController
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config, initialize_channel_factory

def save(config='default.yaml'):
    config_data = load_controller_config(config)
    initialize_channel_factory(config_data)
    print('Initializing RobotModel...')
    robot_model = RobotModel('assets/h1_2/h1_2.urdf')
    low_state_topic = config_data.get('topics', {}).get('low_state', 'rt/lowstate')
    robot_model.init_subscriber(low_state_topic=low_state_topic)
    time.sleep(3.0)
    robot_model.update_kinematics()
    print('Saving current configuration...')
    q = robot_model.state['q']
    np.save('./data/h12_configuration.npy', q)

def lock(config='default.yaml'):
    config_data = load_controller_config(config)
    initialize_channel_factory(config_data)
    print('Initializing UpperController...')
    upper_controller = UpperController('assets/h1_2/h1_2.urdf',
                                       'assets/h1_2/h1_2_sphere.urdf',
                                       'assets/h1_2/h1_2_sphere_collision.srdf',
                                       visualize=True,
                                       config=config)

    print('Lock robot in current configuration')
    q = upper_controller.robot_model.state['q']
    np.save('./data/h12_configuration.npy', q)

    while True:
        upper_controller.lock_configuration(q)
        time.sleep(upper_controller.dt)

def goto(config='default.yaml'):
    config_data = load_controller_config(config)
    initialize_channel_factory(config_data)
    print('Initializing UpperController...')
    upper_controller = UpperController('assets/h1_2/h1_2.urdf',
                                       'assets/h1_2/h1_2_sphere.urdf',
                                       'assets/h1_2/h1_2_sphere_collision.srdf',
                                       visualize=True,
                                       config=config)

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
    parser.add_argument('--config', type=str, default='default.yaml', help='YAML file name under config/')

    args = parser.parse_args()

    if args.lock:
        lock(config=args.config)
    elif args.goto:
        goto(config=args.config)
    elif args.save:
        save(config=args.config)
