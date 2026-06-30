import time
import argparse
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.arm_controller import UpperController
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config, initialize_channel_factory
from h12_ros2_controller.utility.path_definition import (
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
)

def save(config_name='debug.yaml'):
    config = load_controller_config(config_name)
    initialize_channel_factory(config)
    print('Initializing RobotModel...')
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    low_state_topic = config.get('topics', {}).get('low_state', 'rt/lowstate')
    robot_model.init_subscriber(low_state_topic=low_state_topic)
    time.sleep(3.0)
    robot_model.update_kinematics()
    print('Saving current configuration...')
    q = robot_model.state['q']
    np.save('./data/h12_configuration.npy', q)

def lock(config_name='debug.yaml'):
    config = load_controller_config(config_name)
    initialize_channel_factory(config)
    print('Initializing UpperController...')
    upper_controller = UpperController(URDF_MAGPIE_PATH,
                                       URDF_MAGPIE_SPHERE_PATH,
                                       SRDF_MAGPIE_SPHERE_PATH,
                                       handless=False,
                                       visualize=True,
                                       config=config)

    print('Lock robot in current configuration')
    q = upper_controller.robot_model.state['q']
    np.save('./data/h12_configuration.npy', q)

    while True:
        upper_controller.lock_configuration(q)
        time.sleep(upper_controller.dt)

def goto(config_name='debug.yaml'):
    config = load_controller_config(config_name)
    initialize_channel_factory(config)
    print('Initializing UpperController...')
    upper_controller = UpperController(URDF_MAGPIE_PATH,
                                       URDF_MAGPIE_SPHERE_PATH,
                                       SRDF_MAGPIE_SPHERE_PATH,
                                       handless=False,
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
    parser.add_argument('--config', type=str, default='debug.yaml', help='YAML file name under config/')

    args = parser.parse_args()

    if args.lock:
        lock(config_name=args.config)
    elif args.goto:
        goto(config_name=args.config)
    elif args.save:
        save(config_name=args.config)
