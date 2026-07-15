import time
import argparse
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.arm_controller import UpperController
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config, init_channel_factory_guard
from h12_ros2_controller.utility.path_definition import (
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
)

DEFAULT_CONFIGURATION_NAME = 'h12_configuration.npy'

def configuration_path(name):
    if not name.endswith('.npy'):
        name = f'{name}.npy'
    return os.path.join('./data', name)

def save(config_name='debug.yaml', name=DEFAULT_CONFIGURATION_NAME):
    config = load_controller_config(config_name)
    init_channel_factory_guard(config)
    print('Initializing RobotModel...')
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    low_state_topic = config.get('topics', {}).get('low_state', 'rt/lowstate')
    robot_model.init_subscriber(low_state_topic=low_state_topic)
    time.sleep(3.0)
    robot_model.update_kinematics()
    path = configuration_path(name)
    print(f'Saving current configuration to {path}...')
    q = robot_model.state['q']
    np.save(path, q)

def lock(config_name='debug.yaml', name=DEFAULT_CONFIGURATION_NAME):
    config = load_controller_config(config_name)
    init_channel_factory_guard(config)
    print('Initializing UpperController...')
    upper_controller = UpperController(URDF_MAGPIE_PATH,
                                       URDF_MAGPIE_SPHERE_PATH,
                                       SRDF_MAGPIE_SPHERE_PATH,
                                       init=False,
                                       handless=False,
                                       visualize=True,
                                       config=config)

    print('Lock robot in current configuration')
    q = upper_controller.robot_model.state['q']
    path = configuration_path(name)
    np.save(path, q)

    while True:
        upper_controller.lock_configuration(q)
        time.sleep(upper_controller.dt)

def goto(config_name='debug.yaml', name=DEFAULT_CONFIGURATION_NAME):
    config = load_controller_config(config_name)
    init_channel_factory_guard(config)
    print('Initializing UpperController...')
    upper_controller = UpperController(URDF_MAGPIE_PATH,
                                       URDF_MAGPIE_SPHERE_PATH,
                                       SRDF_MAGPIE_SPHERE_PATH,
                                       init=False,
                                       handless=False,
                                       visualize=True,
                                       config=config)

    path = configuration_path(name)
    print(f'Goto saved configuration from {path}')
    q = np.load(path)

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
    parser.add_argument('--name', type=str, default=DEFAULT_CONFIGURATION_NAME, help='Saved configuration file name under data/')

    args = parser.parse_args()

    if args.lock:
        lock(config_name=args.config, name=args.name)
    elif args.goto:
        goto(config_name=args.config, name=args.name)
    elif args.save:
        save(config_name=args.config, name=args.name)
