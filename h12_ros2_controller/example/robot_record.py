import time
import argparse
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.dds_init import init_channel_factory_from_env
from h12_ros2_controller.utility.path_definition import URDF_HANDLESS_PATH

def main(save_path):
    init_channel_factory_from_env()
    robot_model = RobotModel(URDF_HANDLESS_PATH, handless=True)
    robot_model.init_visualizer()
    robot_model.config_visualizer(show_sensors=True, show_com=True, show_zmp=True)
    robot_model.init_subscriber()

    # initialize data collection
    state_arr = []
    com_arr = []
    zmp_arr = []

    print(f'Recording robot data to: {save_path}')
    print('Press Ctrl+C to stop recording')

    # main loop recording robot states
    try:
        while True:
            robot_model.update_kinematics()
            robot_model.update_visualizer()

            # record state information
            state_arr.append(robot_model.state)
            com_arr.append(robot_model.dynamics.get_com())
            zmp_arr.append(robot_model.dynamics.get_zmp())

            time.sleep(0.01)

    except KeyboardInterrupt:
        print('Stopping recording...')

    save_results(state_arr, com_arr, zmp_arr, save_path)

    robot_model.shutdown()

def save_results(state_arr, com_arr, zmp_arr, save_path):
    '''Save recorded data to npz file'''
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    # extract arrays from state dicts
    data = {
        'com': np.array(com_arr),
        'zmp': np.array(zmp_arr),
    }
    # add all state fields as arrays
    keys = state_arr[0].keys()
    for key in keys:
        data[key] = np.array([state[key] for state in state_arr])

    np.savez(save_path, **data)
    print(f'Data saved to {save_path}')
    print(f'Recorded {len(state_arr)} frames')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Record robot state, CoM, and ZMP')
    parser.add_argument('--save', type=str, required=True, help='Filename to save under data/robot_record/')
    args = parser.parse_args()

    # set save path using command line argument
    save_path = f'./data/robot_record/{args.save}'
    main(save_path)
