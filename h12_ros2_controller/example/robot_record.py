import time
import argparse
import numpy as np
from collections import defaultdict

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel

def main():
    parser = argparse.ArgumentParser(description='Record robot state, CoM, and ZMP')
    parser.add_argument('--save', type=str, required=True, help='Filename to save under data/robot_record/')
    args = parser.parse_args()

    ChannelFactoryInitialize()
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_visualizer()
    robot_model.config_visualizer(show_sensors=True, show_com=True, show_zmp=True)
    robot_model.init_subscriber()

    # initialize data collection
    recorded_data = defaultdict(list)
    timestamps = []
    start_time = time.time()

    # set save path using command line argument (file under data/robot_record)
    save_path = f'./data/robot_record/{args.save}'

    print(f'Recording robot data to: {save_path}')
    print('Press Ctrl+C to stop recording')

    # main loop recording robot states
    try:
        while True:
            robot_model.update_kinematics()
            robot_model.update_visualizer()

            # record timestamp
            current_time = time.time() - start_time
            timestamps.append(current_time)

            # record state information
            state = robot_model.state
            for key, value in state.items():
                if isinstance(value, np.ndarray):
                    recorded_data[key].append(value.copy())
                else:
                    recorded_data[key].append(value)

            # record CoM and ZMP
            try:
                com = robot_model.get_com()
                recorded_data['com'].append(com.copy())
            except Exception as e:
                print(f'Warning: could not compute CoM: {e}')

            try:
                zmp = robot_model.get_zmp()
                recorded_data['zmp'].append(zmp.copy())
            except Exception as e:
                print(f'Warning: could not compute ZMP: {e}')

            time.sleep(0.01)

    except KeyboardInterrupt:
        print('Stopping recording...')

    # convert lists to numpy arrays
    for key in recorded_data:
        if key not in ['mode', 'motor_state']:  # skip non-numeric data
            try:
                recorded_data[key] = np.array(recorded_data[key])
            except (ValueError, TypeError):
                pass

    # add timestamps
    recorded_data['timestamps'] = np.array(timestamps)

    # create save directory if it doesn't exist
    save_dir = os.path.dirname(save_path)
    if save_dir and not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # save the data
    np.savez(save_path, **recorded_data)
    print(f'Data saved to {save_path}')
    print(f'Recorded {len(timestamps)} frames over {timestamps[-1]:.2f} seconds')

    robot_model.shutdown()


if __name__ == '__main__':
    main()
