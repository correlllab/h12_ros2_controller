import time
import argparse
import numpy as np
import meshcat.geometry as geo

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel

def get_replay_options():
    '''get replay options from command line input'''
    print('=== Replay options ===')
    print('1. Play once')
    print('2. Loop playback')
    print('3. Custom speed (default: 1.0x)')

    option = input('Select option (1-3, default: 1): ').strip() or '1'

    loop = False
    speed_factor = 1.0

    if option == '2':
        loop = True
    elif option == '3':
        speed_input = input('Enter speed factor (default: 1.0): ').strip() or '1.0'
        try:
            speed_factor = float(speed_input)
            if speed_factor <= 0:
                print('Invalid speed factor, using 1.0')
                speed_factor = 1.0
        except ValueError:
            print('Invalid speed factor, using 1.0')
            speed_factor = 1.0

    return loop, speed_factor


def main():
    parser = argparse.ArgumentParser(description='replay recorded robot motion')
    parser.add_argument('--load', type=str, required=True, help='Filename to load under data/robot_record/')
    args = parser.parse_args()

    # set load path using command line argument (file under data/robot_record)
    load_path = f'./data/robot_record/{args.load}'

    # load the data
    if not os.path.exists(load_path):
        print(f'Error: file {load_path} not found')
        return

    try:
        data = np.load(load_path, allow_pickle=True)
    except Exception as e:
        print(f'Error loading data: {e}')
        return

    # get replay options
    loop, speed_factor = get_replay_options()

    ChannelFactoryInitialize()
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_visualizer()
    robot_model.config_visualizer(show_com=True)

    print(f'Starting replay from: {load_path}')
    print(f'Loop: {loop}, Speed: {speed_factor}x')
    print('Press Ctrl+C to stop')

    # extract data
    if 'q' not in data:
        print('Error: no \'q\' (joint positions) found in recorded data')
        return

    q_data = data['q']
    timestamps = data.get('timestamps', None)
    imu_state_data = data.get('imu_state', None)
    zmp_data = data.get('zmp', None)
    num_frames = len(q_data)

    print(f'Replaying {num_frames} frames')

    try:
        frame_idx = 0
        replay_time = 0.0

        while True:
            # loop back if requested
            if frame_idx >= num_frames:
                if loop:
                    frame_idx = 0
                    replay_time = 0.0
                    print('Looping...')
                else:
                    print('Replay finished')
                    break

            # calculate actual playback time
            if timestamps is not None and frame_idx > 0:
                # original time between frames
                original_dt = timestamps[frame_idx] - timestamps[frame_idx - 1]
                # adjusted for speed factor
                playback_dt = original_dt / speed_factor
            else:
                playback_dt = 0.01 / speed_factor  # default 0.01s

            start_frame_time = time.time()

            # set joint angles manually (no state subscriber)
            robot_model._q = q_data[frame_idx].copy()

            # extract other state data if available
            if 'dq' in data:
                robot_model._dq = data['dq'][frame_idx].copy()
            if 'tau' in data:
                robot_model._tau = data['tau'][frame_idx].copy()

            # update kinematics and visualization (pass imu quaternion if available)
            imu_quat = None
            if imu_state_data is not None:
                imu_quat = imu_state_data[frame_idx].quaternion
            robot_model.update_kinematics(imu_quat=imu_quat)
            robot_model.update_visualizer()

            # visualize saved ZMP if available
            if zmp_data is not None and frame_idx < len(zmp_data):
                zmp_pos = zmp_data[frame_idx]
                transform = np.eye(4)
                transform[:3, 3] = zmp_pos
                viewer = robot_model.viz.viewer
                viewer['zmp'].set_object(geo.Sphere(0.02))
                viewer['zmp'].set_transform(transform)
                viewer['zmp'].set_property('color', (0.0, 1.0, 0.0, 0.8))

            # print frame info
            if frame_idx % 100 == 0:
                print(f'Frame {frame_idx}/{num_frames}')

            # maintain playback timing
            elapsed = time.time() - start_frame_time
            if elapsed < playback_dt:
                time.sleep(playback_dt - elapsed)

            replay_time += playback_dt
            frame_idx += 1

    except KeyboardInterrupt:
        print('Stopping replay...')

    robot_model.shutdown()


if __name__ == '__main__':
    main()
