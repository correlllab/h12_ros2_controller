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
    # choose playback mode
    print('=== Playback mode ===')
    print('1. Play once')
    print('2. Loop playback')
    mode = input('Select mode (1-2, default: 1): ').strip() or '1'
    loop = (mode == '2')

    # choose playback speed
    print('=== Playback speed ===')
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

def run_replay_loop(robot_model, data, loop, speed_factor):
    '''Run the replay loop with given data and loop speed'''
    q_arr = data['q']
    time_stamp_arr = data.get('time_stamp', None)
    imu_state_arr = data.get('imu_state', None)
    zmp_arr = data.get('zmp', None)
    com_arr = data.get('com', None)
    num_frames = len(q_arr)

    print(f'Replaying {num_frames} frames')

    frame_idx = 0
    while True:
        # loop back if requested
        if frame_idx >= num_frames:
            if loop:
                frame_idx = 0
                print('Looping...')
            else:
                print('Replay finished')
                break

        # calculate playback timing
        if time_stamp_arr is not None and frame_idx > 0:
            original_dt = time_stamp_arr[frame_idx] - time_stamp_arr[frame_idx - 1]
            playback_dt = original_dt / speed_factor
        else:
            playback_dt = 0.01 / speed_factor

        start_frame_time = time.time()

        # set joint state
        robot_model._q = q_arr[frame_idx].copy()
        robot_model._dq = data['dq'][frame_idx].copy()
        robot_model._tau = data['tau'][frame_idx].copy()

        # update kinematics with imu orientation
        imu_quat = imu_state_arr[frame_idx].quaternion if imu_state_arr is not None else None
        robot_model.update_kinematics(imu_quat=imu_quat)
        robot_model.update_visualizer()

        # visualize saved ZMP
        if zmp_arr is not None:
            transform = np.eye(4)
            transform[:3, 3] = zmp_arr[frame_idx]
            viewer = robot_model.viz.viewer
            viewer['zmp'].set_object(geo.Sphere(0.02))
            viewer['zmp'].set_transform(transform)
            viewer['zmp'].set_property('color', (0.0, 1.0, 0.0, 0.8))

        # visualize saved CoM
        if com_arr is not None:
            transform = np.eye(4)
            transform[:3, 3] = com_arr[frame_idx]
            viewer = robot_model.viz.viewer
            viewer['com'].set_object(geo.Sphere(0.02))
            viewer['com'].set_transform(transform)
            viewer['com'].set_property('color', (1.0, 0.0, 0.0, 0.8))

        # print frame info
        if frame_idx % 10 == 0:
            print(f'Frame {frame_idx}/{num_frames}')

        # maintain playback timing
        elapsed = time.time() - start_frame_time
        if elapsed < playback_dt:
            time.sleep(playback_dt - elapsed)

        frame_idx += 1

def main(load_path):
    # load the data
    if not os.path.exists(load_path):
        print(f'Error: file {load_path} not found')
        return

    try:
        data = np.load(load_path, allow_pickle=True)
    except Exception as e:
        print(f'Error loading data: {e}')
        return

    if 'q' not in data:
        print('Error: no q (joint positions) found in recorded data')
        return

    # get replay options
    loop, speed_factor = get_replay_options()

    # initialize robot model and visualizer
    ChannelFactoryInitialize()
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_visualizer()

    print(f'Starting replay from: {load_path}')
    print(f'Loop: {loop}, Speed: {speed_factor}x')
    print('Press Ctrl+C to stop')

    # run replay loop
    try:
        run_replay_loop(robot_model, data, loop, speed_factor)
    except KeyboardInterrupt:
        print('Stopping replay...')

    robot_model.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='replay recorded robot motion')
    parser.add_argument('--load', type=str, required=True, help='Filename to load under data/robot_record/')
    args = parser.parse_args()

    # set load path using command line argument (file under data/robot_record)
    load_path = f'./data/robot_record/{args.load}.npz'

    main(load_path)
