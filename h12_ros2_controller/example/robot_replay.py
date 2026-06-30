import time
import argparse
import numpy as np
import meshcat.geometry as geo

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.dds_init import init_channel_factory_from_env
from h12_ros2_controller.utility.path_definition import URDF_MAGPIE_PATH
from h12_ros2_controller.plot.plot_robot_record import create_com_zmp_plot, update_com_zmp_plot, clear_com_zmp_plot

def run_replay_loop(robot_model, data, loop, replay_speed, show_plot):
    '''Run the replay loop with given data and loop speed'''
    q_arr = data['q']
    time_stamp_arr = data.get('time_stamp', None)
    imu_state_arr = data.get('imu_state', None)
    com_arr = data.get('com', None)
    zmp_arr = data.get('zmp', None)
    num_frames = len(q_arr)

    # create plot if requested
    plot_elements = None
    if show_plot and zmp_arr is not None:
        plot_elements = create_com_zmp_plot(num_frames=num_frames)

    print(f'Replaying {num_frames} frames')

    frame_idx = 0
    while True:
        # loop back if requested
        if frame_idx >= num_frames:
            if loop:
                frame_idx = 0
                if plot_elements is not None:
                    clear_com_zmp_plot(plot_elements)
                print('Looping...')
            else:
                print('Replay finished')
                break

        # calculate playback timing
        if time_stamp_arr is not None and frame_idx > 0:
            original_dt = time_stamp_arr[frame_idx] - time_stamp_arr[frame_idx - 1]
            playback_dt = original_dt / replay_speed
        else:
            playback_dt = 0.01 / replay_speed

        start_time = time.time()

        # set joint state
        robot_model._q = q_arr[frame_idx].copy()
        robot_model._dq = data['dq'][frame_idx].copy()
        robot_model._tau = data['tau'][frame_idx].copy()

        # update kinematics with imu orientation
        imu_quat = imu_state_arr[frame_idx].quaternion if imu_state_arr is not None else None
        robot_model.update_kinematics(imu_quat=imu_quat)
        robot_model.update_visualizer()

        # visualize saved CoM
        if com_arr is not None:
            transform = np.eye(4)
            transform[:3, 3] = com_arr[frame_idx]
            viewer = robot_model.viz.viewer
            viewer['com'].set_object(geo.Sphere(0.02))
            viewer['com'].set_transform(transform)
            viewer['com'].set_property('color', (1.0, 0.0, 0.0, 0.8))

        # visualize saved ZMP
        if zmp_arr is not None:
            transform = np.eye(4)
            transform[:3, 3] = zmp_arr[frame_idx]
            viewer = robot_model.viz.viewer
            viewer['zmp'].set_object(geo.Sphere(0.02))
            viewer['zmp'].set_transform(transform)
            viewer['zmp'].set_property('color', (0.0, 1.0, 0.0, 0.8))

        # update plot if enabled
        if plot_elements is not None:
            com_pos = com_arr[frame_idx]
            zmp_pos = zmp_arr[frame_idx]
            left_foot_pos = robot_model.get_frame_position('left_ankle_roll_link')
            right_foot_pos = robot_model.get_frame_position('right_ankle_roll_link')
            update_com_zmp_plot(plot_elements, frame_idx, com_pos, zmp_pos, left_foot_pos, right_foot_pos)

        # print frame info
        if frame_idx % 10 == 0:
            print(f'Frame {frame_idx}/{num_frames}')

        # maintain playback timing
        time.sleep(max(0.0, playback_dt - (time.time() - start_time)))

        frame_idx += 1

def main(load_path, loop, replay_speed, show_plot):
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

    # initialize robot model and visualizer
    init_channel_factory_from_env()
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_visualizer()

    print(f'Starting replay from: {load_path}')
    print(f'Loop: {loop}, Speed: {replay_speed}x, Plot: {show_plot}')
    print('Press Ctrl+C to stop')

    # run replay loop
    try:
        run_replay_loop(robot_model, data, loop, replay_speed, show_plot)
    except KeyboardInterrupt:
        print('Stopping replay...')

    robot_model.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='replay recorded robot motion')
    parser.add_argument('--load', type=str, required=True, help='Filename to load under data/robot_record/')
    parser.add_argument('--loop', action='store_true', help='Loop the replay')
    parser.add_argument('--speed', type=float, default=1.0, help='Replay speed factor')
    parser.add_argument('--plot', action='store_true', help='Show ZMP plot')
    args = parser.parse_args()

    # set load path using command line argument
    load_path = f'./data/robot_record/{args.load}.npz'

    main(load_path, args.loop, args.speed, args.plot)
