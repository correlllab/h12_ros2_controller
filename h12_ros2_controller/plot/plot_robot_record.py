import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.path_definition import URDF_MAGPIE_PATH

def create_com_zmp_plot(num_frames=None, window_size=500):
    '''
    Create plot figure and return plot elements
    num_frames: if set, use fixed horizon (for replay); if None, use rolling window
    window_size: rolling window size when num_frames is None
    '''
    plt.ion()
    fig = plt.figure(figsize=(16, 6))
    ax_2d = fig.add_subplot(2, 3, (1, 4))
    ax_com_x = fig.add_subplot(2, 3, 2)
    ax_com_y = fig.add_subplot(2, 3, 5)
    ax_zmp_x = fig.add_subplot(2, 3, 3)
    ax_zmp_y = fig.add_subplot(2, 3, 6)

    # 2D position plot
    ax_2d.set_xlabel('X (m)')
    ax_2d.set_ylabel('Y (m)')
    ax_2d.set_title('CoM, ZMP, and Foot Positions')
    ax_2d.set_aspect('equal')
    ax_2d.grid(True)
    com_plot, = ax_2d.plot([], [], '^', color='magenta', markersize=12, label='CoM')
    zmp_plot, = ax_2d.plot([], [], 'go', markersize=10, label='ZMP')
    left_foot_plot, = ax_2d.plot([], [], 'bs', markersize=10, label='Left Foot')
    right_foot_plot, = ax_2d.plot([], [], 'rs', markersize=10, label='Right Foot')
    midpoint_plot, = ax_2d.plot([], [], 'kx', markersize=10, label='Midpoint')
    ax_2d.legend()

    # com position plots
    ax_com_x.set_ylabel('X Position (m)')
    ax_com_x.set_title('CoM Position')
    ax_com_x.grid(True)
    com_x_line, = ax_com_x.plot([], [], '-', color='cyan')

    ax_com_y.set_xlabel('Frame')
    ax_com_y.set_ylabel('Y Position (m)')
    ax_com_y.grid(True)
    com_y_line, = ax_com_y.plot([], [], '-', color='brown')

    # zmp error plots
    ax_zmp_x.set_ylabel('X Error (m)')
    ax_zmp_x.set_title('ZMP Error (relative to foot midpoint)')
    ax_zmp_x.grid(True)
    zmp_x_err_line, = ax_zmp_x.plot([], [], '-', color='orange')

    ax_zmp_y.set_xlabel('Frame')
    ax_zmp_y.set_ylabel('Y Error (m)')
    ax_zmp_y.grid(True)
    zmp_y_err_line, = ax_zmp_y.plot([], [], '-', color='purple')

    # set fixed x limits if num_frames is given
    if num_frames is not None:
        ax_com_x.set_xlim(0, num_frames)
        ax_com_y.set_xlim(0, num_frames)
        ax_zmp_x.set_xlim(0, num_frames)
        ax_zmp_y.set_xlim(0, num_frames)

    plot_elements = {
        'fig': fig,
        'ax_2d': ax_2d,
        'ax_com_x': ax_com_x,
        'ax_com_y': ax_com_y,
        'ax_zmp_x': ax_zmp_x,
        'ax_zmp_y': ax_zmp_y,
        'com_plot': com_plot,
        'zmp_plot': zmp_plot,
        'left_foot_plot': left_foot_plot,
        'right_foot_plot': right_foot_plot,
        'midpoint_plot': midpoint_plot,
        'com_x_line': com_x_line,
        'com_y_line': com_y_line,
        'zmp_x_err_line': zmp_x_err_line,
        'zmp_y_err_line': zmp_y_err_line,
        'frames': [],
        'com_x_pos': [],
        'com_y_pos': [],
        'zmp_x_errors': [],
        'zmp_y_errors': [],
        'num_frames': num_frames,
        'window_size': window_size,
    }
    return plot_elements

def update_com_zmp_plot(plot_elements, frame_idx,
                        com_pos, zmp_pos,
                        left_foot_pos, right_foot_pos):
    '''Update plot with new data'''
    midpoint = 0.5 * (left_foot_pos + right_foot_pos)

    # compute zmp error
    zmp_x_err = zmp_pos[0] - midpoint[0]
    zmp_y_err = zmp_pos[1] - midpoint[1]

    # append to arrays
    plot_elements['frames'].append(frame_idx)
    plot_elements['com_x_pos'].append(com_pos[0])
    plot_elements['com_y_pos'].append(com_pos[1])
    plot_elements['zmp_x_errors'].append(zmp_x_err)
    plot_elements['zmp_y_errors'].append(zmp_y_err)

    # trim to window size for rolling view (only if not fixed horizon)
    if plot_elements['num_frames'] is None:
        window_size = plot_elements['window_size']
        if len(plot_elements['frames']) > window_size:
            plot_elements['frames'] = plot_elements['frames'][-window_size:]
            plot_elements['com_x_pos'] = plot_elements['com_x_pos'][-window_size:]
            plot_elements['com_y_pos'] = plot_elements['com_y_pos'][-window_size:]
            plot_elements['zmp_x_errors'] = plot_elements['zmp_x_errors'][-window_size:]
            plot_elements['zmp_y_errors'] = plot_elements['zmp_y_errors'][-window_size:]

    # update 2D plot
    plot_elements['com_plot'].set_data([com_pos[0]], [com_pos[1]])
    plot_elements['zmp_plot'].set_data([zmp_pos[0]], [zmp_pos[1]])
    plot_elements['left_foot_plot'].set_data([left_foot_pos[0]], [left_foot_pos[1]])
    plot_elements['right_foot_plot'].set_data([right_foot_pos[0]], [right_foot_pos[1]])
    plot_elements['midpoint_plot'].set_data([midpoint[0]], [midpoint[1]])

    # auto-scale 2D axes
    all_x = [com_pos[0], zmp_pos[0], left_foot_pos[0], right_foot_pos[0], midpoint[0]]
    all_y = [com_pos[1], zmp_pos[1], left_foot_pos[1], right_foot_pos[1], midpoint[1]]
    margin = 0.1
    plot_elements['ax_2d'].set_xlim(min(all_x) - margin, max(all_x) + margin)
    plot_elements['ax_2d'].set_ylim(min(all_y) - margin, max(all_y) + margin)

    # update com position plots
    plot_elements['com_x_line'].set_data(
        plot_elements['frames'], plot_elements['com_x_pos']
    )
    plot_elements['com_y_line'].set_data(
        plot_elements['frames'], plot_elements['com_y_pos']
    )

    # update zmp error plots
    plot_elements['zmp_x_err_line'].set_data(
        plot_elements['frames'], plot_elements['zmp_x_errors']
    )
    plot_elements['zmp_y_err_line'].set_data(
        plot_elements['frames'], plot_elements['zmp_y_errors']
    )

    # auto-scale axes
    if len(plot_elements['frames']) > 1:
        # only update x limits for rolling window mode
        if plot_elements['num_frames'] is None:
            plot_elements['ax_com_x'].set_xlim(
                plot_elements['frames'][0], plot_elements['frames'][-1]
            )
            plot_elements['ax_com_y'].set_xlim(
                plot_elements['frames'][0], plot_elements['frames'][-1]
            )
            plot_elements['ax_zmp_x'].set_xlim(
                plot_elements['frames'][0], plot_elements['frames'][-1]
            )
            plot_elements['ax_zmp_y'].set_xlim(
                plot_elements['frames'][0], plot_elements['frames'][-1]
            )

        pos_margin = 0.01
        plot_elements['ax_com_x'].set_ylim(
            min(plot_elements['com_x_pos']) - pos_margin,
            max(plot_elements['com_x_pos']) + pos_margin
        )
        plot_elements['ax_com_y'].set_ylim(
            min(plot_elements['com_y_pos']) - pos_margin,
            max(plot_elements['com_y_pos']) + pos_margin
        )

        err_margin = 0.01
        plot_elements['ax_zmp_x'].set_ylim(
            min(plot_elements['zmp_x_errors']) - err_margin,
            max(plot_elements['zmp_x_errors']) + err_margin
        )
        plot_elements['ax_zmp_y'].set_ylim(
            min(plot_elements['zmp_y_errors']) - err_margin,
            max(plot_elements['zmp_y_errors']) + err_margin
        )

    # redraw plots
    plot_elements['fig'].canvas.draw()
    plot_elements['fig'].canvas.flush_events()
    plt.pause(0.01)

def clear_com_zmp_plot(plot_elements):
    '''Clear error history for looping'''
    plot_elements['frames'].clear()
    plot_elements['com_x_pos'].clear()
    plot_elements['com_y_pos'].clear()
    plot_elements['zmp_x_errors'].clear()
    plot_elements['zmp_y_errors'].clear()

def plot_com_zmp_loop(robot_model, data):
    '''Real-time looping plot of ZMP, feet, and error'''
    q_arr = data['q']
    com_arr = data.get('com', None)
    zmp_arr = data.get('zmp', None)
    imu_state_arr = data.get('imu_state', None)
    num_frames = len(q_arr)

    if zmp_arr is None:
        print('Error: no zmp data found')
        return

    plot_elements = create_com_zmp_plot(num_frames)
    print(f'Plotting {num_frames} frames (Ctrl+C to stop)')

    frame_idx = 0
    while True:
        # loop back
        if frame_idx >= num_frames:
            frame_idx = 0
            clear_com_zmp_plot(plot_elements)

        # set robot state
        q = q_arr[frame_idx]
        robot_model._q = q.copy()
        imu_quat = imu_state_arr[frame_idx].quaternion if imu_state_arr is not None else None
        robot_model.update_kinematics(imu_quat=imu_quat)

        # get positions
        com_pos = com_arr[frame_idx] if com_arr is not None else robot_model.dynamics.get_com()
        zmp_pos = zmp_arr[frame_idx]
        left_foot_pos = robot_model.get_frame_position('left_ankle_roll_link')
        right_foot_pos = robot_model.get_frame_position('right_ankle_roll_link')

        update_com_zmp_plot(plot_elements, frame_idx, com_pos, zmp_pos, left_foot_pos, right_foot_pos)

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

    # initialize robot model (no visualizer needed)
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    print(f'Plotting from: {load_path}')

    # run plot loop
    try:
        plot_com_zmp_loop(robot_model, data)
    except KeyboardInterrupt:
        print('Stopping plot...')

    plt.close('all')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot robot recording data')
    parser.add_argument('--load', type=str, required=True,
                        help='Filename to load under data/robot_record/')
    args = parser.parse_args()

    # set load path using command line argument
    load_path = f'./data/robot_record/{args.load}.npz'

    main(load_path)
