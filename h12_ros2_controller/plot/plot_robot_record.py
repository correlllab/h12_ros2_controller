import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel

def create_zmp_plot(num_frames=None, window_size=500):
    '''Create plot figure and return plot elements.
    num_frames: if set, use fixed horizon (for replay). if None, use rolling window.
    window_size: rolling window size when num_frames is None.
    '''
    plt.ion()
    fig = plt.figure(figsize=(12, 6))
    ax_2d = fig.add_subplot(1, 2, 1)
    ax_x = fig.add_subplot(2, 2, 2)
    ax_y = fig.add_subplot(2, 2, 4)

    # 2D position plot
    ax_2d.set_xlabel('X (m)')
    ax_2d.set_ylabel('Y (m)')
    ax_2d.set_title('ZMP and Foot Positions')
    ax_2d.set_aspect('equal')
    ax_2d.grid(True)
    zmp_plot, = ax_2d.plot([], [], 'go', markersize=10, label='ZMP')
    left_foot_plot, = ax_2d.plot([], [], 'bs', markersize=10, label='Left Foot')
    right_foot_plot, = ax_2d.plot([], [], 'rs', markersize=10, label='Right Foot')
    midpoint_plot, = ax_2d.plot([], [], 'kx', markersize=10, label='Midpoint')
    ax_2d.legend()

    # error plots
    ax_x.set_ylabel('X Error (m)')
    ax_x.set_title('ZMP Error (relative to foot midpoint)')
    ax_x.grid(True)
    x_err_line, = ax_x.plot([], [], '-', color='orange')

    ax_y.set_xlabel('Frame')
    ax_y.set_ylabel('Y Error (m)')
    ax_y.grid(True)
    y_err_line, = ax_y.plot([], [], '-', color='purple')

    # set fixed x limits if num_frames is given
    if num_frames is not None:
        ax_x.set_xlim(0, num_frames)
        ax_y.set_xlim(0, num_frames)

    plot_elements = {
        'fig': fig,
        'ax_2d': ax_2d,
        'ax_x': ax_x,
        'ax_y': ax_y,
        'zmp_plot': zmp_plot,
        'left_foot_plot': left_foot_plot,
        'right_foot_plot': right_foot_plot,
        'midpoint_plot': midpoint_plot,
        'x_err_line': x_err_line,
        'y_err_line': y_err_line,
        'frames': [],
        'x_errors': [],
        'y_errors': [],
        'num_frames': num_frames,
        'window_size': window_size,
    }
    return plot_elements

def update_zmp_plot(plot_elements, frame_idx, zmp_pos, left_foot_pos, right_foot_pos):
    '''Update plot with new data'''
    midpoint = 0.5 * (left_foot_pos + right_foot_pos)

    # compute error
    x_err = zmp_pos[0] - midpoint[0]
    y_err = zmp_pos[1] - midpoint[1]

    # append to arrays
    plot_elements['frames'].append(frame_idx)
    plot_elements['x_errors'].append(x_err)
    plot_elements['y_errors'].append(y_err)

    # trim to window size for rolling view (only if not fixed horizon)
    if plot_elements['num_frames'] is None:
        window_size = plot_elements['window_size']
        if len(plot_elements['frames']) > window_size:
            plot_elements['frames'] = plot_elements['frames'][-window_size:]
            plot_elements['x_errors'] = plot_elements['x_errors'][-window_size:]
            plot_elements['y_errors'] = plot_elements['y_errors'][-window_size:]

    # update 2D plot
    plot_elements['zmp_plot'].set_data([zmp_pos[0]], [zmp_pos[1]])
    plot_elements['left_foot_plot'].set_data([left_foot_pos[0]], [left_foot_pos[1]])
    plot_elements['right_foot_plot'].set_data([right_foot_pos[0]], [right_foot_pos[1]])
    plot_elements['midpoint_plot'].set_data([midpoint[0]], [midpoint[1]])

    # auto-scale 2D axes
    all_x = [zmp_pos[0], left_foot_pos[0], right_foot_pos[0], midpoint[0]]
    all_y = [zmp_pos[1], left_foot_pos[1], right_foot_pos[1], midpoint[1]]
    margin = 0.1
    plot_elements['ax_2d'].set_xlim(min(all_x) - margin, max(all_x) + margin)
    plot_elements['ax_2d'].set_ylim(min(all_y) - margin, max(all_y) + margin)

    # update error plots
    plot_elements['x_err_line'].set_data(plot_elements['frames'], plot_elements['x_errors'])
    plot_elements['y_err_line'].set_data(plot_elements['frames'], plot_elements['y_errors'])

    # auto-scale error axes
    if len(plot_elements['frames']) > 1:
        # only update x limits for rolling window mode
        if plot_elements['num_frames'] is None:
            plot_elements['ax_x'].set_xlim(plot_elements['frames'][0], plot_elements['frames'][-1])
            plot_elements['ax_y'].set_xlim(plot_elements['frames'][0], plot_elements['frames'][-1])
        err_margin = 0.01
        plot_elements['ax_x'].set_ylim(min(plot_elements['x_errors']) - err_margin, max(plot_elements['x_errors']) + err_margin)
        plot_elements['ax_y'].set_ylim(min(plot_elements['y_errors']) - err_margin, max(plot_elements['y_errors']) + err_margin)

    plot_elements['fig'].canvas.draw()
    plot_elements['fig'].canvas.flush_events()
    plt.pause(0.01)

def clear_zmp_plot(plot_elements):
    '''Clear error history for looping'''
    plot_elements['frames'].clear()
    plot_elements['x_errors'].clear()
    plot_elements['y_errors'].clear()

def plot_zmp_loop(robot_model, data):
    '''Real-time looping plot of ZMP, feet, and error'''
    q_arr = data['q']
    zmp_arr = data.get('zmp', None)
    imu_state_arr = data.get('imu_state', None)
    num_frames = len(q_arr)

    if zmp_arr is None:
        print('Error: no zmp data found')
        return

    plot_elements = create_zmp_plot(num_frames)
    print(f'Plotting {num_frames} frames (Ctrl+C to stop)')

    frame_idx = 0
    while True:
        # loop back
        if frame_idx >= num_frames:
            frame_idx = 0
            clear_zmp_plot(plot_elements)

        # set robot state
        q = q_arr[frame_idx]
        robot_model._q = q.copy()
        imu_quat = imu_state_arr[frame_idx].quaternion if imu_state_arr is not None else None
        robot_model.update_kinematics(imu_quat=imu_quat)

        # get positions
        zmp_pos = zmp_arr[frame_idx]
        left_foot_pos = robot_model.get_frame_position('left_ankle_roll_link')
        right_foot_pos = robot_model.get_frame_position('right_ankle_roll_link')

        update_zmp_plot(plot_elements, frame_idx, zmp_pos, left_foot_pos, right_foot_pos)

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
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    print(f'Plotting from: {load_path}')

    # run plot loop
    try:
        plot_zmp_loop(robot_model, data)
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
