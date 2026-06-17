import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.dds_init import init_channel_factory_from_env
from h12_ros2_controller.utility.path_definition import URDF_HANDLESS_PATH


def create_overlay_plot(time_s, joint_name, joint_data, quantity, output_path):
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    components = [('x', 0), ('y', 1), ('z', 2)]

    damped = np.asarray(joint_data[f'{quantity}_damped'])
    raw = np.asarray(joint_data[f'{quantity}_raw'])
    n = min(len(time_s), len(damped), len(raw))
    if n == 0:
        plt.close(fig)
        return

    t = np.asarray(time_s[:n])
    damped = damped[:n]
    raw = raw[:n]

    damped_mag = np.linalg.norm(damped, axis=1)
    raw_mag = np.linalg.norm(raw, axis=1)
    axes[0].plot(t, damped_mag, label='damped', linewidth=1.6)
    axes[0].plot(t, raw_mag, label='raw', linewidth=1.0, linestyle='--')

    for axis_idx, (axis_name, comp_idx) in enumerate(components, start=1):
        axes[axis_idx].plot(
            t,
            damped[:, comp_idx],
            label='damped',
            linewidth=1.6,
        )
        axes[axis_idx].plot(
            t,
            raw[:, comp_idx],
            label='raw',
            linewidth=1.0,
            linestyle='--',
        )
        axes[axis_idx].set_ylabel(axis_name)

    axes[0].set_ylabel('magnitude')
    axes[0].set_title(f'{joint_name} {quantity}: damped vs raw')
    axes[-1].set_xlabel('time [s]')

    for ax in axes:
        ax.grid(True, linestyle=':', alpha=0.5)

    axes[0].legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def main(save_folder):
    init_channel_factory_from_env()
    joint_names = ['left_wrist_yaw_link', 'right_wrist_yaw_link']
    robot_model = RobotModel(URDF_HANDLESS_PATH, handless=True)
    robot_model.init_visualizer()
    robot_model.config_visualizer(show_sensors=True, show_com=False, show_zmp=False)
    robot_model.init_subscriber()

    save_dir = os.path.join('figures', 'force', save_folder)
    os.makedirs(save_dir, exist_ok=True)

    recorded = {
        joint_name: {
            'force_damped': [],
            'force_raw': [],
            'torque_damped': [],
            'torque_raw': [],
        }
        for joint_name in joint_names
    }
    time_s = []
    start_time = time.time()
    frame_idx = 0

    print(f'Recording joints: {joint_names}')
    print('Press Ctrl+C to stop and save plots')

    try:
        while True:
            robot_model.update_kinematics()
            robot_model.update_visualizer()

            now = time.time() - start_time

            for joint_name in joint_names:
                wrench_damped = robot_model.get_frame_wrench(joint_name)
                wrench_raw = robot_model.get_frame_wrench_raw(joint_name)

                recorded[joint_name]['force_damped'].append(wrench_damped[:3].copy())
                recorded[joint_name]['force_raw'].append(wrench_raw[:3].copy())
                recorded[joint_name]['torque_damped'].append(wrench_damped[3:].copy())
                recorded[joint_name]['torque_raw'].append(wrench_raw[3:].copy())

            time_s.append(now)

            if frame_idx % 100 == 0:
                jac = robot_model.get_frame_jacobian(joint_names[0])
                cond_num = np.linalg.cond(jac)
                print(f'frame={frame_idx:6d} cond({joint_names[0]})={cond_num:.2f}')

            frame_idx += 1
            time.sleep(0.01)

    except KeyboardInterrupt:
        print('\nCtrl+C received, saving plots')

    finally:
        robot_model.shutdown()

    if len(time_s) == 0:
        print('No samples recorded, skip plotting')
        return

    output_files = []
    for joint_name in joint_names:
        force_plot_path = os.path.join(save_dir, f'{joint_name}_force.png')
        torque_plot_path = os.path.join(save_dir, f'{joint_name}_torque.png')
        create_overlay_plot(time_s, joint_name, recorded[joint_name], 'force', force_plot_path)
        create_overlay_plot(time_s, joint_name, recorded[joint_name], 'torque', torque_plot_path)
        output_files.append(force_plot_path)
        output_files.append(torque_plot_path)

    for file_path in output_files:
        print(f'Saved plot: {file_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Record damped/raw frame wrench estimates')
    parser.add_argument(
        '--save',
        required=True,
        help='Save folder name under figures/force/<folder_name>',
    )
    args = parser.parse_args()

    main(args.save)
