import argparse
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

def load_recording_data(filename):
    '''Load recording data from file and return data dictionary'''
    filepath = f'data/control_record/{filename}.npz'
    if not os.path.exists(filepath):
        print(f'Error: File "{filepath}" does not exist.')
        return None

    print(f'Loading data from: {filepath}')
    data = np.load(filepath)
    return {
        'ids': data['ids'],
        'com': data['com'],
        'q': data['q'],
        'dq': data['dq'],
        'tau': data['tau'],
        'q_cmd': data['q_cmd'],
        'dq_cmd': data['dq_cmd'],
        'tau_cmd': data['tau_cmd'],
        'torque_cmd': data['torque_cmd']
    }

def compare_recordings(filenames, savepath):
    '''Compare multiple recordings by plotting them on overlay graphs'''

    # load all data
    all_data = []
    for filename in filenames:
        data = load_recording_data(filename)
        if data is None:
            print(f'Skipping {filename} due to loading error')
            continue
        all_data.append((filename, data))

    if len(all_data) == 0:
        print('No valid data files found!')
        return

    os.makedirs(savepath, exist_ok=True)

    # get joint information from first dataset (assuming all have same structure)
    reference_data = all_data[0][1]
    ids = reference_data['ids']

    # define colors for different recordings
    colors = plt.cm.tab10(np.linspace(0, 1, len(all_data)))

    # create comparison plot for each joint
    for i in range(reference_data['q'].shape[1]):
        joint_id = ids[i]
        joint_name = BODY_JOINTS[joint_id]

        fig, axes = plt.subplots(3, 1, figsize=(14, 8))

        # plot position (q) comparison
        ax = axes[0]
        for j, (filename, data) in enumerate(all_data):
            ax.plot(data['q'][:, i], label=f'{filename} (actual)',
                   color=colors[j], linewidth=2, alpha=0.8)
            ax.plot(data['q_cmd'][:, i], label=f'{filename} (cmd)',
                   color=colors[j], linewidth=2, alpha=0.6, linestyle='--')
        ax.set_title(f'{joint_name} - Position Comparison')
        ax.set_ylabel('Position (rad)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.grid(True, alpha=0.3)

        # plot velocity (dq) comparison
        ax = axes[1]
        for j, (filename, data) in enumerate(all_data):
            ax.plot(data['dq'][:, i], label=f'{filename} (actual)',
                   color=colors[j], linewidth=2, alpha=0.8)
            ax.plot(data['dq_cmd'][:, i], label=f'{filename} (cmd)',
                   color=colors[j], linewidth=2, alpha=0.6, linestyle='--')
        ax.set_title(f'{joint_name} - Velocity Comparison')
        ax.set_ylabel('Velocity (rad/s)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.grid(True, alpha=0.3)

        # plot torque (tau) comparison
        ax = axes[2]
        for j, (filename, data) in enumerate(all_data):
            ax.plot(data['tau'][:, i], label=f'{filename} (actual)',
                   color=colors[j], linewidth=2, alpha=0.8)
            ax.plot(data['tau_cmd'][:, i], label=f'{filename} (tau_cmd)',
                   color=colors[j], linewidth=2, alpha=0.6, linestyle='--')
            ax.plot(data['torque_cmd'][:, i], label=f'{filename} (torque_cmd)',
                   color=colors[j], linewidth=2, alpha=0.4, linestyle=':')
        ax.set_title(f'{joint_name} - Torque Comparison')
        ax.set_xlabel('Time Steps')
        ax.set_ylabel('Torque (Nm)')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(f'{savepath}/{joint_name}_comparison.png')
        plt.close()

    # create center of mass comparison plot
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))

    # define RGB color palettes for each component
    red_colors = plt.cm.Reds(np.linspace(0.4, 1.0, len(all_data)))
    green_colors = plt.cm.Greens(np.linspace(0.4, 1.0, len(all_data)))
    blue_colors = plt.cm.Blues(np.linspace(0.4, 1.0, len(all_data)))

    # X component comparison (Red shades)
    ax = axes[0]
    for j, (filename, data) in enumerate(all_data):
        ax.plot(data['com'][:, 0], label=f'{filename}',
               color=red_colors[j], linewidth=2, alpha=0.8)
    ax.set_title('Center of Mass - X Component Comparison')
    ax.set_ylabel('X Position (m)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True, alpha=0.3)

    # Y component comparison (Green shades)
    ax = axes[1]
    for j, (filename, data) in enumerate(all_data):
        ax.plot(data['com'][:, 1], label=f'{filename}',
               color=green_colors[j], linewidth=2, alpha=0.8)
    ax.set_title('Center of Mass - Y Component Comparison')
    ax.set_ylabel('Y Position (m)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True, alpha=0.3)

    # Z component comparison (Blue shades)
    ax = axes[2]
    for j, (filename, data) in enumerate(all_data):
        ax.plot(data['com'][:, 2], label=f'{filename}',
               color=blue_colors[j], linewidth=2, alpha=0.8)
    ax.set_title('Center of Mass - Z Component Comparison')
    ax.set_xlabel('Time Steps')
    ax.set_ylabel('Z Position (m)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{savepath}/center_of_mass_comparison.png', dpi=150, bbox_inches='tight')
    plt.close()

    # create overall summary plot with all joints for position
    fig, ax = plt.subplots(figsize=(15, 8))
    for j, (filename, data) in enumerate(all_data):
        # Plot RMS of all joint positions over time
        q_rms = np.sqrt(np.mean(data['q']**2, axis=1))
        q_cmd_rms = np.sqrt(np.mean(data['q_cmd']**2, axis=1))
        ax.plot(q_rms, label=f'{filename} (actual)',
               color=colors[j], linewidth=2, alpha=0.8)
        ax.plot(q_cmd_rms, label=f'{filename} (cmd)',
               color=colors[j], linewidth=2, alpha=0.6, linestyle='--')
    ax.set_title('Overall Joint Position RMS Comparison')
    ax.set_xlabel('Time Steps')
    ax.set_ylabel('RMS Position (rad)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{savepath}/overall_position_rms_comparison.png')
    plt.close()

    # create overall summary plot with all joints for velocity
    fig, ax = plt.subplots(figsize=(15, 8))
    for j, (filename, data) in enumerate(all_data):
        # Plot RMS of all joint velocities over time
        dq_rms = np.sqrt(np.mean(data['dq']**2, axis=1))
        dq_cmd_rms = np.sqrt(np.mean(data['dq_cmd']**2, axis=1))
        ax.plot(dq_rms, label=f'{filename} (actual)',
               color=colors[j], linewidth=2, alpha=0.8)
        ax.plot(dq_cmd_rms, label=f'{filename} (cmd)',
               color=colors[j], linewidth=2, alpha=0.6, linestyle='--')
    ax.set_title('Overall Joint Velocity RMS Comparison')
    ax.set_xlabel('Time Steps')
    ax.set_ylabel('RMS Velocity (rad/s)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{savepath}/overall_velocity_rms_comparison.png')
    plt.close()

    # create overall summary plot with all joints for torque
    fig, ax = plt.subplots(figsize=(15, 8))
    for j, (filename, data) in enumerate(all_data):
        # Plot RMS of all joint torques over time
        tau_rms = np.sqrt(np.mean(data['tau']**2, axis=1))
        tau_cmd_rms = np.sqrt(np.mean(data['tau_cmd']**2, axis=1))
        torque_cmd_rms = np.sqrt(np.mean(data['torque_cmd']**2, axis=1))
        ax.plot(tau_rms, label=f'{filename} (actual)',
               color=colors[j], linewidth=2, alpha=0.8)
        ax.plot(tau_cmd_rms, label=f'{filename} (tau_cmd)',
               color=colors[j], linewidth=2, alpha=0.6, linestyle='--')
        ax.plot(torque_cmd_rms, label=f'{filename} (torque_cmd)',
               color=colors[j], linewidth=2, alpha=0.4, linestyle=':')
    ax.set_title('Overall Joint Torque RMS Comparison')
    ax.set_xlabel('Time Steps')
    ax.set_ylabel('RMS Torque (Nm)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{savepath}/overall_torque_rms_comparison.png')
    plt.close()

    print(f'Comparison plots saved to: {savepath}')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare multiple control recording data')
    parser.add_argument('filenames', nargs='+', help='Recording filenames to compare (e.g., record1 record2 record3)')
    parser.add_argument('--save', '-s', type=str, help='Output directory name (default: comparison_<first_filename>)')
    args = parser.parse_args()

    # set save path
    if args.save:
        savepath = f'figures/control_record/{args.save}'
    else:
        savepath = f'figures/control_record/comparison_{args.filenames[0]}'

    compare_recordings(args.filenames, savepath)
