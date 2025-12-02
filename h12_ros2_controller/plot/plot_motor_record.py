import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

def plot_recording(filename, savepath):
    data = np.load(filename)
    joint_name = data['joint_name']
    q = data['q']  # joint positions
    dq = data['dq']  # joint velocities
    tau = data['tau']  # joint torques
    q_cmd = data['q_cmd']  # commanded joint positions
    dq_cmd = data['dq_cmd']  # commanded joint velocities
    tau_cmd = data['tau_cmd'] # commanded torques
    torque_cmd = data['torque_cmd'] # total torque based on equation

    # create folder
    os.makedirs(savepath, exist_ok=True)

    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(q, label='q')
    plt.plot(q_cmd, '--', label='q_cmd')
    plt.title(f'{joint_name} - Position')
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(dq, label='dq')
    plt.plot(dq_cmd, '--', label='dq_cmd')
    plt.title('Velocity')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(tau, label='tau')
    plt.plot(tau_cmd, '--', label='tau_cmd')
    plt.plot(torque_cmd, '-.', label='torque_cmd')
    plt.title('Torque')
    plt.xlabel('Time')
    plt.ylabel('Torque')
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig(f'{savepath}/{joint_name}.png')
    plt.close()

def plot_comparison(joint_name, left_filename, right_filename, savepath):
    left_data = np.load(left_filename)
    right_data = np.load(right_filename)

    q_left = left_data['q']
    dq_left = left_data['dq']
    tau_left = left_data['tau']
    q_cmd_left = left_data['q_cmd']
    dq_cmd_left = left_data['dq_cmd']
    tau_cmd_left = left_data['tau_cmd']
    torque_cmd_left = left_data['torque_cmd']

    q_right = right_data['q']
    dq_right = right_data['dq']
    tau_right = right_data['tau']
    q_cmd_right = right_data['q_cmd']
    dq_cmd_right = right_data['dq_cmd']
    tau_cmd_right = right_data['tau_cmd']
    torque_cmd_right = right_data['torque_cmd']

    # create folder
    os.makedirs(savepath, exist_ok=True)

    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(q_left, label='Left q')
    plt.plot(q_right, label='Right q')
    plt.plot(q_cmd_left, '--', label='Left q_cmd')
    plt.plot(q_cmd_right, '--', label='Right q_cmd')
    plt.title('Joint Position Comparison')
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(dq_left, label='Left dq')
    plt.plot(dq_right, label='Right dq')
    plt.plot(dq_cmd_left, '--', label='Left dq_cmd')
    plt.plot(dq_cmd_right, '--', label='Right dq_cmd')
    plt.title('Joint Velocity Comparison')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(tau_left, label='Left tau')
    plt.plot(tau_right, label='Right tau')
    plt.plot(tau_cmd_left, '--', label='Left tau_cmd')
    plt.plot(tau_cmd_right, '--', label='Right tau_cmd')
    plt.plot(torque_cmd_left, '-.', label='Left torque_cmd')
    plt.plot(torque_cmd_right, '-.', label='Right torque_cmd')
    plt.title('Joint Torque Comparison')
    plt.xlabel('Time')
    plt.ylabel('Torque')
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig(f'{savepath}/comparison_{joint_name}.png')
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot motor recording data')
    parser.add_argument('folder',
                        help='Recording folder in data/motor_record/')
    args = parser.parse_args()

    folder = args.folder
    savepath = f'figures/motor_record/{folder}'

    # filename = f'data/motor_record/{folder}/left_wrist_yaw_joint.npz'
    # plot_recording(filename, savepath)

    joint_name_list = [
        # 'hip_yaw_joint', 'hip_pitch_joint', 'hip_roll_joint',
        # 'knee_joint', 'ankle_pitch_joint', 'ankle_roll_joint',
        'shoulder_pitch_joint', 'shoulder_roll_joint', 'shoulder_yaw_joint',
        'elbow_joint', 'wrist_roll_joint', 'wrist_pitch_joint', 'wrist_yaw_joint'
    ]

    for variant in ['free', 'static', 'all']:
        # single torso joint
        # plot_recording(f'data/motor_record/{folder}/torso_joint_{variant}.npz', f'{savepath}/{variant}')
        for joint_name in joint_name_list:
            left_filename = f'data/motor_record/{folder}/left_{joint_name}_{variant}.npz'
            right_filename = f'data/motor_record/{folder}/right_{joint_name}_{variant}.npz'
            plot_recording(left_filename, f'{savepath}/{variant}')
            plot_recording(right_filename, f'{savepath}/{variant}')
            plot_comparison(joint_name, left_filename, right_filename, f'{savepath}/{variant}')
