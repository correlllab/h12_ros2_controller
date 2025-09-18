import os
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

    # create folder
    os.makedirs(savepath, exist_ok=True)
    plt.figure()
    plt.plot(q, label='q')
    plt.plot(dq, label='dq')
    plt.plot(tau, label='tau')
    plt.plot(q_cmd, label='q_cmd')
    plt.plot(dq_cmd, label='dq_cmd')
    plt.legend()
    plt.title(joint_name)
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.grid()
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

    q_right = right_data['q']
    dq_right = right_data['dq']
    tau_right = right_data['tau']
    q_cmd_right = right_data['q_cmd']
    dq_cmd_right = right_data['dq_cmd']

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
    plt.title('Joint Torque Comparison')
    plt.xlabel('Time')
    plt.ylabel('Torque')
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig(f'{savepath}/{joint_name}_comparison.png')
    plt.close()


if __name__ == '__main__':
    joint_name_list = ['hip_yaw_joint', 'hip_pitch_joint', 'hip_roll_joint',
                       'knee_joint', 'ankle_pitch_joint', 'ankle_roll_joint']

    for joint_name in joint_name_list:
        left_filename = f'./data/motor_record/left_{joint_name}.npz'
        right_filename = f'./data/motor_record/right_{joint_name}.npz'
        savepath = f'./figures/motor_record'
        plot_recording(left_filename, savepath)
        plot_recording(right_filename, savepath)
        plot_comparison(joint_name, left_filename, right_filename, savepath)
