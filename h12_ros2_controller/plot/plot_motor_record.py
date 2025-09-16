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

if __name__ == '__main__':
    joint_name = 'right_knee_joint'
    filename = f'./data/motor_record/{joint_name}.npz'
    savepath = f'./figures/motor_record'
    plot_recording(filename, savepath)
