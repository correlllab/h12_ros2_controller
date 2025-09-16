import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

def plot_recording(filename, savepath):
    data = np.load(filename)
    ids = data['ids']  # motor ids
    q = data['q']  # joint positions
    dq = data['dq']  # joint velocities
    tau = data['tau']  # joint torques
    q_cmd = data['q_cmd']  # commanded joint positions
    dq_cmd = data['dq_cmd']  # commanded joint velocities
    tau_cmd = data['tau_cmd']  # commanded joint torques
    torque_cmd = data['torque_cmd']  # commanded overall torque

    # create folder
    os.makedirs(savepath, exist_ok=True)

    # create graph for each motor
    for i in range(q.shape[1]):
        joint_id = ids[i]
        joint_name = BODY_JOINTS[joint_id]

        plt.figure()
        plt.plot(q[:, i], label='q')
        plt.plot(dq[:, i], label='dq')
        plt.plot(tau[:, i], label='tau')
        plt.plot(q_cmd[:, i], label='q_cmd')
        plt.plot(dq_cmd[:, i], label='dq_cmd')
        plt.plot(tau_cmd[:, i], label='tau_cmd')
        plt.plot(torque_cmd[:, i], label='torque_cmd')
        plt.title(joint_name)
        plt.xlabel('Time')
        plt.ylabel('Value')
        plt.legend()
        plt.grid()
        plt.savefig(f'{savepath}/{joint_name}.png')
        plt.close()

if __name__ == '__main__':
    filename = 'data/control_record/record_acc_limit.npz'
    savepath = 'figures/control_record/record_acc_limit'
    plot_recording(filename, savepath)
