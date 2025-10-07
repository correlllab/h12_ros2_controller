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

        plt.figure(figsize=(12, 8))

        # Top panel: Position (q)
        plt.subplot(3, 1, 1)
        plt.plot(q[:, i], label='q', linewidth=2)
        plt.plot(q_cmd[:, i], '--', label='q_cmd', linewidth=2)
        plt.title(f'{joint_name} - Position')
        plt.ylabel('Position (rad)')
        plt.legend()
        plt.grid()

        # Middle panel: Velocity (dq)
        plt.subplot(3, 1, 2)
        plt.plot(dq[:, i], label='dq', linewidth=2)
        plt.plot(dq_cmd[:, i], '--', label='dq_cmd', linewidth=2)
        plt.title('Velocity')
        plt.ylabel('Velocity (rad/s)')
        plt.legend()
        plt.grid()

        # Bottom panel: Torque
        plt.subplot(3, 1, 3)
        plt.plot(tau[:, i], label='tau', linewidth=2)
        plt.plot(tau_cmd[:, i], '--', label='tau_cmd', linewidth=2)
        plt.plot(torque_cmd[:, i], '-.', label='torque_cmd', linewidth=2)
        plt.title('Torque')
        plt.xlabel('Time Steps')
        plt.ylabel('Torque (Nm)')
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.savefig(f'{savepath}/{joint_name}.png')
        plt.close()

if __name__ == '__main__':
    path = 'record_real'
    filename = f'data/control_record/{path}.npz'
    savepath = f'figures/control_record/{path}'
    plot_recording(filename, savepath)
