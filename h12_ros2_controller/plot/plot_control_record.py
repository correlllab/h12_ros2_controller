import argparse
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

def plot_recording(filepath, savepath):
    print(f'Loading data from: {filepath}')
    data = np.load(filepath)
    print(f'{dir(data)=}')
    ids = data['ids']  # motor ids
    com = data['com']  # center of mass
    q = data['q']  # joint positions
    dq = data['dq']  # joint velocities
    tau = data['tau']  # joint torques
    q_cmd = data['q_cmd']  # commanded joint positions
    dq_cmd = data['dq_cmd']  # commanded joint velocities
    tau_cmd = data['tau_cmd']  # commanded joint torques
    torque_cmd = data['torque_cmd']  # commanded overall torque
    l = [
            ('ids', ids),
            ('com', com),
            ('q', q),
            ('dq', dq),
            ('tau', tau),
            ('q_cmd', q_cmd),
            ('dq_cmd', dq_cmd),
            ('tau_cmd', tau_cmd),
            ('torque_cmd', torque_cmd)
        ]
    for name, d in l:
        print(f'{name}: {len(d)}')
        print(d)

    # create folder
    os.makedirs(savepath, exist_ok=True)

    # plot center of mass
    fig, axes = plt.subplots(3, 1, figsize=(12, 8))

    # X component
    axes[0].plot(com[:, 0], 'r-', linewidth=2)
    axes[0].set_title('Center of Mass - X Component')
    axes[0].set_ylabel('X Position (m)')
    axes[0].grid(True)

    # Y component
    axes[1].plot(com[:, 1], 'g-', linewidth=2)
    axes[1].set_title('Center of Mass - Y Component')
    axes[1].set_ylabel('Y Position (m)')
    axes[1].grid(True)

    # Z component
    axes[2].plot(com[:, 2], 'b-', linewidth=2)
    axes[2].set_title('Center of Mass - Z Component')
    axes[2].set_xlabel('Time Steps')
    axes[2].set_ylabel('Z Position (m)')
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig(f'{savepath}/center_of_mass.png')
    plt.close()

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

    print(f'Plots saved to: {savepath}')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot control recording data')
    parser.add_argument('filename',
                        help='Recording filename in data/control_record/')
    args = parser.parse_args()

    # check if file exists
    filepath = f'data/control_record/{args.filename}.npz'
    if not os.path.exists(filepath):
        print(f"Error: File '{filepath}' does not exist.")
        sys.exit(1)
    # set save path
    savepath = f'figures/control_record/{args.filename}'

    plot_recording(filepath, savepath)
