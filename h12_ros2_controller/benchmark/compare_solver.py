import argparse
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.arm_controller import ArmController
from h12_ros2_controller.benchmark.precision_benchmark import PrecisionBenchmark

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

def run_benchmark(solver, filepath, mode):
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   'assets/h1_2/h1_2_sphere.urdf',
                                   'assets/h1_2/h1_2_sphere_collision.srdf',
                                   dt=0.04,
                                   vlim=0.8,
                                   wlim=1.5,
                                   dmin=0.05,
                                   visualize=False)
    arm_controller.solver = solver

    target_poses = [
        np.array([0.3, 0.2, 0.2, 0, 0, 0]),
        np.array([0.3, 0.2, 0.3, 0, 0, 0]),
        np.array([0.3, 0.2, 0.4, 0, 0, 0]),
        np.array([0.3, 0.3, 0.2, 0, 0, 0]),
        np.array([0.3, 0.3, 0.3, 0, 0, 0]),
        np.array([0.3, 0.3, 0.4, 0, 0, 0]),
        np.array([0.3, 0.4, 0.2, 0, 0, 0]),
        np.array([0.3, 0.4, 0.3, 0, 0, 0]),
        np.array([0.3, 0.4, 0.4, 0, 0, 0]),
        np.array([0.4, 0.4, 0.4, 0, 0, 0]),
        np.array([0.4, 0.4, 0.4, 0, 45, 0]),
        np.array([0.4, 0.4, 0.4, 0, 60, 0]),
        np.array([0.4, 0.4, 0.4, 0, 90, 0]),
    ]

    benchmark = PrecisionBenchmark(arm_controller, target_poses)
    benchmark.run_benchmark(mode)
    benchmark.save_results(f'data/{filepath}/{solver}.npz')

    # reset to neutral position before shutdown
    if mode == 'pin':
        benchmark.reset_to_neutral_pin()
    elif mode == 'real':
        benchmark.reset_to_neutral_real()
    arm_controller.shutdown()

def plot_benchmark(solver, filepath):
    data = np.load(f'data/{filepath}/{solver}.npz')
    linear_error_log = data['linear_error_log']  # shape: (N, T)
    angular_error_log = data['angular_error_log']  # shape: (N, T)

    # make dir
    os.makedirs(f'figures/{filepath}', exist_ok=True)

    # plot linear error
    plt.figure()
    for i in range(linear_error_log.shape[0]):
        plt.plot(linear_error_log[i], label=f'Target {i+1}')
    plt.title(f'Linear Error - {solver}')
    plt.xlabel('Time Step')
    plt.ylabel('Linear Error')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'figures/{filepath}/{solver}_linear_error.png')
    plt.close()

    # plot angular error
    plt.figure()
    for i in range(angular_error_log.shape[0]):
        plt.plot(angular_error_log[i], label=f'Target {i+1}')
    plt.title(f'Angular Error - {solver}')
    plt.xlabel('Time Step')
    plt.ylabel('Angular Error')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'figures/{filepath}/{solver}_angular_error.png')
    plt.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare solvers for arm controller.')
    parser.add_argument('--run', action='store_true', help='Run benchmark')
    parser.add_argument('--plot', action='store_true', help='Plot benchmark results')
    args = parser.parse_args()

    solvers = ['osqp', 'proxqp', 'daqp', 'quadprog']
    filepath = 'compare_solver_onrobot'
    mode = 'real'

    if args.run:
        ChannelFactoryInitialize()
        for solver in solvers:
            run_benchmark(solver, filepath, mode)
    if args.plot:
        for solver in solvers:
            plot_benchmark(solver, filepath)
