import numpy as np
import matplotlib.pyplot as plt

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
                                   dt=0.02,
                                   vlim=1.0,
                                   wlim=2.0,
                                   dmin=0.01,
                                   visualize=False)
    arm_controller.solver = solver

    target_poses = [
        np.array([0.3, 0.2, 0.1, 0, 0, 0]),
        np.array([0.3, 0.2, 0.2, 0, 0, 0]),
        np.array([0.3, 0.2, 0.3, 0, 0, 0]),
        np.array([0.3, 0.3, 0.1, 0, 0, 0]),
        np.array([0.3, 0.3, 0.2, 0, 0, 0]),
        np.array([0.3, 0.3, 0.3, 0, 0, 0]),
        np.array([0.4, 0.4, 0.4, 0, 0, 0]),
        np.array([0.4, 0.4, 0.4, 0, 45, 0]),
        np.array([0.4, 0.4, 0.4, 0, 60, 0]),
        np.array([0.4, 0.4, 0.4, 0, 90, 0]),
    ]

    benchmark = PrecisionBenchmark(arm_controller, target_poses)
    benchmark.run_benchmark(mode)
    benchmark.save_results(f'{filepath}/{solver}.npz')

    # reset to neutral position before shutdown
    benchmark.reset_to_neutral_real()
    arm_controller.shutdown()

def plot_benchmark(solver):
    data = np.load(f'data/compare_solver/{solver}.npz')
    traj_log = data['traj_log']
    linear_error_log = data['linear_error_log']
    angular_error_log = data['angular_error_log']

    # TODO plot error and trajectory

if __name__ == '__main__':
    ChannelFactoryInitialize()
    for solver in ['osqp', 'proxqp', 'daqp', 'quadprog']:
        run_benchmark(solver, 'data/compare_solver_real', mode='real')
