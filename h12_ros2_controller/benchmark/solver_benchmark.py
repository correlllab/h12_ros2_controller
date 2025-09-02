import numpy as np

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
                                   dt=0.03,
                                   v_lim=1.0,
                                   w_lim=2.0,
                                   dq_lim=2.0,
                                   d_min=0.02,
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

if __name__ == '__main__':
    solvers = ['osqp', 'proxqp', 'daqp', 'quadprog']
    filepath = 'compare_solver_onrobot'
    mode = 'real'

    ChannelFactoryInitialize()
    for solver in solvers:
        run_benchmark(solver, filepath, mode)
