import argparse
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.arm_controller import ArmController
from h12_ros2_controller.benchmark.precision_benchmark import PrecisionBenchmark
from h12_ros2_controller.utility.controller_config import load_controller_config, initialize_channel_factory

def run_benchmark(solver, filepath, mode, config):
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   'assets/h1_2/h1_2_sphere.urdf',
                                   'assets/h1_2/h1_2_sphere_collision.srdf',
                                   visualize=False,
                                   config=config)
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
    benchmark.save_results(f'data/compare_solver/{filepath}/{solver}.npz')

    # reset to neutral position before shutdown
    if mode == 'pin':
        benchmark.reset_to_neutral_pin()
    elif mode == 'real':
        benchmark.reset_to_neutral_real()
    arm_controller.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Solver Benchmark Script')
    parser.add_argument('--save', type=str, required=True,
                        help='Folder name to save benchmark results in data/compare_solver/')
    parser.add_argument('--config', type=str, default='debug.yaml', help='YAML file name under config/')

    # Mutually exclusive group for mode selection
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('--real', action='store_true',
                            help='Run benchmark with real robot routine')
    mode_group.add_argument('--pin', action='store_true',
                            help='Run benchmark with pure Pinocchio kinematics')

    args = parser.parse_args()

    # Set mode based on arguments
    mode = 'real' if args.real else 'pin'

    solvers = ['osqp', 'proxqp', 'daqp', 'quadprog']

    config = load_controller_config(args.config)
    initialize_channel_factory(config)
    for solver in solvers:
        run_benchmark(solver, args.save, mode, config)
