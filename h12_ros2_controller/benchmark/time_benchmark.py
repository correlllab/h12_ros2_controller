import os
import time
import argparse
import numpy as np
from tqdm import tqdm
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.arm_controller import ArmController
from h12_ros2_controller.utility.controller_config import load_controller_config, initialize_channel_factory

class TimeBenchmark:
    def __init__(self, arm_controller, target_poses,
                 iter=100):
        '''
        @param arm_controller: your ArmController instance
        @param target_poses: list of np.array (each pose is a vector of joint angles or end-effector pose)
        @param iter: number of iterations to run for timing benchmark
        '''
        self.arm_controller = arm_controller
        self.target_poses = target_poses
        self.neutral_pose = arm_controller.robot_model.zero_q_body
        self.neutral_pose_reduced = arm_controller.robot_model.zero_q_body_reduced

        # parameters
        self.iter = iter
        # timing logs - 2D array: [num_target_poses, num_iters]
        self.iter_times = []

    def reset_to_neutral(self):
        '''Reset arm to the neutral pose'''
        self.arm_controller.robot_model.state_subscriber = None
        self.arm_controller.robot_model._q = self.neutral_pose
        self.arm_controller.update_robot_model()
        self.arm_controller.update_ik_solver()

    def run_benchmark(self):
        '''Run the timing benchmark sequence'''
        print(f'Running timing benchmark for {len(self.target_poses)} target poses, {self.iter} iterations each...')

        for pose_idx, target_pose in enumerate(tqdm(self.target_poses, desc='Target poses')):
            # reset to neutral position
            self.reset_to_neutral()

            # set target pose
            target_pose[3:] = np.deg2rad(target_pose[3:])
            self.arm_controller.left_ee_target_pose = target_pose

            # run iter iterations for this target pose
            pose_times = []
            for _ in tqdm(range(self.iter), desc=f'Pose {pose_idx+1}/{len(self.target_poses)}', leave=False):
                start_time = time.perf_counter()
                self.arm_controller.sim_step_reduced()
                end_time = time.perf_counter()
                pose_times.append(end_time - start_time)

            self.iter_times.append(pose_times)

        # convert to numpy array for easier manipulation
        self.iter_times = np.array(self.iter_times)

    def compute_summary(self):
        '''Compute average, min, and max iteration times'''
        if len(self.iter_times) == 0:
            return {
                'avg_time': 0.0,
                'min_time': 0.0,
                'max_time': 0.0,
                'std_time': 0.0
            }

        # Flatten the 2D array to get all timing measurements
        all_times = self.iter_times.flatten()
        return {
            'avg_time': np.mean(all_times),
            'min_time': np.min(all_times),
            'max_time': np.max(all_times),
            'std_time': np.std(all_times)
        }

    def print_summary(self):
        '''Print timing benchmark results'''
        summary = self.compute_summary()
        total_measurements = len(self.target_poses) * self.iter
        print(f'\nTiming Benchmark Results ({len(self.target_poses)} poses × {self.iter} iterations = {total_measurements} measurements):')
        print(f'Average time: {summary["avg_time"]*1000:.3f} ms')
        print(f'Min time:     {summary["min_time"]*1000:.3f} ms')
        print(f'Max time:     {summary["max_time"]*1000:.3f} ms')
        print(f'Std dev:      {summary["std_time"]*1000:.3f} ms')

    def save_results(self, filename):
        '''Save the benchmark results to a file'''
        # check if directory exists
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        summary = self.compute_summary()
        np.savez(filename,
                 iter_times=self.iter_times,  # 2D array: [num_poses, num_iters]
                 avg_time=summary['avg_time'],
                 min_time=summary['min_time'],
                 max_time=summary['max_time'],
                 std_time=summary['std_time'],
                 num_poses=len(self.target_poses),
                 iter_count=self.iter)
        print(f'Results saved to {filename}')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Timing benchmark script')
    parser.add_argument('--config', type=str, default='default.yaml', help='YAML file name under config/')
    args = parser.parse_args()

    config_data = load_controller_config(args.config)
    initialize_channel_factory(config_data)
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   'assets/h1_2/h1_2_sphere.urdf',
                                   'assets/h1_2/h1_2_sphere_collision.srdf',
                                   dt=0.02,
                                   visualize=True,
                                   config=args.config)

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

    time_benchmark = TimeBenchmark(arm_controller, target_poses, iter=30)
    time_benchmark.run_benchmark()
    time_benchmark.print_summary()
