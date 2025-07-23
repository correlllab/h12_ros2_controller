import os
import time
import numpy as np
from tqdm import tqdm

class PrecisionBenchmark:
    def __init__(self, arm_controller, target_poses, time=5.0):
        '''
        @param arm_controller: your ArmController instance
        @param target_poses: list of np.array (each pose is a vector of joint angles or end-effector pose)
        '''
        self.time = time
        self.arm_controller = arm_controller
        self.target_poses = target_poses
        self.neutral_pose = arm_controller.robot_model.zero_q

        self.traj_log = []
        self.linear_error_log = []
        self.angular_error_log = []

    def reset_to_neutral_pin(self):
        '''Reset arm to the neutral pose'''
        self.arm_controller.robot_model._q = self.neutral_pose
        self.arm_controller.robot_model.update_kinematics()

    def reset_to_neutral_real(self):
        '''Reset arm to the neutral pose for Mujoco'''
        for _ in tqdm(range(300)):
            self.arm_controller.goto_configuration(self.neutral_pose)
            time.sleep(0.02)

    def run_benchmark(self, mode):
        '''Run the benchmark sequence'''
        for target_pose in tqdm(self.target_poses):
            if mode == 'pin':
                self.reset_to_neutral_pin()
            elif mode == 'real':
                self.reset_to_neutral_real()
            traj_arr, linear_error_arr, angular_error_arr = self.track_trajectory(target_pose, mode)
            self.traj_log.append(traj_arr)
            self.linear_error_log.append(linear_error_arr)
            self.angular_error_log.append(angular_error_arr)

    def track_trajectory(self, target_pose, mode):
        '''
        Track while moving to target_pose.
        Returns trajectory and errors.
        '''
        traj_arr = []
        linear_error_arr = []
        angular_error_arr = []

        # set target pose
        target_pose[3:] = np.deg2rad(target_pose[3:])
        self.arm_controller.left_ee_target_pose = target_pose

        step = int(self.time / self.arm_controller.dt)
        for _ in tqdm(range(step)):
            frame_start_time = time.time()
            if mode == 'pin':
                self.arm_controller.sim_dual_arm_step()
            elif mode == 'real':
                self.arm_controller.control_dual_arm_step()

            # get current pose
            current_pose = self.arm_controller.left_ee_pose
            # get error
            linear_error = np.linalg.norm(self.arm_controller.left_ee_error[:3])
            angular_error = np.linalg.norm(self.arm_controller.left_ee_error[3:])
            # record
            traj_arr.append(current_pose)
            linear_error_arr.append(linear_error)
            angular_error_arr.append(angular_error)

            time.sleep(max(self.arm_controller.dt - (time.time() - frame_start_time), 0.0))

        return np.array(traj_arr), np.array(linear_error_arr), np.array(angular_error_arr)

    def compute_summary(self):
        '''Compute RMS or max errors'''
        all_linear_errors = np.concatenate(self.linear_error_log)
        all_angular_errors = np.concatenate(self.angular_error_log)
        rms_linear = np.sqrt(np.mean(np.square(all_linear_errors), axis=0))
        max_linear = np.max(np.abs(all_linear_errors), axis=0)
        rms_angular= np.sqrt(np.mean(np.square(all_angular_errors), axis=0))
        max_angular= np.max(np.abs(all_angular_errors), axis=0)
        return {
            'rms_linear': rms_linear,
            'max_linear': max_linear,
            'rms_angular': rms_angular,
            'max_angular': max_angular
        }

    def save_results(self, filename):
        '''Save the benchmark results to a file'''
        # check if directory exists
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        np.savez(filename,
                 traj_log=self.traj_log,
                 linear_error_log=self.linear_error_log,
                 angular_error_log=self.angular_error_log)
        print(f"Results saved to {filename}")
