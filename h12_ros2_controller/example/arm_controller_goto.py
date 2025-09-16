import time
import argparse
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.arm_controller import ArmController
from h12_ros2_controller.utility.named_configs import NAMED_CONFIGS

def input_keyword_or_poses():
    '''
    Ask user for keyword command or manual pose input
    Returns (keyword, left_pose, right_pose) tuple
    If keyword is provided, poses will be None
    If no keyword, poses will be provided
    '''
    print(f'Available keywords: {list(NAMED_CONFIGS.keys())}')

    choice = input('Enter keyword (or press Enter for manual poses): ').strip()

    if choice:
        # user entered a keyword
        return choice, None, None
    else:
        # user wants manual pose input
        left_pose = input_pose('left')
        right_pose = input_pose('right')
        return '', left_pose, right_pose

def input_pose(side):
    print(f'{side.capitalize()} end-effector pose...')

    while True:
        input_pose = input('Enter x y z roll pitch yaw (separated by space): ')
        parts = input_pose.strip().split()

        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue
        try:
            values = [float(val) for val in parts]
            # degrees to radian
            values[3:] = np.deg2rad(values[3:])
            return values
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')
            continue


def main(timeout=10.0,
         threshold_linear=5e-3,
         threshold_angular=2e-2,
         use_sport_mode=False):
    ChannelFactoryInitialize()
    # initialize arm controller
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   'assets/h1_2/h1_2_sphere.urdf',
                                   'assets/h1_2/h1_2_sphere_collision.srdf',
                                   dt=0.03,
                                   v_lim=1.0,
                                   w_lim=2.0,
                                   dq_lim=2.0,
                                   d_min=0.02,
                                   visualize=False,
                                   use_sport_mode=use_sport_mode)

    try:
        while True:
            # get keyword or target poses
            keyword, left_pose, right_pose = input_keyword_or_poses()

            # handle keyword configuration
            if keyword in NAMED_CONFIGS:
                print(f'Going to named configuration: {keyword}')
                q_reduced = NAMED_CONFIGS[keyword]
                # compute target end-effector poses from q_reduced
                arm_controller.left_ee_target_transformation = (
                    arm_controller.robot_model.get_frame_transformation_reduced(
                        arm_controller.left_ee_name, q_reduced
                    )
                )
                arm_controller.right_ee_target_transformation = (
                    arm_controller.robot_model.get_frame_transformation_reduced(
                        arm_controller.right_ee_name, q_reduced
                    )
                )
                # use goto as step function
                # step_function = lambda: arm_controller.goto_reduced_configuration(q_reduced)
                step_function = lambda: arm_controller.sim_goto_reduced_configuration(q_reduced)
            elif keyword != '':
                print(f'Unknown keyword: {keyword}')
                continue
            else:
                # manual pose input
                print('Going to target end-effector poses')
                # set target poses
                arm_controller.left_ee_target_pose = left_pose
                arm_controller.right_ee_target_pose = right_pose
                step_function = lambda: arm_controller.control_dual_arm_step()
                # step_function = lambda: arm_controller.sim_dual_arm_step()

            start_time = time.time()
            arm_controller.start_recording()
            while time.time() - start_time < timeout:
                frame_start_time = time.time()
                # control one step
                step_function()

                # print errors
                left_error_linear = np.linalg.norm(arm_controller.left_ee_error[:3])
                left_error_angular = np.linalg.norm(arm_controller.left_ee_error[3:])
                right_error_linear = np.linalg.norm(arm_controller.right_ee_error[:3])
                right_error_angular = np.linalg.norm(arm_controller.right_ee_error[3:])

                print(f'Left Error Linear: {left_error_linear:.4f}, '
                      f'Left Error Angular: {left_error_angular:.4f}, '
                      f'Right Error Linear: {right_error_linear:.4f}, '
                      f'Right Error Angular: {right_error_angular:.4f}')

                # early break
                if (left_error_linear < threshold_linear and right_error_linear < threshold_linear and
                    left_error_angular < threshold_angular and right_error_angular < threshold_angular):
                    print('Target reached!')
                    break

                time.sleep(max(0.0, arm_controller.dt - (time.time() - frame_start_time)))

            for _ in range(50):
                frame_start_time = time.time()
                step_function()
                time.sleep(max(0.0, arm_controller.dt - (time.time() - frame_start_time)))
            arm_controller.stop_recording()

            input('Press any key to continue...') # flush the input buffer
            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    except Exception as e:
        print(f'Exception occurred: {e}')
    finally:
        print('Shutting down...')
        arm_controller.stop_recording()
        arm_controller.save_recording('data/control_record/record_acc_limit.npz')
        arm_controller.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arm Controller Goto')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--debug', action='store_true', help='Run in debug mode (use_sport_mode=False)')
    group.add_argument('--sport', action='store_true', help='Run in sport mode (use_sport_mode=True)')
    args = parser.parse_args()

    if args.sport:
        main(timeout=15.0, use_sport_mode=True)
    elif args.debug:
        main(timeout=15.0, use_sport_mode=False)
    else:
        print('Invalid argument! Use --debug or --sport')
