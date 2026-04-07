import time
import argparse
import numpy as np
import pinocchio as pin

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.utility.controller_config import (
    load_controller_config,
    initialize_channel_factory,
    maybe_start_controller_logging,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS

def input_keyword_or_frame_task():
    '''
    Ask user for keyword command or manual frame task input
    Returns (keyword, frame_name, pose) tuple
    If keyword is provided, frame_name and pose will be None
    If no keyword, frame_name and pose will be provided
    '''
    print(f'Available keywords: {list(NAMED_CONFIGS.keys())}')

    choice = input('Enter keyword (or press Enter for manual frame task): ').strip()

    if choice:
        # user entered a keyword
        return choice, None, None
    else:
        # user wants manual frame task input
        frame_name, pose = input_frame_task()
        return '', frame_name, pose

def input_frame_task():
    '''Get frame name and pose from user'''
    frame_name = input('Enter frame name: ')
    while True:
        input_pose = input('Enter x y z roll pitch yaw (separated by space): ')
        parts = input_pose.strip().split()

        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue
        try:
            values = [float(val) for val in parts]
            # degrees to radians for roll, pitch, yaw
            values[3:] = np.deg2rad(values[3:])
            return frame_name, np.array(values)
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')
            continue

def main(timeout=10.0,
         threshold_linear=5e-3,
         threshold_angular=2e-2,
         com=False,
         config_name='debug.yaml'):
    config= load_controller_config(config_name)
    initialize_channel_factory(config)
    # initialize upper task controller
    frame_controller = FrameController('assets/h1_2/h1_2_handless.urdf',
                                       'assets/h1_2/h1_2_handless_sphere.urdf',
                                       'assets/h1_2/h1_2_handless_sphere_collision.srdf',
                                       handless=True,
                                       visualize=True,
                                       config=config)
    maybe_start_controller_logging(frame_controller)

    try:
        while True:
            # get keyword or frame task input
            keyword, frame_name, pose = input_keyword_or_frame_task()

            # handle keyword configuration
            if keyword in NAMED_CONFIGS:
                print(f'Going to named configuration: {keyword}')
                q_reduced = NAMED_CONFIGS[keyword]
                # use goto as step function
                step_function = lambda: frame_controller.goto_reduced_configuration(q_reduced)
                # step_function = lambda: frame_controller.sim_goto_reduced_configuration(q_reduced)
            elif keyword != '':
                print(f'Unknown keyword: {keyword}')
                continue
            else:
                # manual frame task input
                print('Going to manual frame task')
                task_name = f'{frame_name}_task'
                # add frame task
                frame_controller.clear_frame_tasks()
                frame_controller.add_frame_task(task_name, frame_name, pose)
                step_function = lambda: frame_controller.control_step_reduced(com=com)
                # step_function = lambda: frame_controller.sim_step_reduced(com=com)

            # update ik solver with current state
            frame_controller.update_ik_solver()

            # main loop
            start_time = time.time()
            while time.time() - start_time < timeout:
                frame_start_time = time.time()
                # control one step
                step_function()

                # print error (only for frame task mode)
                if keyword not in NAMED_CONFIGS:
                    error = frame_controller.get_frame_task_error(task_name)
                    linear_error = np.linalg.norm(error[:3])
                    angular_error = np.linalg.norm(error[3:])

                    print(f'Linear Error: {linear_error:.4f}, Angular Error: {angular_error:.4f}')

                    # early break
                    if linear_error < threshold_linear and angular_error < threshold_angular:
                        print('Target reached!')
                        break

                time.sleep(max(0.0, frame_controller.dt - (time.time() - frame_start_time)))

            for _ in range(50):
                frame_start_time = time.time()
                step_function()
                time.sleep(max(0.0, frame_controller.dt - (time.time() - frame_start_time)))

            input('Press any key to continue...') # flush the input buffer
            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    except Exception as e:
        print(f'Exception occurred: {e}')
    finally:
        print('Shutting down...')
        frame_controller.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Frame Controller Goto')
    parser.add_argument('--config', type=str, default='debug.yaml', help='YAML file name under config/')
    parser.add_argument('--com', action='store_true', help='Use center of mass control')
    args = parser.parse_args()
    main(timeout=10.0, com=args.com, config_name=args.config)
