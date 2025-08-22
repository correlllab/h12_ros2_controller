import time
import argparse
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.upper_task_controller import UpperTaskController

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
            return frame_name, values
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')
            continue

def main(timeout=10.0,
         threshold_linear=5e-3,
         threshold_angular=2e-2,
         use_sport_mode=False):
    ChannelFactoryInitialize()
    # initialize upper task controller
    upper_task_controller = UpperTaskController('assets/h1_2/h1_2.urdf',
                                                'assets/h1_2/h1_2_sphere.urdf',
                                                'assets/h1_2/h1_2_sphere_collision.srdf',
                                                dt=0.03,
                                                v_lim=1.0,
                                                w_lim=2.0,
                                                dq_lim=2.0,
                                                d_min=0.02,
                                                visualize=True,
                                                use_sport_mode=use_sport_mode)

    try:
        while True:
            # get frame task input
            frame_name, pose = input_frame_task()
            task_name = f'{frame_name}_task'
            # add frame task
            upper_task_controller.add_frame_task(task_name, frame_name, pose)

            start_time = time.time()
            while time.time() - start_time < timeout:
                frame_start_time = time.time()
                upper_task_controller.sim_step_reduced()

                # print error
                error = upper_task_controller.get_frame_task_error(task_name)
                linear_error = np.linalg.norm(error[:3])
                angular_error = np.linalg.norm(error[3:])

                print(f'Linear Error: {linear_error:.4f}, Angular Error: {angular_error:.4f}')

                # early break
                if linear_error < threshold_linear and angular_error < threshold_angular:
                    print('Target reached!')
                    break

                time.sleep(max(0.0, upper_task_controller.dt - (time.time() - frame_start_time)))

            input('Press any key to continue...') # flush the input buffer
            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    except Exception as e:
        print(f'Exception occurred: {e}')
    finally:
        print('Shutting down...')
        upper_task_controller.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upper Task Controller Goto')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--debug', action='store_true', help='Run in debug mode (use_sport_mode=False)')
    group.add_argument('--sport', action='store_true', help='Run in sport mode (use_sport_mode=True)')
    args = parser.parse_args()

    if args.sport:
        main(timeout=10.0, use_sport_mode=True)
    elif args.debug:
        main(timeout=10.0, use_sport_mode=False)
    else:
        print('Invalid argument! Use --debug or --sport')
