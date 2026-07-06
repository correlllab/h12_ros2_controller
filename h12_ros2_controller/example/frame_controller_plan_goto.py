import time
import argparse
import numpy as np

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
from h12_ros2_controller.utility.path_definition import (
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
)


def input_keyword_or_frame_task():
    '''Ask user for a named config or one manual frame task'''
    print(f'Available named configs: {list(NAMED_CONFIGS.keys())}')
    choice = input('Enter named config (or press Enter for frame task): ').strip()
    if choice:
        return choice, None, None

    frame_name, pose = input_frame_task()
    return '', frame_name, pose


def input_frame_task():
    '''Get frame name and pose from user'''
    frame_name = input('Enter frame name: ').strip()
    while True:
        input_pose = input('Enter x y z roll pitch yaw (degrees): ')
        parts = input_pose.strip().split()
        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue

        try:
            values = [float(val) for val in parts]
            values[3:] = np.deg2rad(values[3:])
            return frame_name, np.array(values)
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')


def init_frame_controller(config_name):
    '''Initialize controller, DDS channel, and Meshcat visualizer'''
    config = load_controller_config(config_name)
    initialize_channel_factory(config)
    frame_controller = FrameController(
        URDF_MAGPIE_PATH,
        URDF_MAGPIE_SPHERE_PATH,
        SRDF_MAGPIE_SPHERE_PATH,
        init=True,
        handless=False,
        visualize=True,
        config=config,
    )
    maybe_start_controller_logging(frame_controller)
    return frame_controller


def plan_goal(frame_controller, keyword, frame_name, pose, ik_timeout, ik_alpha,
              keep_grasp_height, z_margin):
    '''Plan one named config or frame-task goal'''
    start_time = time.perf_counter()

    # named configs plan directly in reduced joint space
    if keyword in NAMED_CONFIGS:
        print(f'Planning to named configuration: {keyword}')
        path = frame_controller.plan_to_configuration(
            NAMED_CONFIGS[keyword],
            keep_grasp_height=keep_grasp_height,
            z_margin=z_margin,
        )
    elif keyword != '':
        raise ValueError(f'Unknown named configuration: {keyword}')
    else:
        # frame goals are configured by FrameController, then solved by IK
        print(f'Planning to frame task: {frame_name}')
        frame_controller.clear_frame_tasks()
        frame_controller.add_frame_task(f'{frame_name}_task', frame_name, pose)
        path = frame_controller.plan_to_ik_target(
            ik_timeout=ik_timeout,
            ik_alpha=ik_alpha,
            keep_grasp_height=keep_grasp_height,
            z_margin=z_margin,
        )
    planning_time = time.perf_counter() - start_time
    return path, planning_time


def main(config_name='debug.yaml', ik_timeout=1.0, ik_alpha=0.1,
         allow_grasp_dip=False, z_margin=0.0):
    frame_controller = init_frame_controller(config_name)

    try:
        while True:
            keyword, frame_name, pose = input_keyword_or_frame_task()
            try:
                # plan and visualize first; execution waits for operator input
                path, planning_time = plan_goal(
                    frame_controller,
                    keyword,
                    frame_name,
                    pose,
                    ik_timeout=ik_timeout,
                    ik_alpha=ik_alpha,
                    keep_grasp_height=not allow_grasp_dip,
                    z_margin=z_margin,
                )
            except Exception as err:
                print(f'Planning failed: {err}')
                continue

            print(f'Planning time: {planning_time:.4f} s')
            print(f'Path shape: {path.shape}')
            frame_controller.visualize_path(path)

            input('Press ENTER to execute the plan...')
            frame_controller.execute_path(path)
            print('Plan execution finished')

            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        print('Shutting down...')
        frame_controller.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Frame Controller Plan Goto')
    parser.add_argument(
        '--config',
        type=str,
        default='debug.yaml',
        help='YAML file name under config/',
    )
    parser.add_argument('--ik-timeout', type=float, default=1.0)
    parser.add_argument('--ik-alpha', type=float, default=0.1)
    parser.add_argument('--allow-grasp-dip', action='store_true')
    parser.add_argument('--grasp-z-margin', type=float, default=0.0)
    args = parser.parse_args()
    main(
        config_name=args.config,
        ik_timeout=args.ik_timeout,
        ik_alpha=args.ik_alpha,
        allow_grasp_dip=args.allow_grasp_dip,
        z_margin=args.grasp_z_margin,
    )
