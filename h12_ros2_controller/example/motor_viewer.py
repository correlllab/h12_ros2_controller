import time
import argparse
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import LowCmdPublisher
from h12_ros2_controller.utility.dds_init import init_channel_factory_from_env
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, UPPER_BODY_JOINTS, LOWER_BODY_JOINTS
from h12_ros2_controller.utility.path_definition import URDF_HANDLESS_PATH

def print_joint_positions(robot_model, joint_groups):
    '''Print joint positions in an organized format by groups'''
    state = robot_model.state

    print('='*80)
    print(f'{"JOINT POSITIONS":^80}')
    print('='*80)

    for group_name, joint_names in joint_groups.items():
        print(f'\n{group_name.upper()}:')
        print('-' * 40)

        for joint_name in joint_names:
            try:
                joint_idx = BODY_JOINTS.index(joint_name)
                position = state['q'][joint_idx]
                velocity = state['dq'][joint_idx]
                torque = state['tau'][joint_idx]
                print(f'  {joint_name:26} | Pos: {position:8.4f} | Vel: {velocity:8.4f} | Tau: {torque:8.4f}')
            except ValueError:
                print(f'  {joint_name:26} | Joint not found in BODY_JOINTS')

    print('='*80)

def setup_low_damping(command_publisher, damping_value=0.5):
    '''Setup very low damping for all joints'''
    print(f'Setting damping to {damping_value} for all joints...')

    # set low damping for all joints
    for i in range(len(BODY_JOINTS)):
        command_publisher.kp[i] = 0.0  # no position control
        command_publisher.kd[i] = damping_value  # low damping
        command_publisher.q[i] = 0.0
        command_publisher.dq[i] = 0.0
        command_publisher.tau[i] = 0.0

def main(damping_value, update_rate):
    # initialize channel
    init_channel_factory_from_env()

    # initialize robot model
    robot_model = RobotModel(URDF_HANDLESS_PATH, handless=True)
    robot_model.init_subscriber()

    # wait for initial state
    print('Waiting for robot connection...')
    time.sleep(1.0)
    robot_model.update_kinematics()

    command_publisher = None
    if damping_value is not None:
        # initialize command publisher only if damping is provided
        command_publisher = LowCmdPublisher()
        # setup low damping
        setup_low_damping(command_publisher, damping_value)
        # enable all motors with low damping
        motor_ids = list(range(27))
        init_q = robot_model.state['q']
        command_publisher.enable_motors(motor_ids, init_q)
        command_publisher.start()
        print(f'Starting position monitoring with damping={damping_value}, update_rate={update_rate}Hz')
    else:
        print(f'Starting position monitoring (listener only), update_rate={update_rate}Hz')

    # define joint groups using predefined definitions from joint_definition
    joint_groups = {
        'lower_body': LOWER_BODY_JOINTS,
        'upper_body': UPPER_BODY_JOINTS
    }

    print('Press Ctrl+C to stop...')

    try:
        while True:
            robot_model.update_kinematics()
            print_joint_positions(robot_model, joint_groups)
            time.sleep(1.0 / update_rate)

    except KeyboardInterrupt:
        print('Stopping motor position checker...')

    finally:
        # shutdown
        if command_publisher is not None:
            command_publisher.shutdown()
        robot_model.shutdown()
        print('Motor position checker stopped.')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Motor position checker with optional damping')
    parser.add_argument('--damping', type=float, default=None,
                        help='Damping value to set for all joints (optional, listener only if not provided)')
    parser.add_argument('--rate', type=float, default=30.0,
                        help='Update rate in Hz (default: 30.0)')
    args = parser.parse_args()

    main(args.damping, args.rate)
