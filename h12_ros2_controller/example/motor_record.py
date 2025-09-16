import time
import numpy as np
import tkinter as tk

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import CommandPublisher
from h12_ros2_controller.utility.joint_definition import ALL_JOINTS, BODY_JOINTS
from h12_ros2_controller.example.motor_debug import setup_gains

def record_linear_motion(robot_model, command_publisher, joint_name, q_end, steps):
    # empty array for recording
    q_arr = []
    dq_arr = []
    tau_arr = []
    q_cmd_arr = []
    dq_cmd_arr = []

    # sync initial state
    robot_model.sync_subscriber()
    robot_model.update_kinematics()
    q_start = robot_model.q[ALL_JOINTS.index(joint_name)]

    for step in range(steps):
        # sync robot state
        robot_model.sync_subscriber()
        robot_model.update_kinematics()

        # linear interpolation
        alpha = (step + 1) / steps
        q_target = (1 - alpha) * q_start + alpha * q_end
        command_publisher.q[BODY_JOINTS.index(joint_name)] = q_target
        position_error = q_target - robot_model.q[ALL_JOINTS.index(joint_name)]
        command_publisher.dq[BODY_JOINTS.index(joint_name)] = position_error

        # sync robot state
        robot_model.sync_subscriber()
        robot_model.update_kinematics()
        # record
        q_arr.append(robot_model.q[ALL_JOINTS.index(joint_name)])
        dq_arr.append(robot_model.dq[ALL_JOINTS.index(joint_name)])
        tau_arr.append(robot_model.tau[ALL_JOINTS.index(joint_name)])
        q_cmd_arr.append(command_publisher.q[BODY_JOINTS.index(joint_name)])
        dq_cmd_arr.append(command_publisher.dq[BODY_JOINTS.index(joint_name)])

        time.sleep(0.01)

    return {
        'joint_name': joint_name,
        'q': np.array(q_arr),
        'dq': np.array(dq_arr),
        'tau': np.array(tau_arr),
        'q_cmd': np.array(q_cmd_arr),
        'dq_cmd': np.array(dq_cmd_arr),
    }

def main(joint_name, q_start, q_end, steps=100):
    # initialize channel
    ChannelFactoryInitialize()

    # initialize robot model and command publisher
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_subscriber()
    command_publisher = CommandPublisher()

    # wait for initial state sync
    time.sleep(1.0)
    robot_model.sync_subscriber()
    robot_model.update_kinematics()

    # get initial joint positions
    init_q = robot_model.q[robot_model.body_q_ids]

    # setup motor gains
    setup_gains(command_publisher)

    # enable all motors at initial positions
    motor_ids = list(range(27))
    command_publisher.enable_motor(motor_ids, init_q)
    command_publisher.start_publisher()

    # move to start position
    _ = record_linear_motion(robot_model, command_publisher, joint_name, q_start, steps)
    # record linear motion
    data = record_linear_motion(robot_model, command_publisher, joint_name, q_end, steps)
    # go back to start
    _ = record_linear_motion(robot_model, command_publisher, joint_name, q_start, steps)

    # shutdown
    command_publisher.shutdown()
    robot_model.shutdown()

    return data

def save_results(data, filename):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    np.savez(filename,
             joint_name=data['joint_name'],
             q=data['q'],
             dq=data['dq'],
             tau=data['tau'],
             q_cmd=data['q_cmd'],
             dq_cmd=data['dq_cmd'])

if __name__ == '__main__':
    path = './data/motor_record'
    # joint_name = 'left_knee_joint'
    joint_name = 'right_knee_joint'
    q_start = 0.0
    q_end = 1.0
    steps = 200

    data = main(joint_name, q_start, q_end, steps)
    save_results(data, f'{path}/{joint_name}.npz')
