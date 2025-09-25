import time
import numpy as np
import tkinter as tk
import pinocchio as pin

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import CommandPublisher
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS
from h12_ros2_controller.example.motor_debug_gui import setup_gains

def record_linear_motion(robot_model, command_publisher, joint_name, q_end, steps):
    # get joint index
    joint_idx = BODY_JOINTS.index(joint_name)
    # empty array for recording
    q_arr = []
    dq_arr = []
    tau_arr = []
    q_cmd_arr = []
    dq_cmd_arr = []

    # sync initial state
    robot_model.update_kinematics()
    q_start = robot_model.state['q'][joint_idx]

    for step in range(steps):
        # sync robot state
        robot_model.update_kinematics()

        # linear interpolation
        alpha = (step + 1) / steps
        q_target = (1 - alpha) * q_start + alpha * q_end
        command_publisher.q[joint_idx] = q_target
        command_publisher.dq[joint_idx] = (q_end - q_start) / (steps * 0.01)

        # sync robot state
        robot_model.update_kinematics()
        # record
        q_arr.append(robot_model.state['q'][joint_idx])
        dq_arr.append(robot_model.state['dq'][joint_idx])
        tau_arr.append(robot_model.state['tau'][joint_idx])
        q_cmd_arr.append(command_publisher.q[joint_idx])
        dq_cmd_arr.append(command_publisher.dq[joint_idx])

        time.sleep(0.01)

    # zero out publisher
    command_publisher.q[joint_idx] = q_end
    command_publisher.dq[joint_idx] = 0.0

    # stationary sleep
    for _ in range(50):
        robot_model.update_kinematics()
        q_arr.append(robot_model.state['q'][joint_idx])
        dq_arr.append(robot_model.state['dq'][joint_idx])
        tau_arr.append(robot_model.state['tau'][joint_idx])
        q_cmd_arr.append(command_publisher.q[joint_idx])
        dq_cmd_arr.append(command_publisher.dq[joint_idx])
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
    robot_model.update_kinematics()

    # setup motor gains
    setup_gains(command_publisher)

    # enable all motors at initial positions
    motor_ids = list(range(27))
    init_q = robot_model.state['q']
    command_publisher.enable_motor(motor_ids, init_q)
    command_publisher.start_publisher()

    # move to start position
    _ = record_linear_motion(robot_model, command_publisher, joint_name, q_start, steps)
    # record linear motion
    data_go = record_linear_motion(robot_model, command_publisher, joint_name, q_end, steps)
    # go back to start
    data_back = record_linear_motion(robot_model, command_publisher, joint_name, q_start, steps)

    data = {
        'joint_name': data_go['joint_name'],
        'q': np.concatenate([data_go['q'], data_back['q']]),
        'dq': np.concatenate([data_go['dq'], data_back['dq']]),
        'tau': np.concatenate([data_go['tau'], data_back['tau']]),
        'q_cmd': np.concatenate([data_go['q_cmd'], data_back['q_cmd']]),
        'dq_cmd': np.concatenate([data_go['dq_cmd'], data_back['dq_cmd']])
    }

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
    path = './data/motor_record_mj'
    joint_name_list = [
        # 'left_hip_yaw_joint', 'right_hip_yaw_joint',
        'left_hip_pitch_joint', 'right_hip_pitch_joint',
        # 'left_hip_roll_joint', 'right_hip_roll_joint',
        # 'left_knee_joint', 'right_knee_joint',
        # 'left_ankle_pitch_joint', 'right_ankle_pitch_joint',
        # 'left_ankle_roll_joint', 'right_ankle_roll_joint'
    ]
    q_start_list = [
        # 0.0, 0.0,
        0.0, 0.0,
        # 0.0, 0.0,
        # 0.0, 0.0,
        # 0.0, 0.0,
        # 0.0, 0.0
    ]
    q_end_list = [
        # 0.3, -0.3, # hip yaw
        -0.5, -0.5, # hip pitch
        # 0.5, -0.5, # hip roll
        # 1.0, 1.0, # knee
        # -0.5, -0.5, # ankle pitch
        # 0.25, -0.25 # ankle roll
    ]
    steps = 100

    for joint_name, q_start, q_end in zip(joint_name_list, q_start_list, q_end_list):
        data = main(f'{joint_name}', q_start, q_end, steps)
        save_results(data, f'{path}/{joint_name}.npz')
