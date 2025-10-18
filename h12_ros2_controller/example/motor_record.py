import time
import argparse
import numpy as np
from tqdm import tqdm

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import CommandPublisher, setup_gains_mj, setup_gains_real
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

def get_state(robot_model, command_publisher, joint_idx):
    state = robot_model.state
    feedback_state = {
        'q': state['q'][joint_idx],
        'dq': state['dq'][joint_idx],
        'ddq': state['ddq'][joint_idx],
        'tau': state['tau'][joint_idx],
    }
    cmd_state = {
        'kp': command_publisher.kp[joint_idx],
        'kd': command_publisher.kd[joint_idx],
        'q_cmd': command_publisher.q[joint_idx],
        'dq_cmd': command_publisher.dq[joint_idx],
        'tau_cmd': command_publisher.tau[joint_idx],
    }
    full_state = {**feedback_state, **cmd_state}
    return full_state

def record_linear_motion(robot_model, command_publisher, joint_name, q_end, steps):
    # get joint index
    joint_idx = BODY_JOINTS.index(joint_name)
    # empty array for recording states
    state_arr = []

    q_start = robot_model.state['q'][joint_idx]

    for step in range(steps):
        # linear interpolation
        alpha = (step + 1) / steps
        q_target = (1 - alpha) * q_start + alpha * q_end
        # command positional control with gravity compensation
        command_publisher.q[joint_idx] = q_target
        command_publisher.tau[joint_idx] = robot_model.get_gravity_compensation()[joint_idx]

        # record state
        state_arr.append(get_state(robot_model, command_publisher, joint_idx))
        time.sleep(0.01)

    # fix on final state
    command_publisher.q[joint_idx] = q_end
    command_publisher.dq[joint_idx] = 0.0
    command_publisher.tau[joint_idx] = robot_model.get_gravity_compensation()[joint_idx]

    return state_arr

def record_static_motion(robot_model, command_publisher, joint_name, steps):
    # get joint index
    joint_idx = BODY_JOINTS.index(joint_name)
    # empty array for recording states
    state_arr = []

    for _ in range(steps):
        # command gravity compensation only
        command_publisher.tau[joint_idx] = robot_model.get_gravity_compensation()[joint_idx]

        # record state
        state_arr.append(get_state(robot_model, command_publisher, joint_idx))
        time.sleep(0.01)

    return state_arr

def record_free_motion(robot_model, command_publisher, joint_name, steps):
    # get joint index
    joint_idx = BODY_JOINTS.index(joint_name)
    # empty array for recording states
    state_arr = []

    for _ in range(steps):
        state_arr.append(get_state(robot_model, command_publisher, joint_idx))
        time.sleep(0.01)

    return state_arr

def main(joint_name_list, q_start_list, q_end_list, steps, savepath):
    # initialize channel
    ChannelFactoryInitialize()

    # initialize robot model and command publisher
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_subscriber()
    command_publisher = CommandPublisher()

    # wait for initial state
    time.sleep(1.0)
    robot_model.update_kinematics()

    # record free motion
    for joint_name in joint_name_list:
        states = record_free_motion(robot_model, command_publisher, joint_name, 200)
        save_results(states, joint_name, f'{savepath}/{joint_name}_free.npz')

    # setup motor gains
    # setup_gains_mj(command_publisher)
    setup_gains_real(command_publisher)

    # enable all motors at initial positions
    motor_ids = list(range(27))
    init_q = robot_model.state['q']
    command_publisher.enable_motor(motor_ids, init_q)
    command_publisher.start_publisher()

    # move elbow to 0 position
    record_linear_motion(robot_model, command_publisher, 'left_elbow_joint', 0.0, steps)
    record_linear_motion(robot_model, command_publisher, 'right_elbow_joint', 0.0, steps)

    joint_data = list(zip(joint_name_list, q_start_list, q_end_list))
    for joint_name, q_start, q_end in tqdm(joint_data):
        # move to start position
        _ = record_linear_motion(robot_model, command_publisher, joint_name, q_start, steps)
        _ = record_static_motion(robot_model, command_publisher, joint_name, steps)
        # record linear motion
        states_go = record_linear_motion(robot_model, command_publisher, joint_name, q_end, steps)
        states_go_static = record_static_motion(robot_model, command_publisher, joint_name, steps)
        time.sleep(3.0)
        states_static = record_static_motion(robot_model, command_publisher, joint_name, 200)
        # go back to start
        states_back = record_linear_motion(robot_model, command_publisher, joint_name, q_start, steps)
        states_back_static = record_static_motion(robot_model, command_publisher, joint_name, steps)

        # combine state arrays
        states = states_go + states_go_static + states_back + states_back_static

        save_results(states_static, joint_name, f'{savepath}/{joint_name}_static.npz')
        save_results(states, joint_name, f'{savepath}/{joint_name}_all.npz')

    command_publisher.shutdown()
    robot_model.shutdown()

def save_results(states, joint_name, filename):
    os.makedirs(os.path.dirname(filename), exist_ok=True)

    q_arr = np.array([state['q'] for state in states])
    dq_arr = np.array([state['dq'] for state in states])
    ddq_arr = np.array([state['ddq'] for state in states])
    tau_arr = np.array([state['tau'] for state in states])
    q_cmd_arr = np.array([state['q_cmd'] for state in states])
    dq_cmd_arr = np.array([state['dq_cmd'] for state in states])
    tau_cmd_arr = np.array([state['tau_cmd'] for state in states])

    # compute PD torque
    kp_arr = np.array([state['kp'] for state in states])
    kd_arr = np.array([state['kd'] for state in states])
    torque_cmd_arr = kp_arr * (q_cmd_arr - q_arr) + kd_arr * (dq_cmd_arr - dq_arr) + tau_cmd_arr

    # Save to npz file
    np.savez(filename,
             joint_name=joint_name,
             kp=kp_arr,
             kd=kd_arr,
             q=q_arr,
             dq=dq_arr,
             ddq=ddq_arr,
             tau=tau_arr,
             q_cmd=q_cmd_arr,
             dq_cmd=dq_cmd_arr,
             tau_cmd=tau_cmd_arr,
             torque_cmd=torque_cmd_arr)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Motor recording script')
    parser.add_argument('--save', type=str, required=True,
                        help='Folder name to save motor recordings in data/motor_record/')
    args = parser.parse_args()

    # set path using command line argument
    savepath = f'./data/motor_record/{args.save}'

    joint_name_list = [
        'left_hip_yaw_joint', 'right_hip_yaw_joint',
        'left_hip_pitch_joint', 'right_hip_pitch_joint',
        'left_hip_roll_joint', 'right_hip_roll_joint',
        'left_knee_joint', 'right_knee_joint',
        'left_ankle_pitch_joint', 'right_ankle_pitch_joint',
        'left_ankle_roll_joint', 'right_ankle_roll_joint',
        'torso_joint',
        'left_shoulder_pitch_joint', 'right_shoulder_pitch_joint',
        'left_shoulder_roll_joint', 'right_shoulder_roll_joint',
        'left_shoulder_yaw_joint', 'right_shoulder_yaw_joint',
        'left_elbow_joint', 'right_elbow_joint',
        'left_wrist_roll_joint', 'right_wrist_roll_joint',
        'left_wrist_pitch_joint', 'right_wrist_pitch_joint',
        'left_wrist_yaw_joint', 'right_wrist_yaw_joint',
    ]
    q_start_list = [
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
    ]
    q_end_list = [
        0.3, -0.3, # hip yaw
        -0.5, -0.5, # hip pitch
        0.3, -0.3, # hip roll
        1.0, 1.0, # knee
        -0.5, -0.5, # ankle pitch
        0.25, -0.25, # ankle roll
        1.0, # torso
        -0.5, -0.5, # shoulder pitch
        1.0, -1.0, # shoulder roll
        1.0, -1.0, # shoulder yaw
        -0.5, -0.5, # elbow
        1.0, -1.0, # wrist roll
        0.3, 0.3, # wrist pitch
        0.8, -0.8, # wrist yaw
    ]
    steps = 50

    main(joint_name_list, q_start_list, q_end_list, steps, savepath)
