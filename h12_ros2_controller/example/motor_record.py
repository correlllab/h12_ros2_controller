import time
import argparse
import numpy as np
from tqdm import tqdm

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import LowCmdPublisher
from h12_ros2_controller.utility.controller_config import (
    load_controller_config,
    get_publisher_clip_limits,
    init_channel_factory_guard,
)
from h12_ros2_controller.utility.joint_limits import setup_gains
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS
from h12_ros2_controller.utility.path_definition import URDF_MAGPIE_PATH

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

def duration_to_steps(duration, dt):
    return max(1, int(round(duration / dt)))

def record_linear_motion(robot_model,
                         command_publisher,
                         joint_name,
                         q_end,
                         duration,
                         dt):
    # get joint index
    joint_idx = BODY_JOINTS.index(joint_name)
    # empty array for recording states
    state_arr = []

    q_start = robot_model.state['q'][joint_idx]
    steps = duration_to_steps(duration, dt)

    for step in range(steps):
        start_time = time.time()
        # linear interpolation
        alpha = (step + 1) / steps
        q_target = (1 - alpha) * q_start + alpha * q_end
        # command positional control with gravity compensation
        command_publisher.q[joint_idx] = q_target
        command_publisher.tau[joint_idx] = (
            robot_model.dynamics.get_gravity_compensation()[joint_idx]
        )

        # record state
        state_arr.append(get_state(robot_model, command_publisher, joint_idx))
        time.sleep(max(0.0, dt - (time.time() - start_time)))

    # fix on final state
    command_publisher.q[joint_idx] = q_end
    command_publisher.dq[joint_idx] = 0.0
    command_publisher.tau[joint_idx] = robot_model.dynamics.get_gravity_compensation()[joint_idx]

    return state_arr

def record_static_motion(robot_model,
                         command_publisher,
                         joint_name,
                         duration,
                         dt):
    # get joint index
    joint_idx = BODY_JOINTS.index(joint_name)
    # empty array for recording states
    state_arr = []
    steps = duration_to_steps(duration, dt)

    for _ in range(steps):
        start_time = time.time()
        # command gravity compensation only
        command_publisher.tau[joint_idx] = (
            robot_model.dynamics.get_gravity_compensation()[joint_idx]
        )

        # record state
        state_arr.append(get_state(robot_model, command_publisher, joint_idx))
        time.sleep(max(0.0, dt - (time.time() - start_time)))

    return state_arr

def record_free_motion(robot_model, command_publisher, duration, dt):
    # empty array for recording states
    joint_states = [[] for _ in range(len(BODY_JOINTS))]
    steps = duration_to_steps(duration, dt)

    for _ in range(steps):
        start_time = time.time()
        for joint_idx in range(len(BODY_JOINTS)):
            joint_states[joint_idx].append(
                get_state(robot_model, command_publisher, joint_idx)
            )
        time.sleep(max(0.0, dt - (time.time() - start_time)))

    return joint_states

def main(joint_name_list,
         q_start_list,
         q_end_list,
         savepath,
         duration=5.0,
         config_name='debug.yaml'):
    # initialize channel
    config = load_controller_config(config_name)
    dt = 1.0 / float(config['frequency']['ctrl_hz'])
    publisher_dt = 1.0 / float(config['frequency']['pub_hz'])
    init_channel_factory_guard(config)

    # initialize robot model and command publisher
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_subscriber(low_state_topic=config['topics']['low_state'])
    command_publisher = LowCmdPublisher(
        dt=publisher_dt,
        low_cmd_topic=config['topics']['low_cmd'],
        clip_limits=get_publisher_clip_limits(config),
    )

    # wait for initial state
    time.sleep(1.0)
    robot_model.update_kinematics()

    # record free motion
    print('Recording free motion noise...')
    joint_states = record_free_motion(robot_model, command_publisher, duration, dt)
    for joint_name in joint_name_list:
        joint_idx = BODY_JOINTS.index(joint_name)
        states = joint_states[joint_idx]
        save_results(states, joint_name, f'{savepath}/{joint_name}_free.npz')

    # setup motor gains
    setup_gains(command_publisher, config['gains'])

    # enable all motors at initial positions
    motor_ids = list(range(27))
    init_q = robot_model.state['q']
    command_publisher.enable_motors(motor_ids, init_q)
    command_publisher.start()

    # move elbow to 0 position
    print('Moving elbows to 0 position...')
    record_linear_motion(
        robot_model,
        command_publisher,
        'left_elbow_joint',
        0.0,
        duration,
        dt,
    )
    record_linear_motion(
        robot_model,
        command_publisher,
        'right_elbow_joint',
        0.0,
        duration,
        dt,
    )

    joint_data = list(zip(joint_name_list, q_start_list, q_end_list))
    for joint_name, q_start, q_end in tqdm(joint_data):
        # move to start position
        _ = record_linear_motion(
            robot_model,
            command_publisher,
            joint_name,
            q_start,
            duration,
            dt,
        )
        _ = record_static_motion(
            robot_model,
            command_publisher,
            joint_name,
            duration,
            dt,
        )
        # record linear motion
        states_go = record_linear_motion(
            robot_model,
            command_publisher,
            joint_name,
            q_end,
            duration,
            dt,
        )
        states_go_static = record_static_motion(
            robot_model,
            command_publisher,
            joint_name,
            duration,
            dt,
        )
        time.sleep(3.0)
        states_static = record_static_motion(
            robot_model,
            command_publisher,
            joint_name,
            duration,
            dt,
        )
        # go back to start
        states_back = record_linear_motion(
            robot_model,
            command_publisher,
            joint_name,
            q_start,
            duration,
            dt,
        )
        states_back_static = record_static_motion(
            robot_model,
            command_publisher,
            joint_name,
            duration,
            dt,
        )

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
    parser.add_argument('--config', type=str, default='debug.yaml', help='YAML file name under config/')
    parser.add_argument('--time', type=float, default=1.0,
                        help='Action duration in seconds for each segment')
    args = parser.parse_args()

    # set path using command line argument
    savepath = f'./data/motor_record/{args.save}'

    joint_name_list = [
        # 'left_hip_yaw_joint', 'right_hip_yaw_joint',
        # 'left_hip_pitch_joint', 'right_hip_pitch_joint',
        # 'left_hip_roll_joint', 'right_hip_roll_joint',
        # 'left_knee_joint', 'right_knee_joint',
        # 'left_ankle_pitch_joint', 'right_ankle_pitch_joint',
        # 'left_ankle_roll_joint', 'right_ankle_roll_joint',
        # 'torso_joint',
        'left_shoulder_pitch_joint', 'right_shoulder_pitch_joint',
        'left_shoulder_roll_joint', 'right_shoulder_roll_joint',
        'left_shoulder_yaw_joint', 'right_shoulder_yaw_joint',
        'left_elbow_joint', 'right_elbow_joint',
        'left_wrist_roll_joint', 'right_wrist_roll_joint',
        'left_wrist_pitch_joint', 'right_wrist_pitch_joint',
        'left_wrist_yaw_joint', 'right_wrist_yaw_joint',
    ]
    q_start_list = [
        # 0.0, 0.0,
        # 0.0, 0.0,
        # 0.0, 0.0,
        # 0.0, 0.0,
        # 0.0, 0.0,
        # 0.0, 0.0,
        # 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
    ]
    q_end_list = [
        # 0.3, -0.3, # hip yaw
        # -0.5, -0.5, # hip pitch
        # 0.3, -0.3, # hip roll
        # 1.0, 1.0, # knee
        # -0.5, -0.5, # ankle pitch
        # 0.25, -0.25, # ankle roll
        # 1.0, # torso
        # -0.5, -0.5, # shoulder pitch
        # 1.0, -1.0, # shoulder roll
        # 1.0, -1.0, # shoulder yaw
        # -0.5, -0.5, # elbow
        # 1.0, -1.0, # wrist roll
        # 0.3, 0.3, # wrist pitch
        # 0.8, -0.8, # wrist yaw
        -1.5, -1.5, # shoulder pitch
        2.0, -2.0, # shoulder roll
        2.0, -2.0, # shoulder yaw
        -0.8, -0.8, # elbow
        2.0, -2.0, # wrist roll
        0.3, 0.3, # wrist pitch
        1.0, -1.0, # wrist yaw
    ]
    main(joint_name_list,
         q_start_list,
         q_end_list,
         savepath,
         duration=args.time,
         config_name=args.config)
