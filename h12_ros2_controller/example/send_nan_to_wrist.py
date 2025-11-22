import time
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import LowCmdPublisher
from h12_ros2_controller.utility.robot_setting import setup_gains
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

def main():
    # initialize channel
    ChannelFactoryInitialize()

    # initialize robot model and command publisher
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_subscriber()
    command_publisher = LowCmdPublisher()
    setup_gains(command_publisher)

    # wait for initial state
    time.sleep(1.0)
    robot_model.update_kinematics()

    # do bad thing to left_wrist
    joint_name = 'left_wrist_yaw_joint'
    joint_idx = BODY_JOINTS.index(joint_name)
    command_publisher.enable_motors([joint_idx], [robot_model.state['q'][joint_idx]])
    command_publisher.start()

    start_time = time.time()
    q = []
    dq = []
    ddq = []
    tau = []
    while time.time() - start_time < 0.1:
        command_publisher.q[joint_idx] = np.nan
        q.append(robot_model.state['q'][joint_idx])
        dq.append(robot_model.state['dq'][joint_idx])
        ddq.append(robot_model.state['ddq'][joint_idx])
        tau.append(robot_model.state['tau'][joint_idx])

    command_publisher.estop()
    q = np.array(q)
    dq = np.array(dq)
    ddq = np.array(ddq)
    tau = np.array(tau)
    filename = './data/motor_record/record_nan/left_wrist_yaw_joint.npz'
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    np.savez(
        filename,
        joint_name=joint_name,
        kp=np.zeros_like(q),
        kd=np.zeros_like(q),
        q=q,
        dq=dq,
        ddq=ddq,
        tau=tau,
        q_cmd=np.zeros_like(q),
        dq_cmd=np.zeros_like(q),
        tau_cmd=np.zeros_like(q),
        torque_cmd=np.zeros_like(q)
    )

if __name__ == '__main__':
    main()
