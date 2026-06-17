import time
import numpy as np
import tkinter as tk
import pinocchio as pin

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import ArmSDKPublisher
from h12_ros2_controller.utility.dds_init import init_channel_factory_from_env
from h12_ros2_controller.utility.joint_definition import LEFT_ARM_INDEX, RIGHT_ARM_INDEX
from h12_ros2_controller.utility.path_definition import URDF_HANDLESS_PATH

def main():
    init_channel_factory_from_env()
    robot_model = RobotModel(URDF_HANDLESS_PATH, handless=True)
    robot_model.init_subscriber()
    time.sleep(1.0)
    robot_model.update_kinematics()

    arm_sdk_publisher = ArmSDKPublisher()

    # gain for shoulder
    arm_sdk_publisher.kp[13:15] = 200.0
    arm_sdk_publisher.kd[13:15] = 6.0
    arm_sdk_publisher.kp[20:22] = 200.0
    arm_sdk_publisher.kd[20:22] = 6.0
    # gain for shoulder yaw
    arm_sdk_publisher.kp[15] = 150.0
    arm_sdk_publisher.kd[15] = 4.0
    arm_sdk_publisher.kp[22] = 150.0
    arm_sdk_publisher.kd[22] = 4.0
    # gain for elbow
    arm_sdk_publisher.kp[16] = 150.0
    arm_sdk_publisher.kd[16] = 4.0
    arm_sdk_publisher.kp[23] = 150.0
    arm_sdk_publisher.kd[23] = 4.0
    # gain for wrist
    arm_sdk_publisher.kp[17:20] = 50.0
    arm_sdk_publisher.kd[17:20] = 3.0
    arm_sdk_publisher.kp[24:27] = 50.0
    arm_sdk_publisher.kd[24:27] = 3.0

    motor_ids = [i for i in range(13, 27)]
    init_q = robot_model.state['q'][LEFT_ARM_INDEX + RIGHT_ARM_INDEX]
    arm_sdk_publisher.enable_motors(motor_ids, init_q)
    arm_sdk_publisher.start()

    control_idx = 14
    root = tk.Tk()
    root.title(f'Arm Joint {control_idx} Control')
    slider = tk.Scale(root, label=f'Joint {control_idx} Position',
                      from_=-2.0, to=2.0,
                      resolution=0.01, orient=tk.HORIZONTAL, length=400)
    slider.pack()
    slider.set(robot_model.state['q'][control_idx])
    root.update()

    try:
        while True:
            root.update()
            robot_model.update_kinematics()

            target_q = robot_model.state['q']
            target_q[control_idx] = slider.get()
            q_diff = target_q - robot_model.state['q']

            tau = robot_model.get_gravity_compensation(target_q)

            arm_sdk_publisher.q[control_idx] = target_q[control_idx]
            arm_sdk_publisher.dq[control_idx] = q_diff[control_idx]
            arm_sdk_publisher.tau[control_idx] = tau[control_idx]

            print(f'Error: {arm_sdk_publisher.q[control_idx] - robot_model.state["q"][control_idx]:.4f}')
            time.sleep(0.01)
    except Exception as e:
        print(f'Exception occurred: {e}')
    finally:
        print('Shutting down...')
        arm_sdk_publisher.shutdown()

if __name__ == '__main__':
    main()
