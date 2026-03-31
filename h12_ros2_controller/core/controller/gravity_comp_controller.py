import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

class GravityCompController(UpperController):
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 visualize=False,
                 config='default.yaml'):
        # initialize base controller
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            visualize=visualize,
            config=config,
        )

        # i control on dq (motor-only, 27)
        self.dq_i = np.zeros(self.robot_model.model_body.nv)
        self.ki = np.zeros(self.robot_model.model_body.nv)
        # gain for shoulder pitch
        self.ki[BODY_JOINTS.index('left_shoulder_pitch_joint')] = 250.0
        self.ki[BODY_JOINTS.index('right_shoulder_pitch_joint')] = 250.0
        # gain for shoulder roll
        self.ki[BODY_JOINTS.index('left_shoulder_roll_joint')] = 350.0
        self.ki[BODY_JOINTS.index('right_shoulder_roll_joint')] = 350.0
        # gain for shoulder yaw
        self.ki[BODY_JOINTS.index('left_shoulder_yaw_joint')] = 100.0
        self.ki[BODY_JOINTS.index('right_shoulder_yaw_joint')] = 100.0
        # gain for elbow
        self.ki[BODY_JOINTS.index('left_elbow_joint')] = 100.0
        self.ki[BODY_JOINTS.index('right_elbow_joint')] = 100.0
        # gain for wrist
        self.ki[BODY_JOINTS.index('left_wrist_pitch_joint')] = 100.0
        self.ki[BODY_JOINTS.index('left_wrist_roll_joint')] = 100.0
        self.ki[BODY_JOINTS.index('left_wrist_yaw_joint')] = 100.0
        self.ki[BODY_JOINTS.index('right_wrist_pitch_joint')] = 100.0
        self.ki[BODY_JOINTS.index('right_wrist_roll_joint')] = 100.0
        self.ki[BODY_JOINTS.index('right_wrist_yaw_joint')] = 100.0

        # damp all joints except torso
        self.damp_mode(6.0)
        # fix torso joint
        torso_idx = BODY_JOINTS.index('torso_joint')
        self.command_publisher.q[torso_idx] = 0.0
        self.command_publisher.dq[torso_idx] = 0.0
        self.command_publisher.tau[torso_idx] = 0.0
        self.command_publisher.kp[torso_idx] = 200.0
        self.command_publisher.kd[torso_idx] = 10.0

    def gravity_compensation_step(self):
        left_wrench = self.robot_model.get_frame_wrench(self.left_ee_name)
        right_wrench = self.robot_model.get_frame_wrench(self.right_ee_name)
        left_force = np.linalg.norm(left_wrench[:3])
        right_force = np.linalg.norm(right_wrench[:3])
        left_torque = np.linalg.norm(left_wrench[3:])
        right_torque = np.linalg.norm(right_wrench[3:])
        # threshold for left shoulder pitch
        if left_force > 24.0:
            self.dq_i[BODY_JOINTS.index('left_shoulder_pitch_joint')] = 0.0
        # threshold for left shoulder roll
        if left_force > 30.0:
            self.dq_i[BODY_JOINTS.index('left_shoulder_roll_joint')] = 0.0
        # threshold for left shoulder yaw
        if left_force > 18.0:
            self.dq_i[BODY_JOINTS.index('left_shoulder_yaw_joint')] = 0.0
        # threshold for left elbow joints
        if left_force > 20.0:
            self.dq_i[BODY_JOINTS.index('left_elbow_joint')] = 0.0
        # threshold for left wrist joints
        if left_force > 12.0:
            self.dq_i[BODY_JOINTS.index('left_wrist_roll_joint')] = 0.0
            self.dq_i[BODY_JOINTS.index('left_wrist_pitch_joint')] = 0.0
            self.dq_i[BODY_JOINTS.index('left_wrist_yaw_joint')] = 0.0

        # threshold for right shoulder pitch
        if right_force > 24.0:
            self.dq_i[BODY_JOINTS.index('right_shoulder_pitch_joint')] = 0.0
        # threshold for right shoulder roll
        if right_force > 30.0:
            self.dq_i[BODY_JOINTS.index('right_shoulder_roll_joint')] = 0.0
        # threshold for right shoulder yaw
        if right_force > 18.0:
            self.dq_i[BODY_JOINTS.index('right_shoulder_yaw_joint')] = 0.0
        # threshold for right elbow joints
        if right_force > 20.0:
            self.dq_i[BODY_JOINTS.index('right_elbow_joint')] = 0.0
        # threshold for right wrist joints
        if right_force > 12.0:
            # self.dq_i[24:27] = 0.0
            self.dq_i[BODY_JOINTS.index('right_wrist_roll_joint')] = 0.0
            self.dq_i[BODY_JOINTS.index('right_wrist_pitch_joint')] = 0.0
            self.dq_i[BODY_JOINTS.index('right_wrist_yaw_joint')] = 0.0

        # threshold for left shoulder yaw joints
        if left_torque > 4.0:
            self.dq_i[BODY_JOINTS.index('left_shoulder_yaw_joint')] = 0.0
        # threshold for left elbow roll joints
        if left_torque > 2.0:
            self.dq_i[BODY_JOINTS.index('left_elbow_roll_joint')] = 0.0
        # threshold for right shoulder yaw joints
        if right_torque > 4.0:
            self.dq_i[BODY_JOINTS.index('right_shoulder_yaw_joint')] = 0.0
        # threshold for right elbow roll joints
        if right_torque > 2.0:
            self.dq_i[BODY_JOINTS.index('right_elbow_roll_joint')] = 0.0

        # integrate dq
        self.dq_i += self.robot_model.state['dq'] * self.dt

        # compute tau for gravity compensation using model_body
        tau = pin.computeGeneralizedGravity(
            self.robot_model.model_body,
            self.robot_model.data_body,
            self.robot_model.state['q']
        )
        # gravity comp tau + i control on dq
        self.command_publisher.tau = tau - self.ki * self.dq_i

        self.update_robot_model()
