import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.upper_controller import UpperController

class GravityCompController(UpperController):
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 dt=0.02, v_lim=1.0, w_lim=2.0, dq_lim=2.0, d_min=0.02,
                 visualize=False,
                 use_sport_mode=False):
        # initialize base controller
        super().__init__(urdf_path, urdf_sphere_path, srdf_sphere_path,
                         dt, v_lim, w_lim, dq_lim, d_min,
                         visualize, use_sport_mode)

        # i control on dq
        self.dq_i = np.zeros(self.robot_model.model.nv)
        self.ki = np.zeros(self.robot_model.model.nv)
        # gain for shoulder pitch
        self.ki[13] = 250.0
        self.ki[20] = 250.0
        # gain for shoulder roll
        self.ki[14] = 250.0
        self.ki[21] = 250.0
        # gain for shoulder yaw
        self.ki[15] = 100.0
        self.ki[22] = 100.0
        # gain for elbow
        self.ki[16] = 100.0
        self.ki[23] = 100.0
        # gain for wrist
        self.ki[17:20] = 80.0
        self.ki[24:27] = 80.0

        # damp all joints except torso
        self.damp_mode(6.0)
        # fix torso joint
        self.command_publisher.q[12] = 0.0
        self.command_publisher.dq[12] = 0.0
        self.command_publisher.tau[12] = 0.0
        self.command_publisher.kp[12] = 200.0
        self.command_publisher.kd[12] = 10.0

    def gravity_compensation_step(self):
        left_wrench = self.robot_model.get_frame_wrench(self.left_ee_name)
        right_wrench = self.robot_model.get_frame_wrench(self.right_ee_name)
        left_force = np.linalg.norm(left_wrench[:3])
        right_force = np.linalg.norm(right_wrench[:3])
        left_torque = np.linalg.norm(left_wrench[3:])
        right_torque = np.linalg.norm(right_wrench[3:])
        # threshold for left shoulder joints
        if left_force > 24.0:
            self.dq_i[13:16] = 0.0
        # threshold for left elbow joints
        if left_force > 20.0:
            self.dq_i[16:18] = 0.0
        # threshold for left wrist joints
        if left_force > 12.0:
            self.dq_i[18:20] = 0.0
        # threshold for right shoulder joints
        if right_force > 24.0:
            self.dq_i[20:23] = 0.0
        # threshold for right elbow joints
        if right_force > 20.0:
            self.dq_i[23:25] = 0.0
        # threshold for right wrist joints
        if right_force > 12.0:
            self.dq_i[25:27] = 0.0

        # threshold for left shoulder yaw joints
        if left_torque > 4.0:
            self.dq_i[15] = 0.0
        # threshold for left elbow roll joints
        if left_torque > 2.0:
            self.dq_i[17] = 0.0
        # threshold for right shoulder yaw joints
        if right_torque > 4.0:
            self.dq_i[22] = 0.0
        # threshold for right elbow roll joints
        if right_torque > 2.0:
            self.dq_i[24] = 0.0

        # integrate dq
        self.dq_i += self.robot_model.state['dq'] * self.dt

        # compute tau for gravity compensation
        tau = pin.computeGeneralizedGravity(
            self.robot_model.model,
            self.robot_model.data,
            self.robot_model.state['q']
        )
        # gravity comp tau + i control on dq
        self.command_publisher.tau = tau - self.ki * self.dq_i

        self.update_robot_model()
