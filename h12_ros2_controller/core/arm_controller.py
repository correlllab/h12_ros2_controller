import os
import time
import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.upper_controller import UpperController
from h12_ros2_controller.utility.joint_definition import LEFT_ARM_INDEX, RIGHT_ARM_INDEX

class ArmController(UpperController):
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

        # add frame tasks to IK solver
        self.ik_solver.add_frame_task('left_ee', self.left_ee_name)
        self.ik_solver.add_frame_task('right_ee', self.right_ee_name)

        # set initial targets for IK solver from current configuration
        self.ik_solver.update_configurations()
        self.ik_solver.set_from_configuration()

        # variable for easier access
        self._left_arm_action = np.zeros(7)
        self._right_arm_action = np.zeros(7)

        # i control on dq
        self.dq_i = np.zeros(self.robot_model.model.nv)
        self.ki = np.zeros(self.robot_model.model.nv)
        # gain for shoulder
        self.ki[13:16] = 320.0
        self.ki[32:35] = 320.0
        # gain for elbow
        self.ki[16:18] = 220.0
        self.ki[35:37] = 220.0
        # gain for wrist
        self.ki[18:20] = 120.0
        self.ki[37:39] = 120.0

    '''
    joint position for left and right arms
    '''
    @property
    def left_arm_q(self):
        return np.copy(self.robot_model.q[LEFT_ARM_INDEX])

    @property
    def right_arm_q(self):
        return np.copy(self.robot_model.q[RIGHT_ARM_INDEX])

    '''
    joint action for left and right arms
    '''
    @property
    def left_arm_action(self):
        return np.copy(self._left_arm_action)

    @property
    def right_arm_action(self):
        return np.copy(self._right_arm_action)

    '''
    left end effector properties
    left_ee_transformation: transformation matrix of the left end effector
    left_ee_position: position of the left end effector
    left_ee_rotation: rotation matrix of the left end effector
    left_ee_rpy: roll, pitch, yaw of the left end effector
    left_ee_pose: pose of the left end effector (x, y, z, roll, pitch, yaw)
    '''
    @property
    def left_ee_transformation(self):
        return self.robot_model.get_frame_transformation(self.left_ee_name)

    @property
    def left_ee_position(self):
        return self.robot_model.get_frame_position(self.left_ee_name)

    @property
    def left_ee_rotation(self):
        return self.robot_model.get_frame_rotation(self.left_ee_name)

    @property
    def left_ee_rpy(self):
        return pin.rpy.matrixToRpy(self.left_ee_rotation)

    @property
    def left_ee_pose(self):
        return np.concatenate([self.left_ee_position, self.left_ee_rpy])

    '''
    right end effector properties
    right_ee_transformation: transformation matrix of the right end effector
    right_ee_position: position of the right end effector
    right_ee_rotation: rotation matrix of the right end effector
    right_ee_rpy: roll, pitch, yaw of the right end effector
    right_ee_pose: pose of the right end effector (x, y, z, roll, pitch, yaw)
    '''
    @property
    def right_ee_transformation(self):
        return self.robot_model.get_frame_transformation(self.right_ee_name)

    @property
    def right_ee_position(self):
        return self.robot_model.get_frame_position(self.right_ee_name)

    @property
    def right_ee_rotation(self):
        return self.robot_model.get_frame_rotation(self.right_ee_name)

    @property
    def right_ee_rpy(self):
        return pin.rpy.matrixToRpy(self.right_ee_rotation)

    @property
    def right_ee_pose(self):
        return np.concatenate([self.right_ee_position, self.right_ee_rpy])

    '''
    left end effector target properties
    left_ee_target_transformation: transformation matrix of the left end effector target
    left_ee_target_position: position of the left end effector target
    left_ee_target_rotation: rotation matrix of the left end effector target
    left_ee_target_rpy: roll, pitch, yaw of the left end effector target
    left_ee_target_pose: pose of the left end effector target (x, y, z, roll, pitch, yaw)
    left_ee_error: error between the current left end effector and the target
    '''
    @property
    def left_ee_target_transformation(self):
        task = self.ik_solver.frame_tasks.get('left_ee')
        return np.copy(task.transform_target_to_world.np)

    @left_ee_target_transformation.setter
    def left_ee_target_transformation(self, transformation):
        assert(transformation.shape == (4, 4)), 'Transformation should be a 4x4 matrix.'
        task = self.ik_solver.frame_tasks.get('left_ee')
        task.transform_target_to_world = pin.SE3(transformation)

    @property
    def left_ee_target_position(self):
        task = self.ik_solver.frame_tasks.get('left_ee')
        return np.copy(task.transform_target_to_world.translation)

    @left_ee_target_position.setter
    def left_ee_target_position(self, position):
        assert(len(position) == 3), 'Position should be a list of 3 elements (x, y, z).'
        task = self.ik_solver.frame_tasks.get('left_ee')
        task.transform_target_to_world.translation = np.array(position)

    @property
    def left_ee_target_rotation(self):
        task = self.ik_solver.frame_tasks.get('left_ee')
        return np.copy(task.transform_target_to_world.rotation)

    @left_ee_target_rotation.setter
    def left_ee_target_rotation(self, rotation):
        assert(rotation.shape == (3, 3)), 'Rotation should be a 3x3 matrix.'
        task = self.ik_solver.frame_tasks.get('left_ee')
        task.transform_target_to_world.rotation = np.array(rotation)

    @property
    def left_ee_target_rpy(self):
        return pin.rpy.matrixToRpy(self.left_ee_target_rotation)

    @left_ee_target_rpy.setter
    def left_ee_target_rpy(self, rpy):
        assert(len(rpy) == 3), 'Rpy should be a list of 3 elements (roll, pitch, yaw).'
        self.left_ee_target_rotation = pin.rpy.rpyToMatrix(np.array(rpy))

    @property
    def left_ee_target_pose(self):
        position = self.left_ee_target_position
        rpy = self.left_ee_target_rpy
        return np.concatenate([position, rpy])

    @left_ee_target_pose.setter
    def left_ee_target_pose(self, pose):
        assert(len(pose) == 6), 'Pose should be a list of 6 elements (x, y, z, roll, pitch, yaw).'
        self.left_ee_target_position = pose[:3]
        self.left_ee_target_rpy = pose[3:]

    @property
    def left_ee_error(self):
        task = self.ik_solver.frame_tasks.get('left_ee')
        return task.compute_error(self.ik_solver.reduced_configuration)

    '''
    right end effector target properties
    right_ee_target_transformation: transformation matrix of the right end effector target
    right_ee_target_position: position of the right end effector target
    right_ee_target_rotation: rotation matrix of the right end effector target
    right_ee_target_rpy: roll, pitch, yaw of the right end effector target
    right_ee_target_pose: pose of the right end effector target (x, y, z, roll, pitch, yaw)
    right_ee_error: error between the current right end effector and the target
    '''
    @property
    def right_ee_target_transformation(self):
        task = self.ik_solver.frame_tasks.get('right_ee')
        return np.copy(task.transform_target_to_world.np)

    @right_ee_target_transformation.setter
    def right_ee_target_transformation(self, transformation):
        assert(transformation.shape == (4, 4)), 'Transformation should be a 4x4 matrix.'
        task = self.ik_solver.frame_tasks.get('right_ee')
        task.transform_target_to_world = pin.SE3(transformation)

    @property
    def right_ee_target_position(self):
        task = self.ik_solver.frame_tasks.get('right_ee')
        return np.copy(task.transform_target_to_world.translation)

    @right_ee_target_position.setter
    def right_ee_target_position(self, position):
        assert(len(position) == 3), 'Position should be a list of 3 elements (x, y, z).'
        task = self.ik_solver.frame_tasks.get('right_ee')
        task.transform_target_to_world.translation = np.array(position)

    @property
    def right_ee_target_rotation(self):
        task = self.ik_solver.frame_tasks.get('right_ee')
        return np.copy(task.transform_target_to_world.rotation)

    @right_ee_target_rotation.setter
    def right_ee_target_rotation(self, rotation):
        assert(rotation.shape == (3, 3)), 'Rotation should be a 3x3 matrix.'
        task = self.ik_solver.frame_tasks.get('right_ee')
        task.transform_target_to_world.rotation = np.array(rotation)

    @property
    def right_ee_target_rpy(self):
        return pin.rpy.matrixToRpy(self.right_ee_target_rotation)

    @right_ee_target_rpy.setter
    def right_ee_target_rpy(self, rpy):
        assert(len(rpy) == 3), 'Rpy should be a list of 3 elements (roll, pitch, yaw).'
        self.right_ee_target_rotation = pin.rpy.rpyToMatrix(np.array(rpy))

    @property
    def right_ee_target_pose(self):
        position = self.right_ee_target_position
        rpy = self.right_ee_target_rpy
        return np.concatenate([position, rpy])

    @right_ee_target_pose.setter
    def right_ee_target_pose(self, pose):
        assert(len(pose) == 6), 'Pose should be a list of 6 elements (x, y, z, roll, pitch, yaw).'
        self.right_ee_target_position = pose[:3]
        self.right_ee_target_rpy = pose[3:]

    @property
    def right_ee_error(self):
        task = self.ik_solver.frame_tasks.get('right_ee')
        return task.compute_error(self.ik_solver.reduced_configuration)

    @property
    def joint_error(self):
        return self.ik_solver.config_task.compute_error(self.ik_solver.configuration)

    @property
    def joint_error_reduced(self):
        return self.ik_solver.config_task.compute_error(self.ik_solver.reduced_configuration)

    def sync_robot_model(self):
        # call parent sync method
        super().sync_robot_model()
        # add end effector specific visualization
        if self.visualize:
            self.robot_model.visualize_wrench(self.left_ee_name)

    def apply_joint_vel(self, vel):
        # call parent method
        super().apply_joint_vel(vel)

        # update joint action for end effectors
        self._left_arm_action = vel[LEFT_ARM_INDEX] * self.dt
        self._right_arm_action = vel[RIGHT_ARM_INDEX] * self.dt

    def control_full_body_step(self):
        # sync robot model
        self.sync_robot_model()
        # solve IK and apply the control
        vel = self.ik_solver.ik_step()
        vel = self.limit_joint_vel(vel)
        self.apply_joint_vel(vel)

    def control_dual_arm_step(self):
        # sync robot model
        self.sync_robot_model()
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced()
        vel = self.limit_joint_vel(vel)
        self.apply_joint_vel(vel)

        # enforce torso neutral position
        self.command_publisher.q[12] = 0

    def sim_full_body_step(self):
        # solve IK and apply the control
        vel = self.ik_solver.ik_step()
        vel = self.limit_joint_vel(vel)
        self.apply_joint_vel(vel)
        # directly update the robot model for visualization
        self.ik_solver.update_visualizer()
        self.robot_model._q = self.robot_model.q + vel * self.dt
        self.robot_model.update_kinematics()
        self.robot_model.update_visualizer()

    def sim_dual_arm_step(self):
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced()
        vel = self.limit_joint_vel(vel)
        self.apply_joint_vel(vel)
        # directly update the robot model for visualization
        self.ik_solver.update_visualizer()
        self.robot_model._q = self.robot_model.q + vel * self.dt
        self.robot_model.update_kinematics()
        self.robot_model.update_visualizer()

    def gravity_compensation_step(self):
        # sync robot model
        self.sync_robot_model()

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
            self.dq_i[32:35] = 0.0
        # threshold for right elbow joints
        if right_force > 20.0:
            self.dq_i[35:37] = 0.0
        # threshold for right wrist joints
        if right_force > 12.0:
            self.dq_i[37:39] = 0.0

        # threshold for left shoulder yaw joints
        if left_torque > 4.0:
            self.dq_i[15] = 0.0
        # threshold for left elbow roll joints
        if left_torque > 2.0:
            self.dq_i[17] = 0.0
        # threshold for right shoulder yaw joints
        if right_torque > 4.0:
            self.dq_i[34] = 0.0
        # threshold for right elbow roll joints
        if right_torque > 2.0:
            self.dq_i[36] = 0.0

        # integral dq
        self.dq_i += self.robot_model.dq * self.dt

        # compute tau for gravity compensation
        tau = pin.computeGeneralizedGravity(
            self.robot_model.model,
            self.robot_model.data,
            self.robot_model.q
        )

        self.command_publisher.tau = (tau - self.ki * self.dq_i)[self.robot_model.body_q_ids]

    def impedance_step(self, x_target):
        # sync robot model
        self.sync_robot_model()

        # solve IK to get joint velocity
        vel = self.ik_solver.ik_step_reduced()

        # get states in Cartesian space
        x = self.left_ee_position
        dx = self.robot_model.get_frame_twist(self.left_ee_name)[0:3]

        # spring damper
        kp = np.array([100.0, 100.0, 100.0])
        kd = np.array([5.0, 5.0, 5.0])
        x_target = self.left_ee_target_position
        F = kp * (x_target - x) + kd * (-dx)

        # inverse dynamics
        J_left = self.robot_model.get_frame_jacobian(self.left_ee_name)
        tau = J_left.T @ np.concatenate([F, np.zeros(3)])
        tau_gravity = pin.computeGeneralizedGravity(
            self.robot_model.model,
            self.robot_model.data,
            self.robot_model.q
        )
        tau_cmd = tau + tau_gravity

        # apply the control
        self.command_publisher.tau = tau_cmd[self.robot_model.body_q_ids]
