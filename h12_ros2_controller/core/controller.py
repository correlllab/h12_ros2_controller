import time
import numpy as np
import tkinter as tk

import pink
import qpsolvers
import pinocchio as pin

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.utils.thread import RecurrentThread

from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import CommandPublisher
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.path_definition import URDF_SPHERE_PATH, SRDF_SPHERE_PATH, MODEL_H12_PATH

class ArmController:
    def __init__(self, filename,
                 dt=0.02, vlim=1.0, wlim=3.0, visualize=False):
        # initialize robot model
        self.robot_model = RobotModel(filename)
        self.dt = dt
        self.vlim = vlim
        self.wlim = wlim
        self.visualize = visualize

        # initialize channel
        ChannelFactoryInitialize()

        # initialize subscriber in robot model
        self.robot_model.init_subscriber()
        time.sleep(0.5)
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()

        # define enabled ids and frozen ids
        motor_ids = np.array([i for i in range(13, 27)])
        self.robot_model.init_reduced_model(ENABLED_JOINTS)

        # initialize command publisher for upper body motors
        self.command_publisher = CommandPublisher()

        # gain for shoulder
        self.command_publisher.kp[13:16] = 180.0
        self.command_publisher.kd[13:16] = 3.0
        self.command_publisher.kp[20:23] = 180.0
        self.command_publisher.kd[20:23] = 3.0
        # gain for elbow
        self.command_publisher.kp[16:18] = 150.0
        self.command_publisher.kd[16:18] = 3.0
        self.command_publisher.kp[23:25] = 150.0
        self.command_publisher.kd[23:25] = 3.0
        # gain for wrist
        self.command_publisher.kp[18:20] = 50.0
        self.command_publisher.kd[18:20] = 2.0
        self.command_publisher.kp[25:27] = 50.0
        self.command_publisher.kd[25:27] = 2.0
        # enable upper body motors
        init_q = self.robot_model.q_reduced
        self.command_publisher.enable_motor(motor_ids, init_q)

        # enable torso motor such that it's locked in pace
        self.command_publisher.kp[12] = 100.0
        self.command_publisher.kd[12] = 3.0
        self.command_publisher.enable_motor([12], [self.robot_model.q[12]])
        self.command_publisher.start_publisher()

        # initialize IK tasks
        # left arm end effector task
        self.left_ee_name = 'left_wrist_yaw_link'
        self.left_ee_id = self.robot_model.model.getFrameId(self.left_ee_name)
        self.left_ee_task = pink.FrameTask(
            self.left_ee_name,
            position_cost=50.0,
            orientation_cost=30.0,
            lm_damping=1.0
        )
        # right arm end effector task
        self.right_ee_name = 'right_wrist_yaw_link'
        self.right_ee_id = self.robot_model.model.getFrameId(self.right_ee_name)
        self.right_ee_task = pink.FrameTask(
            self.right_ee_name,
            position_cost=50.0,
            orientation_cost=30.0,
            lm_damping=1.0
        )
        # posture task as regularization
        self.posture_task = pink.PostureTask(
            cost=1e-3
        )
        # variable for easier access
        self._left_arm_action = np.zeros(7)
        self._right_arm_action = np.zeros(7)

        # joint level task for joint-level control (goto_configuration)
        self.joint_task = pink.PostureTask(
            cost=30.0
        )

        # # sphere self collision
        # self.sphere_model, _, self.collision_model = pin.buildModelsFromUrdf(
        #     filename=f'{root_path}/assets/h1_2/h1_2_sphere.urdf',
        #     package_dirs=f'{root_path}/assets/h1_2',
        # )
        # self.collision_data = pink.utils.process_collision_pairs(
        #     self.sphere_model,
        #     self.collision_model,
        #     f'{root_path}/assets/h1_2/h1_2_sphere_collision.srdf',
        # )
        # # mesh self collision
        # self.collision_model = self.robot_model.collision_model
        # self.collision_data = pink.utils.process_collision_pairs(
        #     self.robot_model.model,
        #     self.robot_model.collision_model,
        #     f'{root_path}/assets/h1_2/h1_2_collision.srdf',
        # )

        # reduced sphere self collision
        self.sphere_model, _, self.collision_model = pin.buildModelsFromUrdf(
            filename=URDF_SPHERE_PATH,
            package_dirs=MODEL_H12_PATH,
        )
        self.sphere_model_reduced, self.collision_model_reduced = pin.buildReducedModel(
            self.sphere_model,
            self.collision_model,
            self.robot_model.frozen_ids,
            self.robot_model.zero_q
        )
        self.collision_data_reduced = pink.utils.process_collision_pairs(
            self.sphere_model_reduced,
            self.collision_model_reduced,
            SRDF_SPHERE_PATH,
        )
        # # reduced mesh self collision
        # self.collision_model_reduced = self.robot_model.collision_model_reduced
        # self.collision_data_reduced = pink.utils.process_collision_pairs(
        #     self.robot_model.reduced_model,
        #     self.robot_model.collision_model_reduced,
        #     f'{root_path}/assets/h1_2/h1_2_collision.srdf'
        # )

        # configuration trakcing robot states
        self.configuration = pink.Configuration(
            self.robot_model.model,
            self.robot_model.data,
            self.robot_model.zero_q,
            # collision_model=self.collision_model,
            # collision_data=self.collision_data
        )
        self.reduced_configuration = pink.Configuration(
            self.robot_model.model_reduced,
            self.robot_model.data_reduced,
            self.robot_model.zero_q_reduced,
            collision_model=self.collision_model_reduced,
            collision_data=self.collision_data_reduced
        )

        # # full collision barriers
        # self.collision_barrier = pink.barriers.SelfCollisionBarrier(
        #     n_collision_pairs=len(self.collision_model.collisionPairs),
        #     gain=20.0,
        #     safe_displacement_gain=1.0,
        #     d_min=0.05,
        # )
        # reduced collision barriers
        self.collision_barrier = pink.barriers.SelfCollisionBarrier(
            n_collision_pairs=len(self.collision_model_reduced.collisionPairs),
            gain=20.0,
            safe_displacement_gain=1.0,
            d_min=0.05,
        )

        # # spherical collision barriers
        # self.ee_barrier = pink.barriers.BodySphericalBarrier(
        #     ('left_wrist_yaw_link', 'right_wrist_yaw_link'),
        #     d_min=0.2,
        #     gain = 100.0,
        #     safe_displacement_gain=1.0
        # )

        # set initial target for all tasks
        self.tasks = [self.left_ee_task, self.right_ee_task, self.posture_task]
        for task in self.tasks:
            task.set_target_from_configuration(self.configuration)
        # set collision barrier
        # self.barriers = []
        # self.barriers = [self.ee_barrier]
        self.barriers = [self.collision_barrier]
        # select solver
        self.solver = qpsolvers.available_solvers[0]
        if 'osqp' in qpsolvers.available_solvers:
            self.solver = 'osqp'

        if self.visualize:
            self.robot_model.init_visualizer()

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
        return np.copy(self.robot_model.q[13:20])

    @property
    def right_arm_q(self):
        return np.copy(self.robot_model.q[32:39])

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
        return np.concatenate([self.left_ee_position,
                               self.left_ee_rpy])

    '''
    left end effector properties
    left_ee_transformation: transformation matrix of the left end effector
    left_ee_position: position of the left end effector
    left_ee_rotation: rotation matrix of the left end effector
    left_ee_rpy: roll, pitch, yaw of the left end effector
    left_ee_pose: pose of the left end effector (x, y, z, roll, pitch, yaw)
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
        return np.concatenate([self.right_ee_position,
                               self.right_ee_rpy])

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
        return self.left_ee_task.transform_target_to_world.np

    @left_ee_target_transformation.setter
    def left_ee_target_transformation(self, transformation):
        assert(transformation.shape == (4, 4)), 'Transformation should be a 4x4 matrix.'
        self.left_ee_task.transform_target_to_world = pin.SE3(transformation)

    @property
    def left_ee_target_position(self):
        return self.left_ee_task.transform_target_to_world.translation

    @left_ee_target_position.setter
    def left_ee_target_position(self, position):
        assert(len(position) == 3), 'Position should be a list of 3 elements (x, y, z).'
        self.left_ee_task.transform_target_to_world.translation = np.array(position)

    @property
    def left_ee_target_rotation(self):
        return self.left_ee_task.transform_target_to_world.rotation

    @left_ee_target_rotation.setter
    def left_ee_target_rotation(self, rotation):
        assert(rotation.shape == (3, 3)), 'Rotation should be a 3x3 matrix.'
        self.left_ee_task.transform_target_to_world.rotation = rotation

    @property
    def left_ee_target_rpy(self):
        return pin.rpy.matrixToRpy(self.left_ee_target_rotation)

    @left_ee_target_rpy.setter
    def left_ee_target_rpy(self, rpy):
        assert(len(rpy) == 3), 'Rpy should be a list of 3 elements (roll, pitch, yaw).'
        self.left_ee_target_rotation = pin.rpy.rpyToMatrix(np.array(rpy))

    @property
    def left_ee_target_pose(self):
        return np.concatenate([self.left_ee_target_position,
                               self.left_ee_target_rpy])

    @left_ee_target_pose.setter
    def left_ee_target_pose(self, pose):
        assert(len(pose) == 6), 'Pose should be a list of 6 elements (x, y, z, roll, pitch, yaw).'
        self.left_ee_target_position = pose[:3]
        self.left_ee_target_rpy = pose[3:]

    @property
    def left_ee_error(self):
        return self.left_ee_task.compute_error(self.reduced_configuration)

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
        return self.right_ee_task.transform_target_to_world.np

    @right_ee_target_transformation.setter
    def right_ee_target_transformation(self, transformation):
        assert(transformation.shape == (4, 4)), 'Transformation should be a 4x4 matrix.'
        self.right_ee_task.transform_target_to_world = pin.SE3(transformation)

    @property
    def right_ee_target_position(self):
        return self.right_ee_task.transform_target_to_world.translation

    @right_ee_target_position.setter
    def right_ee_target_position(self, position):
        assert(len(position) == 3), 'Position should be a list of 3 elements (x, y, z).'
        self.right_ee_task.transform_target_to_world.translation = np.array(position)

    @property
    def right_ee_target_rotation(self):
        return self.right_ee_task.transform_target_to_world.rotation

    @right_ee_target_rotation.setter
    def right_ee_target_rotation(self, rotation):
        assert(rotation.shape == (3, 3)), 'Rotation should be a 3x3 matrix.'
        self.right_ee_task.transform_target_to_world.rotation = rotation

    @property
    def right_ee_target_rpy(self):
        return pin.rpy.matrixToRpy(self.right_ee_target_rotation)

    @right_ee_target_rpy.setter
    def right_ee_target_rpy(self, rpy):
        assert(len(rpy) == 3), 'Rpy should be a list of 3 elements (roll, pitch, yaw).'
        self.right_ee_target_rotation = pin.rpy.rpyToMatrix(np.array(rpy))

    @property
    def right_ee_target_pose(self):
        return np.concatenate([self.right_ee_target_position,
                               self.right_ee_target_rpy])

    @right_ee_target_pose.setter
    def right_ee_target_pose(self, pose):
        assert(len(pose) == 6), 'Pose should be a list of 6 elements (x, y, z, roll, pitch, yaw).'
        self.right_ee_target_position = pose[:3]
        self.right_ee_target_rpy = pose[3:]

    @property
    def right_ee_error(self):
        return self.right_ee_task.compute_error(self.reduced_configuration)

    def sync_robot_model(self):
        # sync robot model and compute forward kinematics
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()

    def update_robot_model(self):
        # update visualizer if needed
        if self.visualize:
            self.robot_model.update_visualizer()
            self.robot_model.visualize_wrench(self.left_ee_name)
        # update configuration
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

    def limit_joint_vel(self, vel):
        # get end effector twist
        twist_left = self.robot_model.compute_frame_twist(self.left_ee_name, vel)
        twist_right = self.robot_model.compute_frame_twist(self.right_ee_name, vel)
        # compute end effector velocity and angular velocity
        v_left, w_left = twist_left[:3], twist_left[3:]
        v_right, w_right = twist_right[:3], twist_right[3:]
        # limit end effector velocity and angular velocity
        v_scaler = np.min([1.0,
                           self.vlim / (np.linalg.norm(v_left) + 1e-3),
                           self.vlim / (np.linalg.norm(v_right) + 1e-3)])
        w_scaler = np.min([1.0,
                           self.wlim / (np.linalg.norm(w_left) + 1e-3),
                           self.wlim / (np.linalg.norm(w_right) + 1e-3)])

        return np.min([v_scaler, w_scaler]) * vel

    def apply_joint_vel(self, vel):
        # solve dynamics
        tau = pin.rnea(self.robot_model.model,
                       self.robot_model.data,
                       self.robot_model.q + vel * self.dt,
                       self.robot_model.dq,
                       np.zeros(self.robot_model.model.nv))

        # update joint action
        self._left_arm_action = vel[13:20] * self.dt
        self._right_arm_action = vel[32:39] * self.dt

        # send the velocity command to the robot
        self.command_publisher.q = (self.robot_model.q + vel * self.dt)[self.robot_model.body_q_ids]
        self.command_publisher.dq = vel[self.robot_model.body_q_ids]
        self.command_publisher.tau = tau[self.robot_model.body_q_ids]

    def lock_configuration(self, q):
        # sync and update robot model
        self.sync_robot_model()
        self.update_robot_model()

        # compute tau and enforce same q
        tau = pin.rnea(self.robot_model.model,
                       self.robot_model.data,
                       q,
                       np.zeros(self.robot_model.model.nv),
                       np.zeros(self.robot_model.model.nv))

        # send command to lock the robot in current configuration
        self.command_publisher.q = q
        self.command_publisher.dq = np.zeros(self.robot_model.model.nv)
        self.command_publisher.tau = tau

    def goto_configuration(self, q):
        # sync and update robot model
        self.sync_robot_model()
        self.update_robot_model()

        # use the joint task to solve joint velocity update
        self.joint_task.set_target(q)
        vel = pink.solve_ik(
            self.configuration,
            [self.joint_task],
            dt=self.dt,
            solver=self.solver,
            barriers=[],
            safety_break=False
        )

        # apply the control
        vel = self.limit_joint_vel(vel)
        self.apply_joint_vel(vel)

    def solve_ik(self):
        # update posture task
        self.posture_task.set_target_from_configuration(self.configuration)

        # solve IK
        vel = pink.solve_ik(
            self.configuration,
            self.tasks,
            dt=self.dt,
            solver=self.solver,
            barriers=[],
            safety_break=False
        )

        vel = self.limit_joint_vel(vel)

        return vel

    def solve_reduced_ik(self):
        # update posture task
        self.posture_task.set_target_from_configuration(self.reduced_configuration)

        # solve IK
        vel = pink.solve_ik(
            self.reduced_configuration,
            self.tasks,
            dt=self.dt,
            solver=self.solver,
            barriers=self.barriers,
            safety_break=False
        )

        vel_full = np.zeros(self.robot_model.model.nv)
        vel_full[self.robot_model.reduced_mask] = vel
        print(f'raw vel: {vel}')
        vel_full = self.limit_joint_vel(vel_full)

        return vel_full

    def control_full_body_step(self):
        # t = time.time()
        # sync and update robot model
        self.sync_robot_model()
        self.update_robot_model()

        # solve IK and apply the control
        vel = self.solve_ik()
        self.apply_joint_vel(vel)

        # print(f'Time: {time.time() - t:.4f}s')

    def control_dual_arm_step(self):
        # t = time.time()
        # sync and update robot model
        self.sync_robot_model()
        self.update_robot_model()

        # solve IK and apply the control
        vel = self.solve_reduced_ik()
        print(f'scaled vel: {vel}')
        self.apply_joint_vel(vel)

        # print(f'Time: {time.time() - t:.4f}s')

    def sim_full_body_step(self):
        t = time.time()
        # update robot model
        self.update_robot_model()

        # solve IK and apply the control
        vel = self.solve_ik()
        self.robot_model._q = self.robot_model.q + vel * self.dt
        self.robot_model.update_kinematics()

        print(f'Time: {time.time() - t:.4f}s')

    def sim_dual_arm_step(self):
        t = time.time()
        self.update_robot_model()

        # solve IK and apply the control
        vel = self.solve_reduced_ik()
        self.robot_model._q = self.robot_model.q + vel * self.dt
        self.robot_model.update_kinematics()

        print(f'Time: {time.time() - t:.4f}s')

    def estop(self):
        self.command_publisher.estop()

    def damp_mode(self, kd=3.0):
        # zero out kp
        self.command_publisher.kp.fill(0.0)
        # gain on kd for damping
        self.command_publisher.kd.fill(kd)
        self.command_publisher.dq.fill(0.0)
        print(f'Set kp to zero, kd to {kd} and dq to 0')

    def gravity_compensation_step(self):
        # sync and update robot model
        self.sync_robot_model()
        self.update_robot_model()

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
        # sync and update robot model
        self.sync_robot_model()
        self.update_robot_model()

        # solve IK to get joint velocity
        vel = self.solve_reduced_ik()

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

        ## ! cursed code causing catastrophic failure
        # q = self.robot_model.q
        # q_target = self.robot_model.q + vel * self.dt
        # dq = self.robot_model.dq
        # # joint space control
        # tau_positional = 100 * (q_target - q) - 5 * dq
        ## ! cursed code causing catastrophic failure

        # apply the control
        self.command_publisher.tau = tau_cmd[self.robot_model.body_q_ids]
