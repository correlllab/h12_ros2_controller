import os
import time
import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import CommandPublisher, ArmSDKPublisher
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS, LEFT_ARM_INDEX, RIGHT_ARM_INDEX

class UpperController:
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 dt=0.02, v_lim=1.0, w_lim=2.0, dq_lim=2.0, d_min=0.02,
                 visualize=False,
                 use_sport_mode=False):
        # initialize robot model
        self.robot_model = RobotModel(urdf_path)
        self.dt = dt
        self.vlim = v_lim
        self.wlim = w_lim
        self.dq_lim = dq_lim
        self.dmin = d_min
        self.visualize = visualize

        # initialize subscriber in robot model
        self.robot_model.init_subscriber()
        time.sleep(0.5)
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()

        # define enabled ids and frozen ids
        motor_ids = np.array([i for i in range(13, 27)])
        self.robot_model.init_reduced_model(ENABLED_JOINTS)

        # initialize command publisher for upper body motors
        if use_sport_mode:
            self.command_publisher = ArmSDKPublisher()
        else:
            self.command_publisher = CommandPublisher()

        # gain for shoulder
        self.command_publisher.kp[13:15] = 200.0
        self.command_publisher.kd[13:15] = 6.0
        self.command_publisher.kp[20:22] = 200.0
        self.command_publisher.kd[20:22] = 6.0
        # gain for shoulder yaw
        self.command_publisher.kp[15] = 150.0
        self.command_publisher.kd[15] = 4.0
        self.command_publisher.kp[22] = 150.0
        self.command_publisher.kd[22] = 4.0
        # gain for elbow
        self.command_publisher.kp[16] = 150.0
        self.command_publisher.kd[16] = 4.0
        self.command_publisher.kp[23] = 150.0
        self.command_publisher.kd[23] = 4.0
        # gain for wrist
        self.command_publisher.kp[17:20] = 50.0
        self.command_publisher.kd[17:20] = 3.0
        self.command_publisher.kp[24:27] = 50.0
        self.command_publisher.kd[24:27] = 3.0
        # enable upper body motors
        init_q = self.robot_model.q_reduced
        self.command_publisher.enable_motor(motor_ids, init_q)

        # enable torso motor such that it's locked in pace
        self.command_publisher.kp[12] = 150.0
        self.command_publisher.kd[12] = 3.0
        self.command_publisher.enable_motor([12], [0.0])
        self.command_publisher.start_publisher()

        # initialize IK solver
        self.ik_solver = IKSolver(
            robot_model=self.robot_model,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            dt=self.dt,
            dmin=self.dmin
        )

        if self.visualize:
            self.robot_model.init_visualizer()

        # default end effector frame names for velocity limiting
        self.left_ee_name = 'left_wrist_yaw_link'
        self.right_ee_name = 'right_wrist_yaw_link'

    def limit_joint_vel(self, vel):
        # get end effector twist
        twist_left = self.robot_model.compute_frame_twist(self.left_ee_name, vel)
        twist_right = self.robot_model.compute_frame_twist(self.right_ee_name, vel)
        # compute end effector velocity and angular velocity
        v_left, w_left = twist_left[:3], twist_left[3:]
        v_right, w_right = twist_right[:3], twist_right[3:]

        # joint-level limit
        left_dq_scaler = np.min(self.dq_lim / (np.abs(vel[LEFT_ARM_INDEX]) + 1e-6))
        right_dq_scaler = np.min(self.dq_lim / (np.abs(vel[RIGHT_ARM_INDEX]) + 1e-6))

        # scale left and right end effectors
        left_scaler = np.min([
            1.0,
            left_dq_scaler,
            self.vlim / (np.linalg.norm(v_left) + 1e-6),
            self.wlim / (np.linalg.norm(w_left) + 1e-6)
        ])
        right_scaler = np.min([
            1.0,
            right_dq_scaler,
            self.vlim / (np.linalg.norm(v_right) + 1e-6),
            self.wlim / (np.linalg.norm(w_right) + 1e-6)
        ])

        vel_scaled = np.zeros_like(vel)
        vel_scaled[LEFT_ARM_INDEX] = left_scaler * vel[LEFT_ARM_INDEX]
        vel_scaled[RIGHT_ARM_INDEX] = right_scaler * vel[RIGHT_ARM_INDEX]

        return vel_scaled

    def goto_configuration(self, q):
        # sync robot model
        self.sync_robot_model()
        # use the IK solver to solve joint velocity update
        vel = self.ik_solver.goto_configuration(q)
        vel = self.limit_joint_vel(vel)
        self.apply_joint_vel(vel)

    def goto_reduced_configuration(self, q_reduced):
        # sync robot model
        self.sync_robot_model()
        # use the IK solver to solve joint velocity update
        vel = self.ik_solver.goto_reduced_configuration(q_reduced)
        vel = self.limit_joint_vel(vel)
        self.apply_joint_vel(vel)

    def sync_robot_model(self):
        # sync robot model and compute forward kinematics
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()
        # update visualizer if needed
        if self.visualize:
            self.ik_solver.update_visualizer()
            self.robot_model.update_visualizer()

    def apply_joint_vel(self, vel):
        # solve dynamics
        tau = pin.rnea(self.robot_model.model,
                       self.robot_model.data,
                       self.robot_model.q + vel * self.dt,
                       self.robot_model.dq,
                       np.zeros(self.robot_model.model.nv))

        # send the velocity command to the robot
        self.command_publisher.q = (self.robot_model.q + vel * self.dt)[self.robot_model.body_q_ids]
        self.command_publisher.dq = vel[self.robot_model.body_q_ids]
        self.command_publisher.tau = tau[self.robot_model.body_q_ids]

    def lock_configuration(self, q):
        # sync robot model
        self.sync_robot_model()
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

    def estop(self):
        self.command_publisher.estop()

    def shutdown(self):
        self.robot_model.shutdown()
        self.command_publisher.shutdown()

    def damp_mode(self, kd=3.0):
        # zero out kp
        self.command_publisher.kp.fill(0.0)
        # gain on kd for damping
        self.command_publisher.kd.fill(kd)
        self.command_publisher.dq.fill(0.0)
        print(f'Set kp to zero, kd to {kd} and dq to 0')
