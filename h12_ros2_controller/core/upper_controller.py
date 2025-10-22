import os
import time
import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import CommandPublisher, ArmSDKPublisher
from h12_ros2_controller.utility.robot_setting import setup_gains_mj, setup_gains_real
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, UPPER_BODY_JOINTS, ENABLED_JOINTS, LEFT_ARM_INDEX, RIGHT_ARM_INDEX

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
        self.robot_model.update_kinematics()

        # init reduced model and collision model
        self.robot_model.init_reduced_model(ENABLED_JOINTS)
        self.robot_model.init_collision_model(urdf_sphere_path, srdf_sphere_path)
        self.enabled_ids = [BODY_JOINTS.index(joint) for joint in ENABLED_JOINTS]
        self.upper_ids = [BODY_JOINTS.index(joint) for joint in UPPER_BODY_JOINTS]

        # initialize command publisher for upper body motors
        if use_sport_mode:
            self.command_publisher = ArmSDKPublisher()
        else:
            self.command_publisher = CommandPublisher()

        # setup_gains_mj(self.command_publisher)
        setup_gains_real(self.command_publisher)

        # enable upper body motors
        init_q = self.robot_model.state_reduced['q']
        self.command_publisher.enable_motor(self.enabled_ids, init_q)

        # enable torso motor such that it's locked in place
        self.command_publisher.enable_motor([BODY_JOINTS.index('torso_joint')], [0.0])
        self.command_publisher.start_publisher()

        # initialize IK solver
        self.ik_solver = IKSolver(
            robot_model=self.robot_model,
            dt=self.dt,
            dmin=self.dmin
        )

        if self.visualize:
            self.robot_model.init_visualizer()

        # default end effector frame names for velocity limiting
        self.left_ee_name = 'left_wrist_yaw_link'
        self.right_ee_name = 'right_wrist_yaw_link'

        # variables recording states
        self.recording = False
        self.com_arr = []
        self.q_arr = []
        self.dq_arr = []
        self.tau_arr = []
        self.q_cmd_arr = []
        self.dq_cmd_arr = []
        self.tau_cmd_arr = []
        self.torque_cmd_arr = []

    '''
    joint position for left and right arms
    '''
    @property
    def left_arm_q(self):
        return np.copy(self.robot_model.state['q'][LEFT_ARM_INDEX])

    @property
    def right_arm_q(self):
        return np.copy(self.robot_model.state['q'][RIGHT_ARM_INDEX])

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

    def update_ik_solver(self):
        # update IK solver with the latest robot configuration
        self.ik_solver.update_configurations()

    def update_robot_model(self):
        # update kinematics
        self.robot_model.update_kinematics()
        # update visualizer if needed
        if self.visualize:
            self.ik_solver.update_visualizer()
            self.robot_model.update_visualizer()
            # visualize center of mass
            self.robot_model.visualize_center_of_mass()

    def goto_configuration(self, q):
        # solve IK and apply control
        vel = self.ik_solver.goto_configuration(q)
        vel = self.limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self.apply_joint_position(self.ik_solver.q)
        self.update_robot_model()

    def sim_goto_configuration(self, q):
        # solve IK and apply control
        vel = self.ik_solver.goto_configuration(q)
        vel = self.limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.robot_model.state['q'] + vel * self.dt
        self.update_robot_model()

    def goto_reduced_configuration(self, q_reduced):
        # solve IK and apply control
        vel = self.ik_solver.goto_reduced_configuration(q_reduced)
        vel = self.limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self.apply_joint_position(self.ik_solver.q)
        self.update_robot_model()

    def sim_goto_reduced_configuration(self, q_reduced):
        # solve IK and apply control
        vel = self.ik_solver.goto_reduced_configuration(q_reduced)
        vel = self.limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.ik_solver.q
        self.update_robot_model()

    def lock_configuration(self, q):
        # compute gravity compensation torque
        tau = self.robot_model.get_gravity_compensation(q)

        # send command to lock the robot in current configuration
        self.command_publisher.q = q
        self.command_publisher.dq = np.zeros(self.robot_model.model.nv)
        self.command_publisher.tau = tau
        self.update_robot_model()

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
        vel_scaled[:13] = vel[:13]
        vel_scaled[LEFT_ARM_INDEX] = left_scaler * vel[LEFT_ARM_INDEX]
        vel_scaled[RIGHT_ARM_INDEX] = right_scaler * vel[RIGHT_ARM_INDEX]

        return vel_scaled

    def apply_joint_position(self, q):
        # get gravity compensation torque
        tau = self.robot_model.get_gravity_compensation(self.robot_model.state['q'])
        # send the position command to robot
        self.command_publisher.q = q
        self.command_publisher.dq = np.zeros(self.robot_model.model.nv)
        self.command_publisher.tau = tau

        if self.recording:
            # record center of mass
            com = self.robot_model.get_center_of_mass()
            # get values for upper body joints only
            q = self.robot_model.state['q'][self.upper_ids]
            dq = self.robot_model.state['dq'][self.upper_ids]
            tau = self.robot_model.state['tau'][self.upper_ids]
            q_cmd = self.command_publisher.q[self.upper_ids]
            dq_cmd = self.command_publisher.dq[self.upper_ids]
            tau_cmd = self.command_publisher.tau[self.upper_ids]
            kp = self.command_publisher.kp[self.upper_ids]
            kd = self.command_publisher.kd[self.upper_ids]
            # record
            self.com_arr.append(com)
            self.q_arr.append(q)
            self.dq_arr.append(dq)
            self.tau_arr.append(tau)
            self.q_cmd_arr.append(q_cmd)
            self.dq_cmd_arr.append(dq_cmd)
            self.tau_cmd_arr.append(tau_cmd)
            self.torque_cmd_arr.append(
                tau_cmd + kp * (q_cmd - q) + kd * (dq_cmd - dq)
            )

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

    def start_recording(self):
        self.recording = True

    def stop_recording(self):
        self.recording = False

    def clear_recording(self):
        self.com_arr = []
        self.q_arr = []
        self.dq_arr = []
        self.tau_arr = []
        self.q_cmd_arr = []
        self.dq_cmd_arr = []
        self.tau_cmd_arr = []
        self.torque_cmd_arr = []

    def save_recording(self, filename):
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        np.savez(filename,
                 ids=np.array(self.upper_ids),
                 com=self.com_arr,
                 q=self.q_arr,
                 dq=self.dq_arr,
                 tau=self.tau_arr,
                 q_cmd=self.q_cmd_arr,
                 dq_cmd=self.dq_cmd_arr,
                 tau_cmd=self.tau_cmd_arr,
                 torque_cmd=self.torque_cmd_arr)
