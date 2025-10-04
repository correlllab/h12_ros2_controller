from ast import Del
import os
import time
import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.channel_interface import CommandPublisher, ArmSDKPublisher, setup_gains_mj, setup_gains_real
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, ENABLED_JOINTS, LEFT_ARM_INDEX, RIGHT_ARM_INDEX

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

        # enable torso motor such that it's locked in pace
        self.command_publisher.enable_motor([12], [0.0])
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
        self.q_arr = []
        self.dq_arr = []
        self.tau_arr = []
        self.q_cmd_arr = []
        self.dq_cmd_arr = []
        self.tau_cmd_arr = []
        self.torque_cmd_arr = []

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
            # get values for upper body joints only
            q = self.robot_model.state['q'][12:27]
            dq = self.robot_model.state['dq'][12:27]
            tau = self.robot_model.state['tau'][12:27]
            q_cmd = self.command_publisher.q[12:27]
            dq_cmd = self.command_publisher.dq[12:27]
            tau_cmd = self.command_publisher.tau[12:27]
            kp = self.command_publisher.kp[12:27]
            kd = self.command_publisher.kd[12:27]
            # record
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
                 ids=np.array([i for i in range(12, 27)]),
                 q=self.q_arr,
                 dq=self.dq_arr,
                 tau=self.tau_arr,
                 q_cmd=self.q_cmd_arr,
                 dq_cmd=self.dq_cmd_arr,
                 tau_cmd=self.tau_cmd_arr,
                 torque_cmd=self.torque_cmd_arr)
