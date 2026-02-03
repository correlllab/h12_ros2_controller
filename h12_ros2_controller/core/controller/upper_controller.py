import time
import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.low_cmd_handler import LowCmdHandler
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, UPPER_BODY_JOINTS, LOWER_BODY_JOINTS, ENABLED_JOINTS, LEFT_ARM_INDEX, RIGHT_ARM_INDEX

class UpperController:
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 dt=0.02, v_lim=1.0, w_lim=2.0, dq_lim=1.0, d_min=0.02,
                 visualize=False,
                 sport_mode=False):
        # initialize robot model
        self.robot_model = RobotModel(urdf_path)
        self.dt = dt
        self.v_lim = v_lim
        self.w_lim = w_lim
        self.dq_lim = dq_lim
        self.d_min = d_min
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

        # intialize low cmd publisher
        self.low_cmd_handler = LowCmdHandler(self.robot_model,
                                             sport_mode=sport_mode)

        # enable upper body motors
        init_q = self.robot_model.state_reduced['q']
        self.low_cmd_handler.enable_motors(self.enabled_ids, init_q)

        # enable torso motor such that it's locked in place
        self.low_cmd_handler.enable_motors([BODY_JOINTS.index('torso_joint')], [0.0])

        # enable lower body motor
        lower_ids = [BODY_JOINTS.index(joint) for joint in LOWER_BODY_JOINTS]
        lower_init = self.robot_model.state['q'][lower_ids]
        self.low_cmd_handler.enable_motors(lower_ids, lower_init)

        # start publisher
        self.low_cmd_handler.start()

        # initialize IK solver
        self.ik_solver = IKSolver(
            robot_model=self.robot_model,
            dt=self.dt,
            d_min=self.d_min
        )

        if self.visualize:
            self.robot_model.init_visualizer()
            self.robot_model.config_visualizer(show_com=True, show_zmp=True)

        # default end effector frame names for velocity limiting
        self.left_ee_name = 'left_wrist_yaw_link'
        self.right_ee_name = 'right_wrist_yaw_link'

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

    @property
    def configuration_error(self):
        return self.ik_solver.compute_error(
            self.ik_solver.config_task,
            self.robot_model.state['q']
        )

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

    @property
    def reduced_configuration_error(self):
        return self.ik_solver.compute_error_reduced(
            self.ik_solver.config_task,
            self.robot_model.state_reduced['q']
        )

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
        # always commanding zero velocity (motor-only, 27)
        dq = np.zeros(self.robot_model.model_body.nv)
        # send command to lock the robot in current configuration
        self.low_cmd_handler.set_joint_commands( q, dq, tau)
        self.update_robot_model()

    def control_step(self, com=False):
        '''Solve IK for all tasks'''
        # solve IK and apply the control
        vel = self.ik_solver.ik_step(com=com)
        vel = self.limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self.apply_joint_position(self.ik_solver.q)
        self.update_robot_model()

    def control_step_reduced(self, com=False):
        '''Solve IK for all tasks with the reduced model'''
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced(com=com)
        vel = self.limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self.apply_joint_position(self.ik_solver.q)
        self.update_robot_model()

    def sim_step(self, com=False):
        # solve IK and apply the control
        vel = self.ik_solver.ik_step(com=com)
        vel = self.limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.ik_solver.q
        self.update_robot_model()

    def sim_step_reduced(self, com=False):
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced(com=com)
        vel = self.limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.ik_solver.q
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
        left_scaler = min([
            1.0,
            left_dq_scaler,
            self.v_lim / (np.linalg.norm(v_left) + 1e-6),
            self.w_lim / (np.linalg.norm(w_left) + 1e-6)
        ])
        right_scaler = min([
            1.0,
            right_dq_scaler,
            self.v_lim / (np.linalg.norm(v_right) + 1e-6),
            self.w_lim / (np.linalg.norm(w_right) + 1e-6)
        ])

        vel_scaled = np.zeros_like(vel)
        vel_scaled[:13] = vel[:13]
        vel_scaled[LEFT_ARM_INDEX] = left_scaler * vel[LEFT_ARM_INDEX]
        vel_scaled[RIGHT_ARM_INDEX] = right_scaler * vel[RIGHT_ARM_INDEX]

        return vel_scaled

    def apply_joint_position(self, q):
        # get gravity compensation torque
        tau = self.robot_model.get_gravity_compensation(self.robot_model.state['q'])
        dq = np.zeros(self.robot_model.model_body.nv)
        self.low_cmd_handler.set_joint_commands(q, dq, tau)

    def estop(self):
        self.low_cmd_handler.estop()

    def shutdown(self):
        self.stop_recording()
        self.robot_model.shutdown()
        self.low_cmd_handler.shutdown()

    def damp_mode(self, kd=3.0):
        kp = np.zeros(self.robot_model.model_body.nv)
        kd = kd * np.ones(self.robot_model.model_body.nv)
        dq = np.zeros(self.robot_model.model_body.nv)
        self.low_cmd_handler.set_joint_commands(dq=dq, kp=kp, kd=kd)
        print(f'Set kp to zero, kd to {kd} and dq to 0')

    def start_recording(self, save_path, filename, record_interval=0.01):
        '''Start recording with background saving'''
        self.low_cmd_handler.start_recording(self.upper_ids, save_path, filename, record_interval)

    def stop_recording(self):
        '''Stop recording'''
        self.low_cmd_handler.stop_recording()

    def clear_recording(self):
        '''Clear recorded'''
        self.low_cmd_handler.clear_recording()
