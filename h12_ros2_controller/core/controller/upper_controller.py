import time
import numpy as np
import pinocchio as pin
from tqdm import tqdm

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.low_cmd_handler import LowCmdHandler
from h12_ros2_controller.core.planner import PlannerConfig, PlannerClient
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.joint_definition import (
    BODY_JOINTS,
    ENABLED_JOINTS,
    LEFT_ARM_INDEX,
    RIGHT_ARM_INDEX,
    UPPER_BODY_INDEX,
)

class UpperController:
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 init: bool=True,
                 handless: bool=False,
                 visualize=False,
                 config=None):
        self.config = config if config is not None else load_controller_config()
        controller_cfg = self.config.get('controller', {})

        # initialize robot model
        self.robot_model = RobotModel(urdf_path, handless=handless)
        self.dt = 1.0 / float(self.config.get('frequency', {}).get('ctrl_hz', 50.0))
        self.v_lim = float(controller_cfg.get('v_lim', 1.0))
        self.w_lim = float(controller_cfg.get('w_lim', 2.0))
        self.dq_lim = float(controller_cfg.get('dq_lim', 1.0))
        self.d_min = float(controller_cfg.get('d_min', 0.02))
        self.torso_target = float(controller_cfg.get('torso_target', 0.0))
        self.visualize = visualize

        # initialize subscriber in robot model
        low_state_topic = self.config.get('topics', {}).get('low_state', 'rt/lowstate')
        self.robot_model.init_subscriber(low_state_topic=low_state_topic)
        time.sleep(0.5)
        self.robot_model.update_kinematics()

        # init reduced model and collision model
        self.robot_model.init_reduced_model(ENABLED_JOINTS)
        self.robot_model.init_collision_model(urdf_sphere_path, srdf_sphere_path)
        self.enabled_ids = [BODY_JOINTS.index(joint) for joint in ENABLED_JOINTS]
        self.upper_ids = UPPER_BODY_INDEX

        # intialize low cmd publisher
        self.low_cmd_handler = LowCmdHandler(self.robot_model,
                                             config=self.config)

        # enable upper body motors
        init_q = self.robot_model.state_reduced['q']
        self.low_cmd_handler.enable_motors(self.enabled_ids, init_q)

        # enable torso motor at its current position
        torso_id = BODY_JOINTS.index('torso_joint')
        torso_init = float(self.robot_model.state['q'][torso_id])
        self.low_cmd_handler.enable_motors([torso_id], [torso_init])

        # start publisher
        self.low_cmd_handler.start()

        # initialize IK solver
        self.ik_solver = IKSolver(
            robot_model=self.robot_model,
            dt=self.dt,
            d_min=self.d_min
        )
        # keep the resolved planner config for the point-cloud preload path
        self.planner_config = self._load_planner_config()
        self.planner = PlannerClient(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            planner_config=self.planner_config,
            handless=handless,
        )

        if self.visualize:
            self.robot_model.init_visualizer()
            self.robot_model.config_visualizer(show_com=True, show_zmp=True)
            self._visualize_configured_point_cloud()

        # default end effector frame names for velocity limiting
        self.left_ee_name = 'left_wrist_yaw_link'
        self.right_ee_name = 'right_wrist_yaw_link'

        if init:
            # move through staged init poses before rotating torso to avoid hand/hip scraping
            self._startup_routine(torso_init, self.torso_target)

    def _startup_routine(self, torso_init, torso_target):
        for config_name in ('init_1', 'init_2', 'init_3', 'home'):
            self._move_to_reduced_configuration(
                NAMED_CONFIGS[config_name],
                steps=50,
                desc=f'Moving {config_name}',
            )
        self._move_torso_to_target(torso_init, torso_target)

    def _move_to_reduced_configuration(
        self, q_reduced,
        steps=150, threshold=1e-3,
        desc='Moving configuration',
    ):
        for _ in tqdm(range(steps), desc=desc, leave=False):
            self.goto_reduced_configuration(q_reduced)
            error = np.max(np.abs(self.robot_model.state_reduced['q'] - q_reduced))
            if error < threshold:
                break
            time.sleep(self.dt)

    def _load_planner_config(self):
        '''Load OMPL planner config from controller config'''
        # merge optional yaml planner settings with dataclass defaults
        cfg = self.config.get('planner', {})
        defaults = PlannerConfig()
        return PlannerConfig(
            planner=cfg.get('planner', defaults.planner),
            timeout=float(cfg.get('timeout', defaults.timeout)),
            range=float(cfg.get('range', defaults.range)),
            try_direct=bool(cfg.get('try_direct', defaults.try_direct)),
            simplify=bool(cfg.get('simplify', defaults.simplify)),
            simplify_time=float(cfg.get('simplify_time', defaults.simplify_time)),
            shortcut=bool(cfg.get('shortcut', defaults.shortcut)),
            moving_speed=float(cfg.get('moving_speed', defaults.moving_speed)),
            dt=float(cfg.get('dt', self.dt)),
            min_interpolation_steps=int(
                cfg.get(
                    'min_interpolation_steps',
                    defaults.min_interpolation_steps,
                )
            ),
            max_interpolation_steps=int(
                cfg.get(
                    'max_interpolation_steps',
                    defaults.max_interpolation_steps,
                )
            ),
            validity_resolution=float(
                cfg.get('validity_resolution', defaults.validity_resolution)
            ),
            constraint_check_steps=int(
                cfg.get('constraint_check_steps', defaults.constraint_check_steps)
            ),
            frame_names=tuple(cfg.get('frame_names', defaults.frame_names)),
            frame_z_min=cfg.get('frame_z_min', defaults.frame_z_min),
            frame_z_min_margin=float(
                cfg.get('frame_z_min_margin', defaults.frame_z_min_margin)
            ),
            frame_z_corridor_margin=float(
                cfg.get(
                    'frame_z_corridor_margin',
                    defaults.frame_z_corridor_margin,
                )
            ),
            point_cloud_path=cfg.get(
                'point_cloud_path', defaults.point_cloud_path
            ),
            point_cloud_margin=float(
                cfg.get('point_cloud_margin', defaults.point_cloud_margin)
            ),
        )

    def _move_torso_to_target(self, torso_init, torso_target, threshold=1e-3):
        torso_id = BODY_JOINTS.index('torso_joint')
        torso_speed = 0.3
        step_size = max(torso_speed * self.dt, 1e-6)
        steps = max(1, int(np.ceil(abs(torso_target - torso_init) / step_size)))

        for step in tqdm(range(steps), desc='Moving torso to target', leave=False):
            alpha = (step + 1) / steps
            q_target = (1.0 - alpha) * torso_init + alpha * torso_target
            self.low_cmd_handler.set_joint_commands(
                q=np.array([q_target], dtype=np.float64),
                joint_ids=[torso_id],
            )
            self.update_robot_model()
            torso_position = float(self.robot_model.state['q'][torso_id])
            if abs(torso_position - torso_target) < threshold:
                break
            time.sleep(self.dt)

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
        return self.robot_model.get_frame_transformation_reduced(self.left_ee_name)

    @property
    def left_ee_position(self):
        return self.robot_model.get_frame_position_reduced(self.left_ee_name)

    @property
    def left_ee_rotation(self):
        return self.robot_model.get_frame_rotation_reduced(self.left_ee_name)

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
        return self.robot_model.get_frame_transformation_reduced(self.right_ee_name)

    @property
    def right_ee_position(self):
        return self.robot_model.get_frame_position_reduced(self.right_ee_name)

    @property
    def right_ee_rotation(self):
        return self.robot_model.get_frame_rotation_reduced(self.right_ee_name)

    @property
    def right_ee_rpy(self):
        return pin.rpy.matrixToRpy(self.right_ee_rotation)

    @property
    def right_ee_pose(self):
        return np.concatenate([self.right_ee_position, self.right_ee_rpy])

    def update_ik_solver(self):
        # update IK solver with the latest robot configuration
        self.ik_solver.update_configurations()

    def _as_reduced_path(self, path):
        # normalize path input and enforce reduced-model waypoint shape
        path = np.asarray(path, dtype=float)
        if path.ndim == 1:
            path = path.reshape(1, -1)
        nq = self.robot_model.model_body_reduced.nq
        if path.ndim != 2 or path.shape[1] != nq:
            raise ValueError(f'Path must have shape (N, {nq})')
        if not np.all(np.isfinite(path)):
            raise ValueError('Path must contain only finite values')
        return path

    def _path_or_raise(self, result):
        if not result.success:
            raise RuntimeError(f'Planning failed: {result.reason}')
        return result.path

    def plan_to_configuration(self, q_reduced, start=None, active_mask=None):
        '''Plan from current reduced state to a reduced joint target'''
        if start is None:
            start = self.robot_model.state_reduced['q']
        q_reduced = np.asarray(q_reduced, dtype=float)
        result = self.planner.plan_configuration(
            current_q=self.robot_model.state['q'],
            start=start,
            goal=q_reduced,
            active_mask=active_mask,
        )
        return self._path_or_raise(result)

    def plan_to_ik_target(self, start=None, ik_timeout=1.0, ik_alpha=0.1):
        '''Solve current IK tasks, then plan to the IK joint solution'''
        # restrict planning to joints supporting the active tasks so that
        # untasked limbs (e.g. the other arm) never move during the plan
        active_mask = self._active_task_mask()
        self.update_ik_solver()
        ik_result = self.ik_solver.solve_ik_reduced(
            alpha=ik_alpha,
            timeout=ik_timeout,
        )
        if not ik_result.success:
            raise RuntimeError('IK failed to resolve current targets')

        q_goal = ik_result.q[self.robot_model.reduced_mask]
        start_q = start if start is not None else self.robot_model.state_reduced['q']
        return self.plan_to_configuration(
            q_goal,
            start=start_q,
            active_mask=active_mask,
        )

    def _active_task_mask(self):
        '''Reduced joint mask supporting all active frame tasks'''
        mask = np.zeros(self.robot_model.model_body_reduced.nq, dtype=bool)
        for task in self.ik_solver.frame_tasks.values():
            mask |= self._reduced_support_mask(task.frame)
        return mask

    def _reduced_support_mask(self, frame_name):
        '''Reduced joint mask supporting a frame in the kinematic chain'''
        model = self.robot_model.model_body_reduced
        frame_id = model.getFrameId(frame_name)
        joint_id = model.frames[frame_id].parentJoint

        # walk up the tree to the root, marking supporting joints
        mask = np.zeros(model.nq, dtype=bool)
        while joint_id > 0:
            idx_q = model.joints[joint_id].idx_q
            nq = model.joints[joint_id].nq
            mask[idx_q:idx_q + nq] = True
            joint_id = model.parents[joint_id]
        return mask

    def update_point_cloud(self, points):
        '''Update the obstacle point cloud used by the planner'''
        return self.planner.set_point_cloud(points)

    def _visualize_configured_point_cloud(self):
        '''Draw the point cloud preloaded from the planner config, if any'''
        # the planner subprocess loads the cloud itself; only draw it here
        path = self.planner_config.point_cloud_path
        if not path:
            return
        self.robot_model.visualizer.visualize_point_cloud(np.load(path))

    def visualize_path(self, path):
        '''Draw a reduced joint-space path with Meshcat'''
        if self.robot_model.visualizer is None:
            self.robot_model.init_visualizer()
        self.robot_model.visualizer.visualize_reduced_path(path)

    def play_path(self, path, delay=0.05):
        '''Animate a reduced joint-space path with Meshcat'''
        if self.robot_model.visualizer is None:
            self.robot_model.init_visualizer()
        self.robot_model.visualizer.play_reduced_path(path, delay=delay)

    def execute_path(self, path, validate=True):
        '''Execute reduced joint waypoints as direct position commands'''
        path = self._as_reduced_path(path)

        # apply each reduced waypoint as a direct full motor position command
        for q_reduced in tqdm(path, desc='Executing path', leave=False):
            if validate:
                # keep execution guarded by the same reduced validity checks
                if not self.robot_model.check_within_limits_reduced(q_reduced):
                    raise ValueError('Path waypoint is outside joint limits')
                if not self.robot_model.check_collision_free_reduced(q_reduced):
                    raise ValueError('Path waypoint is in self-collision')

            q = np.copy(self.robot_model.state['q'])
            q[self.robot_model.reduced_mask] = q_reduced
            self._apply_joint_position(q)
            self.update_robot_model()
            time.sleep(self.dt)

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
        vel = self._limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self._apply_joint_position(self.ik_solver.q)
        self.update_robot_model()
        return vel

    def sim_goto_configuration(self, q):
        # solve IK and apply control
        vel = self.ik_solver.goto_configuration(q)
        vel = self._limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.robot_model.state['q'] + vel * self.dt
        self.update_robot_model()
        return vel

    @property
    def reduced_configuration_error(self):
        return self.ik_solver.compute_error_reduced(
            self.ik_solver.config_task,
            self.robot_model.state_reduced['q']
        )

    def set_reduced_configuration_target(self, q_reduced):
        '''Set the reduced joint target used for configuration feedback'''
        self.ik_solver.config_task.set_target(q_reduced)

    def goto_reduced_configuration(self, q_reduced):
        # solve IK and apply control
        vel = self.ik_solver.goto_reduced_configuration(q_reduced)
        vel = self._limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self._apply_joint_position(self.ik_solver.q)
        self.update_robot_model()
        return vel

    def sim_goto_reduced_configuration(self, q_reduced):
        # solve IK and apply control
        vel = self.ik_solver.goto_reduced_configuration(q_reduced)
        vel = self._limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.ik_solver.q
        self.update_robot_model()
        return vel

    def lock_configuration(self, q):
        self._apply_joint_position(q)
        self.update_robot_model()

    def control_step(self, com=False):
        '''Solve IK for all tasks'''
        # solve IK and apply the control
        vel = self.ik_solver.ik_step(com=com)
        vel = self._limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self._apply_joint_position(self.ik_solver.q)
        self.update_robot_model()
        return vel

    def control_step_reduced(self, com=False):
        '''Solve IK for all tasks with the reduced model'''
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced(com=com)
        vel = self._limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self._apply_joint_position(self.ik_solver.q)
        self.update_robot_model()
        return vel

    def sim_step(self, com=False):
        # solve IK and apply the control
        vel = self.ik_solver.ik_step(com=com)
        vel = self._limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.ik_solver.q
        self.update_robot_model()
        return vel

    def sim_step_reduced(self, com=False):
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced(com=com)
        vel = self._limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.ik_solver.q
        self.update_robot_model()
        return vel

    def init_steady_state(self):
        '''Initialize steady-state I control from the final IK command'''
        self._steady_q_cmd = np.copy(self.low_cmd_handler.q_cmd)
        self._steady_dq_cmd = np.zeros(self.robot_model.model_body.nv)
        self._steady_tau_bias = np.zeros(self.robot_model.model_body.nv)
        ki = self.config.get('gains', {}).get('ki')
        if ki is None:
            self._steady_ki = np.zeros(self.robot_model.model_body.nv)
        else:
            self._steady_ki = np.asarray(ki, dtype=np.float64)

        self._steady_tau_bias_limit = np.zeros_like(self.robot_model.model_body.nv)
        tau_clip_limits = self.config.get('limits', {}).get('tau_clip_limits')
        if tau_clip_limits is not None:
            self._steady_tau_bias_limit = np.asarray(tau_clip_limits, dtype=np.float64)

        self._steady_q_clip_limits = None
        q_clip_limits = self.config.get('limits', {}).get('q_clip_limits')
        if q_clip_limits is not None:
            self._steady_q_clip_limits = np.asarray(q_clip_limits, dtype=np.float64)

    def _steady_state_hits_position_clip(self, q_state, tau_bias_increment, threshold):
        if self._steady_q_clip_limits is None:
            return False

        q_limits = self._steady_q_clip_limits[self.upper_ids]
        q_state = q_state[self.upper_ids]
        tau_bias_increment = tau_bias_increment[self.upper_ids]

        near_upper = q_state >= (q_limits[:, 1] - threshold)
        near_lower = q_state <= (q_limits[:, 0] + threshold)
        push_upper = tau_bias_increment > 0.0
        push_lower = tau_bias_increment < 0.0

        return bool(np.any((near_upper & push_upper) | (near_lower & push_lower)))

    def steady_state_step(self, threshold=1e-3):
        '''Run one steady-state I-control tick and return convergence'''
        if not hasattr(self, '_steady_q_cmd'):
            self.init_steady_state()

        # get positional error
        q_state = self.robot_model.state['q']
        q_error = self._steady_q_cmd - q_state
        tau_bias_increment = self._steady_ki * q_error * self.dt
        # close to clipping limit
        if self._steady_state_hits_position_clip(
            q_state,
            tau_bias_increment,
            threshold,
        ):
            tau_gravity = self.robot_model.dynamics.get_gravity_compensation(
                self.robot_model.state['q']
            )
            tau_cmd = tau_gravity + self._steady_tau_bias
            self.low_cmd_handler.set_joint_commands(
                self._steady_q_cmd,
                self._steady_dq_cmd,
                tau_cmd,
            )
            self.update_robot_model()
            return True
        else:
            self._steady_tau_bias += tau_bias_increment
            self._steady_tau_bias = np.clip(
                self._steady_tau_bias,
                -self._steady_tau_bias_limit,
                self._steady_tau_bias_limit,
            )

            # compute total torque to command
            tau_gravity = self.robot_model.dynamics.get_gravity_compensation(
                self.robot_model.state['q']
            )
            tau_cmd = tau_gravity + self._steady_tau_bias
            self.low_cmd_handler.set_joint_commands(
                self._steady_q_cmd,
                self._steady_dq_cmd,
                tau_cmd,
            )

            # check convergence
            self.update_robot_model()
            q_error = self._steady_q_cmd - self.robot_model.state['q']
            return bool(np.max(np.abs(q_error[self.upper_ids])) < threshold)

    def _limit_joint_vel(self, vel):
        # get end effector twist
        twist_left = self.robot_model.dynamics.compute_frame_twist(self.left_ee_name, vel)
        twist_right = self.robot_model.dynamics.compute_frame_twist(self.right_ee_name, vel)
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

    def _apply_joint_position(self, q):
        # get gravity compensation torque
        tau = self.robot_model.dynamics.get_gravity_compensation(self.robot_model.state['q'])
        dq = np.zeros(self.robot_model.model_body.nv)
        self.low_cmd_handler.set_joint_commands(q, dq, tau)

    def estop(self):
        self.low_cmd_handler.estop()

    def shutdown(self):
        self.stop_recording()
        self.planner.shutdown()
        self.robot_model.shutdown()
        self.low_cmd_handler.shutdown()

    def damp_mode(self, kd=3.0):
        kp = np.zeros(self.robot_model.model_body.nv)
        kd = kd * np.ones(self.robot_model.model_body.nv)
        dq = np.zeros(self.robot_model.model_body.nv)
        self.low_cmd_handler.set_joint_gains(kp=kp, kd=kd)
        self.low_cmd_handler.set_joint_commands(dq=dq)
        print(f'Set kp to zero, kd to {kd} and dq to 0')

    def start_recording(self, save_path, filename, record_interval=1.0):
        '''Start recording with background saving'''
        self.low_cmd_handler.start_recording(self.upper_ids, save_path, filename, record_interval)

    def stop_recording(self):
        '''Stop recording'''
        self.low_cmd_handler.stop_recording()

    def clear_recording(self):
        '''Clear recorded'''
        self.low_cmd_handler.clear_recording()
