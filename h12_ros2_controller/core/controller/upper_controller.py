import time
from copy import deepcopy
import numpy as np
import pinocchio as pin
from tqdm import tqdm

from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.low_cmd_handler import LowCmdHandler
from h12_ros2_controller.core.planner import PlannerClient
from h12_ros2_controller.core.planner.kinematic_pipeline import (
    frame_target_snapshot, frame_targets_close, plan_frame_tasks,
    planner_config_from_controller,
)
from h12_ros2_controller.core.planner.reduced_joint_planner import (
    active_grasp_frames, moving_grasp_frames,
)
from h12_ros2_controller.utility.controller_config import dds_domain_id, load_controller_config
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.joint_definition import (
    BODY_JOINTS,
    ENABLED_JOINTS,
    LEFT_ARM_INDEX,
    RIGHT_ARM_INDEX,
    UPPER_BODY_INDEX,
)

# Max per-joint gap [rad] between the last commanded and the measured arm
# position for a new move to start from the command. A larger gap means the arm
# is blocked or was pushed, so the move restarts from the measured position and
# drops the held torque bias, releasing the force instead of continuing it.
COMMAND_SEED_MAX_GAP = 0.1
# Time constant [s] over which the steady-state torque bias fades once a new
# move starts commanding positions. Carrying it over keeps the arm from sagging
# at the start of a move; fading it hands the load back to gravity
# compensation as the arm leaves the pose the bias was integrated for.
TAU_BIAS_FADE_SEC = 1.0
# DDS domain of the real robot's command bus; the sims refuse to run on it
REAL_ROBOT_DOMAIN_ID = 0


class StaleStateError(RuntimeError):
    '''rt/lowstate is too stale to control on'''


class PathExecutionCancelled(RuntimeError):
    '''Trajectory canceled before another position command was issued'''


class UpperController:
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 init: bool=True,
                 handless: bool=False,
                 visualize=False,
                 config=None):
        self.config = deepcopy(config) if config is not None else load_controller_config()
        controller_cfg = self.config.get('controller', {})
        control_dt = 1.0 / float(self.config.get('frequency', {}).get('ctrl_hz', 50.0))
        planner_config = planner_config_from_controller(self.config, dt=control_dt)
        self.execution_start_tolerance = float(
            controller_cfg.get('execution_start_tolerance', 0.01)
        )
        self.command_seed_max_gap = float(
            controller_cfg.get('command_seed_max_gap', COMMAND_SEED_MAX_GAP)
        )
        self.plan_max_age = float(controller_cfg.get('plan_max_age', 30.0))
        for name in ('threshold_linear', 'threshold_angular'):
            value = float(controller_cfg.get(name, 1e-3 if name == 'threshold_linear' else 1e-2))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f'Controller {name} must be finite and positive')
        for name in ('execution_start_tolerance', 'command_seed_max_gap', 'plan_max_age'):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f'Controller {name} must be finite and positive')

        # initialize robot model
        self.robot_model = RobotModel(urdf_path, handless=handless)
        if planner_config.motor_position_limits is not None:
            self.robot_model.apply_position_limits(planner_config.motor_position_limits)
            self.config['limits']['q_clip_limits'] = np.column_stack((
                self.robot_model.model_body.lowerPositionLimit,
                self.robot_model.model_body.upperPositionLimit,
            ))
            planner_config = planner_config_from_controller(self.config, dt=control_dt)
        self.dt = control_dt
        self.v_lim = float(controller_cfg.get('v_lim', 1.0))
        self.w_lim = float(controller_cfg.get('w_lim', 2.0))
        self.dq_lim = float(controller_cfg.get('dq_lim', 1.0))
        # global multiplier on commanded arm speed (1.0 = full speed); set
        # below 1.0 (e.g. 0.25 for slow mode) to move the arms more slowly
        self.speed_scale = 1.0
        # steady-state I-control torque bias [Nm, motor order]; integrated
        # while holding, faded by _apply_joint_position while moving
        self._tau_bias = np.zeros(self.robot_model.model_body.nv)
        self._tau_bias_decay = float(np.exp(-self.dt / TAU_BIAS_FADE_SEC))
        self.d_min = float(controller_cfg.get('d_min', 0.02))
        # controlling on stale state winds up the hold I-term and seeds moves
        # from where the arm was, so motion stops when state is staler than
        # this; real robot only, since the sims stamp tick in sim steps
        self.max_state_lag_ms = (
            float(controller_cfg.get('max_state_lag_ms', 50.0))
            if dds_domain_id(self.config) == REAL_ROBOT_DOMAIN_ID else 0.0
        )
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
            d_min=self.d_min,
            wrist_movement_weight=planner_config.ik_wrist_motion_weight,
            wrist_posture_cost=planner_config.ik_wrist_posture_cost,
            shoulder_seed_offset=planner_config.ik_shoulder_seed_offset,
            elbow_seed_offset=planner_config.ik_elbow_seed_offset,
            batch_frame_lm_damping=planner_config.ik_frame_lm_damping,
            batch_stagnation_iterations=planner_config.ik_stagnation_iterations,
            batch_stagnation_relative_improvement=(
                planner_config.ik_stagnation_relative_improvement
            ),
        )
        self.planner = PlannerClient(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            planner_config=planner_config,
            handless=handless,
            request_timeout=planner_config.worker_request_timeout,
            startup_timeout=planner_config.worker_startup_timeout,
        )
        self._prior_ik = None
        # default end effector frame names for velocity limiting
        self.left_ee_name = 'left_wrist_yaw_link'
        self.right_ee_name = 'right_wrist_yaw_link'

        try:
            if planner_config.prewarm_worker:
                self.planner.warmup_background()
            if self.visualize:
                self.robot_model.init_visualizer()
                self.robot_model.config_visualizer(show_com=True, show_zmp=True)
            if init:
                # move through init poses before rotating torso to avoid hand/hip scraping
                self._startup_routine(torso_init, self.torso_target)
        except Exception:
            self.shutdown()
            raise

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
        return planner_config_from_controller(self.config, dt=self.dt)

    def _move_torso_to_target(self, torso_init, torso_target, threshold=1e-3):
        torso_id = BODY_JOINTS.index('torso_joint')
        limits = self.low_cmd_handler._q_clip_limits[torso_id]
        if (not np.isfinite(torso_init) or not np.isfinite(torso_target) or
                not limits[0] <= torso_target <= limits[1]):
            raise ValueError('Torso target must be finite and inside command position limits')
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
        # measured convergence can occur before the final interpolation sample
        self.low_cmd_handler.set_joint_commands(
            q=np.array([torso_target], dtype=np.float64), joint_ids=[torso_id],
        )

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

    def check_state_fresh(self):
        '''Raise StaleStateError if rt/lowstate is too stale to control on'''
        if self.max_state_lag_ms <= 0.0:
            return
        staleness_ms = self.robot_model.state_staleness_ms
        if staleness_ms > self.max_state_lag_ms:
            raise StaleStateError(
                f'rt/lowstate is {staleness_ms:.0f} ms stale '
                f'(> {self.max_state_lag_ms:.0f} ms); holding the last command'
            )

    def update_ik_solver(self, measured_q=None, command_seed=None):
        self.check_state_fresh()
        if measured_q is None:
            measured_q = np.copy(self.robot_model.state['q'])
        # re-lock the frozen joints (torso, legs) at their measured angles so
        # reduced-model FK agrees with the robot's actual kinematics (and the
        # TF tree); frozen-at-zero, a torso holding e.g. -0.04 rad offsets
        # every arm frame by a constant ~15 mm that IK can never converge away
        frozen = ~self.robot_model.reduced_mask
        if not np.array_equal(
            measured_q[frozen], self.robot_model.reduced_reference_q[frozen]
        ):
            self.robot_model.rebuild_reduced_model(reference_q=measured_q)
            self.ik_solver.rebuild_reduced()
        # start IK from the last command, not the measured state: the arm sits
        # below its command (gravity, PD lag), and restarting from the measured
        # position would step the command down and drop the arm
        q_seed, from_command = self._command_seed(measured_q)
        if command_seed is not None and from_command:
            q_seed = np.asarray(command_seed, dtype=float).copy()
        if not from_command:
            self._tau_bias[:] = 0.0
        self.ik_solver.update_configurations(q_seed)

    def _command_seed(self, measured_q=None):
        '''
        Return (q, from_command): the motor-order configuration a new move
        starts from. Enabled joints take the last commanded position when it
        is within the configured command gap of the measured position on every
        enabled joint; otherwise the whole seed is the measured state.
        '''
        q_seed = np.copy(
            self.robot_model.state['q'] if measured_q is None else measured_q
        )
        q_cmd = self.low_cmd_handler.q_cmd
        ids = self.enabled_ids
        gap = np.max(np.abs(q_cmd[ids] - q_seed[ids]))
        max_gap = getattr(self, 'command_seed_max_gap', COMMAND_SEED_MAX_GAP)
        if not np.isfinite(gap) or gap > max_gap:
            print(
                f'Command-to-measured gap {gap:.3f} rad > '
                f'{max_gap} rad; starting from the measured state',
                flush=True,
            )
            return q_seed, False
        q_seed[ids] = q_cmd[ids]
        return q_seed, True

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

    def _planner_speed_overrides(self):
        '''Per-plan speed settings implied by the current speed_scale

        A planned path is slowed by PLANNING it at a lower grasp-frame speed:
        the planner then emits proportionally more waypoints and execute_path
        still clocks them out at dt. Stretching the step period instead keeps
        the original waypoint spacing and turns a smooth ramp into a staircase.
        Returns (None, None) at full speed so the planner process keeps the
        config it was spawned with.
        '''
        scale = float(self.speed_scale)
        if scale == 1.0:
            return None, None
        config = self.planner.planner_config
        # the waypoint cap bounds a path's duration, so it has to grow with the
        # step count or a long slow path clamps back to near full speed
        return (
            config.moving_speed * scale,
            int(np.ceil(config.max_interpolation_steps / scale)),
        )

    def plan_to_configuration(self, q_reduced, start=None, active_mask=None,
                              current_q=None):
        '''Plan from the move's start configuration to a reduced joint target'''
        self.last_plan_result = None
        self._prior_ik = None
        result = self._plan_configuration_result(
            q_reduced, start=start, active_mask=active_mask, current_q=current_q,
        )
        self.last_plan_result = result
        return self._path_or_raise(result)

    def _plan_configuration_result(self, q_reduced, start=None, active_mask=None,
                                    current_q=None, direct_only=False,
                                    timeout=None, local_frames=None,
                                    response_timeout=None, local_joint_padding=None,
                                    planning_budget=None):
        '''Return the planner result without discarding failure diagnostics'''
        if not getattr(self, 'config', {}).get('planner'):
            raise ValueError('Planned motion requires an explicit planner YAML block')
        self.check_state_fresh()
        if current_q is None:
            current_q = np.copy(self.robot_model.state['q'])
        frozen = ~self.robot_model.reduced_mask
        if not np.array_equal(
            current_q[frozen], self.robot_model.reduced_reference_q[frozen]
        ):
            self.robot_model.rebuild_reduced_model(reference_q=current_q)
            self.ik_solver.rebuild_reduced()
        if start is None:
            start, from_command = self._command_seed(current_q)
            if not from_command:
                self._tau_bias[:] = 0.0
            start = start[self.robot_model.reduced_mask]
        q_reduced = np.asarray(q_reduced, dtype=float)
        if local_frames is None:
            planning_config = self.planner.planner_config
            if planning_budget is None:
                planning_budget = planning_config.named_planning_budget
            if response_timeout is None:
                response_timeout = planning_config.worker_request_timeout
            local_frames = moving_grasp_frames(
                start, q_reduced, planning_config.frame_names,
                active_mask=active_mask,
            )
            if local_joint_padding is None:
                local_joint_padding = 0.0
        moving_speed, max_interpolation_steps = self._planner_speed_overrides()
        result = self.planner.plan_configuration(
            current_q=current_q,
            start=start,
            goal=q_reduced,
            active_mask=active_mask,
            moving_speed=moving_speed,
            max_interpolation_steps=max_interpolation_steps,
            direct_only=direct_only,
            timeout=timeout,
            local_frames=local_frames,
            local_joint_padding=local_joint_padding,
            response_timeout=response_timeout,
            planning_budget=planning_budget,
        )
        if result.success:
            result.metadata['execution_context'] = {
                'planned_at': time.monotonic(),
                'start': np.asarray(start, dtype=float).copy(),
                'frozen': np.asarray(current_q, dtype=float)[
                    ~self.robot_model.reduced_mask
                ].copy(),
                'active_mask': None if active_mask is None else
                np.asarray(active_mask, dtype=bool).copy(),
                'dt': self.dt,
            }
        return result

    def plan_to_ik_target(self, start=None, ik_timeout=None, ik_alpha=None):
        '''Solve current IK tasks, then plan to the IK joint solution'''
        started = time.perf_counter()
        self.last_plan_result = None
        self.last_planning_context = None
        measured_q = np.copy(self.robot_model.state['q'])
        command_q, from_command = self._command_seed(measured_q)
        if start is None:
            start_q = command_q
        else:
            start_q = measured_q.copy()
            start_q[self.robot_model.reduced_mask] = np.asarray(start, dtype=float)
        start_reduced = start_q[self.robot_model.reduced_mask].copy()
        self.last_planning_start = start_q.copy()
        self.update_ik_solver(measured_q=measured_q, command_seed=command_q)
        if not from_command:
            self._tau_bias[:] = 0.0
        # Use the same active joints and start pose for IK and planning.
        active_mask = self._active_task_mask()
        planning_config = self.planner.planner_config
        deadline = started + planning_config.request_planning_budget
        targets = frame_target_snapshot(self.ik_solver.frame_tasks)
        self.last_planning_context = {
            'start_q': start_q.tolist(),
            'measured_q': measured_q.tolist(),
            'tasks': [{
                'frame': name,
                'pose': np.r_[position, pin.rpy.matrixToRpy(rotation)].tolist(),
            } for name, position, rotation in targets],
            'ik_timeout': (planning_config.ik_total_timeout if ik_timeout is None
                           else ik_timeout),
            'ik_alpha': planning_config.ik_alpha if ik_alpha is None else ik_alpha,
            'assets': {
                key: str(getattr(self.planner, key, ''))
                for key in ('urdf_path', 'urdf_sphere_path', 'srdf_sphere_path')
            },
        }
        preferred = None
        previous = getattr(self, '_prior_ik', None)
        if (previous is not None and len(targets) == len(previous['targets']) and
                np.array_equal(active_mask, previous['active_mask'])):
            close_targets = frame_targets_close(targets, previous['targets'], planning_config)
            if close_targets:
                # reuse only active joint angles; re-solve at the new measured torso pose
                preferred = previous['q'][self.robot_model.reduced_mask].copy()
        local_frames = active_grasp_frames(active_mask, planning_config.frame_names)
        controller_cfg = getattr(self, 'config', {}).get('controller', {})

        def plan_goal(goal, mask, frames, direct_only, timeout):
            remaining = deadline - time.perf_counter()
            if remaining <= 0.0:
                raise TimeoutError('Frame planning deadline exceeded')
            return self._plan_configuration_result(
                goal, start=start_reduced, active_mask=mask,
                current_q=measured_q, direct_only=direct_only,
                timeout=timeout, local_frames=frames,
                response_timeout=min(planning_config.worker_request_timeout, remaining),
                planning_budget=remaining,
            )

        ik_result, result, selected, attempts, _, fallback_reason = plan_frame_tasks(
            self.ik_solver, start_q, active_mask, self.robot_model.reduced_mask,
            local_frames,
            plan_goal,
            planning_config, alpha=ik_alpha, ik_timeout=ik_timeout,
            linear_threshold=float(controller_cfg.get('threshold_linear', 1e-3)),
            angular_threshold=float(controller_cfg.get('threshold_angular', 1e-2)),
            preferred_positions=preferred,
            deadline=deadline,
        )
        self.last_ik_result = ik_result
        self.last_plan_attempts = attempts
        self.last_plan_result = result
        if selected is None:
            if ik_result is None:
                raise TimeoutError('Frame planning deadline exceeded before IK')
            if not ik_result.success:
                reasons = ', '.join(
                    f'{attempt.seed_name}: {attempt.reason or "no convergence"} '
                    f'({attempt.linear_error * 1000:.1f}mm, '
                    f'{attempt.angular_error:.4f}rad, {attempt.iterations} iterations)'
                    for attempt in ik_result.attempts
                )
                raise RuntimeError(
                    f'IK failed to resolve current targets ({reasons}); '
                    f'{fallback_reason}'
                )
            raise RuntimeError(
                f'No IK candidate has a valid path: {result.reason if result else "no candidates"}'
            )
        ik_result.q = selected.q.copy()
        ik_result.seed_name = selected.seed_name
        ik_result.linear_error = selected.linear_error
        ik_result.angular_error = selected.angular_error
        self.ik_solver.configuration_reduced.update(
            selected.q[self.robot_model.reduced_mask]
        )
        self._prior_ik = {
            'q': selected.q.copy(), 'targets': targets,
            'active_mask': active_mask.copy(),
        }
        return self._path_or_raise(result)

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

    def execute_path(self, path, validate=True, cancel_check=None):
        '''Execute reduced joint waypoints as direct position commands'''
        path = self._as_reduced_path(path)
        if not len(path):
            raise ValueError('Cannot execute an empty path')
        if validate:
            self.check_state_fresh()
            result = getattr(self, 'last_plan_result', None)
            if result is None or not result.success or not np.array_equal(path, result.path):
                raise ValueError('Path does not match the most recent validated plan')
            context = result.metadata.get('execution_context')
            if context is None:
                raise ValueError('Path is missing its execution context')
            if time.monotonic() - context['planned_at'] > getattr(
                self, 'plan_max_age', 30.0,
            ):
                raise ValueError('Path is too old to execute')
            if not np.isclose(context['dt'], self.dt, rtol=0.0, atol=1e-9):
                raise ValueError('Path period differs from controller period')
            measured = np.asarray(self.robot_model.state['q'], dtype=float)
            seed, _ = self._command_seed(measured)
            reduced_mask = self.robot_model.reduced_mask
            config = self.planner.planner_config
            motor_speed = np.minimum(
                np.broadcast_to(config.joint_velocity_limit, (path.shape[1],)),
                self.robot_model.model_body_reduced.velocityLimit,
            )
            # the first waypoint is another dt-sized step from the last command
            start_allowance = np.minimum.reduce((
                np.full(path.shape[1], getattr(
                    self, 'execution_start_tolerance', 0.01,
                )),
                motor_speed * self.dt,
                np.broadcast_to(config.joint_acceleration_limit, (path.shape[1],)) *
                self.dt ** 2,
            ))
            if np.any(np.abs(path[0] - seed[reduced_mask]) > start_allowance):
                raise ValueError('Robot command state drifted from planned start')
            clip_limits = getattr(self.low_cmd_handler, '_q_clip_limits', None)
            if clip_limits is not None:
                limits = np.asarray(clip_limits)[reduced_mask]
                if (np.any(path < limits[:, 0] - 1e-9) or
                        np.any(path > limits[:, 1] + 1e-9)):
                    waypoint, joint = np.argwhere(
                        (path < limits[:, 0] - 1e-9) | (path > limits[:, 1] + 1e-9)
                    )[0]
                    name = BODY_JOINTS[np.flatnonzero(reduced_mask)[joint]]
                    raise ValueError(
                        f'Path would be clipped by command position limits: '
                        f'waypoint {waypoint}, {name}={path[waypoint, joint]:.6f}, '
                        f'limits=[{limits[joint, 0]:.6f}, {limits[joint, 1]:.6f}]'
                    )
            self._validate_path_timing(path)

        # planner returns waypoints time-parameterized at the control period
        step_period = self.dt
        for q_reduced in tqdm(path, desc='Executing path', leave=False):
            if cancel_check is not None and cancel_check():
                raise PathExecutionCancelled('Path execution canceled')
            step_start_time = time.monotonic()
            self.check_state_fresh()
            q = np.asarray(self.robot_model.state['q'], dtype=float).copy()
            q[self.robot_model.reduced_mask] = q_reduced
            if validate:
                # measured frozen joints are included in each full-body clearance check
                if not self.robot_model.check_within_limits_reduced(q_reduced):
                    raise ValueError('Path waypoint is outside joint limits')
                if not self.robot_model.check_collision_free(
                    q,
                    min_distance=self.planner.planner_config.self_collision_clearance,
                ):
                    raise ValueError('Path waypoint violates current self-collision clearance')

            self._apply_joint_position(q)
            self.update_robot_model()
            # subtract validation and FK work from the next control-period wait
            time.sleep(max(
                0.0,
                step_period - (time.monotonic() - step_start_time)
            ))

    def _validate_path_timing(self, path):
        '''Reject positions whose playback would exceed planned joint rates'''
        config = self.planner.planner_config
        if not np.isclose(config.dt, self.dt, rtol=0.0, atol=1e-9):
            raise ValueError('Path playback period differs from planner period')
        velocity_limit = np.minimum(
            np.broadcast_to(config.joint_velocity_limit, (path.shape[1],)),
            self.robot_model.model_body_reduced.velocityLimit,
        )
        rates = np.diff(path, axis=0) / self.dt
        if np.any(np.abs(rates) > velocity_limit + 1e-6):
            raise ValueError('Path exceeds joint velocity limits')
        accelerations = np.diff(np.vstack([
            np.zeros(path.shape[1]), rates, np.zeros(path.shape[1]),
        ]), axis=0) / self.dt
        if np.any(
            np.abs(accelerations) >
            np.broadcast_to(config.joint_acceleration_limit, (path.shape[1],)) + 1e-6
        ):
            raise ValueError('Path exceeds joint acceleration limits')

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
        self.check_state_fresh()
        # solve IK and apply control
        vel = self.ik_solver.goto_configuration(q)
        vel = self._limit_joint_vel(vel)
        return self._apply_velocity_command(vel)

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
        self.check_state_fresh()
        # solve IK and apply control
        vel = self.ik_solver.goto_reduced_configuration(q_reduced)
        vel = self._limit_joint_vel(vel)
        return self._apply_velocity_command(vel)

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
        self.check_state_fresh()
        # solve IK and apply the control
        vel = self.ik_solver.ik_step(com=com)
        vel = self._limit_joint_vel(vel)
        return self._apply_velocity_command(vel)

    def control_step_reduced(self, com=False):
        '''Solve IK for all tasks with the reduced model'''
        self.check_state_fresh()
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced(com=com)
        vel = self._limit_joint_vel(vel)
        return self._apply_velocity_command(vel)

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
        '''
        Initialize steady-state I control from the final IK command. The
        integration resumes from the current torque bias so the hand-off from
        a move to a hold is continuous.
        '''
        self._steady_q_cmd = np.copy(self.low_cmd_handler.q_cmd)
        self._steady_dq_cmd = np.zeros(self.robot_model.model_body.nv)
        ki = self.config.get('gains', {}).get('ki')
        if ki is None:
            self._steady_ki = np.zeros(self.robot_model.model_body.nv)
        else:
            self._steady_ki = np.asarray(ki, dtype=np.float64)

        self._steady_tau_bias_limit = np.zeros_like(self.robot_model.model_body.nv)
        tau_clip_limits = self.config.get('limits', {}).get('tau_clip_limits')
        if tau_clip_limits is not None:
            self._steady_tau_bias_limit = np.asarray(tau_clip_limits, dtype=np.float64)

        # (joint name, q, limit) for joints the hold is pinned against
        self.steady_state_limited_joints = []
        self._steady_q_clip_limits = None
        q_clip_limits = self.config.get('limits', {}).get('q_clip_limits')
        if q_clip_limits is not None:
            self._steady_q_clip_limits = np.asarray(q_clip_limits, dtype=np.float64)

    def _steady_state_limited_joints(self, q_state, tau_bias_increment, threshold):
        '''
        (joint name, q, limit) for each upper-body joint within `threshold` of
        a position clip limit that the I-term is pushing further into
        '''
        if self._steady_q_clip_limits is None:
            return []
        limited = []
        for i in self._steady_state_control_ids():
            low, high = self._steady_q_clip_limits[i]
            if q_state[i] >= high - threshold and tau_bias_increment[i] > 0.0:
                limited.append((BODY_JOINTS[i], float(q_state[i]), float(high)))
            elif q_state[i] <= low + threshold and tau_bias_increment[i] < 0.0:
                limited.append((BODY_JOINTS[i], float(q_state[i]), float(low)))
        return limited

    def steady_state_step(self, threshold=1e-3):
        '''
        Run one steady-state I-control tick and return convergence. A hold
        pinned against a joint position limit cannot converge: the I-term is
        frozen, steady_state_limited_joints names the joints, and this
        returns False.
        '''
        self.check_state_fresh()
        if not hasattr(self, '_steady_q_cmd'):
            self.init_steady_state()

        # get positional error
        q_state = self.robot_model.state['q']
        q_error = self._steady_q_cmd - q_state
        tau_bias_increment = self._steady_ki * q_error * self.dt
        # close to clipping limit
        self.steady_state_limited_joints = self._steady_state_limited_joints(
            q_state,
            tau_bias_increment,
            threshold,
        )
        if self.steady_state_limited_joints:
            tau_gravity = self.robot_model.dynamics.get_gravity_compensation(
                self.robot_model.state['q']
            )
            tau_cmd = tau_gravity + self._tau_bias
            self._publish_position_command(
                self._steady_q_cmd,
                self._steady_dq_cmd,
                tau_cmd,
            )
            self.update_robot_model()
            return False
        else:
            self._tau_bias += tau_bias_increment
            self._tau_bias = np.clip(
                self._tau_bias,
                -self._steady_tau_bias_limit,
                self._steady_tau_bias_limit,
            )

            # compute total torque to command
            tau_gravity = self.robot_model.dynamics.get_gravity_compensation(
                self.robot_model.state['q']
            )
            tau_cmd = tau_gravity + self._tau_bias
            self._publish_position_command(
                self._steady_q_cmd,
                self._steady_dq_cmd,
                tau_cmd,
            )

            # check convergence
            self.update_robot_model()
            q_error = self._steady_q_cmd - self.robot_model.state['q']
            control_ids = self._steady_state_control_ids()
            return bool(np.max(np.abs(q_error[control_ids])) < threshold)

    def _steady_state_control_ids(self):
        '''Return joints used for steady-state clipping and convergence'''
        return self.upper_ids

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
        vel_scaled[LEFT_ARM_INDEX] = self.speed_scale * left_scaler * vel[LEFT_ARM_INDEX]
        vel_scaled[RIGHT_ARM_INDEX] = self.speed_scale * right_scaler * vel[RIGHT_ARM_INDEX]

        return vel_scaled

    def _apply_joint_position(self, q):
        # gravity compensation plus the held steady-state bias, which fades
        # by one control step per call
        tau = self.robot_model.dynamics.get_gravity_compensation(self.robot_model.state['q'])
        tau = tau + self._tau_bias
        self._tau_bias *= self._tau_bias_decay
        dq = np.zeros(self.robot_model.model_body.nv)
        self._publish_position_command(q, dq, tau)

    def _publish_position_command(self, q, dq, tau):
        '''Publish one positional command generated by frame tracking'''
        # arm-generated full vectors contain measured torso angles, not torso goals
        torso_id = BODY_JOINTS.index('torso_joint')
        q = np.asarray(q).copy()
        dq = np.asarray(dq).copy()
        q[torso_id] = self.low_cmd_handler.q_cmd[torso_id]
        dq[torso_id] = 0.0
        self.low_cmd_handler.set_joint_commands(q, dq, tau)

    def _apply_velocity_command(self, vel):
        self.ik_solver.integrate(vel)
        self._apply_joint_position(self.ik_solver.q)
        self.update_robot_model()
        return vel

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
