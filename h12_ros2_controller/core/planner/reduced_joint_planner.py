import numpy as np
import pinocchio as pin
from scipy.interpolate import PchipInterpolator
from ompl import base as ob
from ompl import geometric as og
from dataclasses import dataclass, field
from time import perf_counter

_ARM_REDUCED_NQ = 14
_LENGTH_ESTIMATE_STEPS = 50
# approximate grasp radius for converting rotation to swept arc length


def moving_grasp_frames(start, goal, frame_names, active_mask=None):
    '''Select configured grasp frames for arms changing joint position'''
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if start.shape != (_ARM_REDUCED_NQ,) or goal.shape != start.shape:
        raise ValueError('Named goto requires 14 reduced joint positions')
    delta = np.abs(goal - start)
    if active_mask is not None:
        mask = np.asarray(active_mask, dtype=bool)
        if mask.shape != start.shape:
            raise ValueError('active_mask must match the reduced configuration')
        delta[~mask] = 0.0
    frames = []
    for name, joints in (('left_grasp_frame', slice(0, 7)),
                         ('right_grasp_frame', slice(7, 14))):
        if np.any(delta[joints] > 1e-6):
            if name not in frame_names:
                raise ValueError(f'Planner locality requires {name} in frame_names')
            frames.append(name)
    return tuple(frames)


@dataclass
class PlannerConfig:
    '''Configuration for reduced joint-space planning'''

    planner: str = 'RRTConnect'
    timeout: float = 1.0
    range: float = 0.0
    try_direct: bool = True
    simplify: bool = True
    simplify_time: float = 1.0
    moving_speed: float = 0.25
    dt: float = 1.0 / 30.0
    joint_velocity_limit: float = 1.0
    joint_acceleration_limit: float = 25.0
    continuous_timing: bool = True
    min_interpolation_steps: int = 2
    max_interpolation_steps: int = 600
    validity_resolution: float = 0.0025
    constraint_check_steps: int = 10
    frame_names: tuple = ('left_grasp_frame', 'right_grasp_frame')
    frame_x_max: float = 0.50
    frame_y_min: float = None
    frame_y_max: float = None
    frame_z_min: float = None
    frame_z_max: float = 0.50
    frame_limit_margin: float = 1e-3
    frame_z_corridor_margin: float = 0.05
    local_x_backtrack: float = 0.15
    max_local_path_ratio: float = 2.0
    elbow_x_backtrack: float = 0.22
    max_hand_deviation: float = 0.45
    inactive_goal_tolerance: float = 1e-6
    frame_rotation_radius: float = 0.20
    max_ik_candidates: int = 3
    ik_max_seeds: int = 8
    ik_seed_timeout: float = 0.30
    ik_wrist_motion_weight: float = 3.0
    ik_wrist_posture_cost: float = 0.005
    ik_shoulder_seed_offset: float = 0.45
    ik_elbow_seed_offset: float = 0.60
    ik_reuse_translation: float = 0.12
    ik_reuse_rotation: float = 0.50
    candidate_planning_budget: float = 2.0
    bimanual_fallback_budget: float = 5.0
    recent_path_cache_size: int = 6
    candidate_ompl_timeout: float = 0.75
    candidate_early_accept_ratio: float = 1.3
    candidate_ik_weight: float = 0.1
    candidate_hand_weight: float = 2.0
    worker_request_timeout: float = 10.0
    worker_startup_timeout: float = 30.0
    # measured starts can sit marginally past joint limits; only clamp small overshoots
    start_limit_tolerance: float = 0.05


@dataclass
class PlanResult:
    '''Result returned by reduced joint-space planning'''

    success: bool
    path: np.ndarray
    reason: str = ''
    planning_time: float = 0.0
    planner_name: str = ''
    metadata: dict = field(default_factory=dict)


class ReducedJointPlanner:
    def __init__(self, robot_model, config: PlannerConfig = None):
        self.robot_model = robot_model
        self.config = config if config is not None else PlannerConfig()
        self._check_robot_model_ready()
        self._check_planner_config()

        self.nq = int(self.robot_model.model_body_reduced.nq)
        self.validity_checks = 0
        self.frame_names = tuple(self.config.frame_names)
        self.frame_min_z = self._make_frame_floor_map(self.config.frame_z_min)
        self.frame_max_z = self._make_frame_floor_map(self.config.frame_z_max)
        self.corridor_min_z = {}
        self.local_min_x = {}
        self.local_elbow_min_x = {}
        # Each planner owns its FK data; never borrow the live model's data
        self._frame_model = getattr(self.robot_model, 'model', None)
        self._frame_data = (
            self._frame_model.createData()
            if self._frame_model is not None and
            hasattr(self.robot_model, 'full_q') else None
        )
        self._frame_ids = (
            {name: self._frame_model.getFrameId(name) for name in self.frame_names}
            if self._frame_data is not None else {}
        )
        self._elbow_ids = {}
        if self._frame_data is not None:
            for side in ('left', 'right'):
                name = f'{side}_elbow_link'
                frame_id = self._frame_model.getFrameId(name)
                if frame_id < len(self._frame_model.frames):
                    self._elbow_ids[f'{side}_grasp_frame'] = frame_id
        # active joint mask and pinned values for inactive joints
        self.active_mask = np.ones(self.nq, dtype=bool)
        self.pinned_q = np.zeros(self.nq)
        self._recent_paths = []

        # build OMPL problem objects once; plan() only resets start/goal
        self.space = self._make_space()
        self.ss = og.SimpleSetup(self.space)
        self.si = self.ss.getSpaceInformation()
        self.ss.setStateValidityChecker(self._make_validity_checker())
        self.si.setStateValidityCheckingResolution(
            float(self.config.validity_resolution)
        )
        self._set_optimization_objective()
        self.ss.setPlanner(self._make_planner())

    def rebind_reduced_model(self):
        '''Refresh OMPL space after the robot's reduced model is rebuilt'''
        self._check_robot_model_ready()
        self._recent_paths = []
        self.nq = int(self.robot_model.model_body_reduced.nq)
        self.active_mask = np.ones(self.nq, dtype=bool)
        self.pinned_q = np.zeros(self.nq)
        self.space = self._make_space()
        self.ss = og.SimpleSetup(self.space)
        self.si = self.ss.getSpaceInformation()
        self.ss.setStateValidityChecker(self._make_validity_checker())
        self.si.setStateValidityCheckingResolution(
            float(self.config.validity_resolution)
        )
        self._set_optimization_objective()
        self.ss.setPlanner(self._make_planner())

    def _set_optimization_objective(self):
        if self.config.planner.lower() in ('bitstar', 'informedrrtstar'):
            self.ss.getProblemDefinition().setOptimizationObjective(
                ob.PathLengthOptimizationObjective(self.si)
            )

    def _check_robot_model_ready(self):
        if not getattr(self.robot_model, 'init_reduced', False):
            raise ValueError('RobotModel reduced model is not initialized')
        if not getattr(self.robot_model, 'init_collision', False):
            raise ValueError('RobotModel collision model is not initialized')

    def _check_planner_config(self):
        if not np.isfinite(self.config.moving_speed) or self.config.moving_speed <= 0.0:
            raise ValueError('Planner moving_speed must be positive')
        if not np.isfinite(self.config.dt) or self.config.dt <= 0.0:
            raise ValueError('Planner dt must be positive')
        for name in ('joint_velocity_limit', 'joint_acceleration_limit'):
            values = np.asarray(getattr(self.config, name), dtype=float)
            if not np.all(np.isfinite(values)) or np.any(values <= 0):
                raise ValueError(f'Planner {name} must be finite and positive')
        for name in ('timeout', 'validity_resolution'):
            value = getattr(self.config, name)
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f'Planner {name} must be finite and positive')
        for name in ('frame_rotation_radius', 'ik_seed_timeout',
                     'ik_wrist_motion_weight', 'ik_wrist_posture_cost',
                     'ik_shoulder_seed_offset', 'ik_elbow_seed_offset',
                     'ik_reuse_translation', 'ik_reuse_rotation',
                     'candidate_planning_budget', 'candidate_ompl_timeout',
                     'bimanual_fallback_budget',
                     'worker_request_timeout', 'worker_startup_timeout'):
            value = getattr(self.config, name)
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f'Planner {name} must be finite and positive')
        for name in ('candidate_early_accept_ratio',):
            value = getattr(self.config, name)
            if not np.isfinite(value) or value < 1.0:
                raise ValueError(f'Planner {name} must be finite and at least 1')
        for name in ('candidate_ik_weight', 'candidate_hand_weight'):
            value = getattr(self.config, name)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f'Planner {name} must be finite and non-negative')
        for name in ('max_ik_candidates', 'ik_max_seeds'):
            value = getattr(self.config, name)
            if not isinstance(value, int) or value < 1:
                raise ValueError(f'Planner {name} must be a positive integer')
        if (not isinstance(self.config.recent_path_cache_size, int) or
                not 1 <= self.config.recent_path_cache_size <= 32):
            raise ValueError('Planner recent_path_cache_size must be between 1 and 32')
        for name in ('range', 'simplify_time', 'frame_limit_margin',
                     'frame_z_corridor_margin', 'start_limit_tolerance',
                     'inactive_goal_tolerance'):
            value = getattr(self.config, name)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f'Planner {name} must be finite and non-negative')
        for name in ('frame_x_max', 'frame_y_min', 'frame_y_max'):
            value = getattr(self.config, name)
            if value is not None and not np.isfinite(value):
                raise ValueError(f'Planner {name} must be finite')
        for name in ('frame_z_min', 'frame_z_max'):
            value = getattr(self.config, name)
            values = value.values() if isinstance(value, dict) else (value,)
            if any(item is not None and not np.isfinite(item) for item in values):
                raise ValueError(f'Planner {name} must contain finite values')
            if isinstance(value, dict) and set(value) - set(self.config.frame_names):
                raise ValueError(f'Planner {name} contains unknown frame names')
        def frame_bounds(value):
            if value is None:
                return {}
            if isinstance(value, dict):
                return value
            return {name: value for name in self.config.frame_names}

        floors = frame_bounds(self.config.frame_z_min)
        ceilings = frame_bounds(self.config.frame_z_max)
        if any(floors[name] > ceilings[name] for name in floors.keys() & ceilings.keys()):
            raise ValueError('Planner frame_z_min must not exceed frame_z_max')
        if self.config.constraint_check_steps < 1:
            raise ValueError('Planner constraint_check_steps must be at least 1')
        if self.config.min_interpolation_steps < 2:
            raise ValueError('Planner min_interpolation_steps must be at least 2')
        if (
            self.config.max_interpolation_steps <
            self.config.min_interpolation_steps
        ):
            raise ValueError(
                'Planner max_interpolation_steps must be at least '
                'min_interpolation_steps'
            )
        if (not np.isfinite(self.config.local_x_backtrack) or
                self.config.local_x_backtrack < 0):
            raise ValueError('Planner local_x_backtrack must be finite and non-negative')
        if (not np.isfinite(self.config.max_local_path_ratio) or
                self.config.max_local_path_ratio < 1):
            raise ValueError('Planner max_local_path_ratio must be finite and at least 1')
        if not np.isfinite(self.config.elbow_x_backtrack) or self.config.elbow_x_backtrack < 0:
            raise ValueError('Planner elbow_x_backtrack must be finite and non-negative')
        if not np.isfinite(self.config.max_hand_deviation) or self.config.max_hand_deviation <= 0:
            raise ValueError('Planner max_hand_deviation must be finite and positive')
        if (
            self.config.frame_y_min is not None and
            self.config.frame_y_max is not None and
            self.config.frame_y_min > self.config.frame_y_max
        ):
            raise ValueError('Planner frame_y_min must not exceed frame_y_max')

    def _make_space(self):
        # copy Pinocchio reduced joint limits into an OMPL real-vector space
        self.model_lower = np.asarray(
            self.robot_model.model_body_reduced.lowerPositionLimit,
            dtype=float,
        )
        self.model_upper = np.asarray(
            self.robot_model.model_body_reduced.upperPositionLimit,
            dtype=float,
        )
        lower, upper = self.model_lower, self.model_upper
        if lower.shape != upper.shape:
            raise ValueError('Reduced model lower and upper bounds differ')
        if lower.shape != (self.nq,):
            raise ValueError('Reduced model bounds do not match nq')
        if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
            raise ValueError('Reduced model bounds must be finite')
        if np.any(lower >= upper):
            raise ValueError(
                'Reduced model lower bounds must be below upper bounds'
            )

        space = ob.RealVectorStateSpace(self.nq)
        space.setBounds(self._make_bounds(lower, upper))
        return space

    def _make_bounds(self, lower, upper):
        bounds = ob.RealVectorBounds(self.nq)
        for idx in range(self.nq):
            bounds.setLow(idx, float(lower[idx]))
            bounds.setHigh(idx, float(upper[idx]))
        return bounds

    def _apply_active_bounds(self):
        '''Collapse OMPL bounds of inactive joints to their pinned value'''
        lower = np.copy(self.model_lower)
        upper = np.copy(self.model_upper)
        # pin inactive joints to a tiny window around their start value
        lower[~self.active_mask] = self.pinned_q[~self.active_mask]
        upper[~self.active_mask] = self.pinned_q[~self.active_mask]
        self.space.setBounds(self._make_bounds(lower, upper))

    def _project_inactive(self, q):
        '''Force inactive joints to their pinned start values'''
        if np.all(self.active_mask):
            return q
        q = np.array(q, dtype=float)
        q[~self.active_mask] = self.pinned_q[~self.active_mask]
        return q

    def _make_planner(self):
        # resolve by name at runtime because OMPL wheels expose different sets
        planner_name = self.config.planner.lower()
        planner_classes = {
            'rrtconnect': 'RRTConnect',
            'rrt': 'RRT',
            'rrtstar': 'RRTstar',
            'bitstar': 'BITstar',
            'informedrrtstar': 'InformedRRTstar',
            'prm': 'PRM',
            'lazyprm': 'LazyPRM',
            'kpiece1': 'KPIECE1',
        }
        class_name = planner_classes.get(planner_name)
        planner_cls = getattr(og, class_name, None) if class_name else None
        if planner_cls is None:
            raise ValueError(
                f'Unsupported OMPL planner: {self.config.planner}'
            )

        planner = planner_cls(self.si)
        if self.config.range > 0.0 and hasattr(planner, 'setRange'):
            planner.setRange(float(self.config.range))
        return planner

    def _make_validity_checker(self):
        # support both common OMPL Python callback binding styles
        if hasattr(ob, 'StateValidityCheckerFn'):
            return ob.StateValidityCheckerFn(self._is_state_valid)
        return self._is_state_valid

    def _as_reduced_array(self, q, name):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape != (self.nq,):
            raise ValueError(f'{name} must have shape ({self.nq},)')
        if not np.all(np.isfinite(q)):
            raise ValueError(f'{name} must contain only finite values')
        return q

    def _empty_path(self):
        return np.empty((0, self.nq), dtype=float)

    def _make_state(self, q):
        # allocate a state using whichever API this OMPL binding supports
        try:
            state = ob.State(self.space)
            values = state()
        except TypeError:
            state = self.space.allocState()
            values = state
        for idx, value in enumerate(q):
            values[idx] = float(value)
        return state

    def _state_to_array(self, state):
        q = np.asarray([state[idx] for idx in range(self.nq)], dtype=float)
        # guarantee inactive joints never drift from their pinned value
        return self._project_inactive(q)

    def _is_state_valid(self, state):
        self.validity_checks += 1
        q = self._state_to_array(state)
        reason = self._validity_failure(q, 'state')
        return reason == ''

    def _validity_failure(self, q, name):
        if not self.robot_model.check_within_limits_reduced(q):
            return (
                f'{name} is outside reduced joint limits: '
                f'{self._format_limit_violations(q)}'
            )
        if not self.robot_model.check_collision_free_reduced(q):
            return f'{name} is in self-collision'
        reason = self._workspace_constraint_failure(q, name)
        if reason:
            return reason
        return ''

    def _format_limit_violations(self, q):
        violations = self.robot_model.reduced_limit_violations(q)
        if not violations:
            return '(no joint outside limits; check tolerance)'
        return ', '.join(
            f'{v["name"]}={v["value"]:.4f} '
            f'(limit {v["bound"]:.4f}, {v["overshoot"]:.4f} rad over)'
            for v in violations
        )

    def _clamp_start_into_limits(self, start):
        '''
        Nudge a start config that sits marginally past a joint limit back into
        bounds. Only overshoots within start_limit_tolerance are clamped;
        larger ones are left untouched so plan() still rejects a bad pose.
        '''
        tol = self.config.start_limit_tolerance
        if tol is None or tol <= 0.0:
            return start
        lower = self.robot_model.model_body_reduced.lowerPositionLimit
        upper = self.robot_model.model_body_reduced.upperPositionLimit
        clamped = np.clip(start, lower, upper)
        delta = clamped - start
        within = np.abs(delta) <= tol
        result = np.where(within, clamped, start)
        moved = within & (delta != 0.0)
        if np.any(moved):
            idx_to_name = self.robot_model._reduced_idx_to_name()
            detail = ', '.join(
                f'{idx_to_name.get(i, f"q[{i}]")} '
                f'{start[i]:.4f}->{result[i]:.4f}'
                for i in np.where(moved)[0]
            )
            self._diagnostics.append(
                f'clamped start into joint limits (tol={tol:.4f}): {detail}'
            )
        return result

    def _motor_q(self, q_reduced):
        q = np.copy(self.robot_model.state['q'])
        q[self.robot_model.reduced_mask] = q_reduced
        return q

    def _sample_path_for_length(self, path):
        if len(path) < 2 or len(path) >= _LENGTH_ESTIMATE_STEPS:
            return path

        # sample sparse joint paths before measuring end-effector motion
        segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        distance = np.concatenate([[0.0], np.cumsum(segment_lengths)])
        if distance[-1] <= 0.0:
            return path

        sample_distance = np.linspace(0.0, distance[-1], _LENGTH_ESTIMATE_STEPS)
        return np.asarray([
            np.interp(sample_distance, distance, path[:, idx])
            for idx in range(path.shape[1])
        ]).T

    def _rotation_angle(self, rotation_a, rotation_b):
        rotation_delta = rotation_a.T @ rotation_b
        cos_angle = 0.5 * (np.trace(rotation_delta) - 1.0)
        return float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

    def _frame_poses(self, q_reduced, rotation=False):
        if self._frame_data is not None:
            pin.forwardKinematics(
                self._frame_model, self._frame_data,
                self.robot_model.full_q(self._motor_q(q_reduced)),
            )
            pin.updateFramePlacements(self._frame_model, self._frame_data)
            return {
                name: (
                    np.array(self._frame_data.oMf[frame_id].translation),
                    np.array(self._frame_data.oMf[frame_id].rotation)
                    if rotation else None,
                )
                for name, frame_id in self._frame_ids.items()
            }
        q = self._motor_q(q_reduced)
        return {
            name: (
                self.robot_model.get_frame_position(name, q),
                self.robot_model.get_frame_rotation(name, q)
                if rotation and hasattr(self.robot_model, 'get_frame_rotation')
                else None,
            )
            for name in self.frame_names
        }

    def _frame_path_length(self, path, frame_name, poses=None):
        if len(path) < 2:
            return 0.0, 0.0, 0.0

        if poses is None:
            poses = [
                self._frame_poses(
                    q, rotation=hasattr(self.robot_model, 'get_frame_rotation')
                ) for q in path
            ]
        positions = np.asarray(
            [pose[frame_name][0] for pose in poses]
        )
        deltas = np.diff(positions, axis=0)
        translation_length = float(np.sum(np.linalg.norm(deltas, axis=1)))

        rotation_length = 0.0
        if poses and poses[0][frame_name][1] is not None:
            rotations = [pose[frame_name][1] for pose in poses]
            rotation_angle = sum(
                self._rotation_angle(rotations[idx], rotations[idx + 1])
                for idx in range(len(rotations) - 1)
            )
            # convert frame rotation to equivalent swept arc length
            rotation_length = self.config.frame_rotation_radius * rotation_angle

        return (
            translation_length + rotation_length,
            translation_length,
            rotation_length,
        )

    def _interpolation_metadata(self, path):
        sample_path = self._sample_path_for_length(path)
        poses = [
            self._frame_poses(q, rotation=hasattr(self.robot_model, 'get_frame_rotation'))
            for q in sample_path
        ] if len(sample_path) >= 2 else []
        frame_lengths = {
            frame_name: self._frame_path_length(sample_path, frame_name, poses)
            for frame_name in self.frame_names
        }
        lengths = {
            frame_name: values[0]
            for frame_name, values in frame_lengths.items()
        }
        translation_lengths = {
            frame_name: values[1]
            for frame_name, values in frame_lengths.items()
        }
        rotation_lengths = {
            frame_name: values[2]
            for frame_name, values in frame_lengths.items()
        }
        path_length = max(lengths.values(), default=0.0)

        speed = float(self.config.moving_speed)
        dt = float(self.config.dt)
        # path duration comes from the longer grasp-frame trajectory
        steps = max(int(self.config.min_interpolation_steps),
                    int(np.ceil(path_length / speed / dt)) + 1)
        duration = max(steps - 1, 1) * dt
        metadata = {
            'waypoint_count': steps,
            'required_points': steps,
            'duration': duration,
            'effective_speed': path_length / duration,
            'path_length': path_length,
            'frame_path_lengths': lengths,
            'frame_translation_lengths': translation_lengths,
            'frame_rotation_lengths': rotation_lengths,
            'length_estimate_samples': len(sample_path),
        }
        return metadata

    def _timed_path(self, sparse_path, profile='segment'):
        '''Sample a sparse geometric path within discrete timing limits'''
        if profile not in ('segment', 'pchip'):
            raise ValueError(f'Unsupported time profile: {profile}')
        metadata = self._interpolation_metadata(sparse_path)
        metadata['time_profile'] = profile
        dt = float(self.config.dt)
        velocity_limit = np.broadcast_to(
            self.config.joint_velocity_limit, (self.nq,)
        ).astype(float).copy()
        model_limit = np.asarray(
            getattr(self.robot_model.model_body_reduced, 'velocityLimit',
                    np.full(self.nq, np.inf)), dtype=float
        )
        if model_limit.shape == (self.nq,):
            velocity_limit = np.minimum(velocity_limit, model_limit)
        acceleration_limit = np.broadcast_to(
            self.config.joint_acceleration_limit, (self.nq,)
        ).astype(float)
        if np.any(velocity_limit <= 0) or not np.all(np.isfinite(velocity_limit)):
            raise ValueError('Reduced model velocity limits must be positive and finite')
        segments = np.diff(sparse_path, axis=0)
        if profile == 'pchip':
            # repeated knots cannot define a PCHIP parameter; retain the
            # original geometry and use segment timing instead
            if len(sparse_path) < 3 or np.any(np.linalg.norm(segments, axis=1) == 0):
                return None, metadata, 'PCHIP requires distinct sparse vertices'
        counts = np.maximum(1, np.ceil(np.max(
            np.abs(segments) / (velocity_limit * dt), axis=1
        )).astype(int))
        lengths = np.linalg.norm(segments, axis=1)
        knots = np.r_[0.0, np.cumsum(lengths)]
        if profile == 'pchip' and np.any(np.diff(knots) <= 0):
            return None, metadata, 'PCHIP requires distinct arc-length knots'
        curve = PchipInterpolator(knots, sparse_path, axis=0) if profile == 'pchip' else None
        if np.sum(lengths) > 0:
            frame_intervals = metadata['required_points'] - 1
            counts = np.maximum(counts, np.ceil(
                frame_intervals * lengths / np.sum(lengths)
            ).astype(int))
        # ease along moving segments, keeping every sparse geometric vertex
        counts = np.where(lengths > 0.0, np.maximum(counts, 2), counts)
        max_points = int(self.config.max_interpolation_steps)
        while True:
            required = int(np.sum(counts)) + 1
            metadata['required_points'] = required
            if required > max_points:
                metadata['waypoint_count'] = required
                metadata['duration'] = (required - 1) * dt
                return None, metadata, (
                    f'timing requires {required} points, exceeds '
                    f'max_interpolation_steps={max_points}'
                )
            samples = [sparse_path[0]]
            for idx, intervals in enumerate(counts):
                t = np.arange(1, intervals + 1, dtype=float) / intervals
                if curve is None:
                    alpha = t * t * (3.0 - 2.0 * t)
                    samples.extend(sparse_path[idx] + alpha[:, None] * segments[idx])
                else:
                    samples.extend(curve(knots[idx] + t * lengths[idx]))
            path = np.asarray(samples)
            path[0] = sparse_path[0]
            path[-1] = sparse_path[-1]
            if curve is not None:
                path[np.cumsum(counts)[:-1]] = sparse_path[1:-1]
            rates = np.diff(path, axis=0) / dt
            accelerations = np.diff(np.vstack([
                np.zeros(self.nq), rates, np.zeros(self.nq)
            ]), axis=0) / dt
            max_joint_speed = float(np.max(np.abs(rates), initial=0.0))
            max_joint_acc = float(np.max(np.abs(accelerations), initial=0.0))
            speed_ratio = float(np.max(np.abs(rates) / velocity_limit, initial=0.0))
            acc_ratio = float(np.max(
                np.abs(accelerations) / acceleration_limit, initial=0.0
            ))
            poses = [self._frame_poses(q, rotation=True) for q in path]
            frame_speeds = {}
            for name in self.frame_names:
                distances = []
                for first, second in zip(poses, poses[1:]):
                    distance = np.linalg.norm(second[name][0] - first[name][0])
                    if first[name][1] is not None and second[name][1] is not None:
                        distance += self.config.frame_rotation_radius * self._rotation_angle(
                            first[name][1], second[name][1]
                        )
                    distances.append(distance)
                frame_speeds[name] = float(max(distances, default=0.0) / dt)
            max_frame_speed = max(frame_speeds.values(), default=0.0)
            frame_ratio = max_frame_speed / self.config.moving_speed
            duration = (len(path) - 1) * dt
            metadata.update({
                'waypoint_count': len(path), 'actual_waypoints': len(path),
                'duration': duration,
                'effective_speed': metadata['path_length'] / duration,
                'actual_max_joint_speed': max_joint_speed,
                'actual_max_joint_acceleration': max_joint_acc,
                'actual_max_frame_speed': max_frame_speed,
                'actual_frame_speeds': frame_speeds,
            })
            ratio = max(speed_ratio, np.sqrt(acc_ratio), frame_ratio)
            if ratio <= 1.0 + 1e-9:
                return path, metadata, ''
            counts = np.maximum(counts + 1, np.ceil(counts * max(1.1, ratio)).astype(int))

    def _select_timed_path(self, sparse_path, start, goal, local_frames):
        '''Accept a fully validated continuous proposal or retain segment timing'''
        if self.config.continuous_timing and len(sparse_path) >= 3:
            proposal, proposed_metadata, rejection = self._timed_path(
                sparse_path, profile='pchip'
            )
            if proposal is not None:
                rejection = self._path_validity_failure(proposal)
                if not rejection:
                    quality, rejection = self._path_quality_failure(
                        proposal, start, goal, local_frames,
                    )
                    proposed_metadata.update(quality)
            if not rejection:
                segment_path, segment_metadata, segment_failure = self._timed_path(
                    sparse_path
                )
                if (not segment_failure and
                        segment_metadata['duration'] < proposed_metadata['duration']):
                    segment_failure = self._path_validity_failure(segment_path)
                    if not segment_failure:
                        quality, segment_failure = self._path_quality_failure(
                            segment_path, start, goal, local_frames,
                        )
                        segment_metadata.update(quality)
                    if not segment_failure:
                        segment_metadata['pchip_rejection'] = 'segment timing is shorter'
                        return segment_path, segment_metadata, ''
                return proposal, proposed_metadata, ''
            path, metadata, failure = self._timed_path(sparse_path)
            metadata['pchip_rejection'] = rejection
            return path, metadata, failure
        return self._timed_path(sparse_path)

    def _workspace_constraint_failure(self, q_reduced, name):
        if (
            not self.frame_min_z and
            not self.frame_max_z and
            not self.corridor_min_z and
            not self.local_min_x and
            not self.local_elbow_min_x and
            self.config.frame_x_max is None and
            self.config.frame_y_min is None and
            self.config.frame_y_max is None
        ):
            return ''

        # keep selected frames within configured workspace limits
        margin = self.config.frame_limit_margin
        poses = self._frame_poses(q_reduced)
        for frame_name, (position, _) in poses.items():
            x = position[0]
            y = position[1]
            z = position[2]
            local_x = self.local_min_x.get(frame_name)
            if local_x is not None and x + margin < local_x:
                return (
                    f'{name} moves {frame_name} behind local x={local_x:.4f} '
                    f'(x={x:.4f})'
                )
            elbow_x = self.local_elbow_min_x.get(frame_name)
            if elbow_x is not None:
                elbow = self._frame_data.oMf[self._elbow_ids[frame_name]].translation
                if elbow[0] + margin < elbow_x:
                    return (
                        f'{name} moves {frame_name} elbow behind local '
                        f'x={elbow_x:.4f} (x={elbow[0]:.4f})'
                    )
            min_z = self.frame_min_z.get(frame_name)
            corridor_z = self.corridor_min_z.get(frame_name)
            if corridor_z is not None:
                min_z = corridor_z if min_z is None else max(min_z, corridor_z)
            if min_z is not None and z + margin < min_z:
                return (
                    f'{name} moves {frame_name} below z={min_z:.4f} '
                    f'(z={z:.4f}, {min_z - z:.4f}m below floor)'
                )
            max_z = self.frame_max_z.get(frame_name)
            if max_z is not None and z - margin > max_z:
                return (
                    f'{name} moves {frame_name} above z={max_z:.4f} '
                    f'(z={z:.4f}, {z - max_z:.4f}m above ceiling)'
                )
            if (
                self.config.frame_x_max is not None and
                x - margin > self.config.frame_x_max
            ):
                return (
                    f'{name} moves {frame_name} beyond '
                    f'x={self.config.frame_x_max:.4f} '
                    f'(x={x:.4f}, {x - self.config.frame_x_max:.4f}m beyond)'
                )
            if (
                self.config.frame_y_min is not None and
                y + margin < self.config.frame_y_min
            ):
                return (
                    f'{name} moves {frame_name} below '
                    f'y={self.config.frame_y_min:.4f} '
                    f'(y={y:.4f}, {self.config.frame_y_min - y:.4f}m below)'
                )
            if (
                self.config.frame_y_max is not None and
                y - margin > self.config.frame_y_max
            ):
                return (
                    f'{name} moves {frame_name} above '
                    f'y={self.config.frame_y_max:.4f} '
                    f'(y={y:.4f}, {y - self.config.frame_y_max:.4f}m above)'
                )
        return ''

    def _make_frame_floor_map(self, min_z):
        if min_z is None:
            return {}
        if isinstance(min_z, dict):
            return {name: float(z) for name, z in min_z.items()}
        return {name: float(min_z) for name in self.frame_names}

    def _set_workspace_corridor(self, start, goal, local_frames=()):
        self.corridor_min_z = {}
        self.local_min_x = {}
        self.local_elbow_min_x = {}
        if not hasattr(self.robot_model, 'get_frame_position'):
            return

        margin = float(self.config.frame_z_corridor_margin)
        start_poses = self._frame_poses(start)
        start_elbows = {
            name: self._frame_data.oMf[frame_id].translation.copy()
            for name, frame_id in self._elbow_ids.items()
            if name in local_frames
        } if self._frame_data is not None else {}
        goal_poses = self._frame_poses(goal)
        for frame_name in self.frame_names:
            start_pos = start_poses[frame_name][0]
            goal_pos = goal_poses[frame_name][0]
            self.corridor_min_z[frame_name] = (
                min(start_pos[2], goal_pos[2]) - margin
            )
            if frame_name in local_frames:
                self.local_min_x[frame_name] = (
                    min(start_pos[0], goal_pos[0]) - self.config.local_x_backtrack
                )
                if frame_name in start_elbows:
                    goal_elbow = self._frame_data.oMf[
                        self._elbow_ids[frame_name]
                    ].translation
                    self.local_elbow_min_x[frame_name] = (
                        min(start_elbows[frame_name][0], goal_elbow[0]) -
                        self.config.elbow_x_backtrack
                    )

    def _path_quality_failure(self, path, start, goal, local_frames):
        length = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        direct = float(np.linalg.norm(goal - start))
        ratio = length / direct if direct > 1e-8 else (
            1.0 if length < 1e-8 else float('inf')
        )
        metrics = {
            'joint_path_length': length,
            'joint_direct_distance': direct,
            'joint_path_ratio': ratio,
        }
        if local_frames:
            min_x = {name: float('inf') for name in local_frames}
            elbows = {name: float('inf') for name in local_frames if name in self._elbow_ids}
            points = {name: [] for name in local_frames}
            for q in path:
                poses = self._frame_poses(q)
                for name in local_frames:
                    position = poses[name][0]
                    min_x[name] = min(min_x[name], float(position[0]))
                    points[name].append(position)
                    if name in elbows:
                        elbow = self._frame_data.oMf[self._elbow_ids[name]].translation
                        elbows[name] = min(elbows[name], float(elbow[0]))
            metrics['min_frame_x'] = min_x
            metrics['min_elbow_x'] = elbows
            metrics['local_x_bounds'] = {
                name: self.local_min_x[name] for name in local_frames
            }
            metrics['elbow_local_x_bounds'] = self.local_elbow_min_x.copy()
            deviations = {}
            for name, positions in points.items():
                positions = np.asarray(positions)
                direction = positions[-1] - positions[0]
                projection = np.clip(
                    (positions - positions[0]) @ direction /
                    max(float(direction @ direction), 1e-12), 0.0, 1.0,
                )
                closest = positions[0] + projection[:, None] * direction
                deviations[name] = float(np.max(np.linalg.norm(positions - closest, axis=1)))
            metrics['hand_segment_deviation'] = deviations
        if local_frames and ratio > self.config.max_local_path_ratio + 1e-6:
            return metrics, (
                f'path detour ratio {ratio:.2f} exceeds local limit '
                f'{self.config.max_local_path_ratio:.2f}'
            )
        for name, deviation in metrics.get('hand_segment_deviation', {}).items():
            if deviation > self.config.max_hand_deviation + 1e-6:
                return metrics, (
                    f'{name} hand deviates {deviation:.3f}m from local segment '
                    f'(limit {self.config.max_hand_deviation:.3f}m)'
                )
        return metrics, ''

    def _path_validity_failure(self, path):
        for idx, q in enumerate(path):
            reason = self._validity_failure(q, f'path waypoint {idx}')
            if reason:
                return reason

        # Match OMPL's joint-space resolution even when output was capped
        resolution = (
            self.space.getMaximumExtent() * self.config.validity_resolution
        )
        for idx in range(len(path) - 1):
            steps = max(
                1, int(self.config.constraint_check_steps),
                int(np.ceil(np.linalg.norm(path[idx + 1] - path[idx]) / resolution)),
            )
            for sub_idx in range(1, steps):
                segment_alpha = sub_idx / steps
                q = (1.0 - segment_alpha) * path[idx]
                q += segment_alpha * path[idx + 1]
                reason = self._validity_failure(
                    q,
                    f'path segment {idx}',
                )
                if reason:
                    return reason
        return ''

    def _infer_active_mask(self, start, goal):
        if self.nq != _ARM_REDUCED_NQ:
            return None

        mask = np.zeros(self.nq, dtype=bool)
        if np.max(np.abs(start[:7] - goal[:7])) > self.config.inactive_goal_tolerance:
            mask[:7] = True
        if np.max(np.abs(start[7:] - goal[7:])) > self.config.inactive_goal_tolerance:
            mask[7:] = True
        return mask

    def _path_context(self, local_frames):
        '''Fingerprint scene and timing assumptions for one reusable path'''
        excluded = {'timeout', 'simplify_time', 'planner', 'range'}
        config = tuple(sorted(
            (key, repr(value)) for key, value in vars(self.config).items()
            if key not in excluded
        ))
        frozen = np.asarray(self.robot_model.state['q'])[~self.robot_model.reduced_mask]
        velocity = np.asarray(
            self.robot_model.model_body_reduced.velocityLimit, dtype=float,
        ).copy()
        return (config, frozen.copy(), self.active_mask.copy(),
                frozenset(local_frames), velocity)

    def _reversed_previous_path(self, start, goal, local_frames):
        '''Reuse a reversed trajectory only after current-context validation'''
        config, frozen, mask, frames, velocity = self._path_context(local_frames)
        for cached in reversed(self._recent_paths):
            if (config != cached['config'] or frames != cached['frames'] or
                    not np.array_equal(frozen, cached['frozen']) or
                    not np.array_equal(mask, cached['mask']) or
                    not np.array_equal(velocity, cached['velocity']) or
                    not np.allclose(start, cached['goal'], atol=1e-6, rtol=0.0) or
                    not np.allclose(goal, cached['start'], atol=1e-6, rtol=0.0)):
                continue
            path = cached['path'][::-1].copy()
            reason = self._path_validity_failure(path)
            if reason:
                continue
            quality, reason = self._path_quality_failure(path, start, goal, local_frames)
            if reason:
                continue
            metadata = cached['metadata'].copy()
            metadata.update(quality)
            metadata['path_kind'] = 'validated reverse reuse'
            metadata['cache_hit'] = True
            metadata['validity_checks'] = self.validity_checks
            return PlanResult(
                success=True, path=path, reason='validated reverse reuse',
                planner_name=self.config.planner, metadata=metadata,
            )
        return None

    def plan(self, start, goal, active_mask=None, direct_only=False,
             local_frames=(), timeout=None, allow_reuse=True):
        '''Plan directly, then try separately moving both arms if necessary'''
        started = perf_counter()
        result = self._plan_once(
            start, goal, active_mask=active_mask, direct_only=direct_only,
            local_frames=local_frames, timeout=timeout, allow_reuse=allow_reuse,
        )
        if (result.success or direct_only or self.nq != _ARM_REDUCED_NQ or
                result.reason.startswith(('start ', 'goal ', 'goal differs'))):
            return result
        return self._staged_two_arm_plan(
            start, goal, active_mask, local_frames, allow_reuse, result, started,
        )

    def _staged_two_arm_plan(self, start, goal, active_mask, local_frames,
                             allow_reuse, result, started):
        '''Validate two separately timed arm legs against the combined goal'''
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        frames = tuple(local_frames)
        if (set(frames) != {'left_grasp_frame', 'right_grasp_frame'} or
                not np.any(np.abs(start[:7] - goal[:7]) > self.config.inactive_goal_tolerance) or
                not np.any(np.abs(start[7:] - goal[7:]) > self.config.inactive_goal_tolerance)):
            return result
        if active_mask is not None:
            active_mask = np.asarray(active_mask, dtype=bool)
            if not np.any(active_mask[:7]) or not np.any(active_mask[7:]):
                return result
        failures = []
        for first_arm in (1, 0):
            second_arm = 1 - first_arm
            mid = start.copy()
            mid[first_arm * 7:first_arm * 7 + 7] = goal[
                first_arm * 7:first_arm * 7 + 7
            ]
            legs = []
            for leg_start, leg_goal, arm in (
                (start, mid, first_arm), (mid, goal, second_arm),
            ):
                remaining = self.config.candidate_planning_budget - (
                    perf_counter() - started
                )
                if remaining <= 0.0:
                    failures.append('staged planning budget exhausted')
                    break
                mask = np.zeros(self.nq, dtype=bool)
                mask[arm * 7:arm * 7 + 7] = (
                    True if active_mask is None else active_mask[arm * 7:arm * 7 + 7]
                )
                name = ('left_grasp_frame', 'right_grasp_frame')[arm]
                leg = self._plan_once(
                    leg_start, leg_goal, active_mask=mask,
                    local_frames=(name,),
                    timeout=min(self.config.timeout, remaining),
                    allow_reuse=allow_reuse,
                )
                if not leg.success:
                    failures.append(f'{name}: {leg.reason}')
                    break
                legs.append(leg)
            if len(legs) != 2:
                continue
            # each leg is independently timed within the per-plan waypoint cap
            path = np.vstack([legs[0].path, legs[1].path[1:]])
            if active_mask is not None and np.any(
                np.abs(path[:, ~active_mask] - start[~active_mask]) >
                self.config.inactive_goal_tolerance
            ):
                failures.append('staged path moves inactive joints')
                continue
            self._set_active_mask(active_mask, start)
            self._set_workspace_corridor(start, goal, frames)
            reason = self._path_validity_failure(path)
            if reason:
                failures.append(reason)
                continue
            quality, quality_reason = self._path_quality_failure(
                path, start, goal, frames,
            )
            if quality_reason:
                failures.append(quality_reason)
                continue
            dt = self.config.dt
            velocities = np.diff(path, axis=0) / dt
            accelerations = np.diff(np.vstack([
                np.zeros(self.nq), velocities, np.zeros(self.nq),
            ]), axis=0) / dt
            joint_limit = np.minimum(
                np.broadcast_to(self.config.joint_velocity_limit, (self.nq,)),
                self.robot_model.model_body_reduced.velocityLimit,
            )
            acceleration_limit = np.broadcast_to(
                self.config.joint_acceleration_limit, (self.nq,),
            )
            if (np.any(np.abs(velocities) > joint_limit + 1e-6) or
                    np.any(np.abs(accelerations) > acceleration_limit + 1e-6)):
                failures.append('staged path exceeds joint timing limits')
                continue
            lengths = {
                name: sum(leg.metadata['frame_path_lengths'][name] for leg in legs)
                for name in self.frame_names
            }
            translations = {
                name: sum(leg.metadata['frame_translation_lengths'][name] for leg in legs)
                for name in self.frame_names
            }
            rotations = {
                name: sum(leg.metadata['frame_rotation_lengths'][name] for leg in legs)
                for name in self.frame_names
            }
            duration = (len(path) - 1) * dt
            length = max(lengths.values(), default=0.0)
            frame_speeds = {
                name: max(leg.metadata['actual_frame_speeds'][name] for leg in legs)
                for name in self.frame_names
            }
            metadata = {
                **quality, 'path_kind': 'staged two-arm path',
                'time_profile': 'staged', 'waypoint_count': len(path),
                'actual_waypoints': len(path), 'required_points': len(path),
                'duration': duration, 'path_length': length,
                'effective_speed': length / duration,
                'frame_path_lengths': lengths,
                'frame_translation_lengths': translations,
                'frame_rotation_lengths': rotations,
                'length_estimate_samples': sum(
                    leg.metadata['length_estimate_samples'] for leg in legs
                ),
                'actual_max_joint_speed': float(np.max(np.abs(velocities))),
                'actual_max_joint_acceleration': float(np.max(np.abs(accelerations))),
                'actual_max_frame_speed': max(frame_speeds.values(), default=0.0),
                'actual_frame_speeds': frame_speeds,
                'validity_checks': self.validity_checks,
                'stage_leg_reasons': [leg.reason for leg in legs],
                'stage_timings': {
                    'original_attempt': result.planning_time,
                    'leg_1': legs[0].planning_time,
                    'leg_2': legs[1].planning_time,
                    'total': perf_counter() - started,
                },
            }
            return PlanResult(
                success=True, path=path, reason='staged two-arm return',
                planning_time=metadata['stage_timings']['total'],
                planner_name=self.config.planner, metadata=metadata,
            )
        result.reason += '; staged fallback: ' + '; '.join(failures)
        return result

    def _plan_once(self, start, goal, active_mask=None, direct_only=False,
                   local_frames=(), timeout=None, allow_reuse=True):
        '''Plan a collision-free path between reduced joint configurations'''
        started = perf_counter()
        stage_start = started
        timings = {}
        self._diagnostics = []

        def finish(result):
            result.planning_time = perf_counter() - started
            result.metadata['stage_timings'] = timings.copy()
            result.metadata['stage_timings']['total'] = result.planning_time
            if self._diagnostics:
                result.metadata['diagnostics'] = self._diagnostics.copy()
            if result.success and not direct_only and allow_reuse:
                config, frozen, mask, frames, velocity = self._path_context(local_frames)
                self._recent_paths.append({
                    'config': config, 'frozen': frozen, 'mask': mask,
                    'frames': frames, 'velocity': velocity,
                    'start': start.copy(), 'goal': goal.copy(),
                    'path': result.path.copy(), 'metadata': result.metadata.copy(),
                })
                self._recent_paths = self._recent_paths[
                    -self.config.recent_path_cache_size:
                ]
            return result

        start = self._as_reduced_array(start, 'start')
        goal = self._as_reduced_array(goal, 'goal')
        local_frames = tuple(local_frames)
        unsupported = set(local_frames) - set(self.frame_names)
        if unsupported:
            raise ValueError(f'Locality frames not configured: {sorted(unsupported)}')
        # the start reflects the measured pose; legalize tiny limit overshoots
        start = self._clamp_start_into_limits(start)
        if active_mask is None:
            active_mask = self._infer_active_mask(start, goal)

        # reject a requested inactive-joint motion instead of silently changing the goal
        self._set_active_mask(active_mask, start)
        inactive_error = np.abs(goal[~self.active_mask] - start[~self.active_mask])
        if np.any(inactive_error > self.config.inactive_goal_tolerance):
            timings['setup'] = perf_counter() - stage_start
            return finish(PlanResult(
                success=False, path=self._empty_path(),
                reason='goal differs from the start on inactive joints',
                planner_name=self.config.planner,
            ))
        goal = self._project_inactive(goal)
        self._set_workspace_corridor(start, goal, local_frames)

        # reject invalid endpoints before OMPL does any sampling
        self.validity_checks = 0
        for name, q in [('start', start), ('goal', goal)]:
            reason = self._validity_failure(q, name)
            if reason:
                timings['setup'] = perf_counter() - stage_start
                return finish(PlanResult(
                    success=False,
                    path=self._empty_path(),
                    reason=reason,
                    planner_name=self.config.planner,
                    metadata={'validity_checks': self.validity_checks},
                ))

        timings['setup'] = perf_counter() - stage_start
        stage_start = perf_counter()
        self.validity_checks = 0
        if not direct_only and allow_reuse:
            reused = self._reversed_previous_path(start, goal, local_frames)
            if reused is not None:
                timings['reverse_reuse'] = perf_counter() - stage_start
                return finish(reused)
        direct_failure = ''
        if self.config.try_direct or direct_only:
            path, metadata, direct_failure = self._timed_path(
                np.vstack([start, goal])
            )
            metadata['path_kind'] = 'direct path'
            timings['direct'] = perf_counter() - stage_start
            stage_start = perf_counter()
            if path is not None:
                direct_failure = self._path_validity_failure(path)
            if not direct_failure and path is not None:
                quality, direct_failure = self._path_quality_failure(
                    path, start, goal, local_frames,
                )
                metadata.update(quality)
            timings['direct_validation'] = perf_counter() - stage_start
            if not direct_failure:
                metadata['validity_checks'] = self.validity_checks
                return finish(PlanResult(
                    success=True,
                    path=path,
                    reason='direct path',
                    planner_name=self.config.planner,
                    metadata=metadata,
                ))

        if direct_only:
            return finish(PlanResult(
                success=False,
                path=self._empty_path(),
                reason=f'direct path failed: {direct_failure}',
                planner_name=self.config.planner,
                metadata={**metadata, 'direct_failure': direct_failure},
            ))

        stage_start = perf_counter()
        # reset OMPL state; collapse inactive joint bounds to pin them
        self.si.setStateValidityCheckingResolution(
            float(self.config.validity_resolution)
        )
        self._apply_active_bounds()
        self.ss.clear()
        self.ss.setStartAndGoalStates(
            self._make_state(start),
            self._make_state(goal),
        )

        solve_timeout = self.config.timeout if timeout is None else float(timeout)
        if not np.isfinite(solve_timeout) or solve_timeout <= 0.0:
            raise ValueError('Planner timeout must be finite and positive')
        solved = self.ss.solve(solve_timeout)
        timings['ompl'] = perf_counter() - stage_start
        if not bool(solved):
            return finish(PlanResult(
                success=False,
                path=self._empty_path(),
                reason=self._timeout_reason(direct_failure),
                planner_name=self.config.planner,
                metadata={'validity_checks': self.validity_checks},
            ))

        # bool(PlannerStatus) is True for APPROXIMATE_SOLUTION as well as
        # EXACT_SOLUTION. An approximate path ends at the closest state OMPL
        # reached, NOT at the goal, so executing it drives the arm to a
        # near-miss and leaves the steady-state controller to chase a gap it
        # cannot close. Treat it as a planning failure instead.
        if not self.ss.haveExactSolutionPath():
            return finish(PlanResult(
                success=False,
                path=self._empty_path(),
                reason=self._approximate_reason(direct_failure),
                planner_name=self.config.planner,
                metadata={'validity_checks': self.validity_checks},
            ))

        # simplify once, then validate the final candidate path
        stage_start = perf_counter()
        path = self.ss.getSolutionPath()
        self._shorten_path(path)
        sparse_path = self._path_to_array(path, cap=False)
        if (len(sparse_path) < 2 or
                not np.all(np.isfinite(sparse_path)) or
                not np.allclose(sparse_path[0], start, rtol=0, atol=1e-6) or
                not np.allclose(sparse_path[-1], goal, rtol=0, atol=1e-6)):
            return finish(PlanResult(
                success=False, path=self._empty_path(),
                reason='OMPL solution does not connect the requested endpoints',
                planner_name=self.config.planner,
                metadata={'validity_checks': self.validity_checks},
            ))
        path_array, metadata, timing_failure = self._select_timed_path(
            sparse_path, start, goal, local_frames,
        )
        metadata['path_kind'] = 'ompl path'
        timings['postprocess'] = perf_counter() - stage_start
        if timing_failure:
            metadata['validity_checks'] = self.validity_checks
            return finish(PlanResult(
                success=False, path=self._empty_path(), reason=timing_failure,
                planner_name=self.config.planner, metadata=metadata,
            ))
        stage_start = perf_counter()
        metadata['actual_waypoints'] = path_array.shape[0]
        reason = self._path_validity_failure(path_array)
        if not reason:
            quality, reason = self._path_quality_failure(
                path_array, start, goal, local_frames,
            )
            metadata.update(quality)
        timings['validation'] = perf_counter() - stage_start
        metadata['validity_checks'] = self.validity_checks
        if reason:
            return finish(PlanResult(
                success=False,
                path=path_array,
                reason=reason,
                planner_name=self.config.planner,
                metadata=metadata,
            ))

        return finish(PlanResult(
            success=True,
            path=path_array,
            reason='success',
            planner_name=self.config.planner,
            metadata=metadata,
        ))

    def _timeout_reason(self, direct_failure):
        reason = 'OMPL failed to find a solution before timeout'
        if direct_failure:
            reason = f'{reason}; direct path failed: {direct_failure}'
        return reason

    def _approximate_reason(self, direct_failure):
        reason = (
            'OMPL returned only an approximate solution '
            '(path does not reach the goal)'
        )
        if direct_failure:
            reason = f'{reason}; direct path failed: {direct_failure}'
        return reason

    def _path_to_array(self, path, cap=True):
        # convert OMPL state objects into a dense numpy waypoint array
        states = path.getStates()
        path_array = np.asarray(
            [self._state_to_array(state) for state in states],
            dtype=float,
        )
        return path_array

    def _smooth_path_timing(self, path):
        if len(path) < 3:
            return path

        t = np.linspace(0.0, 1.0, len(path))
        alpha = self._smoothstep(len(path))
        return np.asarray([
            np.interp(alpha, t, path[:, idx])
            for idx in range(path.shape[1])
        ]).T

    def _smoothstep(self, steps):
        # slow start and stop while preserving path endpoints
        t = np.linspace(0.0, 1.0, steps)
        return t * t * (3.0 - 2.0 * t)

    def _set_active_mask(self, active_mask, start):
        '''Configure which joints may move and pin the rest to start'''
        if active_mask is None:
            self.active_mask = np.ones(self.nq, dtype=bool)
        else:
            active_mask = np.asarray(active_mask, dtype=bool).reshape(-1)
            if active_mask.shape != (self.nq,):
                raise ValueError(f'active_mask must have shape ({self.nq},)')
            self.active_mask = active_mask
        self.pinned_q = np.array(start, dtype=float)

    def _direct_path(self, start, goal):
        path, metadata, failure = self._timed_path(np.vstack([start, goal]))
        metadata['path_kind'] = 'direct path'
        return path, metadata, failure

    def _shorten_path(self, path, smooth=True):
        '''Shortcut and smooth an OMPL path to prefer efficient motion'''
        if not self.config.simplify:
            return

        # OMPL's time-limited simplification already includes shortcutting
        self.ss.simplifySolution(float(self.config.simplify_time))
