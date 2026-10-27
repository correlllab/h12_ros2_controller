import time
from dataclasses import dataclass, field

import pink
import qpsolvers
import meshcat_shapes
import numpy as np
import pinocchio as pin
from pink.tasks import LinearHolonomicTask

from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS


@dataclass
class IKAttemptResult:
    '''Result for one IK seed attempt'''

    q: np.ndarray
    success: bool
    linear_error: float
    angular_error: float
    seed_name: str
    iterations: int
    elapsed_time: float
    reason: str = ''
    score: float = float('inf')
    frame_errors: dict = field(default_factory=dict)


@dataclass
class IKResult:
    '''Final IK result with all seed attempts'''

    q: np.ndarray
    success: bool
    linear_error: float
    angular_error: float
    seed_name: str
    attempts: list = field(default_factory=list)
    candidates: list = field(default_factory=list)


class IKSolver:
    wrist_movement_weight = 3.0
    wrist_posture_cost = 5e-3
    shoulder_seed_offset = 0.45
    elbow_seed_offset = 0.60
    frame_lm_damping = 3.0
    batch_frame_lm_damping = None
    batch_stagnation_iterations = 12
    batch_stagnation_relative_improvement = 0.001

    def __init__(self,
                 robot_model: RobotModel,
                 dt: float = 0.02,
                 d_min: float = 0.05,
                 *, wrist_movement_weight=None, wrist_posture_cost=None,
                 shoulder_seed_offset=None, elbow_seed_offset=None,
                 frame_lm_damping=3.0, batch_frame_lm_damping=None,
                 batch_stagnation_iterations=12,
                 batch_stagnation_relative_improvement=0.001):
        '''
        Initialize the IK solver with robot model and collision parameters
        Uses model_body (no free-flyer) for IK solving

        @param robot_model: the robot model instance
        @param dt: time step for IK solving
        @param d_min: minimum distance for collision avoidance
        '''
        self.robot_model = robot_model
        self.dt = dt
        self.d_min = d_min
        frame_lm_damping = float(frame_lm_damping)
        if not np.isfinite(frame_lm_damping) or frame_lm_damping < 0.0:
            raise ValueError('frame_lm_damping must be finite and nonnegative')
        if (isinstance(batch_stagnation_iterations, bool) or
                not isinstance(batch_stagnation_iterations, int) or
                batch_stagnation_iterations < 1):
            raise ValueError('batch_stagnation_iterations must be a positive integer')
        improvement = float(batch_stagnation_relative_improvement)
        if not np.isfinite(improvement) or not 0.0 < improvement < 1.0:
            raise ValueError(
                'batch_stagnation_relative_improvement must be finite and between 0 and 1'
            )
        self.frame_lm_damping = frame_lm_damping
        if batch_frame_lm_damping is not None:
            batch_frame_lm_damping = float(batch_frame_lm_damping)
            if not np.isfinite(batch_frame_lm_damping) or batch_frame_lm_damping < 0.0:
                raise ValueError('batch_frame_lm_damping must be finite and nonnegative')
        self.batch_frame_lm_damping = batch_frame_lm_damping
        self.batch_stagnation_iterations = batch_stagnation_iterations
        self.batch_stagnation_relative_improvement = improvement
        for name, value in (
            ('wrist_movement_weight', wrist_movement_weight),
            ('wrist_posture_cost', wrist_posture_cost),
            ('shoulder_seed_offset', shoulder_seed_offset),
            ('elbow_seed_offset', elbow_seed_offset),
        ):
            if value is not None:
                value = float(value)
                if not np.isfinite(value) or value <= 0.0:
                    raise ValueError(f'{name} must be finite and positive')
                setattr(self, name, value)

        # task management
        self.frame_tasks = {}  # maps task_name -> FrameTask
        # basic tasks
        self.posture_task = pink.tasks.PostureTask(cost=1e-3)
        self.config_task = pink.tasks.PostureTask(cost=30.0)

        # center of mass task
        self.com_task = pink.tasks.ComTask(cost=30.0)
        self.com_task_reduced = pink.tasks.ComTask(cost=30.0)

        assert(self.robot_model.init_collision), 'Collision model is not initialized.'

        # configuration using model_body (no free-flyer)
        self.configuration = pink.Configuration(
            robot_model.model_body,
            robot_model.data_body,
            robot_model.state['q']
        )
        # set target for CoM task
        self.com_task.set_target_from_configuration(self.configuration)

        # limits using model_body (no free-flyer)
        self.limits = [
            pink.limits.ConfigurationLimit(robot_model.model_body),
            pink.limits.VelocityLimit(robot_model.model_body),
            pink.limits.AccelerationLimit(robot_model.model_body,
                                          25.0 * np.ones(robot_model.model_body.nv))
        ]

        # reduced-model bindings (configuration, limits, collision barrier)
        self.rebuild_reduced()

        # solver selection
        self.solver = qpsolvers.available_solvers[0]
        for preferred in ['daqp', 'proxqp', 'quadprog', 'osqp']:
            if preferred in qpsolvers.available_solvers:
                self.solver = preferred
                break

    def rebuild_reduced(self):
        '''
        Rebind all objects that hold references to the reduced model.
        Call after RobotModel.rebuild_reduced_model() so the IK, limits and
        collision barrier all see the newly locked frozen-joint reference.
        '''
        robot_model = self.robot_model
        self.configuration_reduced = pink.Configuration(
            robot_model.model_body_reduced,
            robot_model.data_body_reduced,
            robot_model.state_reduced['q'],
            collision_model=robot_model.collision_model_body_reduced,
            collision_data=robot_model.collision_data_body_reduced
        )
        self.com_task_reduced.set_target_from_configuration(self.configuration_reduced)
        self.collision_barrier_reduced = pink.barriers.SelfCollisionBarrier(
            n_collision_pairs=len(robot_model.collision_model_body_reduced.collisionPairs),
            gain=20.0,
            safe_displacement_gain=1.0,
            d_min=self.d_min,
        )
        self.limits_reduced = [
            pink.limits.ConfigurationLimit(robot_model.model_body_reduced),
            pink.limits.VelocityLimit(robot_model.model_body_reduced),
            pink.limits.AccelerationLimit(robot_model.model_body_reduced,
                                          25.0 * np.ones(robot_model.model_body_reduced.nv))
        ]

    @property
    def q(self):
        '''Get current joint configuration (motor-only, 27)'''
        return np.copy(self.configuration.q)

    @property
    def q_reduced(self):
        '''Get current reduced joint configuration'''
        return np.copy(self.configuration_reduced.q)

    def compute_error(self, task: pink.Task, q: np.ndarray) -> np.ndarray:
        '''Compute the error for a specific task (motor-only q, 27)'''
        configuration = pink.Configuration(
            self.robot_model.model_body,
            self.robot_model.data_body,
            q
        )
        return task.compute_error(configuration)

    def compute_error_reduced(self, task: pink.Task, q_reduced: np.ndarray) -> np.ndarray:
        '''Compute the error for a specific task on reduced model'''
        configuration_reduced = pink.Configuration(
            self.robot_model.model_body_reduced,
            self.robot_model.data_body_reduced,
            q_reduced,
        )
        return task.compute_error(configuration_reduced)

    def add_frame_task(self, task_name: str, frame_name: str,
                       position_cost: float = 50.0,
                       orientation_cost: float = 30.0,
                       lm_damping: float = None):
        '''Add a frame task to the IK solver'''
        self.frame_tasks[task_name] = pink.tasks.FrameTask(
            frame_name,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=self.frame_lm_damping if lm_damping is None else lm_damping
        )
        # add frame to visualizer
        visualizer = self.robot_model.visualizer
        if visualizer is not None and visualizer.viz is not None:
            viewer = visualizer.viz.viewer
            meshcat_shapes.frame(viewer[task_name], opacity=1.0)
            if self._frame_is_valid(frame_name):
                meshcat_shapes.frame(viewer[frame_name], opacity=1.0)

    def remove_frame_task(self, task_name: str):
        '''Remove a frame task from the IK solver'''
        frame_name = self.frame_tasks[task_name].frame
        self.frame_tasks.pop(task_name, None)
        # remove frame from visualizer
        visualizer = self.robot_model.visualizer
        if visualizer is not None and visualizer.viz is not None:
            viewer = visualizer.viz.viewer
            viewer[task_name].delete()
            viewer[frame_name].delete()

    def clear_frame_tasks(self):
        '''Clear all frame tasks'''
        task_names = list(self.frame_tasks.keys())
        for task_name in task_names:
            self.remove_frame_task(task_name)

    def update_visualizer(self):
        visualizer = self.robot_model.visualizer
        if visualizer is not None and visualizer.viz is not None:
            viewer = visualizer.viz.viewer
            pelvis_to_world = self.robot_model.get_frame_transformation('pelvis')
            for task_name, task in self.frame_tasks.items():
                target_to_world = pelvis_to_world @ task.transform_target_to_world.np
                viewer[task_name].set_transform(target_to_world)
                frame_name = task.frame
                if not self._frame_is_valid(frame_name):
                    continue
                viewer[frame_name].set_transform(
                    self.robot_model.get_frame_transformation(frame_name)
                )

    def _frame_is_valid(self, frame_name):
        frame_id = self.robot_model.model.getFrameId(frame_name)
        return frame_id < len(self.robot_model.model.frames)

    def update_configurations(self, q=None):
        '''Update Pink configurations with q (motor-only, 27; default: measured state)'''
        if q is None:
            q = self.robot_model.state['q']
        q = np.asarray(q, dtype=float)
        self.configuration.update(q)
        self.configuration_reduced.update(q[self.robot_model.reduced_mask])

    def set_from_configuration(self):
        '''Set initial targets for all tasks from current configuration'''
        self.update_configurations()
        for task in self.frame_tasks.values():
            task.set_target_from_configuration(self.configuration)

    def set_from_configuration_reduced(self):
        '''Set initial targets for all tasks from reduced configuration'''
        self.update_configurations()
        for task in self.frame_tasks.values():
            task.set_target_from_configuration(self.configuration_reduced)

    def integrate(self, vel: np.ndarray):
        '''Integrate the current configuration with given joint velocity (motor-only, 27)'''
        vel_reduced = vel[self.robot_model.reduced_mask]
        self.configuration.integrate_inplace(vel, self.dt)
        self.configuration_reduced.integrate_inplace(vel_reduced, self.dt)

    def integrate_reduced(self, vel_reduced: np.ndarray):
        '''Integrate the current reduced configuration with given reduced joint velocity'''
        vel = self.robot_model.zero_q_body
        vel[self.robot_model.reduced_mask] = vel_reduced
        self.configuration.integrate_inplace(vel, self.dt)
        self.configuration_reduced.integrate_inplace(vel_reduced, self.dt)

    def _ik(self, tasks, dt):
        '''Helper function solving IK on body model for all tasks'''
        return pink.solve_ik(
            self.configuration,
            tasks,
            dt=dt,
            solver=self.solver,
            limits=self.limits,
            barriers=[],
            safety_break=False
        )

    def _ik_reduced(self, tasks, dt, constraints=None):
        '''Helper function solving IK on reduced model for all tasks'''
        return pink.solve_ik(
            self.configuration_reduced,
            tasks,
            dt=dt,
            solver=self.solver,
            limits=self.limits_reduced,
            barriers=[self.collision_barrier_reduced],
            constraints=constraints,
            safety_break=False
        )

    def _frame_task_errors(self, configuration):
        '''Compute worst physical frame position and orientation residuals'''
        linear_err = 0.0
        angular_err = 0.0
        for task in self.frame_tasks.values():
            linear, angular = self._frame_task_residual(task, configuration)
            linear_err = max(linear_err, linear)
            angular_err = max(angular_err, angular)
        return linear_err, angular_err

    def _frame_task_residual(self, task, configuration):
        '''Measure Cartesian error rather than coupled SE(3) log translation'''
        target = getattr(task, 'transform_target_to_world', None)
        if target is not None and hasattr(configuration, 'get_transform_frame_to_world'):
            current = configuration.get_transform_frame_to_world(task.frame)
            linear = np.linalg.norm(current.translation - target.translation)
            angular = np.linalg.norm(pin.log3(current.rotation.T @ target.rotation))
        else:
            error = task.compute_error(configuration)
            linear = np.linalg.norm(error[:3])
            angular = np.linalg.norm(error[3:])
        if not np.isfinite(linear) or not np.isfinite(angular):
            return float('inf'), float('inf')
        return float(linear), float(angular)

    def _frame_error_details(self, configuration):
        '''Return individual frame residuals for candidate diagnostics'''
        return {
            getattr(task, 'frame', name): self._frame_task_residual(task, configuration)
            for name, task in self.frame_tasks.items()
            if hasattr(task, 'compute_error')
        }

    def _task_error_score(self, attempt, linear_threshold, angular_threshold):
        linear_scale = max(float(linear_threshold), 1e-12)
        angular_scale = max(float(angular_threshold), 1e-12)
        return attempt.linear_error / linear_scale + attempt.angular_error / angular_scale

    def _reduced_q_to_full(self, q_reduced):
        q = np.copy(getattr(self, '_batch_full_reference', self.robot_model.state['q']))
        q[self.robot_model.reduced_mask] = q_reduced
        return q

    def _default_seed_items(self, start_q=None):
        # measured pose first for the full-IK first-success fallback
        return [
            ('current', self.robot_model.state['q'] if start_q is None else start_q),
            ('home', NAMED_CONFIGS['home']),
            ('t_pose', NAMED_CONFIGS['t_pose']),
            ('arms_front_45', NAMED_CONFIGS['arms_front_45']),
        ]

    def _extra_seed_items(self, initial_positions):
        if initial_positions is None:
            return []

        try:
            seeds = np.asarray(initial_positions, dtype=float)
            if seeds.ndim == 1:
                return [('initial_0', seeds)]
            if seeds.ndim == 2:
                return [(f'initial_{idx}', seed) for idx, seed in enumerate(seeds)]
        except (TypeError, ValueError):
            pass

        items = []
        for idx, item in enumerate(initial_positions):
            if (
                isinstance(item, tuple) and
                len(item) == 2 and
                isinstance(item[0], str)
            ):
                items.append((item[0], item[1]))
            else:
                items.append((f'initial_{idx}', item))
        return items

    def _normalize_full_seed(self, seed, seed_name):
        q = np.asarray(seed, dtype=float).reshape(-1)
        if q.shape == (self.robot_model.model_body.nq,):
            normalized = q
        elif q.shape == (self.robot_model.model_body_reduced.nq,):
            normalized = self._reduced_q_to_full(q)
        else:
            raise ValueError(
                f'{seed_name} seed must have shape '
                f'({self.robot_model.model_body.nq},) or '
                f'({self.robot_model.model_body_reduced.nq},)'
            )
        if not np.all(np.isfinite(normalized)):
            raise ValueError(f'{seed_name} seed must contain only finite values')
        return normalized

    def _normalize_reduced_seed(self, seed, seed_name):
        q = np.asarray(seed, dtype=float).reshape(-1)
        if q.shape == (self.robot_model.model_body_reduced.nq,):
            normalized = q
        elif q.shape == (self.robot_model.model_body.nq,):
            normalized = q[self.robot_model.reduced_mask]
        else:
            raise ValueError(
                f'{seed_name} seed must have shape '
                f'({self.robot_model.model_body_reduced.nq},) or '
                f'({self.robot_model.model_body.nq},)'
            )
        if not np.all(np.isfinite(normalized)):
            raise ValueError(f'{seed_name} seed must contain only finite values')
        return normalized

    def _seed_list(self, reduced, initial_positions, start_q=None, active_mask=None,
                   preferred_positions=None):
        seeds = []
        seed_items = self._default_seed_items(start_q)
        if preferred_positions is not None:
            seed_items.insert(1, ('preferred', preferred_positions))
        seed_items += self._extra_seed_items(initial_positions)
        for seed_name, seed in seed_items:
            try:
                if reduced:
                    q = self._normalize_reduced_seed(seed, seed_name)
                    if active_mask is not None:
                        q = np.where(active_mask, q, self._batch_reduced_reference)
                    model = self.robot_model.model_body_reduced
                else:
                    q = self._normalize_full_seed(seed, seed_name)
                    model = self.robot_model.model_body
                if (np.any(q < model.lowerPositionLimit) or
                        np.any(q > model.upperPositionLimit)):
                    raise ValueError(f'{seed_name} seed exceeds joint limits')
            except (TypeError, ValueError, OverflowError) as exc:
                seeds.append((seed_name, None, str(exc)))
                continue
            if not any(previous is not None and np.allclose(q, previous, atol=1e-6, rtol=0)
                       for _, previous, _ in seeds):
                seeds.append((seed_name, q.copy(), ''))
        return seeds

    def _arm_fallback_seeds(self, seeds, active_mask):
        '''Generate bounded shoulder and elbow alternatives from the current pose'''
        model = self.robot_model.model_body_reduced
        names = [''] * model.nq
        for name, idx in zip(model.names[1:], model.idx_qs[1:]):
            if 0 <= idx < model.nq:
                names[idx] = name.lower()
        sides = set()
        for task in self.frame_tasks.values():
            frame = getattr(task, 'frame', '').lower()
            if frame.startswith('left_'):
                sides.add('left')
            elif frame.startswith('right_'):
                sides.add('right')
        if len(sides) > 1:
            return []
        side = next(iter(sides), None)
        mask = np.ones(model.nq, dtype=bool) if active_mask is None else active_mask
        def matches(name, part):
            if part not in name or 'wrist' in name:
                return False
            if side == 'left':
                return name.startswith(('left_', 'l_'))
            if side == 'right':
                return name.startswith(('right_', 'r_'))
            return not name.startswith(('left_', 'right_', 'l_', 'r_'))

        joints = [next((idx for idx, name in enumerate(names)
                        if mask[idx] and matches(name, part)), None)
                  for part in ('shoulder', 'elbow')]
        if all(idx is None for idx in joints):
            return []
        base = self._batch_reduced_reference
        lower = np.asarray(model.lowerPositionLimit)
        upper = np.asarray(model.upperPositionLimit)
        shoulder = self.shoulder_seed_offset
        elbow = self.elbow_seed_offset
        variants = ((shoulder, elbow), (-shoulder, elbow),
                    (shoulder, -elbow), (-shoulder, -elbow))
        generated = []
        for variant, offsets in enumerate(variants):
            q = base.copy()
            for idx, offset in zip(joints, offsets):
                if idx is not None and np.isfinite(lower[idx]) and np.isfinite(upper[idx]) \
                        and lower[idx] <= upper[idx]:
                    q[idx] = np.clip(q[idx] + offset, lower[idx], upper[idx])
            if not np.all(np.isfinite(q)):
                continue
            if any(previous is not None and np.allclose(q, previous, atol=1e-6, rtol=0)
                   for _, previous, _ in seeds + generated):
                continue
            generated.append((f'arm_fallback_{variant}', q, ''))
        return generated

    def _naturalness_score(self, q, reference, reduced):
        model = self.robot_model.model_body_reduced if reduced else self.robot_model.model_body
        names = [''] * model.nq
        for name, idx in zip(model.names[1:], model.idx_qs[1:]):
            if 0 <= idx < model.nq:
                names[idx] = name
        weights = np.array([
            self.wrist_movement_weight if 'wrist' in name.lower() else 1.0
            for name in names
        ])
        movement = np.sum(weights * (q - reference) ** 2)
        lower = np.asarray(model.lowerPositionLimit)
        upper = np.asarray(model.upperPositionLimit)
        span = upper - lower
        valid = np.isfinite(span) & (span > 0)
        margin = np.minimum(q - lower, upper - q)
        penalty = np.sum(weights[valid] *
                         np.maximum(0, 0.1 - margin[valid] / span[valid]) ** 2)
        return float(movement + 10.0 * penalty)

    def _batch_posture_preference(self, q_seed):
        '''Retain comfortable wrists while other joints vary across seeds'''
        model = self.robot_model.model_body_reduced
        preferred = q_seed.copy()
        costs = np.full(model.nv, 1e-3)
        for name, idx in zip(model.names[1:], model.idx_qs[1:]):
            if 'wrist' not in name.lower():
                continue
            lower = model.lowerPositionLimit[idx]
            upper = model.upperPositionLimit[idx]
            if np.isfinite(lower + upper) and upper > lower:
                width = upper - lower
                preferred[idx] = np.clip(
                    self._batch_reduced_reference[idx],
                    lower + 0.1 * width, upper - 0.1 * width,
                )
            costs[idx] = self.wrist_posture_cost
        return preferred, costs

    def _result_from_attempts(self, attempts, linear_threshold, angular_threshold,
                              reduced=False, reference=None, first_success=False):
        candidates = []
        for attempt in attempts:
            if attempt.success and not any(
                    np.allclose(attempt.q, candidate.q, atol=1e-3, rtol=0)
                    for candidate in candidates):
                candidates.append(attempt)
        for attempt in candidates:
            q = attempt.q[self.robot_model.reduced_mask] if reduced else attempt.q
            attempt.score = self._naturalness_score(q, reference, reduced)
        candidates.sort(key=lambda attempt: attempt.score)
        if first_success and candidates:
            best = next(attempt for attempt in attempts if attempt.success)
        elif candidates:
            best = candidates[0]
        else:
            best = min(attempts, key=lambda attempt: self._task_error_score(
                attempt, linear_threshold, angular_threshold))
        return IKResult(
            q=np.copy(best.q),
            success=best.success,
            linear_error=best.linear_error,
            angular_error=best.angular_error,
            seed_name=best.seed_name,
            attempts=attempts,
            candidates=candidates,
        )

    def _solve_ik_attempt(self, seed_name, q_seed, alpha, deadline,
                           linear_threshold, angular_threshold):
        self.configuration.update(q_seed)
        self.posture_task.set_target_from_configuration(self.configuration)
        start_time = time.monotonic()
        iterations = 0
        linear_err = angular_err = float('inf')
        reason = ''
        last_q = np.copy(q_seed)
        try:
            linear_err, angular_err = self._frame_task_errors(self.configuration)
            while (linear_err >= linear_threshold or angular_err >= angular_threshold):
                if not np.all(np.isfinite([linear_err, angular_err])):
                    reason = 'non-finite task error'
                    break
                if time.monotonic() >= deadline:
                    reason = 'timeout'
                    break
                vel = self._ik(list(self.frame_tasks.values()) + [self.posture_task], alpha)
                if not np.all(np.isfinite(vel)):
                    reason = 'non-finite velocity'
                    break
                self.configuration.integrate_inplace(vel, alpha)
                if not np.all(np.isfinite(self.configuration.q)):
                    reason = 'non-finite configuration'
                    break
                last_q = np.copy(self.configuration.q)
                iterations += 1
                linear_err, angular_err = self._frame_task_errors(self.configuration)
        except Exception as exc:
            reason = f'{type(exc).__name__}: {exc}'

        return IKAttemptResult(
            q=last_q,
            success=(
                not reason and
                linear_err < linear_threshold and
                angular_err < angular_threshold
            ),
            linear_error=linear_err,
            angular_error=angular_err,
            seed_name=seed_name,
            iterations=iterations,
            elapsed_time=time.monotonic() - start_time,
            reason=reason,
        )

    def _solve_ik_reduced_attempt(self, seed_name, q_seed, alpha, deadline,
                                    linear_threshold, angular_threshold, active_mask=None):
        self.configuration_reduced.update(q_seed)
        inactive_constraint = None
        if active_mask is not None and not np.all(active_mask):
            model = self.configuration_reduced.model
            if model.nq != model.nv:
                raise ValueError('Reduced IK active mask requires nq == nv')
            selector = np.eye(model.nv)[~active_mask]
            inactive_constraint = LinearHolonomicTask(
                A=selector, b=np.zeros(len(selector)), q_0=q_seed.copy(),
            )
        preferred, costs = self._batch_posture_preference(q_seed)
        original_cost = getattr(self.posture_task, 'cost', None)
        if original_cost is not None:
            self.posture_task.cost = costs
        if hasattr(self.posture_task, 'set_target'):
            self.posture_task.set_target(preferred)
        else:
            self.posture_task.set_target_from_configuration(self.configuration_reduced)
        start_time = time.monotonic()
        iterations = 0
        linear_err = angular_err = float('inf')
        reason = ''
        last_q = np.copy(q_seed)
        best_score = checkpoint_score = float('inf')
        best_errors = (linear_err, angular_err)
        checkpoint_iteration = 0
        original_damping = []
        try:
            if self.batch_frame_lm_damping is not None:
                for task in self.frame_tasks.values():
                    original_damping.append((task, task.lm_damping))
                    task.lm_damping = self.batch_frame_lm_damping
            linear_err, angular_err = self._frame_task_errors(self.configuration_reduced)
            while True:
                if not np.all(np.isfinite([linear_err, angular_err])):
                    reason = 'non-finite task error'
                    break
                score = linear_err / linear_threshold + angular_err / angular_threshold
                converged = linear_err < linear_threshold and angular_err < angular_threshold
                if score < best_score or converged:
                    best_score = score
                    last_q = self.configuration_reduced.q.copy()
                    best_errors = (linear_err, angular_err)
                if converged:
                    break
                # accumulate small gains against a meaningful progress checkpoint
                if (not np.isfinite(checkpoint_score) or score <= checkpoint_score *
                        (1.0 - self.batch_stagnation_relative_improvement)):
                    checkpoint_score = score
                    checkpoint_iteration = iterations
                elif iterations - checkpoint_iteration >= self.batch_stagnation_iterations:
                    reason = 'stagnation'
                    break
                if time.monotonic() >= deadline:
                    reason = 'timeout'
                    break
                constraints = None
                if inactive_constraint is not None:
                    # lock inactive displacement in the QP, not after solving it
                    inactive_constraint.q_0 = self.configuration_reduced.q.copy()
                    constraints = [inactive_constraint]
                tasks = list(self.frame_tasks.values()) + [self.posture_task]
                vel = (self._ik_reduced(tasks, alpha, constraints=constraints)
                       if constraints else self._ik_reduced(tasks, alpha))
                if not np.all(np.isfinite(vel)):
                    reason = 'non-finite velocity'
                    break
                self.configuration_reduced.integrate_inplace(vel, alpha)
                if not np.all(np.isfinite(self.configuration_reduced.q)):
                    reason = 'non-finite configuration'
                    break
                iterations += 1
                linear_err, angular_err = self._frame_task_errors(self.configuration_reduced)
        except Exception as exc:
            reason = f'{type(exc).__name__}: {exc}'
        finally:
            for task, damping in original_damping:
                task.lm_damping = damping
            if original_cost is not None:
                self.posture_task.cost = original_cost

        self.configuration_reduced.update(last_q)
        linear_err, angular_err = best_errors
        return IKAttemptResult(
            q=self._reduced_q_to_full(last_q),
            success=(
                not reason and
                linear_err < linear_threshold and
                angular_err < angular_threshold
            ),
            linear_error=linear_err,
            angular_error=angular_err,
            seed_name=seed_name,
            iterations=iterations,
            elapsed_time=time.monotonic() - start_time,
            reason=reason,
        )

    def ik_step(self, com=False):
        '''Solve one step of IK for all tasks and return joint velocity (motor-only, 27)'''
        self.posture_task.set_target_from_configuration(self.configuration)
        tasks = list(self.frame_tasks.values()) + [self.posture_task]
        # add com task if requested
        if com:
            tasks.append(self.com_task)
        return self._ik(tasks, self.dt)

    def ik_step_reduced(self, com=False, constraints=None):
        '''Solve one step of IK on reduced model for all tasks and return motor-only velocity'''
        self.posture_task.set_target_from_configuration(self.configuration_reduced)
        tasks = list(self.frame_tasks.values()) + [self.posture_task]
        # add com task if requested
        if com:
            tasks.append(self.com_task_reduced)
        # convert reduced vel to full motor vel
        vel = self._ik_reduced(
            tasks,
            self.dt,
            constraints=constraints,
        )
        vel_full = self.robot_model.zero_q_body
        vel_full[self.robot_model.reduced_mask] = vel
        return vel_full

    def solve_ik(self, alpha=0.1, timeout=1.0,
                  linear_threshold=1e-3, angular_threshold=1e-2,
                  initial_positions=None, total_timeout=None, max_seeds=8,
                  reference_positions=None, start_q=None):
        '''Solve full IK, retaining first-success selection; timeout is per seed'''
        return self._solve_batch(False, alpha, timeout, linear_threshold,
                                 angular_threshold, initial_positions, total_timeout,
                                 max_seeds, reference_positions, start_q, None)

    def solve_ik_reduced(self, alpha=0.1, timeout=1.0,
                          linear_threshold=1e-3, angular_threshold=1e-2,
                          initial_positions=None, total_timeout=None, max_seeds=8,
                          reference_positions=None, start_q=None, active_mask=None,
                          preferred_positions=None):
        '''Solve reduced IK, trying one preferred full or reduced q after current'''
        return self._solve_batch(True, alpha, timeout, linear_threshold,
                                  angular_threshold, initial_positions, total_timeout,
                                  max_seeds, reference_positions, start_q, active_mask,
                                  preferred_positions)

    def _solve_batch(self, reduced, alpha, timeout, linear_threshold,
                     angular_threshold, initial_positions, total_timeout,
                     max_seeds, reference_positions, start_q, active_mask,
                     preferred_positions=None):
        if not np.isfinite(alpha) or alpha <= 0:
            raise ValueError('alpha must be positive and finite')
        if not np.isfinite(timeout) or timeout < 0:
            raise ValueError('timeout must be nonnegative and finite')
        if total_timeout is None:
            total_timeout = 4 * timeout
        if not np.isfinite(total_timeout) or total_timeout < 0:
            raise ValueError('total_timeout must be nonnegative and finite')
        if not isinstance(max_seeds, int) or max_seeds < 1:
            raise ValueError('max_seeds must be a positive integer')
        if not all(np.isfinite(v) and v > 0 for v in
                   (linear_threshold, angular_threshold)):
            raise ValueError('thresholds must be positive and finite')
        normalize = self._normalize_reduced_seed if reduced else self._normalize_full_seed
        start = self.robot_model.state['q'] if start_q is None else start_q
        reference = normalize(start if reference_positions is None else reference_positions,
                              'reference').copy()
        if reduced:
            self._batch_reduced_reference = self._normalize_reduced_seed(start, 'start').copy()
            if active_mask is not None:
                active_mask = np.asarray(active_mask, dtype=bool)
                if active_mask.shape != (self.robot_model.model_body_reduced.nq,):
                    raise ValueError('active_mask must have reduced joint shape')
            full_start = np.asarray(start, dtype=float).reshape(-1)
            self._batch_full_reference = (full_start.copy() if full_start.shape ==
                                           (self.robot_model.model_body.nq,) else
                                           np.copy(self.robot_model.state['q']))
            if preferred_positions is not None:
                try:
                    preferred = np.asarray(preferred_positions, dtype=float)
                except (TypeError, ValueError, OverflowError) as exc:
                    raise ValueError('preferred seed must be a numeric joint vector') from exc
                if preferred.ndim != 1:
                    raise ValueError('preferred seed must be a single joint vector')
                if not np.all(np.isfinite(preferred)):
                    raise ValueError('preferred seed must contain only finite values')
                if preferred.shape == (self.robot_model.model_body.nq,):
                    frozen = ~self.robot_model.reduced_mask
                    if not np.allclose(preferred[frozen],
                                       self._batch_full_reference[frozen], atol=1e-6, rtol=0):
                        raise ValueError('preferred seed frozen joints must match start')
                preferred_positions = self._normalize_reduced_seed(
                    preferred, 'preferred').copy()
        attempts = []
        fallback = (np.copy(self._batch_full_reference) if reduced else
                    self._normalize_full_seed(start, 'start').copy())
        if not self.frame_tasks:
            return IKResult(fallback, False, float('inf'), float('inf'), 'current',
                            [IKAttemptResult(fallback.copy(), False, float('inf'),
                                             float('inf'), 'current', 0, 0., 'no frame tasks')])
        deadline = time.monotonic() + total_timeout
        seeds = self._seed_list(reduced, initial_positions, start, active_mask,
                                preferred_positions)
        distinct_successes = []
        primary_count = min(len(seeds), max_seeds)
        pending = seeds[:max_seeds]
        index = 0
        while index < len(pending) or (index == primary_count and reduced and
                                       len(distinct_successes) < 2 and
                                       len(pending) < max_seeds):
            if index == primary_count:
                pending.extend(self._arm_fallback_seeds(seeds, active_mask)[
                    :max_seeds - len(pending)])
                primary_count = -1
                if index == len(pending):
                    break
            seed_name, q_seed, error = pending[index]
            index += 1
            if error:
                attempt = IKAttemptResult(fallback.copy(), False, float('inf'),
                                          float('inf'), seed_name, 0, 0.0, error)
            elif time.monotonic() >= deadline:
                break
            else:
                attempt_deadline = min(deadline, time.monotonic() + timeout)
                solver = self._solve_ik_reduced_attempt if reduced else self._solve_ik_attempt
                try:
                    args = (seed_name, q_seed, alpha, attempt_deadline,
                            linear_threshold, angular_threshold)
                    attempt = (solver(*args, active_mask=active_mask) if active_mask is not None
                               else solver(*args))
                except Exception as exc:
                    attempt = IKAttemptResult(fallback.copy(), False, float('inf'),
                                              float('inf'), seed_name, 0, 0.0,
                                              f'{type(exc).__name__}: {exc}')
                if attempt.success and reduced:
                    try:
                        q_reduced = attempt.q[self.robot_model.reduced_mask]
                        model = self.robot_model.model_body_reduced
                        if (np.any(q_reduced < model.lowerPositionLimit) or
                                np.any(q_reduced > model.upperPositionLimit)):
                            attempt.success = False
                            attempt.reason = 'joint limits'
                        elif (hasattr(self.robot_model, 'check_collision_free_reduced') and
                               not self.robot_model.check_collision_free_reduced(
                                   q_reduced, min_distance=self.d_min,
                               )):
                            attempt.success = False
                            attempt.reason = 'self-collision'
                    except Exception as exc:
                        attempt.success = False
                        attempt.reason = f'feasibility check: {type(exc).__name__}: {exc}'
            attempts.append(attempt)
            if attempt.success and all(
                hasattr(task, 'compute_error') for task in self.frame_tasks.values()
            ):
                configuration = (self.configuration_reduced if reduced else
                                 self.configuration)
                configuration.update(
                    attempt.q[self.robot_model.reduced_mask] if reduced else attempt.q
                )
                attempt.frame_errors = self._frame_error_details(configuration)
            if attempt.success and reduced and not any(
                    np.allclose(attempt.q, q, atol=1e-3, rtol=0)
                    for q in distinct_successes):
                distinct_successes.append(attempt.q.copy())
                if len(distinct_successes) >= (3 if index <= primary_count else 2):
                    if index <= primary_count:
                        attempts.extend(IKAttemptResult(
                            fallback.copy(), False, float('inf'), float('inf'),
                            name, 0, 0.0, seed_error,
                        ) for name, _, seed_error in pending[index:primary_count]
                            if seed_error)
                    break
            if attempt.success and not reduced:
                break
        if not attempts:
            attempts.append(IKAttemptResult(fallback.copy(), False, float('inf'),
                                            float('inf'), 'current', 0, 0.0, 'total timeout'))
        result = self._result_from_attempts(
            attempts,
            linear_threshold,
            angular_threshold,
            reduced=reduced,
            reference=reference,
            first_success=not reduced,
        )
        if reduced:
            self.configuration_reduced.update(result.q[self.robot_model.reduced_mask])
        else:
            self.configuration.update(result.q)
        return result

    def goto_configuration(self, q: np.ndarray):
        '''Solve one step of IK to reach a target joint configuration (motor-only, 27)'''
        self.config_task.set_target(q)
        return self._ik([self.config_task], self.dt)

    def goto_reduced_configuration(self, q_reduced: np.ndarray):
        '''Solve one step of IK to reach a target reduced joint configuration'''
        self.config_task.set_target(q_reduced)
        vel = self._ik_reduced([self.config_task], self.dt)
        # convert reduced vel to full motor vel
        vel_full = self.robot_model.zero_q_body
        vel_full[self.robot_model.reduced_mask] = vel
        return vel_full
