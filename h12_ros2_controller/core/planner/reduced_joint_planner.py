import time
import threading
from dataclasses import dataclass, field

import numpy as np
from ompl import base as ob
from ompl import geometric as og


@dataclass
class PlannerConfig:
    '''Configuration for reduced joint-space planning'''

    planner: str = 'RRTConnect'
    timeout: float = 1.0
    range: float = 0.0
    try_direct: bool = True
    simplify: bool = True
    simplify_time: float = 1.0
    shortcut: bool = True
    interpolation_steps: int = 200
    validity_resolution: float = 0.0025
    constraint_check_steps: int = 10
    frame_names: tuple = ('left_grasp_frame', 'right_grasp_frame')
    min_frame_z: float = None
    frame_z_tolerance: float = 1e-3
    # ceiling headroom above endpoint max z; None disables the ceiling
    frame_z_headroom: float = 0.1


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

        self.nq = int(self.robot_model.model_body_reduced.nq)
        self.validity_checks = 0
        self.last_invalid_reason = ''
        self.min_frame_z = self.config.min_frame_z
        self.frame_names = tuple(self.config.frame_names)
        self.frame_min_z = self._make_frame_floor_map(self.min_frame_z)
        self.frame_max_z = {}
        # active joint mask and pinned values for inactive joints
        self.active_mask = np.ones(self.nq, dtype=bool)
        self.pinned_q = np.zeros(self.nq)

        # build OMPL problem objects once; plan() only resets start/goal
        self.space = self._make_space()
        self.ss = og.SimpleSetup(self.space)
        self.si = self.ss.getSpaceInformation()
        self.ss.setStateValidityChecker(self._make_validity_checker())
        self.si.setStateValidityCheckingResolution(
            float(self.config.validity_resolution)
        )
        self.ss.setPlanner(self._make_planner())

    def _check_robot_model_ready(self):
        if not getattr(self.robot_model, 'init_reduced', False):
            raise ValueError('RobotModel reduced model is not initialized')
        if not getattr(self.robot_model, 'init_collision', False):
            raise ValueError('RobotModel collision model is not initialized')

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
        # yield GIL checkpoint so publisher/safety threads are not starved
        time.sleep(0)
        q = self._state_to_array(state)
        try:
            reason = self._validity_failure(q, 'state')
        except Exception as exc:
            self.last_invalid_reason = f'validity exception: {exc}'
            return False
        self.last_invalid_reason = reason
        return reason == ''

    def _validity_failure(self, q, name):
        if not self.robot_model.check_within_limits_reduced(q):
            return f'{name} is outside reduced joint limits'
        if not self.robot_model.check_collision_free_reduced(q):
            return f'{name} is in self-collision'
        reason = self._workspace_constraint_failure(q, name)
        if reason:
            return reason
        return ''

    def _motor_q(self, q_reduced):
        q = np.copy(self.robot_model.state['q'])
        q[self.robot_model.reduced_mask] = q_reduced
        return q

    def _workspace_constraint_failure(self, q_reduced, name):
        if not self.frame_min_z and not self.frame_max_z:
            return ''

        # keep selected frames within their configured z slab
        tol = self.config.frame_z_tolerance
        q = self._motor_q(q_reduced)
        for frame_name in self.frame_names:
            z = self.robot_model.get_frame_position(frame_name, q)[2]
            min_z = self.frame_min_z.get(frame_name)
            if min_z is not None and z + tol < min_z:
                return f'{name} moves {frame_name} below z={min_z:.4f}'
            max_z = self.frame_max_z.get(frame_name)
            if max_z is not None and z - tol > max_z:
                return f'{name} moves {frame_name} above z={max_z:.4f}'
        return ''

    def set_frame_floor(self, min_z, frame_names=None):
        '''Constrain selected frames above a horizontal plane'''
        self.min_frame_z = float(min_z)
        if frame_names is not None:
            self.frame_names = tuple(frame_names)
        self.frame_min_z = self._make_frame_floor_map(self.min_frame_z)

    def set_grasp_floor_from_endpoints(self, start, goal, margin=0.0,
                                       headroom=None):
        '''Constrain grasp frames to a z slab derived from the endpoints'''
        # collect per-frame endpoint z extremes
        z_min = {}
        z_max = {}
        for q_reduced in [start, goal]:
            q = self._motor_q(q_reduced)
            for frame_name in self.frame_names:
                z = self.robot_model.get_frame_position(frame_name, q)[2]
                z_min[frame_name] = min(z_min.get(frame_name, z), z)
                z_max[frame_name] = max(z_max.get(frame_name, z), z)

        # floor keeps frames above the lower endpoint minus a margin
        self.frame_min_z = {
            frame_name: z - float(margin)
            for frame_name, z in z_min.items()
        }
        # ceiling limits how high above the endpoints the path may reach
        headroom = self.config.frame_z_headroom if headroom is None else headroom
        if headroom is None:
            self.frame_max_z = {}
        else:
            self.frame_max_z = {
                frame_name: z + float(headroom)
                for frame_name, z in z_max.items()
            }
        self.min_frame_z = None

    def clear_workspace_constraints(self):
        '''Clear workspace constraints for future plans'''
        self.min_frame_z = self.config.min_frame_z
        self.frame_names = tuple(self.config.frame_names)
        self.frame_min_z = self._make_frame_floor_map(self.min_frame_z)
        self.frame_max_z = {}

    def _make_frame_floor_map(self, min_z):
        if min_z is None:
            return {}
        if isinstance(min_z, dict):
            return {name: float(z) for name, z in min_z.items()}
        return {name: float(min_z) for name in self.frame_names}

    def _path_validity_failure(self, path):
        for idx, q in enumerate(path):
            reason = self._validity_failure(q, f'path waypoint {idx}')
            if reason:
                return reason

        # check extra samples between returned waypoints for z constraints
        steps = max(1, int(self.config.constraint_check_steps))
        for idx in range(len(path) - 1):
            for sub_idx in range(1, steps):
                alpha = sub_idx / steps
                q = (1.0 - alpha) * path[idx] + alpha * path[idx + 1]
                reason = self._validity_failure(q, f'path segment {idx}')
                if reason:
                    return reason
        return ''

    def plan(self, start, goal, active_mask=None):
        '''Plan a collision-free path between reduced joint configurations'''
        start = self._as_reduced_array(start, 'start')
        goal = self._as_reduced_array(goal, 'goal')

        # restrict planning to active joints; pin the rest to the start value
        self._set_active_mask(active_mask, start)
        goal = self._project_inactive(goal)

        # reject invalid endpoints before OMPL does any sampling
        for name, q in [('start', start), ('goal', goal)]:
            reason = self._validity_failure(q, name)
            if reason:
                return PlanResult(
                    success=False,
                    path=self._empty_path(),
                    reason=reason,
                    planner_name=self.config.planner,
                )

        self.validity_checks = 0
        direct_failure = ''
        if self.config.try_direct:
            path = self._direct_path(start, goal)
            direct_failure = self._path_validity_failure(path)
            if not direct_failure:
                return PlanResult(
                    success=True,
                    path=path,
                    reason='direct path',
                    planner_name=self.config.planner,
                    metadata={'validity_checks': self.validity_checks},
                )

        # reset OMPL state; collapse inactive joint bounds to pin them
        self.si.setStateValidityCheckingResolution(
            float(self.config.validity_resolution)
        )
        self._apply_active_bounds()
        self.ss.clear()
        reason = self._ompl_endpoint_failure(start, goal)
        if reason:
            return PlanResult(
                success=False,
                path=self._empty_path(),
                reason=reason,
                planner_name=self.config.planner,
                metadata={'validity_checks': self.validity_checks},
            )
        self.ss.setStartAndGoalStates(
            self._make_state(start),
            self._make_state(goal),
        )

        # run OMPL solve in a worker thread so the GIL is released periodically,
        # allowing the 500Hz publisher thread to keep sending low_cmd packets
        start_time = time.perf_counter()
        solved = self._solve_threaded(float(self.config.timeout))
        planning_time = time.perf_counter() - start_time
        if not bool(solved):
            return PlanResult(
                success=False,
                path=self._empty_path(),
                reason=self._timeout_reason(direct_failure),
                planning_time=planning_time,
                planner_name=self.config.planner,
                metadata={'validity_checks': self.validity_checks},
            )

        # shorten and smooth first; fall back to less smoothing if it breaks
        path_array = None
        for smooth in (True, False):
            path = self.ss.getSolutionPath()
            self._shorten_path(path, smooth=smooth)
            if self.config.interpolation_steps > 0:
                steps = max(2, int(self.config.interpolation_steps))
                path.interpolate(steps)
            candidate = self._path_to_array(path)
            reason = self._path_validity_failure(candidate)
            if not reason:
                path_array = candidate
                break

        # last resort: use the raw solution path without extra smoothing
        if path_array is None:
            path = self.ss.getSolutionPath()
            if self.config.interpolation_steps > 0:
                path.interpolate(max(2, int(self.config.interpolation_steps)))
            path_array = self._path_to_array(path)
            reason = self._path_validity_failure(path_array)
            if reason:
                return PlanResult(
                    success=False,
                    path=path_array,
                    reason=reason,
                    planning_time=planning_time,
                    planner_name=self.config.planner,
                    metadata={'validity_checks': self.validity_checks},
                )

        return PlanResult(
            success=True,
            path=path_array,
            reason='success',
            planning_time=planning_time,
            planner_name=self.config.planner,
            metadata={'validity_checks': self.validity_checks},
        )

    def _path_to_array(self, path):
        # convert OMPL state objects into a dense numpy waypoint array
        states = path.getStates()
        path_array = np.asarray(
            [self._state_to_array(state) for state in states],
            dtype=float,
        )
        max_steps = int(self.config.interpolation_steps)
        if max_steps > 1 and path_array.shape[0] > max_steps:
            indices = np.linspace(0, path_array.shape[0] - 1, max_steps)
            indices = np.unique(indices.astype(int))
            path_array = path_array[indices]
        return path_array

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
        steps = max(2, int(self.config.interpolation_steps))
        alpha = np.linspace(0.0, 1.0, steps).reshape(-1, 1)
        path = (1.0 - alpha) * start + alpha * goal
        # pin inactive joints exactly to the start along the whole path
        path[:, ~self.active_mask] = self.pinned_q[~self.active_mask]
        return path

    def _shorten_path(self, path, smooth=True):
        '''Shortcut and smooth an OMPL path to prefer efficient motion'''
        if not self.config.simplify:
            return

        # run OMPL simplification budget first
        self.ss.simplifySolution(float(self.config.simplify_time))

        # then explicitly shortcut/straighten toward more direct motion
        if self.config.shortcut:
            simplifier = og.PathSimplifier(self.si)
            simplifier.ropeShortcutPath(path)
            simplifier.reduceVertices(path)
            # b-spline smoothing can overshoot into collision; keep optional
            if smooth:
                simplifier.smoothBSpline(path)

    def _solve_threaded(self, timeout):
        '''Run OMPL solve in a worker thread to yield GIL to publisher threads'''
        done = threading.Event()
        result = [None]

        def worker():
            result[0] = self.ss.solve(timeout)
            done.set()

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        # waiting on Event releases the GIL so publisher/safety threads run
        done.wait()
        return result[0]

    def _ompl_endpoint_failure(self, start, goal):
        for name, q in [('start', start), ('goal', goal)]:
            if not self._is_state_valid(self._make_state(q)):
                reason = self.last_invalid_reason or 'unknown invalid state'
                return f'{name} rejected by OMPL validity checker: {reason}'
        return ''

    def _timeout_reason(self, direct_failure):
        reason = 'OMPL failed to find a solution before timeout'
        if direct_failure:
            reason = f'{reason}; direct path failed: {direct_failure}'
        return reason
