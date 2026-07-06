import time
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
    interpolation_steps: int = 200
    validity_resolution: float = 0.0025
    constraint_check_steps: int = 10
    frame_names: tuple = ('left_grasp_frame', 'right_grasp_frame')
    min_frame_z: float = None
    frame_z_tolerance: float = 1e-3


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
        lower = np.asarray(
            self.robot_model.model_body_reduced.lowerPositionLimit,
            dtype=float,
        )
        upper = np.asarray(
            self.robot_model.model_body_reduced.upperPositionLimit,
            dtype=float,
        )
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

        bounds = ob.RealVectorBounds(self.nq)
        for idx in range(self.nq):
            bounds.setLow(idx, float(lower[idx]))
            bounds.setHigh(idx, float(upper[idx]))

        space = ob.RealVectorStateSpace(self.nq)
        space.setBounds(bounds)
        return space

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
        return np.asarray([state[idx] for idx in range(self.nq)], dtype=float)

    def _is_state_valid(self, state):
        self.validity_checks += 1
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
        if not self.frame_min_z:
            return ''

        # keep selected frames above their configured horizontal planes
        q = self._motor_q(q_reduced)
        for frame_name in self.frame_names:
            z = self.robot_model.get_frame_position(frame_name, q)[2]
            min_z = self.frame_min_z.get(frame_name)
            if min_z is not None and z + self.config.frame_z_tolerance < min_z:
                return f'{name} moves {frame_name} below z={min_z:.4f}'
        return ''

    def set_frame_floor(self, min_z, frame_names=None):
        '''Constrain selected frames above a horizontal plane'''
        self.min_frame_z = float(min_z)
        if frame_names is not None:
            self.frame_names = tuple(frame_names)
        self.frame_min_z = self._make_frame_floor_map(self.min_frame_z)

    def set_grasp_floor_from_endpoints(self, start, goal, margin=0.0):
        '''Constrain grasp frames above their endpoint minimum z'''
        frame_min_z = {}
        for q_reduced in [start, goal]:
            q = self._motor_q(q_reduced)
            for frame_name in self.frame_names:
                z = self.robot_model.get_frame_position(frame_name, q)[2]
                if frame_name not in frame_min_z:
                    frame_min_z[frame_name] = z
                else:
                    frame_min_z[frame_name] = min(frame_min_z[frame_name], z)
        self.frame_min_z = {
            frame_name: z - float(margin)
            for frame_name, z in frame_min_z.items()
        }
        self.min_frame_z = None

    def clear_workspace_constraints(self):
        '''Clear workspace constraints for future plans'''
        self.min_frame_z = self.config.min_frame_z
        self.frame_names = tuple(self.config.frame_names)
        self.frame_min_z = self._make_frame_floor_map(self.min_frame_z)

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

    def plan(self, start, goal):
        '''Plan a collision-free path between reduced joint configurations'''
        start = self._as_reduced_array(start, 'start')
        goal = self._as_reduced_array(goal, 'goal')

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

        # reset OMPL state and solve with the current start/goal pair
        self.si.setStateValidityCheckingResolution(
            float(self.config.validity_resolution)
        )
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

        start_time = time.perf_counter()
        solved = self.ss.solve(float(self.config.timeout))
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

        # simplify and interpolate the OMPL path before returning waypoints
        if self.config.simplify:
            self.ss.simplifySolution()

        path = self.ss.getSolutionPath()
        if self.config.interpolation_steps > 0:
            steps = max(2, int(self.config.interpolation_steps))
            path.interpolate(steps)

        path_array = self._path_to_array(path)

        # verify every returned waypoint with the reduced collision model
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

    def _direct_path(self, start, goal):
        steps = max(2, int(self.config.interpolation_steps))
        alpha = np.linspace(0.0, 1.0, steps).reshape(-1, 1)
        return (1.0 - alpha) * start + alpha * goal

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
