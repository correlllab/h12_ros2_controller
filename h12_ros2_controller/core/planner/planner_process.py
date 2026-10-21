import queue
import traceback
import multiprocessing as mp
from contextlib import contextmanager
from dataclasses import asdict

import numpy as np

from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlanResult,
    PlannerConfig,
    ReducedJointPlanner,
)
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS


class PlannerClient:
    '''Client for a planner subprocess'''

    def __init__(self, urdf_path, urdf_sphere_path, srdf_sphere_path,
                 planner_config, handless=False):
        self.urdf_path = urdf_path
        self.urdf_sphere_path = urdf_sphere_path
        self.srdf_sphere_path = srdf_sphere_path
        self.planner_config = planner_config
        self.handless = bool(handless)

        self._ctx = mp.get_context('spawn')
        self._request_queue = None
        self._response_queue = None
        self._process = None

    def plan_configuration(self, current_q, start, goal,
                           active_mask=None, moving_speed=None,
                           max_interpolation_steps=None):
        '''Plan to a reduced joint target in the planner process

        The worker's PlannerConfig is serialized once at spawn, so per-goal
        speed changes have to travel with the request. Leave the overrides at
        None to plan with the config the worker was started with.
        '''
        return self._request({
            'command': 'plan_configuration',
            'current_q': np.asarray(current_q, dtype=float),
            'start': np.asarray(start, dtype=float),
            'goal': np.asarray(goal, dtype=float),
            'active_mask': _optional_array(active_mask, dtype=bool),
            'moving_speed': moving_speed,
            'max_interpolation_steps': max_interpolation_steps,
        })

    def warmup(self):
        '''Initialize the planner worker before its first motion request'''
        self._start()

    def shutdown(self):
        '''Stop the planner process'''
        if self._process is None:
            return
        try:
            self._request_queue.put({'command': 'shutdown'})
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
        finally:
            self._process = None
            self._request_queue = None
            self._response_queue = None

    def _start(self):
        if self._process is not None and self._process.is_alive():
            return

        # lazily start the worker so spawn cost is paid only when needed
        self._request_queue = self._ctx.Queue()
        self._response_queue = self._ctx.Queue()
        init_args = {
            'urdf_path': self.urdf_path,
            'urdf_sphere_path': self.urdf_sphere_path,
            'srdf_sphere_path': self.srdf_sphere_path,
            'planner_config': asdict(self.planner_config),
            'handless': self.handless,
        }
        self._process = self._ctx.Process(
            target=_planner_worker_main,
            args=(init_args, self._request_queue, self._response_queue),
            name='planner_process',
            daemon=True,
        )
        self._process.start()

        try:
            message = self._response_queue.get(timeout=30.0)
        except queue.Empty as exc:
            raise RuntimeError(
                'Planner process did not start in time'
            ) from exc
        if message.get('type') == 'startup_error':
            raise RuntimeError(message['error'])
        if message.get('type') != 'ready':
            raise RuntimeError(
                f'Unexpected planner startup message: {message}'
            )

    def _request(self, request):
        self._start()
        self._request_queue.put(request)
        while True:
            try:
                response = self._response_queue.get(timeout=1.0)
            except queue.Empty as exc:
                if self._process is not None and self._process.is_alive():
                    continue
                raise RuntimeError(
                    'Planner process exited unexpectedly'
                ) from exc
            if isinstance(response, PlanResult):
                return response
            if response.get('type') == 'error':
                raise RuntimeError(response['error'])
            raise RuntimeError(f'Unexpected planner response: {response}')


class _PlannerWorker:
    '''Own the native planner state inside the child process'''

    def __init__(self, init_args):
        # build native state in the child so OMPL and pinocchio stay isolated
        self.robot_model = RobotModel(
            init_args['urdf_path'],
            handless=init_args['handless'],
        )
        self.robot_model.init_reduced_model(ENABLED_JOINTS)
        self.robot_model.init_collision_model(
            init_args['urdf_sphere_path'],
            init_args['srdf_sphere_path'],
        )
        planner_config = PlannerConfig(**init_args['planner_config'])
        self.planner = ReducedJointPlanner(
            self.robot_model,
            config=planner_config,
        )
        self._frozen_reference = self.robot_model.zero_q_body[
            ~self.robot_model.reduced_mask
        ].copy()

    def handle(self, request):
        command = request['command']
        if command == 'plan_configuration':
            return self._plan_configuration(request)
        raise ValueError(f'Unsupported planner command: {command}')

    def _plan_configuration(self, request):
        self._set_current_q(request['current_q'])
        start = np.asarray(request['start'], dtype=float)
        goal = np.asarray(request['goal'], dtype=float)
        active_mask = _optional_array(request.get('active_mask'), dtype=bool)
        with self._config_override(
            request.get('moving_speed'),
            request.get('max_interpolation_steps'),
        ):
            return self.planner.plan(start, goal, active_mask=active_mask)

    @contextmanager
    def _config_override(self, moving_speed, max_interpolation_steps):
        '''Apply per-request planner speed settings, restoring them after'''
        if moving_speed is None and max_interpolation_steps is None:
            yield
            return

        config = self.planner.config
        previous = (config.moving_speed, config.max_interpolation_steps)
        # PlannerConfig is validated at construction only, so an override has
        # to be checked here or a bad value divides by zero when interpolating
        if moving_speed is not None:
            moving_speed = float(moving_speed)
            if not np.isfinite(moving_speed) or moving_speed <= 0.0:
                raise ValueError('Planner moving_speed must be finite and positive')
        if max_interpolation_steps is not None:
            max_interpolation_steps = int(max_interpolation_steps)
            if max_interpolation_steps < config.min_interpolation_steps:
                raise ValueError(
                    'Planner max_interpolation_steps must be at least '
                    'min_interpolation_steps'
                )
        if moving_speed is not None:
            config.moving_speed = moving_speed
        if max_interpolation_steps is not None:
            config.max_interpolation_steps = max_interpolation_steps
        try:
            yield
        finally:
            config.moving_speed, config.max_interpolation_steps = previous

    def _set_current_q(self, q):
        # sync the child process model with the caller's current joint state
        q = np.asarray(q, dtype=float)
        if q.shape != self.robot_model._q.shape or not np.all(np.isfinite(q)):
            raise ValueError('Planner current_q must be a finite motor configuration')
        self.robot_model._q = q.copy()
        self.robot_model._dq = np.zeros_like(self.robot_model._q)
        frozen_reference = q[~self.robot_model.reduced_mask]
        if not np.array_equal(frozen_reference, self._frozen_reference):
            self.robot_model.rebuild_reduced_model(reference_q=q)
            self.planner.rebind_reduced_model()
            self._frozen_reference = frozen_reference.copy()
        self.robot_model.update_kinematics()


def _planner_worker_main(init_args, request_queue, response_queue):
    try:
        worker = _PlannerWorker(init_args)
        # notify the parent only after the child model is fully initialized
        response_queue.put({'type': 'ready'})
    except Exception as exc:
        response_queue.put({
            'type': 'startup_error',
            'error': _format_error(exc),
        })
        return

    while True:
        request = request_queue.get()
        if request.get('command') == 'shutdown':
            return
        try:
            result = worker.handle(request)
            response_queue.put(result)
        except Exception as exc:
            response_queue.put({
                'type': 'error',
                'error': _format_error(exc),
            })


def _optional_array(value, dtype=float):
    if value is None:
        return None
    return np.asarray(value, dtype=dtype)


def _format_error(exc):
    return f'{exc}\n{traceback.format_exc()}'
