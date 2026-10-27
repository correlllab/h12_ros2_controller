import queue
import traceback
import multiprocessing as mp
from contextlib import contextmanager
from dataclasses import asdict
from threading import Event, Lock, Thread
from time import perf_counter

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
                 planner_config, handless=False, ompl_seed=None,
                 request_timeout=10.0, startup_timeout=30.0):
        self.urdf_path = urdf_path
        self.urdf_sphere_path = urdf_sphere_path
        self.srdf_sphere_path = srdf_sphere_path
        self.planner_config = planner_config
        self.handless = bool(handless)
        if ompl_seed is not None and (not isinstance(ompl_seed, int) or ompl_seed < 0):
            raise ValueError('ompl_seed must be a non-negative integer')
        self.ompl_seed = ompl_seed
        self.ompl_seed_effective = None
        self.request_timeout = float(request_timeout)
        if not np.isfinite(self.request_timeout) or self.request_timeout <= 0.0:
            raise ValueError('request_timeout must be finite and positive')
        self.startup_timeout = float(startup_timeout)
        if not np.isfinite(self.startup_timeout) or self.startup_timeout <= 0.0:
            raise ValueError('startup_timeout must be finite and positive')

        self._ctx = mp.get_context('spawn')
        self._lock = Lock()
        self._lifecycle_lock = Lock()
        self._shutdown = Event()
        self._ready = Event()
        self._warmup_thread = None
        self._startup_error = None
        self._startup_duration = None
        self._generation = 0
        self._request_id = 0
        self._request_queue = None
        self._response_queue = None
        self._process = None

    def plan_configuration(self, current_q, start, goal,
                           active_mask=None, moving_speed=None,
                           max_interpolation_steps=None, direct_only=False,
                           local_frames=(), timeout=None, allow_reuse=True,
                           response_timeout=None, local_joint_padding=None,
                           planning_budget=None):
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
            'direct_only': bool(direct_only),
            'local_frames': tuple(local_frames),
            'timeout': timeout,
            'allow_reuse': bool(allow_reuse),
            'local_joint_padding': local_joint_padding,
            'planning_budget': planning_budget,
        }, response_timeout=response_timeout)

    def warmup(self):
        '''Initialize the planner worker before its first motion request'''
        started = perf_counter()
        with self._lock:
            self._start()
            return perf_counter() - started

    def warmup_background(self):
        '''Schedule one nonblocking warmup attempt, returning whether scheduled

        Repeated calls do nothing, including after a failed attempt. Inspect
        startup_error for failures; warmup() can explicitly retry them.
        '''
        with self._lifecycle_lock:
            self._check_shutdown()
            if self._warmup_thread is not None or self.ready:
                return False
            self._warmup_thread = Thread(
                target=self._warmup_background,
                name='planner_warmup',
                daemon=True,
            )
            self._warmup_thread.start()
            return True

    def _warmup_background(self):
        try:
            self.warmup()
        except Exception:
            # _start records failures while holding the worker lock
            pass

    @property
    def ready(self):
        '''Whether the worker has completed startup and is still alive'''
        process = self._process
        return (not self._shutdown.is_set() and self._ready.is_set()
                and process is not None and process.is_alive())

    @property
    def startup_error(self):
        '''Last startup exception, cleared when a new startup attempt begins'''
        return self._startup_error

    @property
    def startup_duration(self):
        '''Seconds spent in the last completed startup attempt, or None'''
        return self._startup_duration

    def shutdown(self):
        '''Permanently close this client, interrupting any pending startup'''
        # signal before taking the worker lock so startup can release it promptly
        with self._lifecycle_lock:
            self._shutdown.set()
        with self._lock:
            self._stop(graceful=True)

    def _check_shutdown(self):
        if self._shutdown.is_set():
            raise RuntimeError('Planner client is shut down')

    def _stop(self, graceful=False):
        self._ready.clear()
        process = self._process
        try:
            if process is not None:
                if graceful and process.is_alive():
                    self._request_queue.put({'command': 'shutdown'})
                    process.join(timeout=2.0)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=1.0)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=1.0)
        finally:
            self._process = None
            self.ompl_seed_effective = None
            for name in ('_request_queue', '_response_queue'):
                connection = getattr(self, name)
                setattr(self, name, None)
                if connection is not None:
                    connection.cancel_join_thread()
                    connection.close()

    def _start(self):
        self._check_shutdown()
        if self.ready:
            return

        started = perf_counter()
        self._startup_error = None
        self._startup_duration = None
        self._stop()
        # lazily start the worker so spawn cost is paid only when needed
        self._generation += 1
        try:
            self._check_shutdown()
            self._request_queue = self._ctx.Queue()
            self._response_queue = self._ctx.Queue()
            init_args = {
                'urdf_path': self.urdf_path,
                'urdf_sphere_path': self.urdf_sphere_path,
                'srdf_sphere_path': self.srdf_sphere_path,
                'planner_config': asdict(self.planner_config),
                'handless': self.handless,
                'ompl_seed': self.ompl_seed,
                'generation': self._generation,
            }
            self._process = self._ctx.Process(
                target=_planner_worker_main,
                args=(init_args, self._request_queue, self._response_queue),
                name='planner_process',
                daemon=True,
            )
            self._process.start()
            deadline = perf_counter() + self.startup_timeout
            while True:
                self._check_shutdown()
                try:
                    message = self._response_queue.get(
                        timeout=min(0.1, max(0.0, deadline - perf_counter()))
                    )
                    break
                except queue.Empty as exc:
                    if not self._process.is_alive():
                        raise RuntimeError('Planner process exited during startup') from exc
                    if perf_counter() >= deadline:
                        raise RuntimeError('Planner process did not start in time') from exc
            self._check_shutdown()
            if message.get('type') == 'startup_error':
                raise RuntimeError(message['error'])
            if message.get('type') != 'ready' or message.get('generation') != self._generation:
                raise RuntimeError(f'Unexpected planner startup message: {message}')
            self.ompl_seed_effective = message.get('ompl_seed')
            self._ready.set()
        except Exception as exc:
            self._startup_error = exc
            self._stop()
            raise
        finally:
            self._startup_duration = perf_counter() - started

    def _request(self, request, response_timeout=None):
        if response_timeout is not None:
            response_timeout = float(response_timeout)
            if not np.isfinite(response_timeout) or response_timeout <= 0.0:
                raise ValueError('response_timeout must be finite and positive')
        started = perf_counter()
        with self._lock:
            acquired = perf_counter()
            self._start()
            ready = perf_counter()
            self._request_id += 1
            request = {**request, 'request_id': self._request_id}
            generation = self._generation
            try:
                self._check_shutdown()
                self._request_queue.put(request)
                deadline = perf_counter() + min(
                    self.request_timeout,
                    response_timeout if response_timeout is not None else self.request_timeout,
                )
                while True:
                    self._check_shutdown()
                    remaining = deadline - perf_counter()
                    if remaining <= 0.0:
                        raise TimeoutError('Planner process response timed out')
                    try:
                        message = self._response_queue.get(timeout=min(0.1, remaining))
                    except queue.Empty as exc:
                        if not self._process.is_alive():
                            raise RuntimeError('Planner process exited unexpectedly') from exc
                        continue
                    if perf_counter() >= deadline:
                        raise TimeoutError('Planner process response timed out')
                    if (message.get('generation') != generation or
                            message.get('request_id') != self._request_id):
                        continue
                    if message.get('type') == 'error':
                        raise RuntimeError(message['error'])
                    if message.get('type') != 'result' or not isinstance(message.get('result'), PlanResult):
                        raise RuntimeError(f'Unexpected planner response: {message}')
                    response = message['result']
                    completed = perf_counter()
                    response.metadata['client_timings'] = {
                        'startup': ready - acquired,
                        'readiness_or_queue_wait': acquired - started,
                        'request_transport_and_worker': completed - ready,
                        'total': completed - started,
                    }
                    return response
            except Exception:
                self._stop()
                raise


class _PlannerWorker:
    '''Own the native planner state inside the child process'''

    def __init__(self, init_args):
        # build native state in the child so OMPL and pinocchio stay isolated
        self.robot_model = RobotModel(
            init_args['urdf_path'],
            handless=init_args['handless'],
        )
        planner_config = PlannerConfig(**init_args['planner_config'])
        if planner_config.motor_position_limits is not None:
            self.robot_model.apply_position_limits(planner_config.motor_position_limits)
        self.robot_model.init_reduced_model(ENABLED_JOINTS)
        self.robot_model.init_collision_model(
            init_args['urdf_sphere_path'],
            init_args['srdf_sphere_path'],
        )
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
        timeout = request.get('timeout')
        if timeout is not None and (not np.isfinite(timeout) or timeout <= 0.0):
            raise ValueError('Planner timeout must be finite and positive')
        budget = request.get('planning_budget')
        if budget is not None:
            budget = float(budget)
            if not np.isfinite(budget) or budget <= 0.0:
                raise ValueError('planning_budget must be finite and positive')
        deadline = None if budget is None else perf_counter() + budget
        with self._config_override(
            request.get('moving_speed'),
            request.get('max_interpolation_steps'),
        ):
            return self.planner.plan(
                start, goal, active_mask=active_mask,
                timeout=timeout,
                direct_only=request.get('direct_only', False),
                local_frames=request.get('local_frames', ()),
                allow_reuse=request.get('allow_reuse', True),
                local_joint_padding=request.get('local_joint_padding'),
                deadline=deadline,
            )

    @contextmanager
    def _config_override(self, moving_speed, max_interpolation_steps, timeout=None):
        '''Apply per-request planner speed settings, restoring them after'''
        if moving_speed is None and max_interpolation_steps is None and timeout is None:
            yield
            return

        config = self.planner.config
        previous = (config.moving_speed, config.max_interpolation_steps,
                    config.max_staged_interpolation_steps, config.timeout)
        # validate overrides before changing the worker's shared config
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
        if timeout is not None:
            timeout = float(timeout)
            if not np.isfinite(timeout) or timeout <= 0.0:
                raise ValueError('Planner timeout must be finite and positive')
        if moving_speed is not None:
            config.moving_speed = moving_speed
        if max_interpolation_steps is not None:
            config.max_interpolation_steps = max_interpolation_steps
            config.max_staged_interpolation_steps = int(np.ceil(
                previous[2] * max_interpolation_steps / previous[1]
            ))
        if timeout is not None:
            config.timeout = timeout
        try:
            yield
        finally:
            (config.moving_speed, config.max_interpolation_steps,
             config.max_staged_interpolation_steps, config.timeout) = previous

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
    generation = init_args['generation']
    try:
        effective_seed = None
        if init_args.get('ompl_seed') is not None:
            from ompl import util as ou
            requested_seed = int(init_args['ompl_seed'])
            ou.RNG.setSeed(requested_seed)
            if ou.RNG.getSeed() == requested_seed:
                effective_seed = requested_seed
        worker = _PlannerWorker(init_args)
        # notify the parent only after the child model is fully initialized
        response_queue.put({
            'type': 'ready', 'generation': generation, 'ompl_seed': effective_seed,
        })
    except Exception as exc:
        response_queue.put({
            'type': 'startup_error',
            'error': _format_error(exc),
            'generation': generation,
        })
        return

    while True:
        request = request_queue.get()
        if request.get('command') == 'shutdown':
            return
        try:
            result = worker.handle(request)
            response_queue.put({
                'type': 'result', 'result': result,
                'generation': generation, 'request_id': request['request_id'],
            })
        except Exception as exc:
            response_queue.put({
                'type': 'error',
                'error': _format_error(exc),
                'generation': generation, 'request_id': request['request_id'],
            })


def _optional_array(value, dtype=float):
    if value is None:
        return None
    return np.asarray(value, dtype=dtype)


def _format_error(exc):
    return f'{exc}\n{traceback.format_exc()}'
