from copy import copy, deepcopy
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass

import numpy as np

from h12_ros2_controller.core.controller.planned_reference import PlanReference


@dataclass(frozen=True)
class SavedFramePlan:
    '''One explicit approach, with absolute targets and original path provenance'''

    reference: PlanReference
    frame_names: tuple
    targets: np.ndarray
    speed_scale: float

    @classmethod
    def capture(cls, controller, path, frame_names, targets):
        targets = np.ascontiguousarray(targets, dtype=np.float64)
        immutable = np.frombuffer(targets.tobytes(), dtype=np.float64).reshape(targets.shape)
        return cls(PlanReference.capture(controller, path), tuple(frame_names),
                   immutable, controller.speed_scale)

    @property
    def start(self):
        return self.reference.path[0]

    def assert_goal(self, goal):
        if goal.plan or goal.relative or goal.slow_mode:
            raise ValueError('Replay accepts only duration and optional cached frame/target assertions')
        if goal.frame_names and tuple(goal.frame_names) != self.frame_names:
            raise ValueError('Replay frame_names differ from the saved approach')


@contextmanager
def planning_only(controller):
    '''Isolate IK/control caches while the existing publisher holds its buffers'''
    ik = controller.ik_solver
    robot = controller.robot_model
    # models stay shared; the planning solver owns independent mutable tasks/data
    memo = {id(robot): robot}
    for name, value in vars(robot).items():
        if name.startswith(('model_', 'collision_model_')):
            memo[id(value)] = value
    planning_ik = copy(ik)
    # reduced planning rebuilds limits/barriers; Pin joint-limit bindings are not pickleable
    bindings = {'limits', 'limits_reduced', 'collision_barrier_reduced'}
    planning_ik.__dict__.update(deepcopy(
        {name: value for name, value in vars(ik).items() if name not in bindings}, memo,
    ))
    planning_ik.rebuild_reduced()
    absent = object()
    names = ('_tau_bias', '_prior_ik', 'speed_scale', 'last_plan_result',
             'last_planning_context', 'last_planning_start', 'last_ik_result',
             'last_plan_attempts', '_reduced_rebuild_pending')
    saved = {name: getattr(controller, name, absent) for name in names}
    # rebuilding at a new frozen-joint pose must not replace the live IK bindings
    robot_names = ('model_body_reduced', 'data_body_reduced', 'reduced_reference_q',
                   'collision_model_body_reduced', 'collision_data_body_reduced')
    robot_saved = {name: getattr(robot, name, absent) for name in robot_names}
    buffers = {name: getattr(controller.low_cmd_handler, name).copy()
               for name in ('q_cmd', 'dq_cmd', 'tau_cmd')}
    controller.ik_solver = planning_ik
    if saved['_tau_bias'] is not absent:
        controller._tau_bias = saved['_tau_bias'].copy()
    try:
        yield
    finally:
        controller.ik_solver = ik
        for name, value in saved.items():
            if value is absent:
                controller.__dict__.pop(name, None)
            else:
                setattr(controller, name, value)
        for name, value in robot_saved.items():
            if value is absent:
                robot.__dict__.pop(name, None)
            else:
                setattr(robot, name, value)
        changed = []
        publisher = getattr(controller.low_cmd_handler, '_command_publisher', None)
        lock = publisher.data_lock if publisher is not None else nullcontext()
        with lock:
            for name, original in buffers.items():
                buffer = (getattr(publisher, name.removesuffix('_cmd'))
                          if publisher is not None else
                          getattr(controller.low_cmd_handler, name))
                if buffer.tobytes() != original.tobytes():
                    changed.append(name)
                    buffer[:] = original
        if changed:
            raise RuntimeError(f'Planning-only changed motor command buffers: {", ".join(changed)}')
