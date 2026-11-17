'''Saved approach action flow using actual playback and offline motor buffers'''

import threading
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock
from pathlib import Path

import numpy as np
import pytest

from h12_ros2_controller.core.saved_frame_plan import SavedFramePlan, planning_only
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.controller.planned_reference import command_context_digest
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from test_planned_comparison import comparison_controller, publisher_buffer_handler, plan_result
from test_ros_planned_comparison import load_server, FakeNode, Pose


def goal(**values):
    request = dict(plan=False, relative=False, slow_mode=False, frame_names=[],
                   frame_targets=[], duration=SimpleNamespace(sec=1, nanosec=0))
    request.update(values)
    return SimpleNamespace(request=SimpleNamespace(**request), is_cancel_requested=False,
                           succeed=Mock(), abort=Mock(), canceled=Mock(), publish_feedback=Mock())


def server_for(controller, mode='frame'):
    namespace = load_server()
    namespace.update(SavedFramePlan=SavedFramePlan, planning_only=planning_only,
                     nullcontext=nullcontext, SLOW_MODE_SCALE=.25)
    server = object.__new__(namespace['FrameTaskServer'])
    FakeNode.__init__(server, 'offline')
    server.controller = controller
    server._controller_lock = threading.Lock()
    server._comparison_state_lock = threading.Lock()
    server._comparison_active = False
    server._saved_frame_plan = None
    server._upper_controller = 'counter'
    server.parameters['control_mode'] = mode
    server.add_on_set_parameters_callback(server._comparison_parameters_callback)
    controller.set_control_mode(mode)
    server.frame_names, server.frame_targets = [], []
    server._frame_publish_snapshot = ((), ())
    server.timeout = 1.
    server.threshold_linear = server.threshold_angular = .01
    server.threshold_joint = .01
    server.config_name = 'offline'
    server.namespace = namespace
    namespace['NAMED_CONFIGS'] = NAMED_CONFIGS
    controller.robot_model.model_body.frames = [SimpleNamespace(name=name) for name in (
        'left_grasp_frame', 'right_grasp_frame', 'pelvis')]
    return server


def save(server, **values):
    handle = goal(plan=True, frame_names=['left_grasp_frame'], frame_targets=[Pose()], **values)
    assert server.plan_frame_task_callback(handle).success, server.logger.error.call_args_list
    handle.succeed.assert_called_once()
    return server._saved_frame_plan


@pytest.mark.parametrize('dtypes', [(np.float32,) * 3, (np.float64, np.float32, np.float32)])
def test_plan_only_preserves_actual_publisher_bytes_and_control_state(monkeypatch, dtypes):
    controller = comparison_controller(monkeypatch)
    handler = publisher_buffer_handler(controller, dtypes)
    server = server_for(controller)
    ik = controller.ik_solver
    ik.frame_tasks['previous'] = np.eye(4)
    prior = controller._prior_ik = {'old': True}
    bias = controller._tau_bias.copy()
    buffers = [getattr(handler, field).tobytes() for field in ('q_cmd', 'dq_cmd', 'tau_cmd')]
    saved = save(server, slow_mode=True)
    assert controller.ik_solver is ik
    assert controller._prior_ik is prior
    assert list(ik.frame_tasks) == ['previous']
    np.testing.assert_array_equal(controller._tau_bias, bias)
    assert controller.speed_scale == 1.
    assert saved.speed_scale == .25
    handler.set_joint_commands.assert_not_called()
    assert buffers == [getattr(handler, field).tobytes() for field in ('q_cmd', 'dq_cmd', 'tau_cmd')]
    assert server._frame_publish_snapshot == ((), ())
    assert getattr(controller, 'last_plan_result', None) is None
    assert saved.reference.dt == controller.dt
    assert saved.reference.config_digest == command_context_digest(controller)
    assert saved.reference.created_at > 0
    with pytest.raises(ValueError):
        saved.reference.path.setflags(write=True)
    with pytest.raises(ValueError):
        saved.targets.setflags(write=True)
    with pytest.raises(ValueError):
        saved.start[0] += 1


@pytest.mark.parametrize('mode', ['frame', 'counter'])
def test_replay_shared_upper_law_counter_reacts_and_reference_survives(monkeypatch, mode):
    controller = comparison_controller(monkeypatch)
    baseline = comparison_controller(monkeypatch)
    baseline.__class__ = FrameController
    server = server_for(controller)
    saved = save(server)
    created = saved.reference.created_at
    # named planning may replace ordinary last_plan_result without invalidating the cache
    controller.last_plan_result = plan_result(controller, np.tile(saved.start, (2, 1)))
    server.parameters['control_mode'] = mode
    baseline.execute_path(saved.reference.path, validation=baseline.revalidate_plan(saved.reference))
    handle = goal()
    assert server.replay_frame_task_callback(handle).success, server.logger.error.call_args_list
    assert server._saved_frame_plan is saved
    assert saved.reference.created_at == created
    assert controller.control_mode == mode
    for field in ('q_cmd', 'dq_cmd', 'tau_cmd'):
        # compare playback (hold follows the same moving law but adds I bias)
        assert np.isfinite(getattr(controller.low_cmd_handler, field)).all()
    writes = controller.low_cmd_handler.calls[:len(saved.reference.path)]
    nominal = baseline.low_cmd_handler.calls
    for actual, expected in zip(writes, nominal):
        for field in ('q', 'dq', 'tau'):
            actual_ids = [list(actual['joint_ids']).index(i) for i in controller.moving_ids]
            expected_ids = [list(expected['joint_ids']).index(i) for i in controller.moving_ids]
            np.testing.assert_array_equal(actual[field][actual_ids], expected[field][expected_ids])
    if mode == 'counter':
        assert np.any(controller.low_cmd_handler.q_cmd[controller.counter_ids] !=
                      baseline.low_cmd_handler.q_cmd[controller.counter_ids])
        reference = controller.q_counter_ref
        controller.prepare_frame_goal(['left_grasp_frame'])
        assert controller.q_counter_ref is reference
    handle.succeed.assert_called_once()


def test_generic_frame_backend_replay_and_multi_frame_plain(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    controller.__class__ = FrameController
    server._upper_controller = 'frame'
    handle = goal(plan=True, frame_names=['left_grasp_frame', 'right_grasp_frame'],
                  frame_targets=[Pose(), Pose()])
    assert server.plan_frame_task_callback(handle).success
    saved = server._saved_frame_plan
    assert server.replay_frame_task_callback(goal()).success, server.logger.error.call_args_list
    assert server._saved_frame_plan is saved


def test_counter_backend_plain_multi_frame_and_counter_rejection(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    handle = goal(plan=True, frame_names=['left_grasp_frame', 'right_grasp_frame'],
                  frame_targets=[Pose(), Pose()])
    assert server.plan_frame_task_callback(handle).success
    saved = server._saved_frame_plan
    server.parameters['control_mode'] = 'counter'
    assert not server.replay_frame_task_callback(goal()).success
    assert not controller.low_cmd_handler.calls
    assert server._saved_frame_plan is saved
    server.parameters['control_mode'] = 'frame'
    assert server.replay_frame_task_callback(goal()).success, server.logger.error.call_args_list


def test_plain_counter_backend_direct_commands_match_frame_backend(monkeypatch):
    controller = comparison_controller(monkeypatch)
    baseline = comparison_controller(monkeypatch)
    baseline.__class__ = FrameController
    controller.set_control_mode('frame')
    controller.prepare_frame_goal(['left_grasp_frame', 'right_grasp_frame'])
    assert not controller.low_cmd_handler.calls
    for item in (controller, baseline):
        target = item.low_cmd_handler.q_cmd.copy()
        target[[13, 20]] += .001
        item.ik_solver = SimpleNamespace(q=target, integrate=Mock())
        item._apply_velocity_command(np.zeros(27))
    for field in ('q_cmd', 'dq_cmd', 'tau_cmd'):
        assert getattr(controller.low_cmd_handler, field).tobytes() == getattr(
            baseline.low_cmd_handler, field).tobytes()


def test_legacy_batch_internal_modes_preserve_persistent_normal_mode(monkeypatch):
    from h12_ros2_controller.core.controller.planned_comparison import PlannedComparison

    controller = comparison_controller(monkeypatch)
    controller.set_control_mode('frame')
    result = PlannedComparison(controller).run('left_grasp_frame', np.eye(4), ('counter', 'frame'))
    assert result['success'], result['reason']
    assert [trial['condition'] for trial in result['trials']] == ['counter', 'frame']
    assert controller.control_mode == 'frame'
    controller.prepare_frame_goal(['left_grasp_frame', 'right_grasp_frame'])
    assert not controller.balanced_live


@pytest.mark.parametrize('invalid', [dict(plan=True), dict(relative=True), dict(slow_mode=True),
                                   dict(frame_names=['right_grasp_frame']),
                                   dict(frame_targets=[Pose(), Pose()])])
def test_replay_rejects_nondefault_semantics_without_publication(monkeypatch, invalid):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server, slow_mode=True)
    assert not server.replay_frame_task_callback(goal(**invalid)).success
    assert not controller.low_cmd_handler.calls
    assert server._saved_frame_plan is saved


def test_replay_endpoint_assertions_not_retargeting_and_resolve_relative_once(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    measured = np.eye(4)
    measured[:3, 3] = [.3, .2, .1]
    controller.get_frame_transformation = Mock(return_value=measured)
    saved = save(server, relative=True)  # zero relative offset -> measured absolute pose
    np.testing.assert_array_equal(saved.targets[0], measured)
    wrong = Pose()
    wrong.position.x = .300001
    wrong.position.y, wrong.position.z = .2, .1
    assert not server.replay_frame_task_callback(goal(frame_targets=[wrong])).success
    assert not controller.low_cmd_handler.calls
    expected = Pose()
    expected.position.x, expected.position.y, expected.position.z = .3, .2, .1
    expected.orientation.w = -2.
    assert server.replay_frame_task_callback(goal(frame_targets=[expected])).success
    assert server._saved_frame_plan is saved


def test_replay_start_guard_then_named_approach_start_uses_existing_planner(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    controller.low_cmd_handler.q_cmd[13:] = 0.
    controller.robot_model.state['q'][13:] = 0.
    assert not server.replay_frame_task_callback(goal()).success
    assert not controller.low_cmd_handler.calls
    assert 'approach_start' in str(server.logger.error.call_args)
    controller.plan_to_configuration = Mock(side_effect=lambda target: (
        setattr(controller, 'last_plan_result', plan_result(
            controller, np.linspace(controller.low_cmd_handler.q_cmd[13:], target, 150))) or
        controller.last_plan_result.path))
    server._named_config_joint_error = lambda: 0.
    server._publish_named_config_feedback = Mock()
    handle = goal(config_name='approach_start', plan=True)
    assert server.named_config_callback(handle).success, server.logger.warn.call_args_list
    controller.plan_to_configuration.assert_called_once()
    np.testing.assert_array_equal(controller.plan_to_configuration.call_args.args[0], saved.start)
    np.testing.assert_array_equal(controller.low_cmd_handler.q_cmd[13:], saved.start)
    assert server._saved_frame_plan is saved
    assert server.replay_frame_task_callback(goal()).success, server.logger.error.call_args_list


@pytest.mark.parametrize('name', ['home', 'approach_start'])
def test_named_goal_prepares_and_finishes_once_after_motor_free_plan(monkeypatch, name):
    controller = comparison_controller(monkeypatch)
    controller.prepare_named_goal = Mock(wraps=controller.prepare_named_goal)
    controller.finish_goal = Mock(wraps=controller.finish_goal)
    server = server_for(controller)
    saved = save(server)
    assert not controller.low_cmd_handler.calls
    controller.prepare_named_goal.assert_not_called()
    controller.finish_goal.assert_not_called()

    def planned_goal(handle, target, timeout):
        assert server._controller_lock.locked()
        controller.prepare_named_goal.assert_called_once_with()
        controller.finish_goal.assert_not_called()
        np.testing.assert_array_equal(target, saved.start if name == 'approach_start'
                                      else NAMED_CONFIGS[name])
        return SimpleNamespace(success=True)

    server._plan_named_config_callback = Mock(side_effect=planned_goal)
    assert server.named_config_callback(goal(config_name=name, plan=True)).success
    server._plan_named_config_callback.assert_called_once()
    controller.prepare_named_goal.assert_called_once_with()
    controller.finish_goal.assert_called_once_with()
    assert not server._controller_lock.locked()
    assert server._saved_frame_plan is saved


def test_failed_or_canceled_plan_does_not_replace_saved_record(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    controller.plan_to_ik_target.side_effect = RuntimeError('offline planner failure')
    assert not server.plan_frame_task_callback(goal(
        plan=True, frame_names=['left_grasp_frame'], frame_targets=[Pose()])).success
    assert server._saved_frame_plan is saved
    assert not controller.low_cmd_handler.calls
    assert not server.plan_frame_task_callback(goal()).success
    assert not server._controller_lock.locked()


def test_ordinary_frame_and_named_plans_never_replace_explicit_cache(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    ordinary = goal(plan=True, frame_names=['left_grasp_frame'], frame_targets=[Pose()])
    assert server.frame_task_callback(ordinary).success, server.logger.warn.call_args_list
    assert server._saved_frame_plan is saved
    server._plan_named_config_callback = Mock(return_value=SimpleNamespace(success=True))
    assert server.named_config_callback(goal(config_name='home', plan=True)).success
    np.testing.assert_array_equal(server._plan_named_config_callback.call_args.args[1],
                                  NAMED_CONFIGS['home'])
    assert server._saved_frame_plan is saved
    assert controller.control_mode == 'frame'


def test_no_cache_errors_and_cancel_do_not_publish(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    assert not server.replay_frame_task_callback(goal()).success
    assert not server.named_config_callback(goal(config_name='approach_start')).success
    handle = goal(plan=True, frame_names=['left_grasp_frame'], frame_targets=[Pose()])
    planner = controller.plan_to_ik_target.side_effect

    def cancel_plan():
        path = planner()
        handle.is_cancel_requested = True
        return path

    controller.plan_to_ik_target.side_effect = cancel_plan
    assert not server.plan_frame_task_callback(handle).success
    handle.canceled.assert_called_once()
    assert server._saved_frame_plan is None
    assert not controller.low_cmd_handler.calls


@pytest.mark.parametrize('mode', ['frame', 'counter'])
@pytest.mark.parametrize('dtypes', [(np.float32,) * 3, (np.float64, np.float32, np.float32)])
def test_saved_replay_actual_publisher_dtype_moving_parity(monkeypatch, mode, dtypes):
    controller = comparison_controller(monkeypatch)
    baseline = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    records = []
    nominal = []
    for item, destination in ((controller, records), (baseline, nominal)):
        handler = publisher_buffer_handler(item, dtypes)
        writer = handler.set_joint_commands

        def record(*args, handler=handler, writer=writer, destination=destination, **kwargs):
            writer(*args, **kwargs)
            destination.append(tuple(getattr(handler, field).copy()
                                     for field in ('q_cmd', 'dq_cmd', 'tau_cmd')))

        handler.set_joint_commands = record
    server.parameters['control_mode'] = mode
    baseline.__class__ = FrameController
    baseline.execute_path(saved.reference.path, validation=baseline.revalidate_plan(saved.reference))
    assert server.replay_frame_task_callback(goal()).success, server.logger.error.call_args_list
    assert len(records) > len(nominal)  # existing endpoint hold and finish follow playback
    for actual, expected in zip(records, nominal):
        for field, reference in zip(actual, expected):
            assert field[controller.moving_ids].tobytes() == reference[controller.moving_ids].tobytes()


def test_mode_parameter_idle_no_publication_busy_rejection_and_goal_freeze(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    change = lambda mode: server.set_parameters_atomically([
        SimpleNamespace(name='control_mode', value=mode)])
    assert change('counter').successful
    assert not controller.low_cmd_handler.calls
    assert server.get_parameter('control_mode').value == 'counter'
    assert controller.control_mode == 'frame'
    with server._controller_lock:
        assert not change('frame').successful
    server._comparison_active = True
    assert not change('frame').successful
    assert not server.plan_frame_task_callback(goal()).success
    assert not server.replay_frame_task_callback(goal()).success
    server._comparison_active = False
    original = controller.execute_path

    def execute(*args, **kwargs):
        assert server._controller_lock.locked()
        assert not change('frame').successful
        assert controller.control_mode == 'counter'
        return original(*args, **kwargs)

    controller.execute_path = execute
    assert server.replay_frame_task_callback(goal()).success, server.logger.error.call_args_list
    assert server._saved_frame_plan is saved
    assert not change('').successful
    assert not change('other').successful
    server._upper_controller = 'frame'
    assert not change('counter').successful


@pytest.mark.parametrize('action', ['frame', 'replay'])
def test_atomic_mode_transaction_later_rejection_and_next_goal_routing(monkeypatch, action):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    handler = publisher_buffer_handler(controller, (np.float64, np.float32, np.float32))
    controller.set_control_mode = Mock(wraps=controller.set_control_mode)
    calls = []

    def validate(parameters):
        calls.append('controller')
        return server._comparison_parameters_callback(parameters)

    def later_validator(parameters):
        calls.append('later')
        return SimpleNamespace(successful=not any(p.name == 'bad_other' for p in parameters))

    # Humble commits only after every on-set callback accepts the whole transaction
    server.parameter_callbacks.clear()
    server.add_on_set_parameters_callback(later_validator)
    server.add_on_set_parameters_callback(validate)
    before = controller.__dict__.copy()
    buffers = tuple(getattr(handler, field).tobytes() for field in ('q_cmd', 'dq_cmd', 'tau_cmd'))
    checksum, created = saved.reference.checksum, saved.reference.created_at
    rejected = server.set_parameters_atomically([
        SimpleNamespace(name='control_mode', value='counter'),
        SimpleNamespace(name='bad_other', value=True),
    ])
    assert not rejected.successful
    assert calls == ['controller', 'later']
    assert server.get_parameter('control_mode').value == 'frame'
    assert not hasattr(server, '_control_mode')
    assert before.keys() == controller.__dict__.keys()
    assert all(controller.__dict__[name] is value for name, value in before.items())
    assert buffers == tuple(getattr(handler, field).tobytes() for field in ('q_cmd', 'dq_cmd', 'tau_cmd'))
    handler.set_joint_commands.assert_not_called()
    controller.set_control_mode.assert_not_called()

    def run(expected):
        def endpoint(*args):
            assert server._controller_lock.locked()
            assert controller.control_mode == expected
            return SimpleNamespace(success=True)

        if action == 'frame':
            server._direct_frame_task_callback = endpoint
            handle = goal(frame_names=['left_grasp_frame'], frame_targets=[Pose()])
            result = server.frame_task_callback(handle)
        else:
            controller.execute_path = Mock()
            server._hold_frame_task_callback = endpoint
            result = server.replay_frame_task_callback(goal())
        assert result.success, server.logger.error.call_args_list
        controller.set_control_mode.assert_called_once_with(expected)

    run('frame')
    controller.set_control_mode.reset_mock()
    before = controller.__dict__.copy()
    buffers = tuple(getattr(handler, field).tobytes() for field in ('q_cmd', 'dq_cmd', 'tau_cmd'))
    assert server.set_parameters_atomically([
        SimpleNamespace(name='control_mode', value='counter')]).successful
    assert server.get_parameter('control_mode').value == 'counter'
    assert controller.control_mode == 'frame'
    assert all(controller.__dict__[name] is value for name, value in before.items())
    assert buffers == tuple(getattr(handler, field).tobytes() for field in ('q_cmd', 'dq_cmd', 'tau_cmd'))
    controller.set_control_mode.assert_not_called()
    run('counter')
    assert server._saved_frame_plan is saved
    assert saved.reference.checksum == checksum
    assert saved.reference.created_at == created
    assert command_context_digest(controller) == saved.reference.config_digest


def test_plan_only_validates_committed_mode_without_configuring_core(monkeypatch):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    controller.set_control_mode = Mock(wraps=controller.set_control_mode)
    assert server.set_parameters_atomically([
        SimpleNamespace(name='control_mode', value='counter')]).successful
    assert controller.control_mode == 'frame'
    multiple = goal(plan=True, frame_names=['left_grasp_frame', 'right_grasp_frame'],
                    frame_targets=[Pose(), Pose()])
    assert not server.plan_frame_task_callback(multiple).success
    assert server._saved_frame_plan is saved
    replacement = save(server)
    assert replacement.reference.config_digest == saved.reference.config_digest
    assert controller.control_mode == 'frame'
    controller.set_control_mode.assert_not_called()
    assert not controller.low_cmd_handler.calls


@pytest.mark.parametrize('action', ['frame', 'replay'])
def test_prevalidated_commit_during_goal_keeps_frozen_execution_mode(monkeypatch, action):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    controller.set_control_mode = Mock(wraps=controller.set_control_mode)
    validated, entered, committed = (threading.Event() for _ in range(3))
    transaction = []

    def later_validator(parameters):
        validated.set()
        assert entered.wait(3.)
        return SimpleNamespace(successful=True)

    server.parameter_callbacks.append(later_validator)

    def set_mode():
        try:
            transaction.append(server.set_parameters_atomically([
                SimpleNamespace(name='control_mode', value='counter')]))
        finally:
            committed.set()

    def endpoint(*args):
        assert server._controller_lock.locked()
        entered.set()
        assert committed.wait(3.)
        assert server.get_parameter('control_mode').value == 'counter'
        assert controller.control_mode == 'frame'
        return SimpleNamespace(success=True)

    if action == 'frame':
        server._direct_frame_task_callback = endpoint
        run = lambda: server.frame_task_callback(goal(
            frame_names=['left_grasp_frame'], frame_targets=[Pose()]))
    else:
        controller.execute_path = Mock()
        server._hold_frame_task_callback = endpoint
        run = lambda: server.replay_frame_task_callback(goal())
    worker = threading.Thread(target=set_mode)
    worker.start()
    try:
        assert validated.wait(3.)
        assert run().success, server.logger.error.call_args_list
    finally:
        entered.set()
        worker.join(timeout=3.)
    assert not worker.is_alive()
    assert transaction[0].successful
    controller.set_control_mode.assert_called_once_with('frame')
    assert controller.control_mode == 'frame'
    controller.set_control_mode.reset_mock()

    def next_endpoint(*args):
        assert server._controller_lock.locked()
        assert controller.control_mode == 'counter'
        return SimpleNamespace(success=True)

    if action == 'frame':
        server._direct_frame_task_callback = next_endpoint
    else:
        server._hold_frame_task_callback = next_endpoint
    assert run().success, server.logger.error.call_args_list
    controller.set_control_mode.assert_called_once_with('counter')
    assert server._saved_frame_plan is saved


@pytest.mark.parametrize('guard', ['config', 'dt', 'collision', 'stale', 'hash'])
def test_replay_guard_failures_abort_without_command_and_keep_cache(monkeypatch, guard):
    controller = comparison_controller(monkeypatch)
    server = server_for(controller)
    saved = save(server)
    if guard == 'config':
        controller.config['controller']['threshold_joint'] = .02
    elif guard == 'dt':
        controller.dt *= 2
    elif guard == 'collision':
        controller.robot_model.check_collision_free = lambda *args, **kwargs: False
    elif guard == 'stale':
        controller.check_state_fresh.side_effect = RuntimeError('stale state')
    else:
        object.__setattr__(saved.reference, 'checksum', 'invalid')
    assert not server.replay_frame_task_callback(goal()).success
    assert not controller.low_cmd_handler.calls
    assert server._saved_frame_plan is saved
    assert not server._controller_lock.locked()


def test_planning_only_restores_diagnostics_and_reduced_bindings_on_exception(monkeypatch):
    controller = comparison_controller(monkeypatch)
    ik = controller.ik_solver
    diagnostics = {name: object() for name in (
        'last_plan_result', 'last_planning_start', 'last_ik_result', 'last_plan_attempts',
    )}
    for name, value in diagnostics.items():
        setattr(controller, name, value)
    controller.__dict__.pop('last_planning_context', None)
    controller._reduced_rebuild_pending = False
    robot = controller.robot_model
    bindings = {name: getattr(robot, name, object()) for name in (
        'model_body_reduced', 'data_body_reduced', 'reduced_reference_q',
        'collision_model_body_reduced',
    )}
    for name, value in bindings.items():
        setattr(robot, name, value)
    robot.__dict__.pop('collision_data_body_reduced', None)
    failure = ValueError('planning failed')
    with pytest.raises(ValueError) as caught:
        with planning_only(controller):
            assert controller.ik_solver is not ik
            for name in (*diagnostics, 'last_planning_context'):
                setattr(controller, name, object())
            controller._reduced_rebuild_pending = True
            for name in (*bindings, 'collision_data_body_reduced'):
                setattr(robot, name, object())
            raise failure
    assert caught.value is failure
    assert controller.ik_solver is ik
    assert all(getattr(controller, name) is value for name, value in diagnostics.items())
    assert all(getattr(robot, name) is value for name, value in bindings.items())
    assert not controller._reduced_rebuild_pending
    assert not hasattr(controller, 'last_planning_context')
    assert not hasattr(robot, 'collision_data_body_reduced')
    assert not controller.low_cmd_handler.calls


def test_planning_only_restores_all_mutated_buffers_under_publisher_lock_before_error(monkeypatch):
    controller = comparison_controller(monkeypatch)
    handler = publisher_buffer_handler(controller, (np.float32, np.float64, np.float32))
    publisher = handler._command_publisher
    restored = []

    class GuardedBuffer(np.ndarray):
        def __setitem__(self, key, value):
            assert publisher.data_lock.locked()
            super().__setitem__(key, value)
            restored.append(id(self))

    for name in ('q', 'dq', 'tau'):
        setattr(publisher, name, getattr(publisher, name).view(GuardedBuffer))
    originals = {name: getattr(publisher, name).tobytes() for name in ('q', 'dq', 'tau')}
    with pytest.raises(RuntimeError, match='q_cmd, dq_cmd, tau_cmd'):
        with planning_only(controller):
            with publisher.data_lock:
                for name in originals:
                    getattr(publisher, name)[:] = 99.
            restored.clear()
    assert restored == [id(getattr(publisher, name)) for name in originals]
    assert all(getattr(publisher, name).tobytes() == value
               for name, value in originals.items())
    assert not publisher.data_lock.locked()
    handler.set_joint_commands.assert_not_called()


def test_planning_only_real_pin_pink_tasks_and_caches_are_isolated(monkeypatch):
    from h12_ros2_controller.core.planner.kinematic_pipeline import KinematicPipeline
    from h12_ros2_controller.core.planner.planner_process import PlannerClient
    from h12_ros2_controller.utility.controller_config import load_controller_config
    import h12_ros2_controller.core.planner.kinematic_pipeline as pipeline_module

    assets = Path(__file__).resolve().parents[4] / 'CL_Assets/ros_assets'
    monkeypatch.setattr(pipeline_module, 'resolve_model_paths', lambda: tuple(
        str(assets / name) for name in ('h1_2_magpie.urdf', 'h1_2_magpie_sphere.urdf',
                                      'h1_2_magpie_sphere_collision.srdf')))
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    ik = pipeline.ik_solver
    ik.add_frame_task('old', 'left_grasp_frame')
    controller = object.__new__(FrameController)
    controller.robot_model = pipeline.robot_model
    controller.ik_solver = ik
    controller._tau_bias = np.ones(27)
    controller.speed_scale = 1.
    controller.low_cmd_handler = SimpleNamespace(q_cmd=np.zeros(27), dq_cmd=np.zeros(27),
                                                tau_cmd=np.zeros(27))
    configuration = ik.configuration_reduced
    before = ik.q.copy()
    with planning_only(controller):
        assert controller.ik_solver is not ik
        assert controller.ik_solver.robot_model is pipeline.robot_model
        assert controller.ik_solver.configuration.model is ik.configuration.model
        controller.clear_frame_tasks()
        controller.ik_solver.update_configurations(np.ones(27) * .01)
        controller._tau_bias[:] = 8.
        assert list(ik.frame_tasks) == ['old']
        np.testing.assert_array_equal(ik.q, before)
    assert controller.ik_solver is ik
    assert ik.configuration_reduced is configuration
    np.testing.assert_array_equal(controller._tau_bias, 1.)
    controller.config = load_controller_config('sim_safety_split.yaml')
    controller.enabled_ids = list(range(13, 27))
    controller.max_state_lag_ms = 0.
    controller.dt = pipeline.planner_config.dt
    controller._prior_ik = None
    controller.low_cmd_handler.set_joint_commands = Mock(side_effect=AssertionError('motor write'))
    controller.planner = PlannerClient(*pipeline.model_paths, pipeline.planner_config, ompl_seed=42)
    try:
        with planning_only(controller):
            controller.clear_frame_tasks()
            controller.add_frame_task('left', 'left_grasp_frame',
                                      np.array([.4, .4, .3, 0., np.deg2rad(-30), 0.]))
            path = controller.plan_to_ik_target()
            reference = SavedFramePlan.capture(controller, path, ['left_grasp_frame'], [np.eye(4)])
        assert len(reference.reference.path) > 1
        assert controller.ik_solver is ik
        assert list(ik.frame_tasks) == ['old']
        np.testing.assert_array_equal(ik.q, before)
        np.testing.assert_array_equal(controller._tau_bias, 1.)
        controller.low_cmd_handler.set_joint_commands.assert_not_called()
        for field in ('q_cmd', 'dq_cmd', 'tau_cmd'):
            np.testing.assert_array_equal(getattr(controller.low_cmd_handler, field), 0.)
    finally:
        controller.planner.shutdown()
