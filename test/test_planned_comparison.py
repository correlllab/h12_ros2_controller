import time
import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from h12_ros2_controller.core.planner import PlannerConfig, PlanResult
from h12_ros2_controller.core.low_cmd_handler import LowCmdHandler
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.core.controller.planned_comparison import PlannedComparison
from h12_ros2_controller.core.controller.planned_reference import PlanReference
from h12_ros2_controller.core.controller.planned_reference import command_context_digest
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from test_counter_frame_controller import action_controller


def comparison_controller(monkeypatch):
    '''Use actual playback, Upper motor law and robust solve with one mock writer'''
    controller = action_controller()
    controller.enabled_ids = list(range(13, 27))
    controller.speed_scale = 1.0
    controller.timeout = 0.04
    controller.visualize = False
    controller._tau_bias = np.full(27, 0.125)
    controller._tau_bias_decay = 0.98
    controller.command_seed_max_gap = 0.1
    controller.execution_start_tolerance = 0.01
    controller.plan_max_age = 30.0
    controller.config.setdefault('controller', {})['threshold_joint'] = 0.01
    controller.config['gains'] = {'ki': np.full(27, 0.1)}
    controller.config['limits']['tau_clip_limits'] = np.full(27, 100.0)
    controller.low_cmd_handler._q_clip_limits = controller.config['limits']['q_clip_limits']
    controller.low_cmd_handler._clip_tau_cmd = lambda value: np.clip(value, -100.0, 100.0)
    controller.robot_model.model_body.names = ['universe'] + [f'joint_{i}' for i in range(27)]
    controller.robot_model.model_body.parents = np.arange(28)
    controller.robot_model.model_body_reduced = SimpleNamespace(
        nq=14, velocityLimit=np.full(14, 3.0),
        names=['universe'] + [f'joint_{i}' for i in range(13, 27)],
    )
    controller.planner = SimpleNamespace(
        planner_config=PlannerConfig(dt=controller.dt, joint_velocity_limit=3.0,
                                     joint_acceleration_limit=25.0),
        urdf_path='model', urdf_sphere_path='spheres', srdf_sphere_path='pairs',
    )
    controller._sync_reduced_model = Mock()
    controller.low_cmd_handler.q_cmd[13:] = NAMED_CONFIGS['prep']
    controller.robot_model.state['q'][13:] = NAMED_CONFIGS['prep']
    controller.robot_model.state_reduced = {'q': controller.robot_model.state['q'][13:]}

    class IK:
        def __init__(self):
            self.q = controller.robot_model.state['q'].copy()
            self.frame_tasks = {}
            self.posture_task = SimpleNamespace(set_target_from_configuration=Mock())
            self.config_task = SimpleNamespace(set_target=Mock())
            self.configuration_reduced = SimpleNamespace()
            self.rebuild_reduced = Mock()

        @property
        def q_reduced(self):
            return self.q[13:]

        def update_configurations(self, q):
            self.q = q.copy()

    controller.ik_solver = IK()
    controller.clear_frame_tasks = lambda: controller.ik_solver.frame_tasks.clear()
    controller.add_frame_task = lambda name, frame, target: (
        controller.ik_solver.frame_tasks.update({name: target.copy()}))
    controller.get_frame_task_error = lambda name: np.zeros(6)
    controller.get_frame_transformation = lambda frame, q: np.eye(4)
    controller.update_robot_model = lambda: controller.robot_model.state['q'].__setitem__(
        slice(12, None), controller.low_cmd_handler.q_cmd[12:],
    )
    controller.prepared_q = controller.low_cmd_handler.q_cmd[13:].copy()
    controller.planning_seeds = []

    def plan_configuration(target):
        np.testing.assert_array_equal(target, NAMED_CONFIGS['prep'])
        start = controller.low_cmd_handler.q_cmd[13:].copy()
        path = np.linspace(start, controller.prepared_q, 8)
        controller.last_plan_result = plan_result(controller, path)
        return path

    def plan_frame():
        controller.planning_seeds.append((controller.ik_solver.q.copy(),
                                          controller._tau_bias.copy()))
        path = np.tile(controller.prepared_q, (5, 1))
        path[:, 0] += [0, 0.001, 0.002, 0.002, 0.002]
        controller.last_plan_result = plan_result(controller, path)
        # planning-only mutations must never leak into the first trial
        controller._tau_bias[:] = 99.0
        controller.ik_solver.q[:] = 99.0
        controller._prior_ik = {'planning_only': True}
        return path

    controller.plan_to_configuration = Mock(side_effect=plan_configuration)
    controller.plan_to_ik_target = Mock(side_effect=plan_frame)
    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.time.sleep',
                        lambda seconds: None)
    return controller


def plan_result(controller, path, age=0):
    return PlanResult(success=True, path=path, metadata={'execution_context': {
        'planned_at': time.monotonic() - age, 'dt': controller.dt,
        'start': path[0].copy(), 'frozen': np.zeros(13), 'active_mask': None,
    }})


@pytest.mark.parametrize('mode', ['frame', 'counter'])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_actual_core_single_writer_and_exact_upper_moving_bytes(monkeypatch, mode, side):
    controller = comparison_controller(monkeypatch)
    baseline = comparison_controller(monkeypatch)
    baseline.__class__ = FrameController
    controller._set_arm_ownership(side)
    controller._comparison_mode = mode
    controller._comparison_playback = True
    controller._comparison_max_deviation = np.zeros(3)
    controller._comparison_audit = Mock()
    q = controller.low_cmd_handler.q_cmd.copy()
    controller._planned_previous = q[13:].copy()
    q[controller.moving_ids[:4]] += [0.001, -0.001, 0.001, -0.001]
    UpperController._apply_joint_position(baseline, q)
    UpperController._apply_joint_position(controller, q)
    assert len(controller.low_cmd_handler.calls) == 1
    assert len(baseline.low_cmd_handler.calls) == 1
    for field in ('q_cmd', 'dq_cmd', 'tau_cmd'):
        actual = getattr(controller.low_cmd_handler, field)
        nominal = getattr(baseline.low_cmd_handler, field)
        assert actual[controller.moving_ids].tobytes() == nominal[controller.moving_ids].tobytes()
    if mode == 'frame':
        assert controller.low_cmd_handler.q_cmd[13:].tobytes() == q[13:].tobytes()
    else:
        assert np.any(controller.low_cmd_handler.q_cmd[controller.counter_ids] !=
                      q[controller.counter_ids])
    np.testing.assert_array_equal(controller._comparison_max_deviation, 0)


def test_moving_torque_parity_includes_existing_upper_motor_clipping(monkeypatch):
    controller = comparison_controller(monkeypatch)
    baseline = comparison_controller(monkeypatch)
    baseline.__class__ = FrameController
    for item in (controller, baseline):
        writer = item.low_cmd_handler.set_joint_commands
        item.low_cmd_handler.set_joint_commands = (
            lambda q, dq, tau, joint_ids, writer=writer:
            writer(q=q, dq=dq, tau=np.clip(tau, -1.0, 1.0), joint_ids=joint_ids))
    controller.low_cmd_handler._clip_tau_cmd = lambda value: np.clip(value, -1.0, 1.0)
    controller._comparison_mode = 'frame'
    controller._comparison_playback = True
    controller._comparison_max_deviation = np.zeros(3)
    controller._comparison_audit = None
    q = controller.low_cmd_handler.q_cmd.copy()
    baseline._apply_joint_position(q)
    controller._apply_joint_position(q)
    assert controller.low_cmd_handler.tau_cmd[13:].tobytes() == baseline.low_cmd_handler.tau_cmd[13:].tobytes()


def publisher_buffer_handler(controller, dtypes):
    '''Use real clipping, writes and property reads without constructing any endpoint'''
    handler = object.__new__(LowCmdHandler)
    previous = controller.low_cmd_handler
    handler._command_publisher = SimpleNamespace(data_lock=threading.Lock(), **{
        name: np.asarray(getattr(previous, f'{name}_cmd'), dtype=dtype).copy()
        for name, dtype in zip(('q', 'dq', 'tau'), dtypes)
    })
    handler._q_clip_limits = previous._q_clip_limits.copy()
    handler._dq_clip_limits = np.full(27, 100.0)
    handler._tau_clip_limits = np.full(27, 100.0)
    handler._recording = False
    handler.set_joint_commands = Mock(wraps=handler.set_joint_commands)
    controller.low_cmd_handler = handler
    return handler


@pytest.mark.parametrize('mode', ['frame', 'counter'])
@pytest.mark.parametrize('dtypes', [
    (np.float32, np.float32, np.float32), (np.float64, np.float32, np.float32),
    (np.float32, np.float64, np.float64),
])
def test_publisher_dtype_parity_matches_upper_exactly(monkeypatch, mode, dtypes):
    controller = comparison_controller(monkeypatch)
    baseline = comparison_controller(monkeypatch)
    handler = publisher_buffer_handler(controller, dtypes)
    baseline_handler = publisher_buffer_handler(baseline, dtypes)
    controller._comparison_mode = mode
    controller._comparison_playback = True
    controller._comparison_max_deviation = np.zeros(3)
    controller._comparison_audit = Mock()
    q = handler.q_cmd.astype(np.float64)
    controller._planned_previous = q[13:].copy()
    q[13] += 0.00123456789
    dq = np.full(27, 0.0123456789)
    tau = np.full(27, 0.123456789)
    assert np.float64(np.float32(tau[13])) != tau[13]
    UpperController._publish_position_command(baseline, q.copy(), dq.copy(), tau.copy())
    controller._publish_position_command(q, dq, tau)
    handler.set_joint_commands.assert_called_once()
    audit = controller._comparison_audit.call_args.args[0]
    for index, (field, dtype) in enumerate(zip(('q_cmd', 'dq_cmd', 'tau_cmd'), dtypes)):
        emitted = getattr(handler, field)[controller.moving_ids]
        expected = getattr(baseline_handler, field)[controller.moving_ids]
        assert emitted.dtype == np.dtype(dtype)
        assert emitted.tobytes() == expected.tobytes()
        np.testing.assert_array_equal(audit['moving_nominal'][index], expected)
        np.testing.assert_array_equal(audit['moving_emitted'][index], expected)
    assert audit['moving_max_deviation'] == [0.0, 0.0, 0.0]


@pytest.mark.parametrize('field', ['q', 'dq', 'tau'])
def test_postpublication_moving_change_above_float32_ulp_still_fails(monkeypatch, field):
    controller = comparison_controller(monkeypatch)
    handler = publisher_buffer_handler(controller, (np.float32,) * 3)
    controller._comparison_mode = 'frame'
    controller._comparison_playback = False
    controller._comparison_max_deviation = np.zeros(3)
    controller._comparison_audit = None
    write = handler.set_joint_commands

    def corrupted_write(*args, **kwargs):
        write(*args, **kwargs)
        buffer = getattr(handler._command_publisher, field)
        for _ in range(2):
            buffer[controller.moving_ids[0]] = np.nextafter(
                buffer[controller.moving_ids[0]], np.float32(np.inf),
            )

    handler.set_joint_commands = corrupted_write
    with pytest.raises(ValueError, match='Emitted moving-arm command differs'):
        controller._publish_position_command(
            handler.q_cmd.astype(np.float64), np.full(27, 0.0123456789),
            np.full(27, 0.123456789),
        )
    assert controller._comparison_max_deviation[['q', 'dq', 'tau'].index(field)] > 0.0


@pytest.mark.parametrize('field', range(3))
def test_sub_float32_ulp_merged_moving_change_rejected_before_write(monkeypatch, field):
    controller = comparison_controller(monkeypatch)
    handler = publisher_buffer_handler(controller, (np.float32,) * 3)
    controller._comparison_mode = 'frame'
    controller._comparison_playback = False
    nominal = (handler.q_cmd.astype(np.float64), np.full(27, 0.0123456789),
               np.full(27, 0.123456789))
    combined = tuple(value.copy() for value in nominal)
    index = controller.moving_ids[0]
    before = combined[field][index]
    combined[field][index] = np.nextafter(before, np.inf)
    assert combined[field][index] != before
    assert np.float32(combined[field][index]) == np.float32(before)
    with pytest.raises(ValueError, match='Combined moving-arm command differs'):
        controller._publish_comparison_command(nominal, combined)
    handler.set_joint_commands.assert_not_called()


@pytest.mark.parametrize('order', [
    ('frame', 'counter'), ('counter', 'frame'),
    ('frame', 'counter', 'counter', 'frame'), ('counter', 'counter'), ('frame', 'frame'),
])
def test_batch_plan_once_same_hash_and_repeated_resets(monkeypatch, order):
    controller = comparison_controller(monkeypatch)
    references, seeds, events = [], [], []
    capture = controller.capture_reference
    controller._capture_reference = Mock(wraps=controller._capture_reference)

    def capture_reference():
        assert not controller._reference_captured
        assert controller._counter_q_command is None
        assert controller.q_counter_ref is None
        assert controller._latched_activation == 0.0
        assert controller._prior_ik is None
        seeds.append((controller.ik_solver.q.copy(), controller._tau_bias.copy()))
        capture()
        references.append(controller.q_counter_ref)

    controller.capture_reference = capture_reference
    summary = PlannedComparison(controller).run('left_grasp_frame', np.eye(4), order, event=events.append)
    assert summary['success'], summary
    assert [trial['condition'] for trial in summary['trials']] == list(order)
    assert len({trial['hash'] for trial in summary['trials']}) == 1
    assert summary['reference']['joint_order'] == [f'joint_{i}' for i in range(13, 27)]
    assert controller.plan_to_ik_target.call_count == 1
    assert controller.plan_to_configuration.call_count == len(order)
    assert len(references) == len(order)
    assert controller._capture_reference.call_count == len(order)
    assert len({id(reference) for reference in references}) == len(order)
    for seed, bias in seeds:
        np.testing.assert_array_equal(seed[13:], controller.prepared_q)
        np.testing.assert_array_equal(bias, 0)
    assert all(trial['moving_max_deviation'] == [0.0, 0.0, 0.0]
               for trial in summary['trials'])
    assert any(item['type'] == 'command' for item in events)
    assert any(item['type'] == 'step' and 'overrun' in item for item in events)
    assert controller._comparison_mode is None
    assert controller.speed_scale == 1.0


def test_reference_immutable_fresh_validation_preserves_original_age(monkeypatch):
    controller = comparison_controller(monkeypatch)
    path = np.tile(controller.prepared_q, (3, 1))
    controller.last_plan_result = plan_result(controller, path, age=90)
    reference = PlanReference.capture(controller, path)
    created = reference.created_at
    with pytest.raises(ValueError, match='too old'):
        UpperController.execute_path(controller, path)
    with pytest.raises(ValueError):
        reference.path.setflags(write=True)
    validation = controller.revalidate_plan(reference)
    assert validation.validated_at > created + 89
    assert reference.created_at == created
    controller._comparison_mode = 'frame'
    controller._comparison_playback = False
    controller._comparison_max_deviation = np.zeros(3)
    controller._comparison_audit = None
    controller.last_plan_result = plan_result(controller, path + 0.001)
    controller.execute_path(reference.path, validation=validation)
    assert reference.created_at == created


def test_reference_accepts_rebuilt_reduced_identity_but_rejects_topology_change(monkeypatch):
    controller = comparison_controller(monkeypatch)
    path = np.tile(controller.prepared_q, (3, 1))
    controller.last_plan_result = plan_result(controller, path)
    reference = PlanReference.capture(controller, path)
    previous = controller.robot_model.model_body_reduced
    controller.robot_model.model_body_reduced = SimpleNamespace(**previous.__dict__)
    controller.robot_model.state['q'][0] += 0.01
    assert command_context_digest(controller) == reference.config_digest
    controller.revalidate_plan(reference)
    controller.robot_model.model_body.names[1] = 'different_joint'
    with pytest.raises(ValueError, match='model/configuration'):
        controller.revalidate_plan(reference)


@pytest.mark.parametrize('fault', ['start', 'config', 'path', 'dt', 'collision', 'limits'])
def test_reference_rejects_changed_context_without_publication(monkeypatch, fault):
    controller = comparison_controller(monkeypatch)
    path = np.tile(controller.prepared_q, (3, 1))
    controller.last_plan_result = plan_result(controller, path)
    reference = PlanReference.capture(controller, path)
    if fault == 'start':
        controller.low_cmd_handler.q_cmd[13] += 0.05
    elif fault == 'config':
        controller.config['changed'] = True
    elif fault == 'path':
        reference = type(reference)(reference.path + 0.001, reference.created_at,
                                    reference.dt, reference.config_digest, reference.checksum)
    elif fault == 'dt':
        controller.dt *= 2
    elif fault == 'collision':
        controller.robot_model.check_collision_free = lambda *args, **kwargs: False
    else:
        controller.robot_model.check_within_limits_reduced = lambda *args: False
    with pytest.raises(ValueError):
        controller.revalidate_plan(reference)
    assert not controller.low_cmd_handler.calls


def test_actual_execution_rechecks_start_after_ticket_and_measured_frozen_joints(monkeypatch):
    controller = comparison_controller(monkeypatch)
    path = np.tile(controller.prepared_q, (3, 1))
    controller.last_plan_result = plan_result(controller, path)
    reference = PlanReference.capture(controller, path)
    ticket = controller.revalidate_plan(reference)
    controller.low_cmd_handler.q_cmd[13] += 0.05
    with pytest.raises(ValueError, match='drifted'):
        UpperController.execute_path(controller, path, validation=ticket)
    assert not controller.low_cmd_handler.calls
    controller.low_cmd_handler.q_cmd[13] -= 0.05
    checks = []
    controller.robot_model.state['q'][0] += 0.03
    controller.robot_model.check_collision_free = lambda q, **kwargs: checks.append(q.copy()) or True
    UpperController.execute_path(controller, path, validation=ticket)
    assert checks[0][0] == controller.robot_model.state['q'][0]


@pytest.mark.parametrize('fault', ['moving_fallback', 'invalid_merge', 'solver_failure'])
def test_invalid_counter_never_writes_merged_command(monkeypatch, fault):
    controller = comparison_controller(monkeypatch)
    controller._comparison_mode = 'counter'
    controller._comparison_playback = True
    controller._comparison_max_deviation = np.zeros(3)
    controller._comparison_audit = None
    q = controller.low_cmd_handler.q_cmd.copy()
    controller._planned_previous = q[13:].copy()
    q[13] += 0.001
    if fault == 'invalid_merge':
        controller.robot_model.check_collision_free = lambda *args, **kwargs: False
    elif fault == 'solver_failure':
        controller._plan_counter_velocity = Mock(side_effect=RuntimeError('failed'))
    else:
        original = controller._write_arm_command

        def changed(arm_q, arm_dq, tau):
            arm_q[controller.moving_local[0]] += 0.001
            return original(arm_q, arm_dq, tau)

        controller._write_arm_command = changed
    with pytest.raises((ValueError, RuntimeError)):
        controller._apply_joint_position(q)
    assert not controller.low_cmd_handler.calls


def test_cancel_stops_batch_holds_last_without_return(monkeypatch):
    controller = comparison_controller(monkeypatch)
    cancel = [False]
    commands = []

    def event(record):
        if record['type'] == 'command':
            commands.append(controller.low_cmd_handler.q_cmd.copy())
            cancel[0] = True

    result = PlannedComparison(controller).run(
        'left_grasp_frame', np.eye(4), ('counter', 'frame'),
        cancel_check=lambda: cancel[0], event=event,
    )
    assert not result['success'] and result['cancelled']
    assert len(result['trials']) == 1
    assert controller.plan_to_configuration.call_count == 1
    np.testing.assert_array_equal(controller.low_cmd_handler.q_cmd, commands[-1])
    assert not np.any(controller.low_cmd_handler.dq_cmd)
    assert controller._comparison_mode is None
    assert controller.planned_balance is False


def test_cancel_during_counter_solve_checks_before_merged_publication(monkeypatch):
    controller = comparison_controller(monkeypatch)
    cancel = [False]
    solve = controller._isolated_velocity_solve

    def request_cancel(*args):
        result = solve(*args)
        cancel[0] = True
        return result

    controller._isolated_velocity_solve = request_cancel
    events = []
    result = PlannedComparison(controller).run(
        'left_grasp_frame', np.eye(4), ('counter', 'frame'),
        cancel_check=lambda: cancel[0], event=events.append,
    )
    assert result['cancelled'] and not result['success']
    assert not any(record['type'] == 'command' for record in events)
    np.testing.assert_array_equal(controller.low_cmd_handler.q_cmd[13:], controller.prepared_q)
    assert controller.plan_to_configuration.call_count == 1


def test_relative_target_resolved_only_once_after_preparation(monkeypatch):
    controller = comparison_controller(monkeypatch)
    pose = np.eye(4)
    pose[:3, 3] = [0.4, 0.2, 0.3]
    getter = Mock(return_value=pose)
    controller.get_frame_transformation = getter
    delta = np.eye(4)
    delta[:3, 3] = [0.01, 0.02, -0.03]
    result = PlannedComparison(controller).run(
        'left_grasp_frame', delta, ('frame', 'frame'), relative=True,
    )
    assert result['success'], result
    getter.assert_called_once()
    np.testing.assert_array_equal(np.asarray(result['reference']['target'])[:3, 3],
                                  pose[:3, 3] + delta[:3, 3])


def test_overall_budget_stops_after_planning_without_main_motion(monkeypatch):
    controller = comparison_controller(monkeypatch)
    clock = [10.0]
    monkeypatch.setattr('h12_ros2_controller.core.controller.planned_comparison.time.monotonic',
                        lambda: clock[0])
    plan = controller.plan_to_ik_target.side_effect

    def delayed_plan():
        result = plan()
        clock[0] += 301.0
        return result

    controller.plan_to_ik_target.side_effect = delayed_plan
    events = []
    result = PlannedComparison(controller).run(
        'left_grasp_frame', np.eye(4), ('frame', 'counter'), event=events.append,
    )
    assert not result['success'] and not result['cancelled']
    assert 'overall wall budget' in result['reason']
    assert not any(record['type'] == 'command' for record in events)
    assert controller.plan_to_configuration.call_count == 1


def test_failed_endpoint_stops_order_and_reports_failure_fields(monkeypatch):
    controller = comparison_controller(monkeypatch)
    controller.get_frame_task_error = lambda name: np.ones(6)
    result = PlannedComparison(controller).run(
        'left_grasp_frame', np.eye(4), ('frame', 'counter'),
    )
    assert not result['success'] and len(result['trials']) == 1
    trial = result['trials'][0]
    assert trial['linear_error'] > 0 and trial['angular_error'] > 0
    assert trial['planned_path_checksum'] == result['hash']
    assert trial['config_digest'] == result['reference']['config_digest']
    assert trial['moving_max_deviation'] == [0.0, 0.0, 0.0]


def test_nonfinite_endpoint_failure_summary_and_events_are_json_safe(monkeypatch):
    controller = comparison_controller(monkeypatch)
    controller.get_frame_task_error = lambda name: np.full(6, np.nan)
    events = []
    result = PlannedComparison(controller).run(
        'left_grasp_frame', np.eye(4), ('frame', 'counter'), event=events.append,
    )
    assert not result['success']
    assert result['trials'][0]['linear_error'] is None
    json.dumps(result, allow_nan=False)
    json.dumps(events, allow_nan=False)


@pytest.mark.parametrize('threshold, blocked, expected', [
    (0.01, False, True), (0.004, False, False), (0.01, True, False),
])
def test_preparation_matches_named_arm_threshold_target_and_limit_guard(
        monkeypatch, threshold, blocked, expected):
    controller = comparison_controller(monkeypatch)
    controller.config['controller']['threshold_joint'] = threshold
    controller.low_cmd_handler.q_cmd[13] += 0.004
    initial = controller.low_cmd_handler.q_cmd[13:].copy()
    measured = controller.robot_model.state['q']

    def update():
        measured[12:] = controller.low_cmd_handler.q_cmd[12:]
        measured[12] += 0.011
        measured[13:] += 0.005

    controller.update_robot_model = update
    update()
    if blocked:
        controller._steady_state_limited_joints = lambda *args: [('torso_joint', 0.0, 0.0)]
    initialize = controller.init_steady_state
    preparation_targets = []

    def init_steady():
        if controller._comparison_mode is None:
            target = controller.ik_solver.config_task.set_target.call_args.args[0]
            np.testing.assert_array_equal(target, NAMED_CONFIGS['prep'])
            assert not np.array_equal(target, initial)
            preparation_targets.append(target.copy())
        initialize()

    controller.init_steady_state = init_steady
    steady = controller.steady_state_step
    prep_results, main_results = [], []

    def steady_step():
        result = steady()
        if controller._comparison_mode is None:
            # this is the planned named handler's actual arm-only success metric
            named_error = float(np.max(np.abs(
                controller.robot_model.state_reduced['q'] - NAMED_CONFIGS['prep']
            )))
            prep_results.append((result, named_error))
        else:
            main_results.append(result)
        return result

    controller.steady_state_step = steady_step
    result = PlannedComparison(controller).run('left_grasp_frame', np.eye(4), ('frame',))
    assert result['success'] is expected, result
    assert len(preparation_targets) == 1
    assert prep_results and all(not boolean for boolean, _ in prep_results)
    assert all((error < threshold) is (threshold == 0.01) for _, error in prep_results)
    if expected:
        assert len(prep_results) == 1
        assert len(main_results) == int(np.ceil(controller.timeout / controller.dt))
    else:
        assert not main_results
        assert ('position limit' if blocked else 'did not settle') in result['reason']
