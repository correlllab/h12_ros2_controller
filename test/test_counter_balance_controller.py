from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance_controller import (
    CounterBalanceController,
)
from h12_ros2_controller.core.ik_solver import IKSolver


def test_free_arm_is_fixed_and_validated_before_initialization():
    with pytest.raises(ValueError, match='free_arm'):
        CounterBalanceController('', '', '', free_arm='both')


def test_frame_ownership_accepts_only_nonempty_free_arm_support():
    controller = CounterBalanceController.__new__(CounterBalanceController)
    frames = [object(), object(), object(), object()]
    controller.robot_model = SimpleNamespace(
        model_body_reduced=SimpleNamespace(
            frames=frames,
            getFrameId=lambda name: {
                'free': 0,
                'counter': 1,
                'mixed': 2,
                'torso': 3,
            }.get(name, len(frames)),
        ),
    )
    controller._free_reduced_mask = np.array([True, True, False, False])
    masks = {
        'free': np.array([True, False, False, False]),
        'counter': np.array([False, False, True, False]),
        'mixed': np.array([True, False, True, False]),
        'torso': np.zeros(4, dtype=bool),
    }
    controller._reduced_support_mask = lambda name: masks[name]

    controller._validate_frame_ownership('free')
    with pytest.raises(ValueError, match='free arm'):
        controller._validate_frame_ownership('torso')
    with pytest.raises(ValueError, match='ownership'):
        controller._validate_frame_ownership('counter')
    with pytest.raises(ValueError, match='ownership'):
        controller._validate_frame_ownership('mixed')
    with pytest.raises(ValueError, match='Unknown'):
        controller._validate_frame_ownership('unknown')


def test_frame_target_is_validated_before_task_mutation():
    controller = CounterBalanceController.__new__(CounterBalanceController)
    calls = []
    controller._validate_frame_ownership = lambda frame: None
    controller._reference_captured = True
    controller.ik_solver = SimpleNamespace(
        add_frame_task=lambda *args, **kwargs: calls.append((args, kwargs)),
    )

    with pytest.raises(ValueError, match='Target'):
        controller.add_frame_task('task', 'frame', target=np.zeros(3))

    assert calls == []


def test_counter_lock_selects_all_seven_reserved_reduced_joints():
    controller = CounterBalanceController.__new__(CounterBalanceController)
    controller.counter_arm_ids = list(range(20, 27))
    controller.robot_model = SimpleNamespace(
        reduced_mask=np.array([False] * 13 + [True] * 14),
        model_body_reduced=SimpleNamespace(nv=14),
        state_reduced={'q': np.arange(14, dtype=np.float64)},
    )
    controller._counter_reduced_mask = controller._motor_ids_to_reduced_mask(
        controller.counter_arm_ids
    )

    lock = controller._make_counter_lock()

    assert lock.A.shape == (7, 14)
    assert np.allclose(lock.A[:, :7], 0.0)
    assert np.allclose(lock.A[:, 7:], np.eye(7))
    assert np.allclose(lock.b, 0.0)
    assert np.allclose(lock.q_0, np.arange(14))


def test_reduced_ik_threads_hard_constraints_to_pink(monkeypatch):
    captured = {}
    solver = IKSolver.__new__(IKSolver)
    solver.configuration_reduced = object()
    solver.solver = 'test_solver'
    solver.limits_reduced = ['limit']
    solver.collision_barrier_reduced = 'barrier'

    def fake_solve_ik(*args, **kwargs):
        captured.update(kwargs)
        return np.zeros(2)

    monkeypatch.setattr('pink.solve_ik', fake_solve_ik)
    constraint = object()

    solver._ik_reduced(['task'], 0.02, constraints=[constraint])

    assert captured['constraints'] == [constraint]
    assert captured['barriers'] == ['barrier']


def test_counter_backtracking_changes_only_counter_joints():
    controller = CounterBalanceController.__new__(CounterBalanceController)
    controller.counter_ids = list(range(20, 24))
    controller.counter_wrist_ids = list(range(24, 27))
    controller.backtrack_scales = [1.0, 0.5, 0.0]
    controller._integrate_candidate = lambda command: np.copy(command)
    controller._candidate_valid = lambda candidate: bool(
        np.max(np.abs(candidate[controller.counter_ids])) <= 0.6
    )
    free = np.zeros(27)
    free[13:20] = np.arange(7) * 0.1

    command, _, scale = controller._backtrack_counter(
        free,
        np.ones(4),
    )

    assert scale == 0.5
    assert np.allclose(command[13:20], free[13:20])
    assert np.allclose(command[20:24], 0.5)
    assert np.allclose(command[24:27], 0.0)


def test_publication_merges_both_arms_atomically_with_q_dq_tau():
    captured = []
    controller = CounterBalanceController.__new__(CounterBalanceController)
    controller.arm_ids = list(range(13, 27))
    controller.counter_ids = list(range(20, 24))
    controller.latest_published = False
    controller.latest_status = 'solved'
    controller.latest_com_target = None
    controller.latest_applied_counter_command = np.zeros(4)
    controller.low_cmd_handler = SimpleNamespace(
        _estopped=False,
        q_cmd=np.zeros(27),
        dq_cmd=np.copy(np.linspace(-0.2, 0.2, 27)),
        set_joint_commands=lambda **kwargs: captured.append(kwargs),
    )
    controller.robot_model = SimpleNamespace(
        state={'q': np.zeros(27)},
        dynamics=SimpleNamespace(
            get_gravity_compensation=lambda q: np.arange(27, dtype=float),
        ),
    )
    command = np.linspace(-0.2, 0.2, 27)
    candidate = np.linspace(-1.0, 1.0, 27)

    assert controller._publish_command(command, candidate)

    assert len(captured) == 1
    assert captured[0]['joint_ids'] == list(range(13, 27))
    assert np.allclose(captured[0]['q'], candidate[13:27])
    assert np.allclose(captured[0]['dq'], command[13:27])
    assert np.allclose(captured[0]['tau'], np.arange(13.0, 27.0))
    assert controller.latest_published
    assert np.allclose(
        controller.latest_applied_counter_command,
        command[20:24],
    )


def test_idle_step_publishes_zero_velocity_hold():
    controller = CounterBalanceController.__new__(CounterBalanceController)
    controller.latest_plan = None
    controller.latest_support = None
    controller.latest_free_command = np.zeros(7)
    controller.latest_counter_pre_limit = np.zeros(4)
    controller.latest_counter_command = np.zeros(4)
    controller.latest_applied_counter_command = np.ones(4)
    controller.latest_com = None
    controller.latest_com_target = None
    controller.latest_applied_com_error = None
    controller.latest_backtrack_scale = 0.0
    controller.latest_clipped = False
    controller.latest_collision_rejection = False
    controller.latest_published = False
    controller.latest_status = 'solved'
    controller.low_cmd_handler = SimpleNamespace(_estopped=False)
    controller.ik_solver = SimpleNamespace(frame_tasks={})
    controller.robot_model = SimpleNamespace(model_body=SimpleNamespace(nv=27))
    calls = []
    controller._publish_hold = lambda: calls.append('hold')
    controller._clear_reference = lambda: calls.append('clear')

    command = controller.control_step()

    assert calls == ['hold', 'clear']
    assert np.allclose(command, 0.0)
    assert controller.latest_status == 'idle'


def test_estop_suppresses_all_publication():
    controller = CounterBalanceController.__new__(CounterBalanceController)
    controller.arm_ids = list(range(13, 27))
    controller.latest_published = False
    controller.latest_status = 'solved'
    controller.low_cmd_handler = SimpleNamespace(
        _estopped=True,
        set_joint_commands=lambda **kwargs: pytest.fail('published after estop'),
    )
    controller.robot_model = SimpleNamespace()

    assert not controller._publish_command(np.zeros(27), np.zeros(27))
    assert controller.latest_status == 'estopped'
    assert not controller.latest_published


@pytest.mark.parametrize(
    ('plan', 'expected'),
    (
        (SimpleNamespace(status='invalid_output'), 'invalid_output'),
        (
            SimpleNamespace(
                status='best_effort',
                zero_com_error=0.01,
                optimized_com_error=0.02,
            ),
            'no_improvement',
        ),
    ),
)
def test_solver_failures_continue_free_arm_motion(plan, expected):
    published = []
    controller = _control_harness(plan, published)

    command = controller.control_step()

    assert controller.latest_status == expected
    assert np.allclose(command[13:20], 0.1)
    assert np.allclose(command[20:27], 0.0)
    assert len(published) == 1


@pytest.mark.parametrize(
    ('command', 'lower', 'upper', 'expected'),
    (
        (np.full(4, np.nan), -np.ones(4), np.ones(4), 'invalid_output'),
        (np.ones(4), -0.5 * np.ones(4), 0.5 * np.ones(4),
         'output_out_of_bounds'),
    ),
)
def test_invalid_solver_commands_continue_free_arm_motion(
        command, lower, upper, expected):
    plan = SimpleNamespace(
        status='best_effort',
        zero_com_error=0.02,
        optimized_com_error=0.01,
        command=command,
        control_lower=lower,
        control_upper=upper,
    )
    published = []
    controller = _control_harness(plan, published)

    result = controller.control_step()

    assert controller.latest_status == expected
    assert np.allclose(result[13:20], 0.1)
    assert np.allclose(result[20:27], 0.0)
    assert len(published) == 1


def _control_harness(plan, published):
    controller = CounterBalanceController.__new__(CounterBalanceController)
    controller.free_arm = 'left'
    controller.counter_arm = 'right'
    controller.free_ids = list(range(13, 20))
    controller.counter_arm_ids = list(range(20, 27))
    controller.counter_ids = list(range(20, 24))
    controller.counter_wrist_ids = list(range(24, 27))
    controller.arm_ids = list(range(13, 27))
    controller.improvement_tolerance = 1e-6
    controller._reference_captured = True
    controller.q_counter_ref = np.zeros(4)
    controller.counter_wrist_ref = np.zeros(3)
    controller.com_offset_ref = np.zeros(2)
    controller.latest_plan = None
    controller.latest_support = None
    controller.latest_status = 'idle'
    controller.latest_free_command = np.zeros(7)
    controller.latest_counter_pre_limit = np.zeros(4)
    controller.latest_counter_command = np.zeros(4)
    controller.latest_applied_counter_command = np.zeros(4)
    controller.latest_com = None
    controller.latest_com_target = None
    controller.latest_applied_com_error = None
    controller.latest_backtrack_scale = 0.0
    controller.latest_clipped = False
    controller.latest_collision_rejection = False
    controller.latest_published = False
    controller.solver_count = 0
    controller.solver_failures = 0
    controller.low_cmd_handler = SimpleNamespace(_estopped=False)
    controller.ik_solver = SimpleNamespace(frame_tasks={'task': object()})
    controller.robot_model = SimpleNamespace(
        model_body=SimpleNamespace(nv=27, nq=27),
        state={'q': np.zeros(27)},
        get_com=lambda: np.array([0.0, 0.0, 1.0]),
        full_q=lambda q: np.concatenate([np.zeros(7), q]),
    )
    support = SimpleNamespace(
        valid=True,
        center=np.zeros(2),
        invalid_reason='',
    )
    controller.update_robot_model = lambda: None
    controller.update_ik_solver = lambda: None
    controller._refresh_counter_lock = lambda: None
    controller._support_rectangle = lambda: support
    free = np.zeros(27)
    free[13:20] = 0.1
    controller._free_arm_command = lambda: np.copy(free)
    controller._integrate_candidate = lambda command: np.copy(command)
    controller._candidate_valid = lambda candidate: True
    controller.counter_balance_ddp = SimpleNamespace(solve=lambda *args, **kwargs: plan)
    controller._publish_command = lambda command, candidate: published.append(
        (np.copy(command), np.copy(candidate))
    )
    controller._publish_hold = lambda: published.append(('hold', None))
    return controller
