from types import SimpleNamespace

import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.controller.zmp.balance_actuator import (
    BalanceActuator,
    format_vector,
    format_vector_map,
    format_status_map,
)
from h12_ros2_controller.core.controller.zmp_controller import ZmpController
from h12_ros2_controller.core.robot_dynamics import MomentumDDP
from h12_ros2_controller.core.controller.zmp.balance_observer import (
    BalanceState,
)
from h12_ros2_controller.core.controller.zmp.momentum_allocator import (
    ArmMomentumTarget,
)
from h12_ros2_controller.core.controller.zmp.momentum_target_estimator import (
    MomentumTargetEstimator,
)
from h12_ros2_controller.core.controller.zmp.perturbation_detector import (
    PerturbationState,
)


def make_actuator(config=None):
    if config is None:
        config = {
            'zmp': {
                'ddp': {},
                'blending': {},
                'solver_failure': {
                    'accept_best_effort': True,
                    'min_alignment': 0.2,
                    'min_useful_momentum': 0.1,
                },
            },
        }
    robot = SimpleNamespace(model_body=SimpleNamespace(nv=27))
    return BalanceActuator(robot, 0.02, config)


def make_plan(solved=False, us=None, peak_momentum=None):
    if us is None:
        us = np.ones((1, 7), dtype=np.float64)
    if peak_momentum is None:
        peak_momentum = np.array([0.5, 0.0, 0.0], dtype=np.float64)
    return SimpleNamespace(
        solved=solved,
        us=np.asarray(us, dtype=np.float64),
        peak_momentum=np.asarray(peak_momentum, dtype=np.float64),
    )


def test_accepts_aligned_unconverged_best_effort_plan():
    actuator = make_actuator()
    plan = make_plan(solved=False, peak_momentum=[0.5, 0.0, 0.0])

    accepted, status, achieved = actuator._evaluate_plan(
        plan,
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
    )

    assert accepted
    assert status == 'best_effort'
    assert np.allclose(achieved, [0.5, 0.0, 0.0])


def test_rejects_empty_nonfinite_and_opposite_plans():
    actuator = make_actuator()
    target = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    empty = make_plan(us=np.empty((0, 7)), peak_momentum=[0.5, 0.0, 0.0])
    nonfinite = make_plan(us=[[np.nan] * 7], peak_momentum=[0.5, 0.0, 0.0])
    opposite = make_plan(us=[[1.0] * 7], peak_momentum=[-0.5, 0.0, 0.0])

    assert actuator._evaluate_plan(empty, target)[1] == 'empty'
    assert actuator._evaluate_plan(nonfinite, target)[1] == (
        'nonfinite_velocity'
    )
    assert actuator._evaluate_plan(opposite, target)[1] == 'opposite'


def test_compact_format_helpers_do_not_emit_numpy_array_repr():
    assert format_vector([0.75, -0.75, 0.0]) == '[+0.75 -0.75 +0.00]'
    assert format_vector_map({'left': [0.5, -0.5, 0.0]}) == (
        '{left:[+0.50 -0.50 +0.00]}'
    )
    assert format_status_map({'left': 'best_effort'}) == '{left:best_effort}'


def test_suppressed_response_clears_stale_achieved_debug_state():
    actuator = make_actuator()
    actuator.latest_per_arm_achieved = {'left': np.ones(3)}
    actuator.latest_combined_achieved = np.ones(3)

    class FailingBehavior:
        def current_arm_q(self):
            return np.zeros(7, dtype=np.float64)

        def solve(self, target, q_ref=None):
            raise RuntimeError('fail')

    actuator._behavior = lambda arm: FailingBehavior()
    started = actuator.maybe_start_response(
        [ArmMomentumTarget('left', np.ones(3, dtype=np.float64))],
        PerturbationState(active=True, severity=1.0),
    )

    assert not started
    assert actuator.latest_response_status == 'suppressed'
    assert actuator.latest_per_arm_achieved == {}
    assert np.allclose(actuator.latest_combined_achieved, np.zeros(3))
    assert actuator.latest_solver_statuses == {'left': 'exception'}


def test_actuator_diagnostics_update_when_executing_mocked_plan():
    actuator = make_actuator()
    command = np.zeros(27, dtype=np.float64)
    command[13] = 2.0
    command[20] = -3.0

    class MockPlan:
        us = [np.ones(7, dtype=np.float64)]

        def body_velocity_at(self, index, nv):
            return np.copy(command)

    actuator.plans = {'left': MockPlan()}
    actuator.state = 'executing_impulse'

    result = actuator.step()

    assert np.allclose(result, command)
    assert actuator.plan_index == 1
    assert actuator.latest_raw_plan_command_norm == np.linalg.norm(command)


def test_direct_response_updates_each_tick_and_returns_after_perturbation():
    actuator = make_actuator({
        'zmp': {
            'execution': {
                'mode': 'direct',
                'direct_damping': 0.1,
                'direct_max_velocity': 2.0,
            },
            'ddp': {},
            'blending': {},
            'solver_failure': {},
        },
    })
    class DirectBehavior:
        arm_ids = np.arange(7)
        arm_model = SimpleNamespace(createData=lambda: object())

        def current_arm_q(self):
            return np.zeros(7, dtype=np.float64)

    actuator._behavior = lambda arm: DirectBehavior()
    actuator._current_arm_q = lambda arm: np.zeros(7, dtype=np.float64)
    target = ArmMomentumTarget(
        'left',
        np.array([0.5, 0.0, 0.0], dtype=np.float64),
    )
    active = PerturbationState(active=True, severity=1.0)
    inactive = PerturbationState(active=False, severity=0.0)

    original = pin.computeCentroidalMap
    pin.computeCentroidalMap = lambda model, data, q: np.vstack((
        np.zeros((3, 7)),
        np.eye(3, 7),
    ))
    try:
        actuator.update_direct_response([target], active)
        command = actuator.step()
        actuator.update_direct_response([], inactive)
    finally:
        pin.computeCentroidalMap = original

    assert actuator.state == 'returning'
    assert command[0] > 0.0
    assert np.max(np.abs(command)) <= 2.0
    assert actuator.latest_response_status == 'direct'


def test_zmp_controller_bypasses_velocity_limit_by_default():
    controller = ZmpController.__new__(ZmpController)
    controller.zmp_enabled = True
    controller.limit_ddp_velocity = False
    controller.dt = 0.02
    controller.robot_model = SimpleNamespace(
        model_body=SimpleNamespace(nv=27),
    )
    raw = np.ones(27, dtype=np.float64)
    controller.actuator = SimpleNamespace(
        step=lambda: np.copy(raw),
        state='executing_impulse',
        plan_index=3,
    )
    controller.update_robot_model = lambda: None
    controller.update_ik_solver = lambda: None
    controller._update_balance_state = lambda: None
    controller._limit_joint_vel = lambda command: (_ for _ in ()).throw(
        AssertionError('limiter should not be called')
    )
    applied = []

    def apply_velocity_command(command):
        applied.append(command)

    controller._apply_velocity_command = apply_velocity_command

    controller.control_step()

    assert len(applied) == 1
    assert np.allclose(applied[0], raw)
    assert controller.latest_raw_command_norm == np.linalg.norm(raw)
    assert controller.latest_applied_command_norm == np.linalg.norm(raw)


def test_zmp_controller_uses_velocity_limit_when_configured():
    controller = ZmpController.__new__(ZmpController)
    controller.zmp_enabled = True
    controller.limit_ddp_velocity = True
    controller.dt = 0.02
    controller.robot_model = SimpleNamespace(
        model_body=SimpleNamespace(nv=27),
    )
    raw = np.ones(27, dtype=np.float64)
    limited = 0.5 * raw
    controller.actuator = SimpleNamespace(
        step=lambda: np.copy(raw),
        state='executing_impulse',
        plan_index=3,
    )
    controller.update_robot_model = lambda: None
    controller.update_ik_solver = lambda: None
    controller._update_balance_state = lambda: None
    controller._limit_joint_vel = lambda command: np.copy(limited)
    applied = []

    def apply_velocity_command(command):
        applied.append(command)

    controller._apply_velocity_command = apply_velocity_command

    controller.control_step()

    assert len(applied) == 1
    assert np.allclose(applied[0], limited)
    assert controller.latest_raw_command_norm == np.linalg.norm(raw)
    assert controller.latest_applied_command_norm == np.linalg.norm(limited)


def test_momentum_ddp_phase_steps_allows_zero_hold():
    behavior = MomentumDDP.__new__(MomentumDDP)
    behavior.dt = 0.02
    behavior.hold_duration = 0.0
    behavior.momentum_duration = 0.24
    behavior.return_duration = 0.32

    assert behavior._phase_steps() == (0, 12, 16)


def test_target_estimator_applies_minimum_active_momentum_norm():
    estimator = MomentumTargetEstimator(
        SimpleNamespace(total_mass=10.0),
        {
            'zmp': {
                'gains': {
                    'zmp': [1.0, 1.0],
                    'center': [0.0, 0.0],
                    'com_velocity': [0.0, 0.0],
                    'angular_acceleration': [0.0, 0.0],
                },
                'target': {
                    'response_time': 0.1,
                    'min_momentum_norm': 0.3,
                    'max_momentum': [1.5, 1.5, 0.0],
                },
            },
        },
    )
    state = BalanceState(
        zmp=np.array([0.001, 0.0], dtype=np.float64),
        zmp_target=np.zeros(2, dtype=np.float64),
        zmp_error=np.array([0.001, 0.0], dtype=np.float64),
        support_margin=0.1,
        com_xy=np.zeros(2, dtype=np.float64),
        center_reference=np.zeros(2, dtype=np.float64),
        center_shift=np.zeros(2, dtype=np.float64),
        com_velocity=np.zeros(2, dtype=np.float64),
        angular_velocity=np.zeros(3, dtype=np.float64),
        angular_acceleration=np.zeros(3, dtype=np.float64),
        force_proxy=0.0,
    )

    target = estimator.estimate(
        state,
        PerturbationState(active=True, severity=1.0),
    )
    quiet_target = estimator.estimate(
        state,
        PerturbationState(active=False, severity=0.0),
    )

    assert np.isclose(np.linalg.norm(target), 0.3)
    assert target[1] > 0.0
    assert np.allclose(quiet_target, np.zeros(3))


def test_momentum_ddp_reads_phase_weight_config(monkeypatch):
    monkeypatch.setattr(
        MomentumDDP,
        '_arm_ids',
        staticmethod(lambda arm: [0]),
    )
    monkeypatch.setattr(
        MomentumDDP,
        '_build_arm_model',
        lambda self: SimpleNamespace(),
    )
    robot = SimpleNamespace(
        state={'q': np.zeros(27, dtype=np.float64)},
    )

    behavior = MomentumDDP(
        robot,
        dt=0.02,
        config={
            'momentum_ddp': {
                'hold_w_q': 11.0,
                'swing_w_q': 0.25,
                'return_w_q': 22.0,
                'terminal_w_q': 33.0,
                'hold_w_momentum': 4.0,
                'return_w_momentum': 5.0,
                'terminal_w_momentum': 6.0,
            },
        },
    )

    assert behavior.hold_w_q == 11.0
    assert behavior.swing_w_q == 0.25
    assert behavior.return_w_q == 22.0
    assert behavior.terminal_w_q == 33.0
    assert behavior.hold_w_momentum == 4.0
    assert behavior.return_w_momentum == 5.0
    assert behavior.terminal_w_momentum == 6.0
