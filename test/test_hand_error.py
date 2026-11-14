import numpy as np
import pytest

import h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller as adapter
from h12_ros2_controller.core.controller.counter_balance.hand_error import (
    HandErrorConfig,
    fold_hand_error,
    hand_momentum_map,
    parse_hand_error_config,
)
from test_counter_ddp_velocity_robust_controller import _robust


def _problem(seed=3):
    rng = np.random.default_rng(seed)
    return (
        rng.normal(size=(2, 4)), rng.normal(size=(2, 4)),
        rng.normal(size=2), rng.normal(size=2), rng.normal(size=4),
        -np.ones(4), np.ones(4),
    )


def test_hand_error_config_defaults_off_and_rejects_bad_values():
    assert parse_hand_error_config(None) == HandErrorConfig()
    assert parse_hand_error_config({}).weight == 0.0
    parsed = parse_hand_error_config({'weight': 2.0, 'velocity_scale': 0.05})
    assert (parsed.weight, parsed.velocity_scale) == (2.0, 0.05)
    for bad in ({'weight': -1.0}, {'velocity_scale': 0.0}, {'frame': 3}, []):
        with pytest.raises(ValueError):
            parse_hand_error_config(bad)


def test_pitch_momentum_moves_raised_hand_forward_by_lever_arm():
    inertia, height = 4.0, 1.2
    hand_map = hand_momentum_map(
        hand=[0.0, 0.0, height], pivot=np.zeros(3),
        com_inertia=inertia * np.eye(3), mass=0.0, com=np.zeros(3),
    )
    velocity = hand_map @ np.array([0.0, 2.0])
    np.testing.assert_allclose(velocity, [2.0 / inertia * height, 0.0, 0.0])


def test_parallel_axis_raises_pivot_inertia():
    near = hand_momentum_map([0, 0, 1.0], np.zeros(3), np.eye(3), 50.0, [0, 0, 0.0])
    far = hand_momentum_map([0, 0, 1.0], np.zeros(3), np.eye(3), 50.0, [0, 0, 1.0])
    assert np.linalg.norm(far) < np.linalg.norm(near)


def test_folded_rows_reproduce_both_weighted_terms():
    _, momentum, _, rhs, _, _, _ = _problem()
    hand_map = np.random.default_rng(5).normal(size=(3, 2))
    config = HandErrorConfig(weight=0.7, velocity_scale=0.1)
    rows, target = fold_hand_error(momentum, rhs, hand_map, 2.0, 1.5, config)
    for x in np.random.default_rng(7).normal(size=(5, 4)):
        residual = momentum @ x - rhs
        expected = (2.0 * np.sum((residual / 1.5) ** 2)
                    + 0.7 * np.sum((hand_map @ residual / 0.1) ** 2))
        assert np.isclose(np.sum((rows @ x - target) ** 2), expected)


def test_disabled_hand_error_leaves_robust_solve_unchanged():
    controller = _robust()
    args = (*_problem(), 1.0)
    reference = adapter.ScipyNominalMixin._isolated_velocity_solve(controller, *args)
    controller.hand_error = HandErrorConfig()
    controller._hand_map = np.ones((3, 2))
    solved = controller._isolated_velocity_solve(*args)
    np.testing.assert_array_equal(
        solved.requested_counter_dq, reference.requested_counter_dq,
    )


def test_enabled_hand_error_solves_folded_problem(monkeypatch):
    controller = _robust()
    controller.com_velocity_scale = 0.1
    controller.hand_error = HandErrorConfig(weight=1.0, velocity_scale=0.1)
    controller._hand_map = np.random.default_rng(9).normal(size=(3, 2))
    seen = {}

    def fake_solve(com, momentum, com_rhs, momentum_rhs, *rest):
        seen.update(momentum=momentum, rhs=momentum_rhs, config=rest[-1])
        return 'solved'

    monkeypatch.setattr(adapter, 'solve_scipy_nominal', fake_solve)
    com, momentum, com_rhs, rhs, posture, lower, upper = _problem()
    assert controller._isolated_velocity_solve(
        com, momentum, com_rhs, rhs, posture, lower, upper, 1.0,
    ) == 'solved'
    rows, target = fold_hand_error(
        momentum, rhs, controller._hand_map, controller.momentum_weight,
        controller.momentum_scale, controller.hand_error,
    )
    np.testing.assert_allclose(seen['momentum'], rows)
    np.testing.assert_allclose(seen['rhs'], target)
    assert (seen['config'].momentum_weight, seen['config'].momentum_scale) == (1.0, 1.0)
