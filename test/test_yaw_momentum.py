import numpy as np
import pytest

import h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller as adapter
from h12_ros2_controller.core.controller.counter_balance.yaw_momentum import (
    YawMomentumConfig,
    parse_yaw_momentum_config,
    yaw_momentum_row,
)
from h12_ros2_controller.core.controller.counter_balance.scipy_nominal_planner import (
    solve_scipy_nominal,
)
from test_counter_ddp_velocity_robust_controller import _robust


def _problem(seed=3):
    rng = np.random.default_rng(seed)
    return (
        rng.normal(size=(2, 4)), rng.normal(size=(2, 4)),
        rng.normal(size=2), rng.normal(size=2), rng.normal(size=4),
        -np.ones(4), np.ones(4),
    )


def test_yaw_config_defaults_off_and_rejects_bad_values():
    assert parse_yaw_momentum_config(None) == YawMomentumConfig()
    parsed = parse_yaw_momentum_config({'weight': 1.5, 'scale': 0.2, 'gyro_gain': 0.1})
    assert (parsed.weight, parsed.scale, parsed.gyro_gain) == (1.5, 0.2, 0.1)
    for bad in ({'weight': -1.0}, {'scale': 0.0}, {'gyro_gain': np.nan}, []):
        with pytest.raises(ValueError):
            parse_yaw_momentum_config(bad)


def test_yaw_row_cancels_moving_arm_yaw_momentum():
    config = YawMomentumConfig(weight=4.0, scale=0.5)
    counter = np.array([1.0, 0.0, 0.0, 0.0])
    row, target = yaw_momentum_row(counter, [2.0, 0, 0, 0, 0, 0, 0], [0.3, 0, 0, 0, 0, 0, 0],
                                   0.0, 1.0, config, default_scale=9.0)
    np.testing.assert_allclose(row, [[2.0 / 0.5, 0, 0, 0]])
    # residual zero when counter yaw momentum equals minus the moving arm's
    assert np.isclose((row @ np.array([-0.6, 0, 0, 0]))[0], target[0])


def test_extra_rows_change_scipy_solution():
    com, momentum, com_rhs, rhs, posture, lower, upper = _problem()
    config = adapter.ScipyNominalConfig(1.0, 2.0, 0.02, 1e-4, 0.1, 1.0, 1.0)
    base = solve_scipy_nominal(com, momentum, com_rhs, rhs, posture, lower, upper, 1.0, config)
    row = np.array([[30.0, 0.0, 0.0, 0.0]])
    pinned = solve_scipy_nominal(com, momentum, com_rhs, rhs, posture, lower, upper, 1.0, config,
                                 extra_rows=row, extra_target=np.zeros(1))
    assert base.accepted and pinned.accepted
    assert abs(pinned.requested_counter_dq[0]) < abs(base.requested_counter_dq[0])
    assert pinned.objective_matrix.shape[0] == base.objective_matrix.shape[0] + 1


def test_disabled_yaw_leaves_robust_solve_unchanged():
    controller = _robust()
    args = (*_problem(), 1.0)
    reference = adapter.ScipyNominalMixin._isolated_velocity_solve(controller, *args)
    controller.yaw_momentum = YawMomentumConfig()
    controller._yaw_terms = (np.ones(4), np.ones(7))
    controller._yaw_inputs = (np.ones(7), 0.5)
    solved = controller._isolated_velocity_solve(*args)
    np.testing.assert_array_equal(solved.requested_counter_dq, reference.requested_counter_dq)


def test_enabled_yaw_appends_row(monkeypatch):
    controller = _robust()
    controller.yaw_momentum = YawMomentumConfig(weight=1.0)
    controller._yaw_terms = (np.array([1.0, 2.0, 0.0, 0.0]), np.ones(7))
    controller._yaw_inputs = (np.full(7, 0.1), 0.0)
    seen = {}

    def fake_solve(*args, extra_rows=None, extra_target=None):
        seen.update(rows=extra_rows, target=extra_target)
        return 'solved'

    monkeypatch.setattr(adapter, 'solve_scipy_nominal', fake_solve)
    assert controller._isolated_velocity_solve(*_problem(), 1.0) == 'solved'
    expected = yaw_momentum_row(*controller._yaw_terms, *controller._yaw_inputs, 1.0,
                                controller.yaw_momentum, controller.momentum_scale)
    np.testing.assert_allclose(seen['rows'], expected[0])
    np.testing.assert_allclose(seen['target'], expected[1])


def test_enabled_yaw_runs_through_control_step():
    controller = _robust()
    controller.yaw_momentum = YawMomentumConfig(weight=1.0)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert controller._yaw_inputs is None          # cleared after the plan
