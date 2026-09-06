from types import SimpleNamespace

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_balance_controller import (
    CounterBalanceController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_robust_controller import (
    CounterResidualH2RobustController,
)
from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    Frozen3CVelocitySolve,
)
from test_counter_residual_h2_controller import _h2_controller


def _robust():
    controller = _h2_controller()
    controller.__class__ = CounterResidualH2RobustController
    controller.latest_nominal_fallback_used = False
    controller.latest_nominal_primary_result = None
    return controller


def test_robust_h2_preserves_accepted_nominal_path():
    baseline = _h2_controller()
    robust = _robust()

    baseline.control_configuration_step(np.zeros(14), np.zeros(14))
    robust.control_configuration_step(np.zeros(14), np.zeros(14))

    assert not robust.latest_nominal_fallback_used
    for name in ('q', 'dq', 'tau'):
        assert np.array_equal(
            robust.low_cmd_handler.calls[-1][name],
            baseline.low_cmd_handler.calls[-1][name],
        )


def test_robust_h2_uses_scipy_only_after_rejected_nominal(monkeypatch):
    controller = _robust()
    rejected = Frozen3CVelocitySolve(
        requested_counter_dq=np.zeros(4),
        accepted=False,
        diagnostics=SimpleNamespace(accepted=False),
    )
    expected = np.array([0.1, -0.2, 0.0, 0.05])
    monkeypatch.setattr(
        CounterDDPVelocityController,
        '_isolated_velocity_solve',
        lambda *args, **kwargs: rejected,
    )
    monkeypatch.setattr(
        CounterBalanceController,
        '_solve_bounded_velocity',
        lambda *args, **kwargs: expected,
    )

    solved = controller._isolated_velocity_solve(
        np.zeros((2, 4)),
        np.zeros((2, 4)),
        np.zeros(2),
        np.zeros(2),
        np.zeros(4),
        -np.ones(4),
        np.ones(4),
        1.0,
    )

    assert solved.accepted
    assert np.array_equal(solved.requested_counter_dq, expected)
    assert controller.latest_nominal_fallback_used
