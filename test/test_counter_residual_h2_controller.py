from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_controller import (
    CounterResidualH2Controller,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_ocp import (
    ResidualH2OCP,
)
from h12_ros2_controller.core.controller.counter_balance.verified_response_parameters import (
    verified_h2_models,
)
from test_counter_ddp_velocity_controller import _controllers


def _h2_controller():
    _, velocity = _controllers()
    controller = CounterResidualH2Controller.__new__(
        CounterResidualH2Controller,
    )
    controller.__dict__.update(velocity.__dict__)
    controller.h2_models = verified_h2_models()
    controller.h2_ocp = ResidualH2OCP(dt=controller.dt, max_iterations=1)
    controller.h2_trust_velocity = np.array([0.1, 0.1, 0.0, 0.1])
    controller.h2_context_limit = 3.0
    controller.h2_shadow = True
    controller._h2_previous_tilt = np.zeros(2)
    controller._h2_previous_rate = np.array([0.08, -0.05])
    controller._h2_previous_moving_momentum = np.zeros(2)
    controller._h2_previous_nominal_momentum = np.zeros(2)
    controller._h2_sequence = 0
    controller._reset_h2_diagnostics()
    controller.robot_model.state['imu_state'].gyroscope = [0.08, -0.05, 0.0]
    return controller


def test_h2_shadow_publishes_exact_nominal_once():
    baseline = _controllers()[1]
    candidate = _h2_controller()
    baseline.robot_model.state['imu_state'].gyroscope = [0.08, -0.05, 0.0]
    q = np.zeros(14)
    dq = np.zeros(14)

    baseline.control_configuration_step(q, dq)
    candidate.control_configuration_step(q, dq)

    assert len(candidate.low_cmd_handler.calls) == 1
    for name in ('q', 'dq', 'tau'):
        assert np.array_equal(
            candidate.low_cmd_handler.calls[-1][name],
            baseline.low_cmd_handler.calls[-1][name],
        )
    assert candidate.latest_h2_residual[2] == 0.0


def test_h2_abstains_when_phase_is_not_confident():
    controller = _h2_controller()
    controller._h2_previous_rate = np.array([-0.08, 0.05])

    controller.control_configuration_step(np.zeros(14), np.zeros(14))

    assert not controller.latest_h2_model_valid
    assert controller.latest_h2_decision == 'abstain'
    assert np.allclose(controller.latest_h2_residual, 0.0, atol=1e-8)


def test_h2_model_failure_falls_back_to_exact_nominal():
    baseline = _controllers()[1]
    candidate = _h2_controller()
    candidate.h2_models = None
    q = np.zeros(14)
    dq = np.zeros(14)
    baseline.robot_model.state['imu_state'].gyroscope = [0.08, -0.05, 0.0]

    baseline.control_configuration_step(q, dq)
    candidate.control_configuration_step(q, dq)

    assert candidate.latest_h2_status == 'model_failure'
    assert np.array_equal(
        candidate.low_cmd_handler.calls[-1]['dq'],
        baseline.low_cmd_handler.calls[-1]['dq'],
    )


def test_h2_active_adds_only_selected_residual_before_finalizer():
    controller = _h2_controller()
    controller.h2_shadow = False
    controller._run_h2 = lambda context, nominal: np.array([
        0.01, -0.01, 0.0, 0.005,
    ])
    nominal = SimpleNamespace(
        requested_counter_dq=np.array([0.2, -0.1, 0.0, 0.05]),
    )

    selected = controller._select_requested_counter_velocity(None, nominal)

    assert np.allclose(selected, [0.21, -0.11, 0.0, 0.055])
    assert np.array_equal(controller._h2_pending_residual, [
        0.01, -0.01, 0.0, 0.005,
    ])


def test_h2_pending_state_uses_applied_residual_after_finalizer():
    controller = _h2_controller()
    controller.h2_shadow = False
    controller._run_h2 = lambda context, nominal: np.array([
        0.005, -0.004, 0.0, 0.003,
    ])

    controller.control_configuration_step(np.zeros(14), np.zeros(14))

    expected = (
        controller.latest_applied_counter_dq
        - controller.latest_backtrack_scale * controller._h2_nominal_for_finalize
    )
    expected[2] = 0.0
    assert np.allclose(controller._h2_pending_residual, expected)


@pytest.mark.parametrize(
    'config',
    (
        {'max_iterations': -1},
        {'trust_velocity': [0.1, 0.1, 0.1, 0.1]},
        {'context_limit': 0.0},
        {'weights': {'rate': -1.0}},
    ),
)
def test_h2_config_rejects_invalid_values(config):
    with pytest.raises(ValueError):
        CounterResidualH2Controller._h2_settings(config)
