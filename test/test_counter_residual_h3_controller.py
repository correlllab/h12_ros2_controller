import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_residual_h3_controller import (
    CounterResidualH3Controller,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h3_ocp import (
    ResidualH3OCP,
)
from h12_ros2_controller.core.controller.counter_balance.verified_response_parameters import (
    verified_h3_models,
)
from test_counter_ddp_velocity_controller import _controllers


def _controller():
    _, velocity = _controllers()
    controller = CounterResidualH3Controller.__new__(CounterResidualH3Controller)
    controller.__dict__.update(velocity.__dict__)
    controller.h2_models = verified_h3_models()
    controller.h2_ocp = ResidualH3OCP(dt=controller.dt, max_iterations=1)
    controller.h2_trust_velocity = np.array([0.01, 0.01, 0.0, 0.01])
    controller.h2_context_limit = 3.0
    controller.h2_shadow = True
    controller._h2_previous_tilt = np.zeros(2)
    controller._h2_previous_rate = np.array([0.08, -0.05])
    controller._h2_previous_moving_momentum = np.zeros(2)
    controller._h2_previous_nominal_momentum = np.zeros(2)
    controller._h2_sequence = 0
    controller._h2_pending_residual = np.zeros(4)
    controller._h2_nominal_for_finalize = np.zeros(4)
    controller._reset_h2_diagnostics()
    controller.robot_model.state['imu_state'].gyroscope = [0.08, -0.05, 0.0]
    return controller


def test_h3_shadow_publishes_exact_frozen_3c():
    baseline = _controllers()[1]
    candidate = _controller()
    baseline.robot_model.state['imu_state'].gyroscope = [0.08, -0.05, 0.0]

    baseline.control_configuration_step(np.zeros(14), np.zeros(14))
    candidate.control_configuration_step(np.zeros(14), np.zeros(14))

    for name in ('q', 'dq', 'tau'):
        assert np.array_equal(
            candidate.low_cmd_handler.calls[-1][name],
            baseline.low_cmd_handler.calls[-1][name],
        )
    diagnostics = candidate.diagnostics()
    assert diagnostics['h3_shadow']
    assert diagnostics['h3_horizon_steps'] == 3
    assert 'h2_status' not in diagnostics
