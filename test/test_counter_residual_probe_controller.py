from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.counter_residual_probe_controller import (
    CounterResidualProbeController,
)
from test_counter_ddp_velocity_controller import _controllers


def _probe(shadow=False):
    _, velocity = _controllers()
    controller = CounterResidualProbeController.__new__(
        CounterResidualProbeController,
    )
    controller.__dict__.update(velocity.__dict__)
    controller.probe_shadow = shadow
    controller.probe_mode = 'joint'
    controller.probe_first_step = 0
    controller.probe_spacing_steps = 2
    controller.probe_pulse_steps = 1
    controller.probe_seed = 0
    controller.probe_sign_scale = 1.0
    controller.probe_realization_delay = 1
    controller.probe_realization_gain = np.ones(4)
    controller.probe_momentum_damping = 1e-6
    controller.probe_max_residual_velocity = 1.0
    controller.probe_sequence = [(0, 0.1, 1), (1, -0.2, 1)]
    controller._probe_started = False
    controller._probe_step = -1
    controller._reset_probe_diagnostics()
    return controller


def test_zero_probe_residual_uses_exact_nominal_and_publishes_once():
    baseline = _probe(shadow=True)
    candidate = _probe(shadow=False)
    candidate.probe_sequence = []

    baseline.control_configuration_step(np.zeros(14), np.zeros(14))
    candidate.control_configuration_step(np.zeros(14), np.zeros(14))

    assert len(candidate.low_cmd_handler.calls) == 1
    for name in ('q', 'dq', 'tau'):
        assert np.array_equal(
            candidate.low_cmd_handler.calls[-1][name],
            baseline.low_cmd_handler.calls[-1][name],
        )


def test_active_probe_adds_bounded_residual_before_shared_finalizer():
    controller = _probe()

    controller.control_configuration_step(np.zeros(14), np.zeros(14))

    assert controller.latest_probe_active
    assert np.allclose(controller.latest_probe_residual, [0.1, 0.0, 0.0, 0.0])
    assert np.allclose(
        controller.latest_requested_counter_dq,
        controller.latest_probe_nominal + controller.latest_probe_residual,
    )
    assert len(controller.low_cmd_handler.calls) == 1


def test_probe_schedule_separates_pulses_by_configured_spacing():
    controller = _probe()

    values = [controller._scheduled_residual(step) for step in range(4)]

    assert values[0]['pulse_index'] == 0
    assert values[1]['pulse_index'] == -1
    assert values[2]['pulse_index'] == 1
    assert values[3]['pulse_index'] == -1


def test_momentum_probe_uses_realization_weighted_damped_inverse():
    controller = _probe()
    controller.probe_mode = 'momentum'
    controller.probe_momentum_damping = 1e-10
    controller.probe_sequence = [(0, 0.2, 2)]
    context = SimpleNamespace(
        momentum_counter=np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ]),
    )

    result = controller._scheduled_residual(0, context)

    assert result['pulse_steps'] == 2
    assert np.allclose(result['residual'], [0.2, 0.0, 0.0, 0.0])
    assert np.allclose(result['expected_momentum'], [0.2, 0.0])


@pytest.mark.parametrize(
    'config',
    (
        {'shadow': 1},
        {'spacing_steps': 0},
        {'pulse_steps': 3, 'spacing_steps': 2},
        {'amplitudes': [0.0]},
        {'sign_scale': 0.0},
        {'axes': [0, 0]},
        {'mode': 'invalid'},
        {'pulse_step_options': [0]},
        {'realization_delay': 2},
        {'realization_gain': [1.0, 1.0, 0.0, 1.0]},
        {'momentum_damping': 0.0},
    ),
)
def test_probe_config_rejects_invalid_values(config):
    with pytest.raises(ValueError):
        CounterResidualProbeController._probe_settings(config)
