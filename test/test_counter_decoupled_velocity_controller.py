from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.counter_decoupled_velocity_controller import (
    CounterDecoupledVelocityController,
)
from test_counter_ddp_velocity_controller import _controllers


def _decoupled(tilt_gain=0.5, max_tilt_error=0.1):
    _, velocity = _controllers()
    controller = CounterDecoupledVelocityController.__new__(
        CounterDecoupledVelocityController,
    )
    controller.__dict__.update(velocity.__dict__)
    controller.tilt_gain = tilt_gain
    controller.max_tilt_error = max_tilt_error
    controller.latest_tilt_error = np.zeros(2)
    controller.latest_clipped_tilt_error = np.zeros(2)
    controller.latest_tilt_feedback = np.zeros(2)
    controller.latest_tilt_feedback_available = False
    controller.latest_momentum_feedforward = np.zeros(2)
    controller.latest_gyro_feedback = np.zeros(2)
    controller.latest_positive_divergence = np.zeros(2)
    controller.tilt_reference = np.zeros(2)
    return controller


def _targets(controller, balance_scale=1.0):
    return controller._reaction_targets(
        np.zeros((2, 7)),
        np.zeros((2, 7)),
        np.zeros(7),
        np.zeros(2),
        np.zeros(3),
        balance_scale,
    )


def test_zero_tilt_gain_reproduces_frozen_3c_target_and_command():
    _, baseline = _controllers()
    controller = _decoupled(tilt_gain=0.0)
    controller.robot_model.state['imu_state'].quaternion = [
        np.cos(0.1), np.sin(0.1), 0.0, 0.0,
    ]
    args = (
        np.zeros((2, 7)),
        np.zeros((2, 7)),
        np.zeros(7),
        np.array([0.01, -0.02]),
        np.array([0.2, -0.1, 0.0]),
        0.7,
    )

    baseline_targets = baseline._reaction_targets(*args)
    candidate_targets = controller._reaction_targets(*args)

    assert all(
        np.array_equal(candidate, expected)
        for candidate, expected in zip(candidate_targets, baseline_targets)
    )

    q = np.zeros(14)
    dq = np.zeros(14)
    baseline.control_configuration_step(q, dq, balance_scale=0.7)
    controller.control_configuration_step(q, dq, balance_scale=0.7)
    for name in ('q', 'dq', 'tau'):
        assert np.array_equal(
            controller.low_cmd_handler.calls[-1][name],
            baseline.low_cmd_handler.calls[-1][name],
        )


def test_tilt_feedback_has_restorative_sign_and_clips():
    controller = _decoupled()
    controller.robot_model.state['imu_state'].quaternion = [
        np.cos(0.1), np.sin(0.1), 0.0, 0.0,
    ]

    _, momentum_rhs = _targets(controller, balance_scale=0.5)

    assert np.allclose(momentum_rhs, [0.025, 0.0])
    assert np.allclose(controller.latest_clipped_tilt_error, [0.1, 0.0])
    assert controller.latest_tilt_feedback_available


def test_invalid_tilt_disables_only_new_feedback():
    controller = _decoupled()
    controller.robot_model.state['imu_state'] = SimpleNamespace(
        quaternion=[np.nan, 0.0, 0.0, 0.0],
        gyroscope=[0.0, 0.0, 0.0],
    )

    com_rhs, momentum_rhs = _targets(controller)

    assert np.array_equal(com_rhs, np.zeros(2))
    assert np.array_equal(momentum_rhs, np.zeros(2))
    assert not controller.latest_tilt_feedback_available


@pytest.mark.parametrize('moving_arm', ['left', 'right'])
def test_feedback_law_is_ownership_independent(moving_arm):
    controller = _decoupled()
    controller.moving_arm = moving_arm
    controller.robot_model.state['imu_state'].quaternion = [
        np.cos(0.025), 0.0, np.sin(0.025), 0.0,
    ]

    _, momentum_rhs = _targets(controller)

    assert np.allclose(momentum_rhs, [0.0, 0.025])


@pytest.mark.parametrize(
    'config',
    [
        {'tilt_gain': -1.0},
        {'tilt_gain': np.nan},
        {'max_tilt_error': 0.0},
        {'max_tilt_error': np.inf},
    ],
)
def test_feedback_config_rejects_invalid_values(config):
    with pytest.raises(ValueError):
        CounterDecoupledVelocityController._feedback_settings(config)
