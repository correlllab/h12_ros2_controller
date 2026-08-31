import numpy as np

from h12_ros2_controller.core.controller.counter_balance.braking import (
    BrakingLimits,
    ReconnectLimits,
    braking_rollout,
    reconnect_trajectory,
)


def _limits(**kwargs):
    values = {
        'dt': 0.02,
        'delay_steps': 2,
        'horizon_steps': 25,
        'max_acceleration': np.full(4, 25.0),
        'braking_effectiveness': np.ones(4),
        'max_acceleration_change': np.full(4, 5.0),
        'max_velocity': np.full(4, 3.0),
        'q_lower': np.full(4, -1.0),
        'q_upper': np.full(4, 1.0),
        'velocity_tolerance': np.full(4, 0.01),
        'acceleration_tolerance': np.full(4, 0.1),
    }
    values.update(kwargs)
    return BrakingLimits(**values)


def test_braking_rollout_stops_centered_counter_state():
    result = braking_rollout(
        np.zeros(4),
        np.array([0.5, -0.5, 0.2, -0.2]),
        np.zeros(4),
        _limits(),
        collision_validator=lambda q: True,
    )

    assert result.valid
    assert result.stopped
    assert np.allclose(result.dq[-1], 0.0)
    assert np.all(result.q >= -1.0)
    assert np.all(result.q <= 1.0)


def test_braking_rollout_rejects_insufficient_position_margin():
    result = braking_rollout(
        np.array([0.99, 0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
        np.array([5.0, 0.0, 0.0, 0.0]),
        _limits(),
        collision_validator=lambda q: True,
    )

    assert not result.valid
    assert result.invalid_reason == 'counter position left braking bounds'


def test_braking_rollout_handles_outward_acceleration_at_zero_velocity():
    result = braking_rollout(
        np.array([0.99, 0.0, 0.0, 0.0]),
        np.zeros(4),
        np.array([10.0, 0.0, 0.0, 0.0]),
        _limits(),
        collision_validator=lambda q: True,
    )

    assert not result.valid
    assert result.q[1, 0] > result.q[0, 0]


def test_braking_rollout_checks_between_knot_turning_point():
    result = braking_rollout(
        np.array([0.9995, 0.0, 0.0, 0.0]),
        np.array([0.2, 0.0, 0.0, 0.0]),
        np.array([-25.0, 0.0, 0.0, 0.0]),
        _limits(delay_steps=0),
        collision_validator=lambda q: True,
    )

    assert not result.valid
    assert result.invalid_reason == 'counter position left braking bounds'


def test_braking_rollout_does_not_stop_inside_delayed_interval():
    result = braking_rollout(
        np.zeros(4),
        np.array([0.05, 0.0, 0.0, 0.0]),
        np.array([-25.0, 0.0, 0.0, 0.0]),
        _limits(
            q_lower=np.array([-0.001, -1.0, -1.0, -1.0]),
        ),
        collision_validator=lambda q: True,
    )

    assert not result.valid
    assert result.invalid_reason == 'counter position left braking bounds'


def _reconnect_limits(**kwargs):
    values = {
        'dt': 0.02,
        'duration': 0.5,
        'max_acceleration': np.full(4, 25.0),
        'max_acceleration_change': np.full(4, 5.0),
        'max_velocity': np.full(4, 3.0),
        'max_torque_rate': np.full(4, 20.0),
        'q_lower': np.full(4, -1.0),
        'q_upper': np.full(4, 1.0),
    }
    values.update(kwargs)
    return ReconnectLimits(**values)


def test_reconnect_trajectory_reaches_upstream_state_smoothly():
    result = reconnect_trajectory(
        np.zeros(4),
        np.zeros(4),
        np.zeros(4),
        np.zeros(4),
        np.full(4, 0.1),
        np.zeros(4),
        np.zeros(4),
        np.full(4, 0.2),
        _reconnect_limits(),
        collision_validator=lambda q: True,
    )

    assert result.valid
    assert np.allclose(result.q[-1], 0.1)
    assert np.allclose(result.dq[-1], 0.0)
    assert np.allclose(result.acceleration[-1], 0.0)
    assert np.allclose(result.torque[-1], 0.2)


def test_reconnect_trajectory_rejects_torque_jump():
    result = reconnect_trajectory(
        np.zeros(4),
        np.zeros(4),
        np.zeros(4),
        np.zeros(4),
        np.zeros(4),
        np.zeros(4),
        np.zeros(4),
        np.full(4, 20.0),
        _reconnect_limits(max_torque_rate=np.full(4, 1.0)),
        collision_validator=lambda q: True,
    )

    assert not result.valid
    assert result.invalid_reason == 'reconnect torque rate left bounds'
