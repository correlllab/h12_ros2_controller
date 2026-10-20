import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.reaction_observer import (
    ReactionObserver,
    predict_frozen_momentum_rate,
    reaction_seed_diagnostic,
)


def test_reaction_observer_filters_finite_difference_rates():
    observer = ReactionObserver(filter_time_constant=0.02)
    first = observer.update(
        1.0, np.zeros(4), np.zeros(2), np.zeros(2), np.zeros(2),
    )
    second = observer.update(
        1.02,
        np.full(4, 0.2),
        np.array([0.2, -0.1]),
        np.array([-0.1, 0.3]),
        np.array([0.04, -0.02]),
    )

    assert not first.valid
    assert second.valid
    assert second.dt == pytest.approx(0.02)
    assert np.allclose(second.counter_acceleration, 5.0)
    assert np.allclose(second.counter_hdot, [5.0, -2.5])
    assert np.allclose(second.moving_hdot, [-2.5, 7.5])
    assert np.allclose(second.base_angular_acceleration, [1.0, -0.5])


def test_reaction_observer_rejects_stale_difference():
    observer = ReactionObserver(max_dt=0.05)
    observer.update(
        1.0, np.zeros(4), np.zeros(2), np.zeros(2), np.zeros(2),
    )

    sample = observer.update(
        1.2, np.ones(4), np.ones(2), np.ones(2), np.ones(2),
    )

    assert not sample.valid
    assert np.allclose(sample.counter_hdot, 0.0)


def test_predict_frozen_momentum_rate_separates_arm_contributions():
    current_counter_map = np.zeros((2, 4))
    current_moving_map = np.zeros((2, 7))
    next_counter_map = np.zeros((2, 4))
    next_moving_map = np.zeros((2, 7))
    next_counter_map[0, 0] = 2.0
    next_moving_map[1, 0] = 3.0

    counter, moving, total = predict_frozen_momentum_rate(
        current_counter_map,
        current_moving_map,
        np.zeros(4),
        np.zeros(7),
        next_counter_map,
        next_moving_map,
        np.array([0.1, 0.0, 0.0, 0.0]),
        np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        0.02,
    )

    assert np.allclose(counter, [10.0, 0.0])
    assert np.allclose(moving, [0.0, 30.0])
    assert np.allclose(total, [10.0, 30.0])


def test_reaction_seed_diagnostic_clips_infeasible_acceleration():
    current_map = np.zeros((2, 4))
    next_map = np.zeros((2, 4))
    next_map[0, 0] = 1.0
    next_map[1, 1] = 1.0

    result = reaction_seed_diagnostic(
        current_map,
        next_map,
        np.zeros(4),
        np.array([2.0, -2.0]),
        np.array([0.1, 0.1]),
        -10.0 * np.ones(4),
        10.0 * np.ones(4),
        0.02,
    )

    assert result.valid
    assert not result.unbounded_feasible
    assert np.allclose(result.clipped_acceleration[:2], [10.0, -10.0])
    assert np.array_equal(result.saturation_mask[:2], [True, True])
    assert np.allclose(result.achieved_measured_rate, [1.0, -1.0])
