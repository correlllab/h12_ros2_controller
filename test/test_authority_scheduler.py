import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.authority_scheduler import (
    AdaptiveAuthorityScheduler,
    AuthorityConfig,
)


def test_scheduler_reaches_full_authority_for_rising_preview():
    scheduler = AdaptiveAuthorityScheduler(AuthorityConfig(
        momentum_scale=1.0,
        momentum_rate_scale=1.0,
        preview_entry=0.1,
        preview_full=0.2,
    ))
    momentum = np.zeros((4, 2))
    rate = np.array([[0.1, 0.0], [0.3, 0.0], [0.2, 0.0]])

    values = [
        scheduler.evaluate(
            momentum, rate, np.zeros(2), np.zeros(2), 1.0,
        )
        for _ in range(5)
    ]

    assert values[-1].authority == pytest.approx(1.0)
    assert values[-1].effective_scale == pytest.approx(1.0)
    assert values[-1].peak_index == 1


def test_post_peak_low_response_attenuates_feedforward():
    scheduler = AdaptiveAuthorityScheduler(AuthorityConfig(
        preview_entry=0.1,
        preview_full=0.2,
    ))
    decision = scheduler.evaluate(
        np.zeros((3, 2)),
        np.array([[0.3, 0.0], [0.1, 0.0]]),
        np.zeros(2),
        np.zeros(2),
        1.0,
    )

    assert decision.peak_index == 0
    assert decision.target_authority == 0.0


def test_invalid_preview_keeps_response_feedback_available():
    scheduler = AdaptiveAuthorityScheduler(AuthorityConfig(
        response_entry=0.1,
        response_full=0.2,
    ))
    decision = scheduler.evaluate(
        np.zeros((0, 2)),
        np.zeros((0, 2)),
        np.array([0.2, 0.0]),
        np.array([0.2, 0.0]),
        0.5,
        preview_valid=False,
    )

    assert decision.feedback_authority == 1.0
    assert decision.effective_scale == pytest.approx(0.1)


def test_scheduler_reset_clears_authority():
    scheduler = AdaptiveAuthorityScheduler()
    scheduler.authority = 1.0

    scheduler.reset()

    assert scheduler.authority == 0.0


def test_preview_only_mode_ignores_response_feedback():
    scheduler = AdaptiveAuthorityScheduler(AuthorityConfig(
        preview_entry=0.1,
        preview_full=0.2,
    ))

    decision = scheduler.evaluate(
        np.zeros((3, 2)),
        np.zeros((2, 2)),
        np.array([1.0, 0.0]),
        np.array([1.0, 0.0]),
        1.0,
        feedback_enabled=False,
        confirmation_enabled=False,
    )

    assert decision.feedback_authority == 1.0
    assert decision.target_authority == 0.0
