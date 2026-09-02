import time

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.authority_scheduler import (
    AdaptiveAuthorityScheduler,
    AuthorityConfig,
)
from h12_ros2_controller.core.controller.counter_balance.counter_adaptive_velocity_controller import (
    CounterAdaptiveVelocityController,
)
from test_counter_ddp_velocity_controller import _controllers


def _adaptive(shadow=True):
    _, velocity = _controllers()
    controller = CounterAdaptiveVelocityController.__new__(
        CounterAdaptiveVelocityController,
    )
    controller.__dict__.update(velocity.__dict__)
    controller.adaptive_shadow = shadow
    controller.adaptive_mode = 'combined'
    controller.preview_steps = 3
    controller.max_preview_age = 0.04
    controller.authority_scheduler = AdaptiveAuthorityScheduler(
        AuthorityConfig(
            preview_entry=0.1,
            preview_full=0.2,
        ),
    )
    controller.latest_authority_decision = None
    controller.latest_adaptive_total_time = 0.0
    controller.adaptive_sequence = 0
    return controller


def _preview(controller):
    q = np.zeros((controller.preview_steps + 1, 14))
    dq = np.zeros_like(q)
    dq[:, controller.moving_local[0]] = [0.0, 0.1, 0.3, 0.1]
    return q, dq


def test_shadow_adaptive_controller_publishes_exact_3c_command():
    _, baseline = _controllers()
    adaptive = _adaptive(shadow=True)
    q, dq = _preview(adaptive)
    generated_at = time.monotonic()

    baseline.control_configuration_step(q[0], dq[0], balance_scale=0.7)
    adaptive.control_preview_step(
        q,
        dq,
        lifecycle_scale=0.7,
        generated_at=generated_at,
        sample_times=(
            generated_at + np.arange(adaptive.preview_steps + 1) * adaptive.dt
        ),
    )

    baseline_call = baseline.low_cmd_handler.calls[-1]
    adaptive_call = adaptive.low_cmd_handler.calls[-1]
    for name in ('q', 'dq', 'tau'):
        assert np.allclose(adaptive_call[name], baseline_call[name])
    assert adaptive.latest_authority_decision is not None
    assert adaptive.diagnostics()['adaptive_shadow']


def test_stale_preview_disables_feedforward_but_keeps_current_sample():
    controller = _adaptive(shadow=False)
    controller.authority_scheduler.authority = 1.0
    q, dq = _preview(controller)
    generated_at = time.monotonic() - 1.0

    controller.control_preview_step(
        q,
        dq,
        lifecycle_scale=1.0,
        generated_at=generated_at,
        sample_times=(
            generated_at
            + np.arange(controller.preview_steps + 1) * controller.dt
        ),
    )

    decision = controller.latest_authority_decision
    assert not decision.preview_valid
    assert decision.feedforward_authority == 0.0
    assert decision.authority == 0.0
    assert controller.low_cmd_handler.calls


def test_estop_resets_scheduler_before_control():
    controller = _adaptive(shadow=False)
    controller.authority_scheduler.authority = 1.0
    controller.low_cmd_handler._estopped = True
    q, dq = _preview(controller)

    controller.control_preview_step(q, dq, lifecycle_scale=1.0)

    assert controller.authority_scheduler.authority == 0.0


def test_preview_signal_failure_resets_scheduler(monkeypatch):
    controller = _adaptive(shadow=False)
    controller.authority_scheduler.authority = 1.0
    monkeypatch.setattr(
        controller,
        '_preview_signals',
        lambda *args: (_ for _ in ()).throw(RuntimeError('failed')),
    )
    q, dq = _preview(controller)
    generated_at = time.monotonic()

    controller.control_preview_step(
        q,
        dq,
        lifecycle_scale=1.0,
        generated_at=generated_at,
        sample_times=(
            generated_at
            + np.arange(controller.preview_steps + 1) * controller.dt
        ),
    )

    assert controller.latest_authority_decision.authority == 0.0


def test_invalid_current_sample_resets_scheduler():
    controller = _adaptive(shadow=False)
    controller.authority_scheduler.authority = 1.0
    q, dq = _preview(controller)
    q[0, 0] = np.nan

    try:
        controller.control_preview_step(q, dq, lifecycle_scale=1.0)
    except ValueError:
        pass
    else:
        raise AssertionError('invalid current sample was accepted')

    assert controller.authority_scheduler.authority == 0.0


def test_adaptive_config_defaults_to_shadow():
    settings = CounterAdaptiveVelocityController._adaptive_settings({})

    assert settings['shadow']
    assert settings['mode'] == 'combined'
    assert settings['preview_steps'] == 10
