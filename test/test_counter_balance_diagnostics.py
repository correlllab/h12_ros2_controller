from copy import deepcopy
from unittest.mock import Mock

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.objective import reaction_targets
from h12_ros2_controller.core.controller.counter_balance.diagnostic_controller import (
    CounterBalanceDiagnosticController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller import (
    CounterDDPVelocityRobustController,
)
from test_counter_balance_controller import _harness


def _controller(moving_arm='left', variant=None):
    controller = _harness(moving_arm)
    controller.__class__ = (
        CounterDDPVelocityRobustController if variant is None
        else CounterBalanceDiagnosticController
    )
    controller.velocity_ocp = None
    controller.latest_velocity_ocp_result = None
    controller.momentum_weight = 2.0
    controller.posture_weight = 0.02
    controller.damping = 0.0001
    if variant is not None:
        controller.configure_diagnostic_trial(variant)
    return controller


@pytest.mark.parametrize('moving_arm', ['left', 'right'])
@pytest.mark.parametrize('variant', ['c0', 'ar_shadow', 'momentum_rate_shadow'])
def test_c0_and_shadows_are_bitwise_nominal_including_buffers(moving_arm, variant):
    baseline = _controller(moving_arm)
    candidate = _controller(moving_arm, variant)
    for index, scale in enumerate((1.0, 0.37, 0.0)):
        q, dq = np.zeros(14), np.zeros(14)
        q[candidate.moving_local] = np.linspace(-0.1, 0.1, 7)
        dq[candidate.moving_local] = np.linspace(-0.3, 0.3, 7)
        for controller in (baseline, candidate):
            controller.robot_model.state['arrival_monotonic'] = 10.0 + index * 0.02
            controller.robot_model.state['dq'][controller.moving_ids] = 0.12
        candidate.set_diagnostic_sample(index * 0.02, 1.0)
        expected = baseline.control_configuration_step(q, dq, balance_scale=scale)
        actual = candidate.control_configuration_step(q, dq, balance_scale=scale)
        assert actual.tobytes() == expected.tobytes()
        for name in ('q_cmd', 'dq_cmd', 'tau_cmd'):
            assert getattr(candidate.low_cmd_handler, name).tobytes() == getattr(
                baseline.low_cmd_handler, name,
            ).tobytes()
        for name in ('com_rhs', 'momentum_rhs', 'lower', 'upper',
                     'objective_matrix', 'objective_target'):
            assert getattr(candidate._latest_nominal_plan, name).tobytes() == getattr(
                baseline._latest_nominal_plan, name,
            ).tobytes()
        assert candidate._nominal_prepared_context == baseline._nominal_prepared_context
    assert len(candidate.low_cmd_handler.calls) == 3
    record = candidate.diagnostics()['counter_balance_diagnostic']
    assert record['actuation'] == 'c0'
    assert record['rate_feedback_enabled'] is False
    assert record['ddq_used'] is False


@pytest.mark.parametrize('variant', CounterBalanceDiagnosticController.VARIANTS)
@pytest.mark.parametrize('scale', [0.0, 0.17, 1.0])
def test_offsets_cancel_only_the_selected_scaled_feedback(variant, scale):
    controller = _controller(variant=variant)
    com = np.arange(14, dtype=float).reshape(2, 7) / 20
    momentum = np.flip(com, axis=1).copy()
    dq = np.linspace(-0.3, 0.2, 7)
    error = np.array([0.02, -0.01])
    gyro = np.array([-0.07, 0.13, 0.9])
    before = deepcopy(controller.config)
    gains = controller.com_gain, controller.gyro_gain
    offsets = controller._reaction_target_offsets(
        com, momentum, dq, error, gyro, scale,
    )
    rhs = reaction_targets(com, momentum, dq, error, gyro, scale, *gains)
    com_enabled = variant not in ('feedback_off', 'com_feedback_off')
    gyro_enabled = variant not in ('feedback_off', 'gyro_feedback_off')
    np.testing.assert_allclose(
        rhs[0] + offsets[0],
        scale * (-com @ dq - (controller.com_gain * error if com_enabled else 0)),
        atol=1e-16,
    )
    np.testing.assert_allclose(
        rhs[1] + offsets[1],
        scale * (-momentum @ dq + (
            controller.gyro_gain * gyro[:2] if gyro_enabled else 0
        )), atol=1e-16,
    )
    assert (controller.com_gain, controller.gyro_gain) == gains
    for key in before['limits']:
        np.testing.assert_array_equal(controller.config['limits'][key], before['limits'][key])
    components = controller._diagnostic_record['target_components']
    np.testing.assert_allclose(components['com_feedback'], -scale * gains[0] * error)
    np.testing.assert_allclose(components['gyro_feedback'], scale * gains[1] * gyro[:2])


@pytest.mark.parametrize('variant', ['feedback_off', 'com_feedback_off', 'gyro_feedback_off'])
def test_feedback_variant_flows_through_existing_plan_and_finalizer(variant):
    controller = _controller(variant=variant)
    controller._reference_captured = True
    controller.q_counter_ref = np.zeros(4)
    controller.counter_wrist_ref = np.zeros(3)
    controller.com_offset_ref = np.array([-0.02, 0.01])
    controller.robot_model.state['imu_state'].gyroscope = [0.2, -0.1, 0.0]
    finalize = Mock(wraps=controller._finalize_counter_velocity)
    controller._finalize_counter_velocity = finalize
    controller.set_diagnostic_sample(1.2, 1.0)
    controller.control_configuration_step(np.zeros(14), np.zeros(14), balance_scale=0.3)
    finalize.assert_called_once()
    assert len(controller.low_cmd_handler.calls) == 1
    nominal = controller._latest_nominal_plan
    np.testing.assert_allclose(nominal.com_rhs, (
        [0.0, 0.0] if variant != 'gyro_feedback_off' else [-0.012, 0.006]
    ), atol=1e-16)
    np.testing.assert_allclose(nominal.momentum_rhs, (
        [0.0, 0.0] if variant != 'com_feedback_off' else [0.006, -0.003]
    ), atol=1e-16)
    assert controller.diagnostics()['counter_balance_diagnostic']['sample']['phase'] == 'post_move'


def test_trial_reset_and_consumed_sample_do_not_leak_history():
    controller = _controller(variant='momentum_rate_shadow')
    preview = {'dq': [1.0, 2.0]}
    controller.set_diagnostic_sample(0.1, 1.0, preview)
    preview['dq'][0] = 99.0
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert controller._diagnostic_record['sample']['reference_preview']['dq'][0] == 1.0
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert controller._diagnostic_record['sample'] is None
    assert controller._diagnostic_record['phase_source'] == 'unavailable'
    controller.configure_diagnostic_trial('feedback_off')
    assert controller._diagnostic_previous == {}
    assert all(not history for history in controller._diagnostic_rates.values())
    assert controller._diagnostic_record == {}
    assert controller._latest_nominal_plan is None
    assert controller._reference_captured is False
    assert controller.q_counter_ref is None


def test_rates_use_separate_timestamps_and_all_filter_windows_without_ddq():
    controller = _controller(variant='momentum_rate_shadow')
    controller.robot_model.state['ddq'] = np.full(27, np.nan)
    for index in range(10):
        controller.robot_model.state['arrival_monotonic'] = 10.0 + index * 0.02
        controller.robot_model.state['dq'][controller.moving_ids[2:4]] = index * 0.02
        controller.set_diagnostic_sample(index * 0.03, 1.0)
        controller.control_configuration_step(np.zeros(14), np.zeros(14))
    rates = controller._diagnostic_record['rates']['sources']
    np.testing.assert_allclose(rates['measured']['raw_rate_xy'], [1.0, 1.0])
    for window in ('3', '5', '9'):
        np.testing.assert_allclose(rates['measured']['filtered_rate_xy'][window], [1.0, 1.0])
        np.testing.assert_array_equal(rates['command']['filtered_rate_xy'][window], [0.0, 0.0])
    assert rates['measured']['dt_s'] == pytest.approx(0.02)
    assert rates['command']['dt_s'] == pytest.approx(0.03)
    controller.robot_model.state['arrival_monotonic'] = 9.0
    controller.set_diagnostic_sample(2.0, 1.0)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    for record in controller._diagnostic_record['rates']['sources'].values():
        assert record['valid'] is False
        assert all(value is None for value in record['filtered_rate_xy'].values())


@pytest.mark.parametrize('variant', ['', 'C0', 'ar', 'recovery', None, 1])
def test_invalid_variant_is_atomic(variant):
    controller = _controller(variant='c0')
    controller.set_diagnostic_sample(0.0, 1.0)
    with pytest.raises(ValueError, match='Unknown diagnostic variant'):
        controller.configure_diagnostic_trial(variant)
    assert controller.diagnostic_variant == 'c0'
    assert controller._diagnostic_sample is not None


@pytest.mark.parametrize('elapsed,duration', [(np.nan, 1), (np.inf, 1), (-1, 1), (0, 0), (0, np.nan)])
def test_nonfinite_or_invalid_phase_rejected(elapsed, duration):
    controller = _controller(variant='c0')
    with pytest.raises(ValueError):
        controller.set_diagnostic_sample(elapsed, duration)


def test_unqualified_model_preview_and_nonfinite_target_rejected():
    controller = _controller(variant='ar_shadow')
    with pytest.raises(ValueError, match='qualified'):
        controller.configure_diagnostic_trial('ar_shadow', prediction_model=Mock())
    with pytest.raises(ValueError, match='finite data'):
        controller.set_diagnostic_sample(0, 1, {'dq': [np.nan]})
    with pytest.raises(ValueError, match='finite'):
        controller._reaction_target_offsets(
            np.zeros((2, 7)), np.zeros((2, 7)), np.zeros(7),
            np.array([np.nan, 0.0]), np.zeros(3), 1.0,
        )


def test_failed_step_clears_stale_diagnostics_and_rate_history():
    controller = _controller(variant='c0')
    controller.set_diagnostic_sample(0, 1)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    with pytest.raises(ValueError, match='finite'):
        controller.control_configuration_step(np.full(14, np.nan), np.zeros(14))
    record = controller.diagnostics()['counter_balance_diagnostic']
    assert 'target_components' not in record
    assert 'rates' not in record
    assert controller._diagnostic_previous == {}


def test_diagnostic_state_cannot_be_reconfigured_during_control():
    controller = _controller(variant='c0')
    controller._diagnostic_in_step = True
    with pytest.raises(RuntimeError):
        controller.configure_diagnostic_trial('feedback_off')
    with pytest.raises(RuntimeError):
        controller.set_diagnostic_sample(0, 1)


@pytest.mark.parametrize('variant', CounterBalanceDiagnosticController.VARIANTS)
def test_all_variants_use_exactly_one_committing_nominal_solve(variant):
    controller = _controller(variant=variant)
    solve = Mock(wraps=controller._isolated_velocity_solve)
    controller._isolated_velocity_solve = solve
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    solve.assert_called_once()
    assert len(controller.low_cmd_handler.calls) == 1


@pytest.mark.parametrize('timestamp', [None, np.nan, np.inf])
def test_missing_or_nonfinite_measured_clock_never_affects_command(timestamp):
    baseline = _controller()
    controller = _controller(variant='momentum_rate_shadow')
    controller.robot_model.state['arrival_monotonic'] = timestamp
    controller.set_diagnostic_sample(0, 1)
    baseline.control_configuration_step(np.zeros(14), np.zeros(14))
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    for name in ('q_cmd', 'dq_cmd', 'tau_cmd'):
        np.testing.assert_array_equal(
            getattr(controller.low_cmd_handler, name),
            getattr(baseline.low_cmd_handler, name),
        )
    rates = controller._diagnostic_record['rates']['sources']
    assert rates['measured']['valid'] is False
    assert rates['measured']['timestamp_s'] is None


def test_nonfinite_measured_rate_signal_is_not_an_actuation_input():
    controller = _controller(variant='c0')
    controller.robot_model.state['dq'][controller.counter_ids] = np.nan
    controller.robot_model.state['arrival_monotonic'] = 10.0
    controller.set_diagnostic_sample(0, 1)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert np.all(np.isfinite(controller.low_cmd_handler.dq_cmd))
    rates = controller._diagnostic_record['rates']['sources']
    assert rates['measured']['momentum_xy'] is None
    assert rates['measured']['valid'] is False
    assert 'measured' not in controller._diagnostic_previous


@pytest.mark.parametrize('failure', ['collision', 'bounds', 'estop'])
def test_inherited_failure_paths_still_publish_at_most_one_hold(failure):
    controller = _controller(variant='feedback_off')
    if failure == 'collision':
        controller.robot_model.check_collision_free = lambda unused: False
    elif failure == 'bounds':
        controller._counter_velocity_bounds = lambda unused: (
            np.ones(4), -np.ones(4),
        )
    else:
        controller.low_cmd_handler._estopped = True
    controller.set_diagnostic_sample(0, 1)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert len(controller.low_cmd_handler.calls) == (0 if failure == 'estop' else 1)
    np.testing.assert_array_equal(controller.latest_applied_counter_dq, np.zeros(4))


def test_diagnostics_return_an_independent_snapshot():
    controller = _controller(variant='c0')
    controller.set_diagnostic_sample(0, 1)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    values = controller.diagnostics()['counter_balance_diagnostic']
    values['target_components']['com_feedforward'][0] = 99.0
    assert controller._diagnostic_record['target_components']['com_feedforward'][0] == 0.0
