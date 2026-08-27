import time

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance import (
    CounterDDPController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_ocp import (
    CounterDDPOCP,
)
from h12_ros2_controller.utility.controller_config import load_controller_config
from test_counter_balance_controller import _harness as _reactive_harness


def _harness(moving_arm='left', shadow=False):
    reactive = _reactive_harness(moving_arm)
    controller = CounterDDPController.__new__(CounterDDPController)
    controller.__dict__.update(reactive.__dict__)
    controller.shadow = shadow
    controller.horizon_steps = 3
    controller.max_acceleration = np.full(4, 25.0)
    controller.max_acceleration_change = np.full(4, 5.0)
    controller.max_velocity = np.ones(4)
    controller.max_excursion = np.full(4, 0.5)
    controller.gyro_rate_threshold = 0.04
    controller.gyro_rate_full_scale = 0.06
    controller.moving_velocity_threshold = 1e-3
    controller.max_forecast_age = 0.1
    controller.max_frozen_map_displacement = 0.1
    controller.metric_tolerance = 0.25
    controller.validation_steps = 1
    controller.timing_guard = 1.0
    controller.ddp = CounterDDPOCP(
        dt=controller.dt,
        horizon_steps=controller.horizon_steps,
        max_iterations=2,
    )
    controller._previous_acceleration = np.zeros(4)
    controller._reset_ddp_diagnostics()
    return controller


def _horizon(controller, moving_position=0.3, moving_velocity=0.2):
    q = np.zeros((controller.horizon_steps + 1, 14))
    dq = np.zeros_like(q)
    q[:, controller.moving_local] = moving_position
    dq[:, controller.moving_local] = moving_velocity
    generated_at = time.monotonic()
    sample_times = generated_at + np.arange(
        controller.horizon_steps + 1,
    ) * controller.dt
    return q, dq, sample_times, generated_at


def test_counter_ddp_is_separate_controller_class():
    assert CounterDDPController is not None
    assert CounterDDPController.__name__ == 'CounterDDPController'


def test_horizon_step_preserves_moving_position_velocity_and_torque():
    controller = _harness()
    q, dq, sample_times, generated_at = _horizon(controller)
    tau = np.linspace(1.0, 14.0, 14)

    result = controller.control_horizon_step(
        q,
        dq,
        moving_tau=tau,
        sample_times=sample_times,
        generated_at=generated_at,
    )

    call = controller.low_cmd_handler.calls[-1]
    assert np.allclose(call['q'][controller.moving_local], 0.3)
    assert np.allclose(call['dq'][controller.moving_local], 0.2)
    assert np.allclose(
        call['tau'][controller.moving_local],
        tau[controller.moving_local],
    )
    assert np.allclose(
        call['q'][controller.counter_wrist_local],
        controller.counter_wrist_ref,
    )
    assert np.allclose(result, call['dq'])
    assert controller.diagnostics()['torque_source'] == 'supplied'
    assert controller.diagnostics()['moving_position_command_error'] == 0.0
    assert controller.diagnostics()['moving_velocity_command_error'] == 0.0


def test_shadow_solves_but_holds_counter_arm():
    controller = _harness(shadow=True)
    controller.robot_model.state['imu_state'].gyroscope = [0.1, 0.0, 0.0]
    q, dq, sample_times, generated_at = _horizon(controller)
    measured = np.copy(controller.robot_model.state['q'][controller.counter_ids])

    controller.control_horizon_step(
        q,
        dq,
        sample_times=sample_times,
        generated_at=generated_at,
    )

    call = controller.low_cmd_handler.calls[-1]
    assert controller.latest_status.startswith('shadow_')
    assert np.allclose(
        call['q'][controller.counter_active_local], measured,
    )
    assert np.allclose(call['dq'][controller.counter_active_local], 0.0)
    assert controller.diagnostics()['optimized_cost'] is not None


def test_invalid_horizon_shape_is_rejected_without_publication():
    controller = _harness()

    with pytest.raises(ValueError, match='moving horizons'):
        controller.control_horizon_step(
            np.zeros((controller.horizon_steps, 14)),
            np.zeros((controller.horizon_steps, 14)),
        )

    assert controller.low_cmd_handler.calls == []


def test_stale_forecast_holds_counter_and_preserves_moving_sample():
    controller = _harness()
    q, dq, sample_times, generated_at = _horizon(controller)
    generated_at -= 1.0
    sample_times -= 1.0

    controller.control_horizon_step(
        q,
        dq,
        sample_times=sample_times,
        generated_at=generated_at,
    )

    call = controller.low_cmd_handler.calls[-1]
    assert controller.latest_status == 'invalid_forecast_time'
    assert np.allclose(call['q'][controller.moving_local], 0.3)
    assert np.allclose(call['dq'][controller.moving_local], 0.2)
    assert np.allclose(call['dq'][controller.counter_active_local], 0.0)


def test_shifted_forecast_timestamps_are_rejected_with_moving_pass_through():
    controller = _harness()
    q, dq, sample_times, generated_at = _horizon(controller)
    sample_times += 0.01

    controller.control_horizon_step(
        q,
        dq,
        sample_times=sample_times,
        generated_at=generated_at,
    )

    call = controller.low_cmd_handler.calls[-1]
    assert controller.latest_status == 'invalid_forecast_time'
    assert np.allclose(call['q'][controller.moving_local], 0.3)
    assert np.allclose(call['dq'][controller.moving_local], 0.2)


def test_timing_overrun_holds_counter_command():
    controller = _harness()
    controller.timing_guard = 1e-12
    controller.robot_model.state['imu_state'].gyroscope = [0.1, 0.0, 0.0]
    q, dq, sample_times, generated_at = _horizon(controller)

    controller.control_horizon_step(
        q,
        dq,
        sample_times=sample_times,
        generated_at=generated_at,
    )

    call = controller.low_cmd_handler.calls[-1]
    assert controller.latest_status == 'timing_overrun'
    assert controller.latest_timing_overrun
    assert np.allclose(call['dq'][controller.counter_active_local], 0.0)


def test_delayed_gravity_is_included_before_timing_guard():
    controller = _harness()
    controller.timing_guard = 0.01
    controller.robot_model.state['imu_state'].gyroscope = [0.1, 0.0, 0.0]
    q, dq, sample_times, generated_at = _horizon(controller)

    def gravity(unused):
        time.sleep(0.02)
        return np.arange(27, dtype=np.float64)

    controller.robot_model.dynamics.get_gravity_compensation = gravity

    controller.control_horizon_step(
        q,
        dq,
        sample_times=sample_times,
        generated_at=generated_at,
    )

    call = controller.low_cmd_handler.calls[-1]
    assert controller.latest_status == 'timing_overrun'
    assert np.allclose(call['dq'][controller.counter_active_local], 0.0)


def test_inactive_hold_reports_delayed_publication():
    controller = _harness()
    controller.timing_guard = 0.01
    q, dq, sample_times, generated_at = _horizon(controller)

    def gravity(unused):
        time.sleep(0.02)
        return np.arange(27, dtype=np.float64)

    controller.robot_model.dynamics.get_gravity_compensation = gravity

    controller.control_horizon_step(
        q,
        dq,
        sample_times=sample_times,
        generated_at=generated_at,
    )

    assert controller.latest_status == 'inactive_timing_overrun'
    assert controller.latest_timing_overrun


def test_first_acceleration_bounds_include_position_velocity_and_slew():
    controller = _harness()
    controller.robot_model.state['q'][controller.counter_ids[0]] = 0.998
    controller.robot_model.state['dq'][controller.counter_ids[0]] = 0.1
    controller.q_counter_ref = np.zeros(4)
    controller._reference_captured = True
    controller._previous_acceleration[0] = 1.0
    q = controller.robot_model.state['q'][controller.counter_ids]
    dq = controller.robot_model.state['dq'][controller.counter_ids]

    lower, upper = controller._control_bounds(q, dq)

    assert upper[0, 0] == pytest.approx(0.0, abs=1e-12)
    assert lower[0, 0] >= -4.0
    assert np.all(upper[1:] == 25.0)


def test_control_bounds_allow_recovery_from_excursion_overshoot():
    controller = _harness()
    controller.q_counter_ref = np.zeros(4)
    controller._reference_captured = True
    counter_q = np.array([0.55, 0.0, 0.0, 0.0])
    counter_dq = np.zeros(4)

    lower, upper = controller._control_bounds(counter_q, counter_dq)

    assert np.all(lower[0] <= upper[0])


def test_partial_backtrack_effective_acceleration_uses_published_velocity():
    controller = _harness()
    controller.backtrack_scales = (0.5,)
    calls = 0

    def collision(candidate):
        nonlocal calls
        calls += 1
        return True

    controller.robot_model.check_collision_free = collision
    controller.counter_wrist_ref = np.zeros(3)
    motor_q = np.copy(controller.robot_model.state['q'])
    arm_q = np.copy(motor_q[controller.arm_ids])
    arm_dq = np.zeros(14)
    counter_q = np.zeros(4)
    counter_dq = np.full(4, 0.5)
    requested_q = np.full(4, 0.01)
    requested_dq = np.full(4, 0.7)

    result, status = controller._backtrack_ddp_counter(
        motor_q,
        arm_q,
        arm_dq,
        counter_q,
        counter_dq,
        requested_q,
        requested_dq,
    )
    _, command_dq, scale = result
    effective = (
        command_dq[controller.counter_active_local] - counter_dq
    ) / controller.dt

    assert status is None
    assert scale == 0.5
    assert np.allclose(effective, -7.5)
    assert calls == 1


def test_inherited_publication_uses_ik_velocity_only_after_sample_zero():
    controller = _harness()
    captured = {}
    controller._inherited_forecast_dq = np.full(14, 0.3)

    def control(q_horizon, dq_horizon, **kwargs):
        captured['q'] = q_horizon
        captured['dq'] = dq_horizon
        captured['kwargs'] = kwargs

    controller.control_horizon_step = control
    q = np.zeros(27)
    dq = np.zeros(27)
    tau = np.ones(27)

    controller._publish_position_command(q, dq, tau)

    assert np.allclose(captured['dq'][0], 0.0)
    assert np.allclose(captured['dq'][1:], 0.3)
    assert np.allclose(captured['q'][1], 0.3 * controller.dt)
    assert captured['kwargs']['forecast_source'] == (
        'inherited_constant_velocity'
    )


def test_configuration_parser_rejects_non_boolean_shadow():
    with pytest.raises(ValueError, match='shadow'):
        CounterDDPController._parse_ddp_config({'shadow': 'false'})


def test_configuration_parser_uses_short_realtime_defaults():
    settings = CounterDDPController._parse_ddp_config({})

    assert settings['shadow']
    assert settings['horizon_steps'] == 5
    assert settings['max_iterations'] == 2
    assert np.array_equal(settings['max_acceleration'], np.full(4, 25.0))


def test_scalar_gyro_activation_uses_moving_horizon():
    controller = _harness()
    controller.q_counter_ref = np.zeros(4)
    controller.counter_wrist_ref = np.zeros(3)
    controller.com_offset_ref = np.zeros(2)
    controller._reference_captured = True
    controller.robot_model.state['imu_state'].gyroscope = [0.06, 0.0, 0.0]
    motor_q = np.copy(controller.robot_model.state['q'])
    q, dq, _, _ = _horizon(controller)

    knots, activation = controller._build_knots(
        motor_q, q, dq, authority_scale=1.0,
    )

    assert len(knots) == controller.horizon_steps
    assert np.allclose(activation, 1.0)
    assert controller.latest_gyro_rate == pytest.approx(0.06)


def test_scalar_gyro_activation_ignores_standing_rate():
    controller = _harness()
    controller.q_counter_ref = np.zeros(4)
    controller.counter_wrist_ref = np.zeros(3)
    controller.com_offset_ref = np.zeros(2)
    controller._reference_captured = True
    controller.robot_model.state['imu_state'].gyroscope = [0.2, 0.0, 0.0]
    motor_q = np.copy(controller.robot_model.state['q'])
    q, dq, _, _ = _horizon(controller)
    dq[:] = 0.0

    _, activation = controller._build_knots(
        motor_q, q, dq, authority_scale=1.0,
    )

    assert np.allclose(activation, 0.0)


def test_inactive_path_skips_solver_and_passes_through_arm_sample():
    controller = _harness()
    q, dq, sample_times, generated_at = _horizon(controller)

    controller.control_horizon_step(
        q,
        dq,
        sample_times=sample_times,
        generated_at=generated_at,
    )

    call = controller.low_cmd_handler.calls[-1]
    assert controller.latest_status == 'inactive_passthrough'
    assert controller.diagnostics()['optimized_cost'] is None
    assert np.allclose(
        call['q'][controller.counter_active_local],
        q[0, controller.counter_active_local],
    )
    assert np.allclose(
        call['dq'][controller.counter_active_local],
        dq[0, controller.counter_active_local],
    )


def test_config_loader_preserves_counter_ddp_mapping(tmp_path):
    config_path = tmp_path / 'ddp.yaml'
    config_path.write_text(
        'mode: debug\n'
        'gains:\n'
        '  kp: 1.0\n'
        '  kd: 0.1\n'
        '  ki: 0.0\n'
        'counter_ddp:\n'
        '  shadow: false\n'
        '  horizon_steps: 3\n',
        encoding='utf-8',
    )

    config = load_controller_config('ddp.yaml', config_dir=tmp_path)

    assert config['counter_ddp'] == {
        'shadow': False,
        'horizon_steps': 3,
    }
