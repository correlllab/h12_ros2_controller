import numpy as np
from types import SimpleNamespace

from h12_ros2_controller.core.controller.zmp_controller import (
    DirectZmpController,
)
from h12_ros2_controller.core.robot_dynamics import predict_balance_error


def test_momentum_change_can_cancel_planar_balance_error():
    balance_error = np.array([0.02, -0.01], dtype=np.float64)
    mass = 50.0
    dt = 0.04
    scale = mass * 9.81 * dt
    momentum_delta = scale * np.array([
        balance_error[1],
        -balance_error[0],
        0.0,
    ])

    predicted = predict_balance_error(
        balance_error,
        momentum_delta,
        np.zeros(3),
        mass,
        dt,
    )

    assert np.allclose(predicted, np.zeros(2))


def test_feedback_error_applies_axis_gain_and_norm_limit():
    controller = DirectZmpController.__new__(DirectZmpController)
    controller.zmp_gain = np.array([2.0, 1.0], dtype=np.float64)
    controller.max_error = 0.02

    error = controller._feedback_error([0.02, 0.02])

    assert np.isclose(np.linalg.norm(error), 0.02)
    assert error[0] > error[1]


def test_feedback_error_is_unclipped_below_limit():
    controller = DirectZmpController.__new__(DirectZmpController)
    controller.zmp_gain = np.array([1.0, 0.5], dtype=np.float64)
    controller.max_error = 0.04

    error = controller._feedback_error([0.01, -0.01])

    assert np.allclose(error, [0.01, -0.005])


def test_pitch_rate_feedback_reverses_balance_error_with_fall_direction():
    controller = DirectZmpController.__new__(DirectZmpController)
    controller.zmp_gain = np.array([1.0, 1.0], dtype=np.float64)
    controller.control_zmp_gain = np.copy(controller.zmp_gain)
    controller.angular_velocity_gain = np.array([0.15, 0.0])
    controller.tilt_gain = np.zeros(2)
    controller.latest_tilt_error = np.zeros(2)
    controller.max_error = 0.04
    state = SimpleNamespace(
        zmp_residual=np.array([-0.01, 0.0]),
        angular_velocity=np.array([0.0, 0.2, 0.0]),
    )

    forward_error = controller._control_error(state)
    state.angular_velocity[1] = -0.2
    backward_error = controller._control_error(state)

    assert forward_error[0] < 0.0
    assert backward_error[0] > 0.0


def test_tilt_feedback_restores_pitch_toward_reference():
    controller = DirectZmpController.__new__(DirectZmpController)
    controller.zmp_gain = np.zeros(2)
    controller.control_zmp_gain = np.zeros(2)
    controller.angular_velocity_gain = np.zeros(2)
    controller.tilt_gain = np.array([0.15, 0.0])
    controller.max_error = 0.04
    controller.latest_tilt_error = np.array([0.0, -0.2])
    state = SimpleNamespace(
        zmp_residual=np.zeros(2),
        angular_velocity=np.zeros(3),
    )

    error = controller._control_error(state)

    assert error[0] > 0.0


def test_reference_does_not_freeze_before_observer_is_armed():
    controller = DirectZmpController.__new__(DirectZmpController)
    controller.output_enabled = True
    controller.active = True
    controller.latest_balance_error = np.array([0.01, 0.0])
    controller.latest_balance_state = SimpleNamespace(armed=False)

    assert not controller._reference_should_freeze()

    controller.latest_balance_state.armed = True
    assert controller._reference_should_freeze()


def test_feedback_activation_uses_hysteresis():
    controller = DirectZmpController.__new__(DirectZmpController)
    controller.enter_threshold = 0.012
    controller.exit_threshold = 0.005
    controller.active = False

    controller._update_active([0.01, 0.0])
    assert not controller.active

    controller._update_active([0.013, 0.0])
    assert controller.active

    controller._update_active([0.006, 0.0])
    assert controller.active

    controller._update_active([0.004, 0.0])
    assert not controller.active


def test_response_direction_uses_zmp_and_stops_after_rate_decay():
    controller = DirectZmpController.__new__(DirectZmpController)
    controller.enter_threshold = 0.012
    controller.rate_peak_threshold = 0.10
    controller.stop_rate_threshold = 0.05
    controller.stop_on_rate_decay = True
    controller.response_pitch_direction = 0.0
    controller.response_peak_seen = False
    trigger_error = np.array([-0.02, 0.0])

    assert not controller._response_should_stop(
        trigger_error,
        pitch_rate=-0.03,
        was_active=False,
    )
    assert controller.response_pitch_direction == 1.0

    assert not controller._response_should_stop(
        trigger_error,
        pitch_rate=0.12,
        was_active=True,
    )
    assert controller.response_peak_seen

    assert controller._response_should_stop(
        trigger_error,
        pitch_rate=0.04,
        was_active=True,
    )


def test_balance_command_publishes_optimized_arm_velocity():
    captured = {}
    controller = DirectZmpController.__new__(DirectZmpController)
    controller.balance_ddp = SimpleNamespace(arm_ids=[1, 2])
    controller.ik_solver = SimpleNamespace(
        q=np.array([0.0, 0.1, 0.2, 0.0]),
        integrate=lambda command: captured.update(integrated=command.copy()),
    )
    controller.robot_model = SimpleNamespace(
        state={'q': np.zeros(4)},
        dynamics=SimpleNamespace(
            get_gravity_compensation=lambda _q: np.arange(4.0),
        ),
    )
    controller.low_cmd_handler = SimpleNamespace(
        set_joint_commands=lambda **kwargs: captured.update(kwargs),
    )
    controller.update_robot_model = lambda: None
    command = np.array([0.0, 0.3, -0.4, 0.0])

    controller._apply_balance_command(command)

    assert np.allclose(captured['integrated'], command)
    assert np.allclose(captured['q'], [0.1, 0.2])
    assert np.allclose(captured['dq'], [0.3, -0.4])
    assert np.allclose(captured['tau'], [1.0, 2.0])
    assert captured['joint_ids'] == [1, 2]
