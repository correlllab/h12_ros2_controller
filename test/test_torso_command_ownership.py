from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, UPPER_BODY_INDEX


TORSO = BODY_JOINTS.index('torso_joint')


def controller_fixture():
    commands = SimpleNamespace(
        q_cmd=np.zeros(27), _q_clip_limits=np.tile([-2.0, 2.0], (27, 1)),
        dq_cmd=np.zeros(27), tau_cmd=np.zeros(27),
    )
    sent = []

    def publish(q, dq=None, tau=None, joint_ids=None):
        ids = np.arange(27) if joint_ids is None else joint_ids
        commands.q_cmd[ids] = q
        if dq is not None:
            commands.dq_cmd[ids] = dq
        if tau is not None:
            commands.tau_cmd[ids] = tau
        sent.append((np.asarray(q).copy(), None if dq is None else dq.copy(), joint_ids))

    commands.set_joint_commands = publish
    controller = object.__new__(UpperController)
    controller.low_cmd_handler = commands
    controller.enabled_ids = list(range(13, 27))
    controller.robot_model = SimpleNamespace(state={'q': np.zeros(27)})
    controller.dt = 0.02
    controller.update_robot_model = lambda: None
    return controller, sent


def test_arm_commands_preserve_explicit_torso_reference_and_inputs():
    controller, sent = controller_fixture()
    controller.low_cmd_handler.q_cmd[TORSO] = 0.15
    q = np.full(27, 0.1)
    dq = np.ones(27)

    for torso in (-0.1, 0.0, 0.25):
        q[TORSO] = torso
        controller._publish_position_command(q, dq, np.zeros(27))
        assert sent[-1][0][0] == 0.15
        assert sent[-1][1][0] == 0.0
        assert sent[-1][2] == list(UPPER_BODY_INDEX)
        np.testing.assert_array_equal(sent[-1][0][1:], q[13:])
        assert q[TORSO] == torso and dq[TORSO] == 1.0


def test_arm_position_refresh_uses_measured_torso_for_dynamics_not_command_target():
    controller, sent = controller_fixture()
    controller.low_cmd_handler.q_cmd[TORSO] = -0.1
    measured = np.zeros(27)
    measured[TORSO] = 0.05
    gravity_inputs = []
    controller.robot_model = SimpleNamespace(
        state={'q': measured}, model_body=SimpleNamespace(nv=27),
        dynamics=SimpleNamespace(get_gravity_compensation=lambda q:
                                  gravity_inputs.append(q.copy()) or np.zeros(27)),
    )
    controller._tau_bias = np.zeros(27)
    controller._tau_bias_decay = 0.95

    controller._apply_joint_position(measured)

    assert gravity_inputs[0][TORSO] == 0.05
    assert sent[0][0][0] == -0.1
    assert measured[TORSO] == 0.05


def test_torso_ramp_latches_exact_target_after_early_measured_convergence(monkeypatch):
    controller, sent = controller_fixture()
    controller.low_cmd_handler.q_cmd[TORSO] = 0.3
    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.time.sleep',
                        lambda seconds: None)

    controller._move_torso_to_target(0.3, 0.0)

    assert len(sent) == 2
    assert sent[0][0][0] != 0.0
    assert sent[-1][0][0] == 0.0
    assert controller.low_cmd_handler.q_cmd[TORSO] == 0.0


@pytest.mark.parametrize('target', (float('nan'), float('inf'), 2.1))
def test_invalid_torso_target_sends_no_commands(target):
    controller, sent = controller_fixture()
    with pytest.raises(ValueError, match='Torso target'):
        controller._move_torso_to_target(0.0, target)
    assert not sent


@pytest.mark.parametrize('mode', ('planned', 'direct'))
def test_upper_motion_holds_zero_torso_without_overwriting_frozen_motor_commands(mode):
    controller, sent = controller_fixture()
    controller.low_cmd_handler.q_cmd[:12] = np.arange(12) * 0.01
    controller.low_cmd_handler.dq_cmd[:12] = 0.4
    controller.low_cmd_handler.tau_cmd[:12] = 0.7
    original = controller.low_cmd_handler.q_cmd.copy()
    measured = np.full(27, 0.03)
    controller.robot_model = SimpleNamespace(
        state={'q': measured}, model_body=SimpleNamespace(nv=27),
        dynamics=SimpleNamespace(get_gravity_compensation=lambda q: np.zeros(27)),
    )
    controller._tau_bias = np.ones(27)
    controller._tau_bias_decay = 0.5
    controller.ik_solver = SimpleNamespace(
        q=measured.copy(), integrate=lambda velocity: None,
    )

    if mode == 'planned':
        controller._apply_joint_position(measured)
    else:
        controller._apply_velocity_command(np.zeros(27))

    assert controller.low_cmd_handler.q_cmd[TORSO] == 0.0
    np.testing.assert_array_equal(controller.low_cmd_handler.q_cmd[:12], original[:12])
    np.testing.assert_array_equal(controller.low_cmd_handler.dq_cmd[:12], np.full(12, 0.4))
    np.testing.assert_array_equal(controller.low_cmd_handler.tau_cmd[:12], np.full(12, 0.7))
    assert sent[-1][2] == list(UPPER_BODY_INDEX)
    np.testing.assert_array_equal(measured, np.full(27, 0.03))
    assert controller._tau_bias[TORSO] == 1.0
    np.testing.assert_array_equal(controller._tau_bias[13:], np.full(14, 0.5))


def test_guarded_seed_reset_preserves_torso_hold_bias():
    controller, _ = controller_fixture()
    q = np.zeros(27)
    controller.robot_model.reduced_mask = np.r_[
        np.zeros(13, dtype=bool), np.ones(14, dtype=bool),
    ]
    controller.robot_model.reduced_reference_q = q.copy()
    controller.check_state_fresh = lambda: None
    controller.low_cmd_handler.q_cmd[13:] = 0.2
    controller._tau_bias = np.ones(27)
    controller.ik_solver = SimpleNamespace(update_configurations=lambda seed: None)

    controller.update_ik_solver()

    assert controller._tau_bias[TORSO] == 1.0
    np.testing.assert_array_equal(controller._tau_bias[13:], np.zeros(14))
