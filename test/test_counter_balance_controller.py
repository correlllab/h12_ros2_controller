from types import SimpleNamespace

import numpy as np
import pytest

import h12_ros2_controller.core.controller.counter_balance.controller as reactive
import h12_ros2_controller.core.controller.counter_balance.objective as objective
from h12_ros2_controller.core.controller.counter_balance import (
    CounterBalanceController,
    ReactiveCounterBalanceController,
)
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS


class _CommandHandler:
    def __init__(self):
        self._estopped = False
        self.q_cmd = np.zeros(27, dtype=np.float64)
        self.dq_cmd = np.zeros(27, dtype=np.float64)
        self.tau_cmd = np.full(27, 0.25, dtype=np.float64)
        self.calls = []

    def set_joint_commands(self, **kwargs):
        self.calls.append(kwargs)
        ids = kwargs['joint_ids']
        self.q_cmd[ids] = kwargs['q']
        self.dq_cmd[ids] = kwargs['dq']
        self.tau_cmd[ids] = kwargs['tau']


def _harness(moving_arm='left'):
    controller = ReactiveCounterBalanceController.__new__(
        ReactiveCounterBalanceController
    )
    controller.moving_arm = moving_arm
    controller.counter_arm = 'right' if moving_arm == 'left' else 'left'
    controller.arm_ids = list(range(13, 27))
    if moving_arm == 'left':
        controller.moving_ids = list(range(13, 20))
        controller.counter_arm_ids = list(range(20, 27))
    else:
        controller.moving_ids = list(range(20, 27))
        controller.counter_arm_ids = list(range(13, 20))
    controller.counter_ids = controller.counter_arm_ids[:4]
    controller.counter_wrist_ids = controller.counter_arm_ids[4:]
    controller.moving_local = controller._arm_local_indices(
        controller.moving_ids
    )
    controller.counter_local = controller._arm_local_indices(
        controller.counter_arm_ids
    )
    controller.counter_active_local = controller.counter_local[:4]
    controller.counter_wrist_local = controller.counter_local[4:]
    controller.motor_q_indices = np.arange(27)
    controller.motor_v_indices = np.arange(27)
    controller.moving_v_indices = np.asarray(controller.moving_ids)
    controller.counter_v_indices = np.asarray(controller.counter_ids)
    controller.dt = 0.02
    controller.dq_lim = 2.0
    controller.com_weight = 1.0
    controller.momentum_weight = 1.0
    controller.posture_weight = 0.0
    controller.com_gain = 2.0
    controller.gyro_gain = 0.1
    controller.posture_gain = 0.5
    controller.damping = 0.0
    controller.com_velocity_scale = 0.1
    controller.momentum_scale = 2.0
    controller.posture_velocity_scale = 0.5
    controller.activation_tilt_threshold = 0.0
    controller.activation_tilt_full_scale = 0.0
    controller.activation_latch = False
    controller.support_geometry = {
        'front': 0.174,
        'rear': 0.086,
        'half_width': 0.043,
        'max_yaw_divergence': np.deg2rad(20.0),
    }
    controller.max_velocity = np.ones(4)
    controller.max_excursion = np.full(4, np.inf)
    controller.backtrack_scales = (1.0, 0.5, 0.0)
    controller.config = {
        'limits': {
            'q_clip_limits': np.tile([-1.0, 1.0], (27, 1)),
            'dq_clip_limits': np.full(27, 0.5),
        },
    }

    q = np.zeros(27, dtype=np.float64)
    q[:13] = np.linspace(-0.12, 0.12, 13)
    q[24:27] = [0.2, -0.3, 0.4]
    state = {
        'q': q,
        'dq': np.zeros(27, dtype=np.float64),
        'imu_state': SimpleNamespace(
            gyroscope=[0.0, 0.0, 0.0],
            quaternion=[1.0, 0.0, 0.0, 0.0],
        ),
    }
    handler = _CommandHandler()
    handler.q_cmd[:] = q
    model_body = SimpleNamespace(
        nq=27,
        nv=27,
        lowerPositionLimit=np.full(27, -2.0),
        upperPositionLimit=np.full(27, 2.0),
        velocityLimit=np.full(27, 3.0),
    )
    robot_model = SimpleNamespace(
        state=state,
        model_body=model_body,
        dynamics=SimpleNamespace(
            get_gravity_compensation=lambda unused: np.arange(
                27, dtype=np.float64
            )
        ),
        check_within_limits=lambda unused: True,
        check_collision_free=lambda unused: True,
    )
    controller.robot_model = robot_model
    controller.low_cmd_handler = handler
    controller.update_robot_model = lambda: None

    support = SimpleNamespace(
        valid=True,
        center=np.zeros(2, dtype=np.float64),
        invalid_reason='',
    )
    com_jacobian = np.zeros((3, 27), dtype=np.float64)
    momentum_map = np.zeros((3, 27), dtype=np.float64)
    for row in range(2):
        com_jacobian[row, controller.moving_v_indices[row]] = 1.0
        com_jacobian[row, controller.counter_v_indices[row]] = 1.0
        momentum_map[row, controller.moving_v_indices[row + 2]] = 1.0
        momentum_map[row, controller.counter_v_indices[row + 2]] = 1.0
    controller._model_terms = lambda unused: (
        support,
        np.array([0.0, 0.0, 0.8]),
        com_jacobian,
        momentum_map,
        np.eye(3),
    )
    controller._reference_captured = False
    controller.q_counter_ref = None
    controller.counter_wrist_ref = None
    controller.com_offset_ref = None
    controller.tilt_reference = None
    controller._latched_activation = 0.0
    controller._counter_q_command = None
    controller._frame_balance_scale = 1.0
    controller._reset_diagnostics()
    return controller


def test_moving_arm_is_validated_before_base_initialization():
    with pytest.raises(ValueError, match='moving_arm'):
        ReactiveCounterBalanceController('', '', '', moving_arm='both')


def test_canonical_controller_extends_frame_controller():
    assert issubclass(CounterBalanceController, FrameController)
    assert ReactiveCounterBalanceController is CounterBalanceController


def test_reactive_config_is_validated_before_base_initialization(monkeypatch):
    calls = []
    monkeypatch.setattr(
        reactive.FrameController,
        '__init__',
        lambda *args, **kwargs: calls.append((args, kwargs)),
    )
    config = {
        'reactive_counter_balance': {'momentum_scale': 0.0},
    }

    with pytest.raises(ValueError, match='momentum_scale'):
        ReactiveCounterBalanceController('', '', '', 'left', config=config)

    assert calls == []


def test_tilt_activation_ramps_and_latches():
    controller = _harness()
    controller.activation_tilt_threshold = 0.07
    controller.activation_tilt_full_scale = 0.10
    controller.activation_latch = True
    controller.tilt_reference = np.zeros(2)

    controller._imu_tilt = lambda: np.array([0.05, 0.0])
    assert controller._balance_activation() == 0.0
    controller._imu_tilt = lambda: np.array([0.085, 0.0])
    assert np.isclose(controller._balance_activation(), 0.5)
    controller._imu_tilt = lambda: np.zeros(2)
    assert np.isclose(controller._balance_activation(), 0.5)
    controller._imu_tilt = lambda: np.array([0.11, 0.0])
    assert controller._balance_activation() == 1.0


@pytest.mark.parametrize(
    ('moving_arm', 'moving_ids', 'counter_ids'),
    (
        ('left', list(range(13, 20)), list(range(20, 27))),
        ('right', list(range(20, 27)), list(range(13, 20))),
    ),
)
def test_arm_ownership_and_body_velocity_column_routing(
        moving_arm, moving_ids, counter_ids):
    controller = _harness(moving_arm)

    assert controller.moving_ids == moving_ids
    assert controller.counter_arm_ids == counter_ids
    assert np.array_equal(controller.moving_v_indices, moving_ids)
    assert np.array_equal(controller.counter_v_indices, counter_ids[:4])
    assert not set(controller.moving_ids) & set(controller.counter_arm_ids)


def test_frame_target_identifies_moving_arm_from_kinematic_support():
    controller = _harness('right')
    controller.clear_frame_tasks = lambda: None
    controller.add_frame_task = lambda *unused: None
    controller._reduced_support_mask = lambda unused: np.array(
        [True] * 7 + [False] * 7,
    )

    controller.set_frame_target('wrist', np.zeros(6))

    assert controller.moving_arm == 'left'
    assert controller.counter_arm == 'right'


def test_named_target_identifies_single_moving_arm(monkeypatch):
    controller = _harness('right')
    target = np.zeros(14)
    target[:7] = 0.4
    captured = []
    monkeypatch.setitem(reactive.NAMED_CONFIGS, 'left_test', target)
    controller.set_reduced_configuration_target = captured.append

    result = controller.set_named_target('left_test')

    assert controller.moving_arm == 'left'
    assert controller.counter_arm == 'right'
    assert np.array_equal(result, target)
    assert np.array_equal(captured[0], target)


def test_inherited_frame_publication_preserves_moving_command():
    controller = _harness('left')
    q = np.copy(controller.robot_model.state['q'])
    dq = np.zeros(27)
    q[controller.moving_ids] = 0.35
    dq[controller.moving_ids] = 0.2

    controller._publish_position_command(q, dq, np.zeros(27))

    call = controller.low_cmd_handler.calls[0]
    assert np.allclose(call['q'][:7], 0.35)
    assert np.allclose(call['dq'][:7], 0.2)
    assert controller._steady_state_control_ids() == controller.moving_ids


def test_body_indices_follow_pinocchio_joint_q_and_v_indices():
    controller = ReactiveCounterBalanceController.__new__(
        ReactiveCounterBalanceController
    )
    joints = [SimpleNamespace(nq=0, nv=0, idx_q=0, idx_v=0)]
    joint_ids = {}
    for motor_id, joint_name in enumerate(BODY_JOINTS):
        joint_ids[joint_name] = motor_id + 1
        joints.append(SimpleNamespace(
            nq=1,
            nv=1,
            idx_q=26 - motor_id,
            idx_v=(motor_id + 7) % 27,
        ))
    controller.robot_model = SimpleNamespace(
        model_body=SimpleNamespace(
            getJointId=lambda name: joint_ids[name],
            joints=joints,
        )
    )

    q_indices, v_indices = controller._body_indices()

    assert np.array_equal(q_indices, np.arange(26, -1, -1))
    assert np.array_equal(v_indices, (np.arange(27) + 7) % 27)


def test_model_terms_use_private_full_body_data_and_live_motor_q(monkeypatch):
    controller = _harness()
    rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    data = SimpleNamespace(
        Ag=np.zeros((6, 27)),
        oMf=[SimpleNamespace(rotation=rotation)],
    )
    model = SimpleNamespace(
        nq=27,
        nv=27,
        getFrameId=lambda unused: 0,
    )
    controller.robot_model.model_body = model
    controller._reactive_data = data
    controller.motor_q_indices = np.arange(26, -1, -1)
    controller._support_rectangle = lambda private_data: SimpleNamespace(
        valid=True,
        center=np.zeros(2),
        invalid_reason='',
    )
    seen = []

    def record_q(unused_model, private_data, body_q):
        assert private_data is data
        seen.append(np.copy(body_q))

    monkeypatch.setattr(reactive.pin, 'forwardKinematics', record_q)
    monkeypatch.setattr(
        reactive.pin,
        'updateFramePlacements',
        lambda unused_model, private_data: None,
    )
    monkeypatch.setattr(
        reactive.pin,
        'centerOfMass',
        lambda unused_model, private_data, body_q: np.array([0.1, 0.2, 0.8]),
    )
    monkeypatch.setattr(
        reactive.pin,
        'jacobianCenterOfMass',
        lambda unused_model, private_data, body_q: np.zeros((3, 27)),
    )
    monkeypatch.setattr(
        reactive.pin,
        'computeCentroidalMap',
        lambda unused_model, private_data, body_q: None,
    )
    motor_q = np.linspace(-0.5, 0.5, 27)

    terms = ReactiveCounterBalanceController._model_terms(controller, motor_q)

    assert np.allclose(seen[0], motor_q[::-1])
    assert np.array_equal(terms[4], rotation)


def test_bounded_least_squares_cancels_normalized_synthetic_motion():
    controller = _harness()
    com_counter = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
    ])
    momentum_counter = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])

    result = controller._solve_bounded_velocity(
        com_counter,
        momentum_counter,
        np.array([-0.1, 0.2]),
        np.array([-0.3, 0.4]),
        np.zeros(4),
        np.full(4, -1.0),
        np.full(4, 1.0),
    )

    assert np.allclose(result, [-0.1, 0.2, -0.3, 0.4])


def test_physical_scales_normalize_least_squares_rows(monkeypatch):
    controller = _harness()
    controller.posture_weight = 1.0
    captured = {}

    def solve(matrix, target, bounds):
        captured['matrix'] = matrix
        captured['target'] = target
        captured['bounds'] = bounds
        return SimpleNamespace(success=True, x=np.zeros(4))

    monkeypatch.setattr(objective, 'lsq_linear', solve)
    controller._solve_bounded_velocity(
        np.ones((2, 4)),
        2.0 * np.ones((2, 4)),
        np.array([0.1, -0.1]),
        np.array([2.0, -2.0]),
        np.full(4, 0.5),
        np.full(4, -1.0),
        np.full(4, 1.0),
    )

    assert np.allclose(captured['matrix'][:2], 10.0)
    assert np.allclose(captured['matrix'][2:4], 1.0)
    assert np.allclose(captured['matrix'][4:], 2.0 * np.eye(4))
    assert np.allclose(captured['target'][:2], [1.0, -1.0])
    assert np.allclose(captured['target'][2:4], [1.0, -1.0])


def test_zero_balance_scale_leaves_posture_regularization_active():
    controller = _harness()
    controller.posture_weight = 1.0
    posture_target = np.array([0.1, -0.2, 0.3, -0.4])

    result = controller._solve_bounded_velocity(
        np.eye(2, 4),
        np.eye(2, 4),
        np.ones(2),
        np.ones(2),
        posture_target,
        np.full(4, -1.0),
        np.full(4, 1.0),
        balance_scale=0.0,
    )

    assert np.allclose(result, posture_target)


@pytest.mark.parametrize(
    ('gyro', 'expected'),
    (
        ([0.2, 0.0, 0.0], [0.02, 0.0]),
        ([0.0, -0.3, 0.0], [0.0, -0.03]),
    ),
)
def test_roll_pitch_gyro_feedback_has_reaction_momentum_sign(gyro, expected):
    controller = _harness()

    _, momentum_rhs = controller._reaction_targets(
        np.zeros((2, 7)),
        np.zeros((2, 7)),
        np.zeros(7),
        np.zeros(2),
        np.asarray(gyro),
        balance_scale=1.0,
    )

    assert np.allclose(momentum_rhs, expected)


def test_callable_body_gyro_is_rotated_into_model_frame():
    controller = _harness()
    controller.robot_model.state['imu_state'] = SimpleNamespace(
        get_gyroscope=lambda: [1.0, 0.0, 0.0],
    )
    rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])

    gyro, available = controller._torso_gyro(rotation)

    assert available
    assert np.allclose(gyro, [0.0, 1.0, 0.0])


def test_counter_velocity_bounds_include_one_step_position_room():
    controller = _harness()
    counter_q = np.array([0.999, 0.0, 0.0, 0.0])

    lower, upper = controller._counter_velocity_bounds(counter_q)

    assert upper[0] == pytest.approx(0.05)
    assert np.all(upper <= 0.5)
    assert np.all(lower >= -0.5)


def test_counter_excursion_bounds_limit_motion_relative_to_reference():
    controller = _harness()
    controller.q_counter_ref = np.zeros(4)
    controller.max_excursion = np.array([0.3, 0.4, 0.5, 0.6])
    counter_q = np.array([0.29, -0.39, 0.0, 0.0])
    lower = np.full(4, -2.0)
    upper = np.full(4, 2.0)

    lower, upper = controller._counter_excursion_bounds(
        counter_q, lower, upper,
    )

    assert np.allclose(lower, [-2.0, -0.5, -2.0, -2.0])
    assert np.allclose(upper, [0.5, 2.0, 2.0, 2.0])


def test_step_preserves_live_lower_body_and_publishes_atomically():
    controller = _harness()
    candidates = []
    controller.robot_model.check_within_limits = lambda q: (
        candidates.append(np.copy(q)) or True
    )
    q_target = np.zeros(14)
    dq_target = np.zeros(14)
    q_target[:7] = 5.0
    dq_target[:4] = [2.0, -2.0, 2.0, -2.0]

    result = controller.control_configuration_step(
        q_target,
        dq_target,
        balance_scale=0.5,
    )

    call = controller.low_cmd_handler.calls[0]
    assert call['joint_ids'] == list(range(13, 27))
    assert call['q'].shape == (14,)
    assert call['dq'].shape == (14,)
    assert call['tau'].shape == (14,)
    assert np.array_equal(call['tau'], np.arange(13.0, 27.0))
    assert np.allclose(call['q'][:7], 5.0)
    assert np.allclose(call['dq'][:4], [2.0, -2.0, 2.0, -2.0])
    assert np.allclose(call['q'][11:14], [0.2, -0.3, 0.4])
    assert np.allclose(call['dq'][11:14], 0.0)
    assert np.array_equal(result, call['dq'])
    assert np.allclose(candidates[0][:13], controller.robot_model.state['q'][:13])
    assert controller.diagnostics()['balance_scale'] == 0.5
    assert not controller.diagnostics()['clipped']
    assert controller.diagnostics()['moving_position_command_error'] == 0.0
    assert controller.diagnostics()['moving_velocity_command_error'] == 0.0


def test_counter_collision_cannot_block_moving_arm():
    controller = _harness()
    controller.robot_model.check_collision_free = lambda unused: False
    q_target = np.zeros(14)
    q_target[:7] = 0.4
    dq_target = np.zeros(14)
    dq_target[:7] = 0.2

    result = controller.control_configuration_step(q_target, dq_target)

    call = controller.low_cmd_handler.calls[0]
    measured = controller.robot_model.state['q'][13:27]
    assert controller.latest_status == 'counter_candidate_invalid'
    assert np.allclose(call['q'][:7], q_target[:7])
    assert np.allclose(call['dq'][:7], dq_target[:7])
    assert np.allclose(call['q'][7:], measured[7:])
    assert np.allclose(call['dq'][7:], 0.0)
    assert np.allclose(result[:7], dq_target[:7])


def test_counter_limit_rejection_cannot_block_moving_arm():
    controller = _harness()
    controller.robot_model.check_within_limits = lambda unused: False
    q_target = np.zeros(14)
    q_target[:7] = 0.4
    dq_target = np.zeros(14)
    dq_target[:7] = 0.2

    result = controller.control_configuration_step(q_target, dq_target)

    call = controller.low_cmd_handler.calls[0]
    measured = controller.robot_model.state['q'][13:27]
    assert controller.latest_status == 'counter_candidate_invalid'
    assert np.allclose(call['q'][:7], q_target[:7])
    assert np.allclose(call['dq'][:7], dq_target[:7])
    assert np.allclose(call['q'][7:], measured[7:])
    assert np.allclose(call['dq'][7:], 0.0)
    assert np.allclose(result[:7], dq_target[:7])


def test_counter_collision_backtracks_to_zero_and_keeps_valid_motion():
    controller = _harness()
    measured_counter = np.copy(
        controller.robot_model.state['q'][controller.counter_ids]
    )
    controller.robot_model.check_collision_free = lambda q: bool(
        np.allclose(q[controller.counter_ids], measured_counter)
    )
    q_target = np.zeros(14)
    q_target[:7] = 0.3
    dq_target = np.zeros(14)
    dq_target[:4] = 0.2

    result = controller.control_configuration_step(q_target, dq_target)

    call = controller.low_cmd_handler.calls[0]
    assert np.allclose(call['q'][:7], 0.3)
    assert np.allclose(result[controller.counter_active_local], 0.0)
    assert controller.latest_status == 'collision_backtracked'
    assert controller.latest_backtrack_scale == 0.0


def test_nonfinite_moving_input_is_rejected_without_publication():
    controller = _harness()
    q_target = np.full(14, 0.4)
    q_target[0] = np.nan

    with pytest.raises(ValueError, match='moving-arm command must be finite'):
        controller.control_configuration_step(q_target, np.zeros(14))

    assert controller.low_cmd_handler.calls == []


def test_gravity_failure_preserves_moving_arm_and_holds_counter_arm():
    controller = _harness()
    controller.robot_model.dynamics.get_gravity_compensation = (
        lambda unused: (_ for _ in ()).throw(RuntimeError('failed'))
    )
    q_target = np.zeros(14)
    q_target[:7] = 0.4
    dq_target = np.zeros(14)
    dq_target[:7] = 0.2

    result = controller.control_configuration_step(q_target, dq_target)

    call = controller.low_cmd_handler.calls[0]
    measured = controller.robot_model.state['q'][13:27]
    assert controller.latest_status == 'gravity_failure'
    assert np.allclose(call['q'][:7], q_target[:7])
    assert np.allclose(call['dq'][:7], dq_target[:7])
    assert np.allclose(call['q'][7:], measured[7:])
    assert np.allclose(call['dq'][7:], 0.0)
    assert np.allclose(call['tau'], 0.25)
    assert np.allclose(result[:7], dq_target[:7])


def test_estop_suppresses_model_update_and_publication():
    controller = _harness()
    controller.low_cmd_handler._estopped = True
    controller.update_robot_model = lambda: pytest.fail('updated after estop')

    result = controller.control_configuration_step(
        np.zeros(14),
        np.zeros(14),
    )

    assert np.allclose(result, 0.0)
    assert controller.latest_status == 'estopped'
    assert controller.low_cmd_handler.calls == []


@pytest.mark.parametrize(
    'config',
    (
        {'weights': {'com': np.nan}},
        {'gains': {'gyro': np.inf}},
        {'damping': 0.0},
        {'max_velocity': [1.0, 1.0, 0.0, 1.0]},
        {'com_velocity_scale': 0.0},
        {'momentum_scale': np.nan},
        {'posture_velocity_scale': -1.0},
    ),
)
def test_reactive_configuration_rejects_nonfinite_or_invalid_values(config):
    controller = ReactiveCounterBalanceController.__new__(
        ReactiveCounterBalanceController
    )

    with pytest.raises(ValueError, match='reactive_counter_balance'):
        controller._load_reactive_config(config)


def test_reactive_configuration_uses_conservative_physical_defaults():
    controller = ReactiveCounterBalanceController.__new__(
        ReactiveCounterBalanceController
    )

    controller._load_reactive_config({})

    assert controller.gyro_gain == 0.1
    assert controller.com_velocity_scale == 0.1
    assert controller.momentum_scale == 1.0
    assert controller.posture_velocity_scale == 1.0


@pytest.mark.parametrize('balance_scale', (-1.0, np.nan, None))
def test_balance_scale_requires_finite_nonnegative_value(balance_scale):
    controller = _harness()

    with pytest.raises(ValueError, match='balance_scale'):
        controller.control_configuration_step(
            np.zeros(14),
            np.zeros(14),
            balance_scale=balance_scale,
        )

    assert controller.low_cmd_handler.calls == []


def test_config_loader_preserves_reactive_physical_scales(tmp_path):
    config_path = tmp_path / 'reactive.yaml'
    config_path.write_text(
        'mode: debug\n'
        'gains:\n'
        '  kp: 1.0\n'
        '  kd: 0.1\n'
        '  ki: 0.0\n'
        'reactive_counter_balance:\n'
        '  com_velocity_scale: 0.2\n'
        '  momentum_scale: 3.0\n'
        '  posture_velocity_scale: 0.8\n',
        encoding='utf-8',
    )

    config = load_controller_config(
        'reactive.yaml',
        config_dir=tmp_path,
    )

    assert config['reactive_counter_balance'] == {
        'com_velocity_scale': 0.2,
        'momentum_scale': 3.0,
        'posture_velocity_scale': 0.8,
    }
