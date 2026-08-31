from types import SimpleNamespace

import numpy as np

import h12_ros2_controller.core.controller.counter_balance.base_response_observation as observation
from h12_ros2_controller.core.controller.counter_balance.base_response_observation import (
    BaseResponseObservationPipeline,
    SupportKinematicsObserver,
    SupportValidityObserver,
    SupportValidityThresholds,
    counter_state_observation,
)


def _state():
    return {
        'arrival_monotonic': 10.0,
        'tick': 123,
        'tick_valid': True,
        'sequence': 4,
        'received': True,
        'q': np.arange(27, dtype=np.float64),
        'dq': np.arange(27, dtype=np.float64) + 1.0,
    }


def test_counter_observation_uses_one_strict_measured_snapshot():
    observation = counter_state_observation(
        _state(), [13, 14, 15, 16], now=10.01, max_age=0.04,
    )

    assert observation.valid
    assert observation.sequence == 4
    assert observation.tick == 123
    assert np.array_equal(observation.q_counter, [13, 14, 15, 16])
    assert np.array_equal(observation.dq_counter, [14, 15, 16, 17])
    assert not observation.q_counter.flags.writeable


def test_counter_observation_rejects_stale_state_without_fallback():
    observation = counter_state_observation(
        _state(), [13, 14, 15, 16], now=10.1, max_age=0.04,
    )

    assert not observation.valid
    assert observation.invalid_reason == 'low-state sample is stale'
    assert np.allclose(observation.q_counter, 0.0)
    assert np.allclose(observation.dq_counter, 0.0)


def _support_sample(
        observer, timestamp, com_x=0.0, left_twist=None):
    return observer.update(
        timestamp,
        left_position=np.array([0.0, 0.1, 0.0]),
        right_position=np.array([0.0, -0.1, 0.0]),
        left_rotation=np.eye(3),
        right_rotation=np.eye(3),
        left_twist=np.zeros(6) if left_twist is None else left_twist,
        right_twist=np.zeros(6),
        com_position=np.array([com_x, 0.0, 0.8]),
        base_position=np.array([0.0, 0.0, 1.0]),
        base_rotation=np.eye(3),
    )


def test_support_observer_builds_canonical_support_and_derivatives():
    observer = SupportKinematicsObserver(filter_time_constant=0.02)
    first_base, first_support = _support_sample(observer, 1.0)
    second_base, second_support = _support_sample(
        observer, 1.02, com_x=0.002,
    )

    assert first_support.valid
    assert np.allclose(first_support.support_origin, 0.0)
    assert np.allclose(first_support.support_rotation, np.eye(3))
    assert not first_base.valid
    assert second_base.valid
    assert second_base.base_height == 1.0
    assert np.allclose(second_base.angular_velocity_xy, [0.0, 0.0])
    assert np.allclose(second_base.com_velocity_xy, [0.05, 0.0])


def test_support_validity_uses_hysteresis_and_foot_twist():
    observer = SupportKinematicsObserver()
    validity = SupportValidityObserver(SupportValidityThresholds(
        enter_samples=2,
        exit_samples=2,
    ))
    _base, support = _support_sample(observer, 1.0)
    validity.capture_reference(support)
    first = validity.update(support, now=1.0)
    second = validity.update(support, now=1.0)
    fast_twist = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])
    _base, moving = _support_sample(
        observer, 1.02, left_twist=fast_twist,
    )
    third = validity.update(moving, now=1.02)
    fourth = validity.update(moving, now=1.02)

    assert not first.double_support_valid
    assert second.double_support_valid
    assert third.double_support_valid
    assert not fourth.double_support_valid


def test_pipeline_uses_one_snapshot_for_all_derived_observations(monkeypatch):
    transforms = {
        0: SimpleNamespace(
            translation=np.array([0.0, 0.1, 0.0]),
            rotation=np.eye(3),
        ),
        1: SimpleNamespace(
            translation=np.array([0.0, -0.1, 0.0]),
            rotation=np.eye(3),
        ),
        2: SimpleNamespace(
            translation=np.array([0.0, 0.0, 0.5]),
            rotation=np.eye(3),
        ),
    }

    class Model:
        def getFrameId(self, name):
            return {
                'left_ankle_roll_link': 0,
                'right_ankle_roll_link': 1,
                'torso_link': 2,
            }[name]

    class Robot:
        model = Model()
        data = SimpleNamespace(oMf=transforms)

        def update_kinematics_from_snapshot(self, state):
            q = np.zeros(34)
            q[6] = 1.0
            return q, np.zeros(33)

    monkeypatch.setattr(
        observation.pin,
        'getFrameVelocity',
        lambda *args: SimpleNamespace(
            linear=np.zeros(3), angular=np.zeros(3),
        ),
    )
    monkeypatch.setattr(
        observation.pin,
        'centerOfMass',
        lambda model, data, *args: (
            setattr(data, 'vcom', [np.zeros(3)])
            or np.array([0.0, 0.0, 0.8])
        ),
    )
    state = _state()
    state['imu_state'] = SimpleNamespace(
        quaternion=[1.0, 0.0, 0.0, 0.0],
        gyroscope=[0.1, 0.0, 0.0],
    )
    pipeline = BaseResponseObservationPipeline(
        Robot(),
        [13, 14, 15, 16],
        validity_observer=SupportValidityObserver(
            SupportValidityThresholds(enter_samples=1),
        ),
    )

    pipeline.capture_reference(state, now=10.01)
    first = pipeline.update(state, now=10.01)
    state['arrival_monotonic'] = 10.02
    second = pipeline.update(state, now=10.03)

    assert first.counter.valid
    assert first.support.valid
    assert first.support_validity.double_support_valid
    assert not first.base.valid
    assert second.base.valid

    state['imu_state'].quaternion = [np.nan, 0.0, 0.0, 0.0]
    invalid = pipeline.update(state, now=10.03)
    assert invalid.base is None


def test_pipeline_reference_capture_rejects_stale_snapshot():
    pipeline = BaseResponseObservationPipeline(
        SimpleNamespace(),
        [13, 14, 15, 16],
    )

    try:
        pipeline.capture_reference(_state(), now=10.1)
    except ValueError as error:
        assert str(error) == 'could not capture support reference'
    else:
        raise AssertionError('stale reference capture did not fail')

    assert not pipeline.reference_captured
