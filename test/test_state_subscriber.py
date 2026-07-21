from types import SimpleNamespace

import numpy as np

import h12_ros2_controller.core.channel_interface as channel_interface
from h12_ros2_controller.utility.joint_definition import NUM_MOTOR


class _Subscriber:
    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type
        self.callback = None

    def Init(self, callback, queue_depth):
        self.callback = callback
        self.queue_depth = queue_depth

    def Close(self):
        pass


def _motor_state(index):
    return SimpleNamespace(
        mode=index % 2,
        q=float(index),
        dq=float(index + 1),
        ddq=float(index + 2),
        tau_est=float(index + 3),
        temperature=[index, index + 1],
        vol=float(index + 4),
        sensor=[index, index + 1],
        motorstate=index,
    )


def test_state_subscriber_preserves_tick_and_copies_consistent_snapshot(
        monkeypatch):
    monkeypatch.setattr(channel_interface, 'ChannelSubscriber', _Subscriber)
    subscriber = channel_interface.StateSubscriber('rt/test_lowstate')
    imu = SimpleNamespace(
        quaternion=[1.0, 0.0, 0.0, 0.0],
        gyroscope=[0.1, 0.2, 0.3],
        accelerometer=[1.0, 2.0, 3.0],
        rpy=[0.0, 0.0, 0.0],
        temperature=20,
    )
    message = SimpleNamespace(
        tick=1234,
        imu_state=imu,
        motor_state=[_motor_state(index) for index in range(NUM_MOTOR)],
    )

    subscriber._subscribe_low_state(message)
    first = subscriber.state
    imu.quaternion[0] = 0.0
    message.motor_state[0].q = -1.0
    first['imu_state'].quaternion[0] = -2.0
    first['q'][0] = -3.0
    second = subscriber.state

    assert second['imu_state'].quaternion == [1.0, 0.0, 0.0, 0.0]
    assert second['q'][0] == 0.0
    assert np.array_equal(second['q'], np.arange(NUM_MOTOR))
