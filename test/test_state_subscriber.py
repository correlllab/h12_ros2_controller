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
    monkeypatch.setattr(channel_interface.time, 'monotonic', lambda: 12.5)
    monkeypatch.setattr(channel_interface.time, 'time', lambda: 100.0)
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
    assert second['tick'] == 1234
    assert second['unwrapped_tick'] == 1234
    assert second['tick_valid']
    assert second['arrival_monotonic'] == 12.5
    assert second['time_stamp'] == 100.0
    assert second['sequence'] == 1
    assert second['received']


def test_state_subscriber_unwraps_tick_and_rejects_out_of_order(monkeypatch):
    monkeypatch.setattr(channel_interface, 'ChannelSubscriber', _Subscriber)
    subscriber = channel_interface.StateSubscriber('rt/test_lowstate')
    message = SimpleNamespace(
        tick=0xfffffffe,
        imu_state=SimpleNamespace(),
        motor_state=[_motor_state(index) for index in range(NUM_MOTOR)],
    )
    subscriber._subscribe_low_state(message)
    message.tick = 1
    subscriber._subscribe_low_state(message)
    wrapped = subscriber.state
    message.tick = 0
    subscriber._subscribe_low_state(message)
    out_of_order = subscriber.state
    message.tick = 2
    subscriber._subscribe_low_state(message)
    recovered = subscriber.state

    assert wrapped['unwrapped_tick'] == 0x100000001
    assert wrapped['tick_valid']
    assert not out_of_order['tick_valid']
    assert out_of_order['unwrapped_tick'] == wrapped['unwrapped_tick']
    assert recovered['tick_valid']
    assert recovered['unwrapped_tick'] == 0x100000002


def test_state_subscriber_accepts_fresh_duplicate_tick(monkeypatch):
    monkeypatch.setattr(channel_interface, 'ChannelSubscriber', _Subscriber)
    subscriber = channel_interface.StateSubscriber('rt/test_lowstate')
    message = SimpleNamespace(
        tick=10,
        imu_state=SimpleNamespace(),
        motor_state=[_motor_state(index) for index in range(NUM_MOTOR)],
    )

    subscriber._subscribe_low_state(message)
    subscriber._subscribe_low_state(message)
    state = subscriber.state

    assert state['tick_valid']
    assert state['unwrapped_tick'] == 10
    assert state['sequence'] == 2
