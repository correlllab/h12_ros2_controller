'''Listener-free DDS reader that returns only the newest sample'''

from typing import Any

from cyclonedds.domain import DomainParticipant
from cyclonedds.internal import InvalidSample
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from unitree_sdk2py.core.channel import ChannelFactory


# KEEP_LAST(1) lets CycloneDDS's C receive thread overwrite the cached sample,
# so a consumer that reads late gets the newest state instead of a queued old
# one. There is deliberately no listener: a Python callback runs on the receive
# thread and must take the GIL, and in a busy process a receive thread waiting
# on the GIL leaves packets queued in the socket, which shows up as state that
# is seconds old.
LATEST_ONLY_QOS = Qos(
    Policy.History.KeepLast(1),
    Policy.Reliability.BestEffort,
)
# Upper bound on samples drained per take; with KEEP_LAST(1) at most one is
# ever cached, so this only guards against a QoS mismatch.
TAKE_BATCH = 8
# LowState_.tick is a uint32 millisecond counter
TICK_WRAP = 1 << 32
# How fast [ms per s] the tick-lag baseline may rise to follow clock drift
# between the robot's tick and this host (500 ppm, well above crystal drift).
# A transport delay that builds faster than this registers as lag.
LAG_DRIFT_MS_PER_S = 0.5


def sdk_participant() -> DomainParticipant:
    '''
    The participant unitree_sdk2py's ChannelFactory created, so readers share
    its domain and network interface. ChannelFactory is a singleton that keeps
    it in a private attribute (on the instance or the class, depending on the
    SDK version); init_channel_factory() must have run first.
    '''
    participant = getattr(ChannelFactory(), '_ChannelFactory__participant', None)
    if participant is None:
        raise RuntimeError('init_channel_factory() must run before creating DDS readers')
    return participant


class LatestReader:
    '''Poll the newest sample of a DDS topic without a receive-thread callback'''
    def __init__(self, participant: DomainParticipant, topic_name: str, msg_type: Any):
        self._topic = Topic(participant, topic_name, msg_type)
        self._reader = DataReader(participant, self._topic, LATEST_ONLY_QOS)

    def take_latest(self) -> Any | None:
        '''Return the newest valid sample not yet taken, or None'''
        latest = None
        for sample in self._reader.take(N=TAKE_BATCH):
            if not isinstance(sample, InvalidSample):
                latest = sample
        return latest

    def close(self) -> None:
        '''Release the reader'''
        self._reader = None
        self._topic = None


class TickLagTracker:
    '''
    How late [ms] samples stamped with the robot's millisecond tick arrive.
    The baseline is the earliest arrival seen; it may rise by
    LAG_DRIFT_MS_PER_S to follow clock drift, so only a delay that builds
    faster than drift registers.
    '''
    def __init__(self):
        self._last_tick = None
        self._wraps = 0
        self._base_ms = None
        self._base_time = 0.0

    def update(self, tick: int, now: float) -> float:
        tick = int(tick)
        if self._last_tick is not None and tick < self._last_tick - TICK_WRAP // 2:
            self._wraps += 1
        self._last_tick = tick
        offset_ms = now * 1000.0 - (tick + self._wraps * TICK_WRAP)
        if self._base_ms is None:
            self._base_ms = offset_ms
        else:
            drift_ms = LAG_DRIFT_MS_PER_S * (now - self._base_time)
            self._base_ms = min(offset_ms, self._base_ms + drift_ms)
        self._base_time = now
        return offset_ms - self._base_ms
