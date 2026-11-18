"""Pure, opt-in post-fade references, NOT a hardware safety controller

The caller owns mode changes, full-robot/swept collision validation, transport,
and an independent expiring command lease. STOP_REQUEST emits no replacement
command: the previously buffered command may remain live until that lease acts.
No device SDK, controller instance, clock, publisher or enable operation is used.
Bounds are engineering assumptions, not commissioned hardware limits.
"""

from dataclasses import dataclass

import numpy as np


def _vector(value):
    array = np.asarray(value, dtype=float)
    if array.ndim != 1 or not array.size or not np.all(np.isfinite(array)):
        raise ValueError('expected a nonempty finite vector')
    return tuple(float(x) for x in array)


@dataclass(frozen=True)
class Command:
    """Immutable owned-joint local-buffer values, including feedforward effort"""

    q: tuple
    dq: tuple
    tau: tuple

    def __post_init__(self):
        for name in ('q', 'dq', 'tau'):
            object.__setattr__(self, name, _vector(getattr(self, name)))
        if len(self.q) != len(self.dq) or len(self.q) != len(self.tau):
            raise ValueError('command dimensions differ')


@dataclass(frozen=True)
class BoundValues:
    """Explicit research-only assumptions; constructing these never enables hardware"""

    dt: float
    max_source_age: float
    max_buffer_age: float
    max_commit_age: float
    q_lower: tuple
    q_upper: tuple
    excursion: tuple
    lead: tuple
    velocity: tuple
    acceleration: tuple
    effort: tuple
    kp: tuple
    kd: tuple

    def __post_init__(self):
        for name in ('dt', 'max_source_age', 'max_buffer_age', 'max_commit_age'):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0:
                raise ValueError('timing bounds must be finite and positive')
            object.__setattr__(self, name, value)
        names = ('q_lower', 'q_upper', 'excursion', 'lead', 'velocity',
                 'acceleration', 'effort', 'kp', 'kd')
        for name in names:
            object.__setattr__(self, name, _vector(getattr(self, name)))
        if len({len(getattr(self, name)) for name in names}) != 1:
            raise ValueError('bound dimensions differ')
        if np.any(np.array(self.q_lower) >= self.q_upper):
            raise ValueError('position interval is empty')
        for name in names[2:]:
            value = np.array(getattr(self, name))
            if np.any(value < 0) or (name not in ('kp', 'kd') and np.any(value == 0)):
                raise ValueError('invalid ' + name)
        if self.max_buffer_age < self.dt:
            raise ValueError('buffer age must permit one nominal step')

    @classmethod
    def research_fixture(cls):
        """Return uncommissioned four-joint prototype hypotheses, not safe defaults"""
        return cls(
            dt=1 / 30, max_source_age=.05, max_buffer_age=.10,
            max_commit_age=.02, q_lower=(-2.,) * 4, q_upper=(2.,) * 4,
            excursion=(.58,) * 4, lead=(.04,) * 4, velocity=(.20,) * 4,
            acceleration=(1.,) * 4, effort=(24., 24., 10.8, 10.8),
            kp=(240., 200., 150., 150.), kd=(12.,) * 4,
        )


@dataclass(frozen=True)
class Proposal:
    """Opaque single-use transaction; commit requires this exact object"""

    command: Command
    trial_id: str
    created_at: float
    source_time: float


class ReferencePlanner:
    """Propose one nominal step, then commit only an exact accepted local buffer

    ``validator(previous, candidate)`` must return literal True after validating
    the assembled full command and swept path outside this reduced planner.
    It must not write commands. A false result or exception fails closed.
    All vectors describe the same owned joints; no other joint is modified.
    A commit is NOT DDS delivery, actuator acknowledgement or lease renewal.
    """

    def __init__(self, bounds, validator):
        if not isinstance(bounds, BoundValues) or not callable(validator):
            raise ValueError('explicit bounds and validator are required')
        self.bounds = bounds
        self.validator = validator
        self.state = 'idle'
        self.reason = None
        self._external_stop = False
        self._trial_id = None
        self._used_trials = set()
        self._last = None
        self._pending = None
        self._rest = None
        self._accepted_at = None
        self._source_time = None

    @property
    def last_accepted(self):
        """Return immutable historical evidence, never a fallback command"""
        return self._last

    @property
    def stop_requested(self):
        return self.state in ('stop_requested', 'stopped')

    def _latch(self, reason):
        self.state = 'stopped' if self._external_stop else 'stop_requested'
        self.reason = reason
        self._pending = None
        return None

    def stop(self, reason='external_stop'):
        """Invalidate pending work terminally; no trial reset clears external STOP"""
        self._external_stop = True
        self._latch(reason)

    def reset_trial(self, trial_id, rest, accepted_buffer, *, accepted_at,
                    mode_ack):
        """Seed from a confirmed local buffer, never from measured position

        A new unique trial and explicit external mode acknowledgement are needed
        after local faults. mode_ack is only a caller assertion, not verification
        of a motor mode or independent lease. External STOP is irreversible.
        """
        if self._external_stop:
            raise RuntimeError('external stop cannot be reset')
        if (not isinstance(trial_id, str) or not trial_id
                or trial_id in self._used_trials or mode_ack is not True):
            raise ValueError('new trial and positive mode acknowledgement required')
        if not isinstance(accepted_buffer, Command):
            raise ValueError('confirmed Command buffer required')
        rest = np.array(_vector(rest))
        if len(rest) != len(self.bounds.kp) or len(accepted_buffer.q) != len(rest):
            raise ValueError('trial dimensions differ')
        if not np.isfinite(accepted_at):
            raise ValueError('nonfinite acceptance time')
        if np.any(rest < self.bounds.q_lower) or np.any(rest > self.bounds.q_upper):
            raise ValueError('rest outside position limits')
        self._trial_id = trial_id
        self._used_trials.add(trial_id)
        self._rest = rest
        self._last = accepted_buffer
        self._accepted_at = float(accepted_at)
        self._source_time = None
        self._pending = None
        self.state = 'ready'
        self.reason = None

    def _guard(self, command, q, dq):
        b = self.bounds
        ref, vel, tau = map(np.array, (command.q, command.dq, command.tau))
        lo = np.maximum(b.q_lower, self._rest - b.excursion)
        hi = np.minimum(b.q_upper, self._rest + b.excursion)
        if np.any(q < lo) or np.any(q > hi):
            return 'measured_position_or_excursion'
        if np.any(ref < lo) or np.any(ref > hi):
            return 'position_or_excursion'
        if np.any(abs(vel) > b.velocity):
            return 'velocity'
        if np.any(abs(ref - q) > b.lead):
            return 'lead'
        effort = np.array(b.kp) * (ref - q) + np.array(b.kd) * (vel - dq) + tau
        if not np.all(np.isfinite(effort)) or np.any(abs(effort) > b.effort):
            return 'total_pd_ff_effort'
        # reserve a nominal response interval plus continuous braking distance
        # this assumes frozen measurement, not a certified moving-plant envelope
        reserve = abs(vel) * b.dt + vel ** 2 / (2 * np.array(b.acceleration))
        stop = ref + np.sign(vel) * reserve
        if (np.any(stop < lo) or np.any(stop > hi)
                or np.any(abs(stop - q) > b.lead)):
            return 'stopping_margin'
        return None

    def propose(self, q, dq, tau, *, now, source_time, trial_id, postfade,
                eligible=True, mode_ack=True):
        """Return a proposal or None; rejected commands always request STOP

        No reference/cadence state changes before activation or on skipped slots.
        dt is always nominal, never elapsed time; missed slots never accumulate.
        Fresh source timestamps and bounded buffer/commit ages are mandatory.
        """
        if self.stop_requested:
            return None
        if self.state == 'idle':
            return self._latch('missing_seed')
        if trial_id != self._trial_id or mode_ack is not True:
            return self._latch('trial_or_mode_changed')
        if self._pending is not None:
            return self._latch('uncommitted_proposal')
        if not postfade or not eligible:
            if self.state == 'active':
                return self._latch('recovery_authorization_revoked')
            return None
        try:
            q, dq, tau = (np.array(_vector(x)) for x in (q, dq, tau))
            if any(len(x) != len(self._rest) for x in (q, dq, tau)):
                raise ValueError('observation dimensions differ')
            if not np.all(np.isfinite([now, source_time])):
                raise ValueError('nonfinite clock')
            b = self.bounds
            age = now - self._accepted_at
            if age < 0 or source_time > now or now - source_time > b.max_source_age:
                return self._latch('stale_or_reversed_clock')
            if age > b.max_buffer_age:
                return self._latch('stale_buffer')
            if age < b.dt - 1e-12:
                return None
            if self._source_time is not None and source_time <= self._source_time:
                return self._latch('stale_source')
            reason = self._guard(self._last, q, dq)
            if reason:
                return self._latch('inherited_' + reason)
            old_q, old_v = np.array(self._last.q), np.array(self._last.dq)
            error = self._rest - old_q
            accel = np.array(b.acceleration)
            # leave braking room rather than arriving with nonzero velocity
            speed = np.sqrt((accel * b.dt) ** 2 + 2 * accel * abs(error)) - accel * b.dt
            target_v = np.sign(error) * np.minimum(
                np.minimum(b.velocity, speed), abs(error) / (4 * b.dt))
            new_v = old_v + np.clip(target_v - old_v, -accel * b.dt, accel * b.dt)
            new_q = old_q + .5 * b.dt * (old_v + new_v)
            if np.any((self._rest - new_q) * error < -1e-14):
                return self._latch('target_overshoot')
            command = Command(new_q, new_v, tau)
            reason = self._guard(command, q, dq)
            if reason:
                return self._latch(reason)
            if self.validator(self._last, command) is not True:
                return self._latch('external_validation')
            proposal = Proposal(command, trial_id, float(now), float(source_time))
        except Exception as exc:
            return self._latch('proposal_error:' + type(exc).__name__)
        self._pending = proposal
        return proposal

    def commit(self, proposal, *, accepted, actual_buffer=None, now,
               error=None):
        """Acknowledge exact local-buffer acceptance, or latch a write failure

        Catch transport exceptions outside the planner and pass them as error.
        Never call the writer on None. False, missing/mismatched buffer readback,
        exception, stale commit or replay invalidate the transaction without
        changing last_accepted. No fault automatically rearms.
        """
        if self.stop_requested:
            return False
        if (proposal is None or proposal is not self._pending
                or accepted is not True or error is not None):
            self._latch('local_buffer_write_failed')
            return False
        try:
            if isinstance(now, (bool, np.bool_)) or np.ndim(now) != 0:
                raise ValueError('clock must be a numeric scalar')
            now = float(now)
        except (ValueError, TypeError, OverflowError):
            self._latch('malformed_commit_clock')
            return False
        if (not np.isfinite(now) or now < proposal.created_at
                or now - proposal.created_at > self.bounds.max_commit_age
                or now - proposal.source_time > self.bounds.max_source_age):
            self._latch('stale_commit')
            return False
        if not isinstance(actual_buffer, Command) or actual_buffer != proposal.command:
            self._latch('local_buffer_mismatch')
            return False
        self._last = actual_buffer
        self._accepted_at = float(now)
        self._source_time = proposal.source_time
        self._pending = None
        self.state = 'active'
        return True
