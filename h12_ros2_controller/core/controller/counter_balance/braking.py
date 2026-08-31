from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BrakingLimits:
    '''Configure a conservative discrete braking rollout'''

    dt: float
    delay_steps: int
    horizon_steps: int
    max_acceleration: np.ndarray
    braking_effectiveness: np.ndarray
    max_acceleration_change: np.ndarray
    max_velocity: np.ndarray
    q_lower: np.ndarray
    q_upper: np.ndarray
    velocity_tolerance: np.ndarray
    acceleration_tolerance: np.ndarray


@dataclass(frozen=True)
class BrakingResult:
    '''Store one worst-case braking rollout and validity result'''

    valid: bool
    stopped: bool
    q: np.ndarray
    dq: np.ndarray
    acceleration: np.ndarray
    invalid_reason: str | None


@dataclass(frozen=True)
class ReconnectLimits:
    '''Configure one bounded reconnect trajectory'''

    dt: float
    duration: float
    max_acceleration: np.ndarray
    max_acceleration_change: np.ndarray
    max_velocity: np.ndarray
    max_torque_rate: np.ndarray
    q_lower: np.ndarray
    q_upper: np.ndarray


@dataclass(frozen=True)
class ReconnectResult:
    '''Store one validated quintic reconnect trajectory'''

    valid: bool
    q: np.ndarray
    dq: np.ndarray
    acceleration: np.ndarray
    torque: np.ndarray
    invalid_reason: str | None


def braking_rollout(
        q, dq, previous_acceleration, limits, collision_validator=None):
    '''Roll out delay and slew-limited worst-case counter braking'''
    q = _vector(q, 'q')
    dq = _vector(dq, 'dq')
    acceleration = _vector(previous_acceleration, 'previous_acceleration')
    parsed = _validated_limits(limits)
    if collision_validator is None:
        raise ValueError('collision_validator is required')
    reason = _state_violation(q, dq, parsed)
    if reason is not None or not collision_validator(q):
        return _result(
            False, np.zeros(4, dtype=bool), [q], [dq], [acceleration],
            reason or 'counter braking state is in collision',
        )

    direction = np.sign(dq)
    direction[direction == 0.0] = np.sign(acceleration[direction == 0.0])
    direction[direction == 0.0] = 1.0
    q_values = [np.copy(q)]
    dq_values = [np.copy(dq)]
    acceleration_values = [np.copy(acceleration)]
    stopped = np.zeros(4, dtype=bool)

    for step in range(parsed.horizon_steps):
        if step < parsed.delay_steps:
            target = direction * parsed.max_acceleration
            effectiveness = np.ones(4)
        else:
            effectiveness = parsed.braking_effectiveness
            target = -dq / (parsed.dt * effectiveness)
            target = np.clip(
                target,
                -parsed.max_acceleration,
                parsed.max_acceleration,
            )
        delta = np.clip(
            target - acceleration,
            -parsed.max_acceleration_change,
            parsed.max_acceleration_change,
        )
        command_acceleration = np.clip(
            acceleration + delta,
            -parsed.max_acceleration,
            parsed.max_acceleration,
        )
        realized_acceleration = effectiveness * command_acceleration
        next_dq = dq + parsed.dt * realized_acceleration
        stop_time = np.full(4, np.nan)
        moving = np.abs(realized_acceleration) > 1e-12
        stop_time[moving] = -dq[moving] / realized_acceleration[moving]

        probe_times = sorted({
            float(value) for value in stop_time
            if np.isfinite(value) and 0.0 < value < parsed.dt
        })
        for current_time in probe_times:
            probe_q = (
                q + current_time * dq
                + 0.5 * current_time ** 2 * realized_acceleration
            )
            probe_dq = dq + current_time * realized_acceleration
            reason = _state_violation(probe_q, probe_dq, parsed)
            if reason is not None or not collision_validator(probe_q):
                return _result(
                    False, stopped, q_values, dq_values,
                    acceleration_values,
                    reason or 'counter braking trajectory is in collision',
                )

        next_q = (
            q + parsed.dt * dq
            + 0.5 * parsed.dt ** 2 * realized_acceleration
        )
        acceleration = np.copy(command_acceleration)
        q = next_q
        dq = next_dq
        stopped = (
            np.abs(dq) <= parsed.velocity_tolerance
        ) & (
            np.abs(acceleration) <= parsed.acceleration_tolerance
        )
        q_values.append(np.copy(q))
        dq_values.append(np.copy(dq))
        acceleration_values.append(np.copy(acceleration))

        reason = _state_violation(q, dq, parsed)
        if reason is not None or not collision_validator(q):
            return _result(
                False, stopped, q_values, dq_values,
                acceleration_values,
                reason or 'counter braking state is in collision',
            )
        if np.all(stopped):
            return _result(
                True, stopped, q_values, dq_values,
                acceleration_values, None,
            )
    return _result(
        False, stopped, q_values, dq_values, acceleration_values,
        'counter velocity did not stop inside braking horizon',
    )


def reconnect_trajectory(
        q, dq, acceleration, torque,
        target_q, target_dq, target_acceleration, target_torque,
        limits, collision_validator=None):
    '''Build a bounded quintic trajectory to the latest upstream reference'''
    q = _vector(q, 'q')
    dq = _vector(dq, 'dq')
    acceleration = _vector(acceleration, 'acceleration')
    torque = _vector(torque, 'torque')
    target_q = _vector(target_q, 'target_q')
    target_dq = _vector(target_dq, 'target_dq')
    target_acceleration = _vector(target_acceleration, 'target_acceleration')
    target_torque = _vector(target_torque, 'target_torque')
    parsed = _validated_reconnect_limits(limits)
    if collision_validator is None:
        raise ValueError('collision_validator is required')
    steps = int(round(parsed.duration / parsed.dt))
    if steps < 1 or not np.isclose(steps * parsed.dt, parsed.duration):
        raise ValueError('reconnect duration must align with dt')

    duration = parsed.duration
    matrix = np.array([
        [duration ** 3, duration ** 4, duration ** 5],
        [3 * duration ** 2, 4 * duration ** 3, 5 * duration ** 4],
        [6 * duration, 12 * duration ** 2, 20 * duration ** 3],
    ])
    coefficients = np.zeros((4, 6))
    coefficients[:, 0] = q
    coefficients[:, 1] = dq
    coefficients[:, 2] = 0.5 * acceleration
    for joint in range(4):
        known_q = q[joint] + duration * dq[joint]
        known_q += 0.5 * duration ** 2 * acceleration[joint]
        known_dq = dq[joint] + duration * acceleration[joint]
        rhs = np.array([
            target_q[joint] - known_q,
            target_dq[joint] - known_dq,
            target_acceleration[joint] - acceleration[joint],
        ])
        coefficients[joint, 3:] = np.linalg.solve(matrix, rhs)

    times = np.arange(steps + 1) * parsed.dt
    q_values = np.zeros((steps + 1, 4))
    dq_values = np.zeros_like(q_values)
    acceleration_values = np.zeros_like(q_values)
    for index, current_time in enumerate(times):
        powers = np.array([current_time ** order for order in range(6)])
        q_values[index] = coefficients @ powers
        dq_values[index] = (
            coefficients[:, 1]
            + 2 * coefficients[:, 2] * current_time
            + 3 * coefficients[:, 3] * current_time ** 2
            + 4 * coefficients[:, 4] * current_time ** 3
            + 5 * coefficients[:, 5] * current_time ** 4
        )
        acceleration_values[index] = (
            2 * coefficients[:, 2]
            + 6 * coefficients[:, 3] * current_time
            + 12 * coefficients[:, 4] * current_time ** 2
            + 20 * coefficients[:, 5] * current_time ** 3
        )
    blend = (times / duration)[:, None]
    torque_values = torque + blend * (target_torque - torque)
    reason = _reconnect_violation(
        q_values, dq_values, acceleration_values, torque_values, parsed,
    )
    if reason is None and any(
            not collision_validator(value) for value in q_values):
        reason = 'reconnect trajectory is in collision'
    return ReconnectResult(
        valid=reason is None,
        q=q_values,
        dq=dq_values,
        acceleration=acceleration_values,
        torque=torque_values,
        invalid_reason=reason,
    )


def _validated_limits(limits):
    if not isinstance(limits, BrakingLimits):
        raise ValueError('limits must be BrakingLimits')
    if (
        not np.isfinite(limits.dt) or limits.dt <= 0.0
        or limits.delay_steps < 0 or limits.horizon_steps < 1
    ):
        raise ValueError('braking timing is invalid')
    values = {
        'max_acceleration': limits.max_acceleration,
        'braking_effectiveness': limits.braking_effectiveness,
        'max_acceleration_change': limits.max_acceleration_change,
        'max_velocity': limits.max_velocity,
        'q_lower': limits.q_lower,
        'q_upper': limits.q_upper,
        'velocity_tolerance': limits.velocity_tolerance,
        'acceleration_tolerance': limits.acceleration_tolerance,
    }
    parsed = {}
    for name, value in values.items():
        parsed[name] = _vector(value, name)
    if any(np.any(parsed[name] <= 0.0) for name in (
            'max_acceleration', 'braking_effectiveness',
            'max_acceleration_change', 'max_velocity',
            'velocity_tolerance', 'acceleration_tolerance')):
        raise ValueError('braking dynamic limits must be positive')
    if np.any(parsed['braking_effectiveness'] > 1.0):
        raise ValueError('braking effectiveness must not exceed one')
    if np.any(parsed['q_lower'] >= parsed['q_upper']):
        raise ValueError('braking position limits are invalid')
    return BrakingLimits(
        dt=float(limits.dt),
        delay_steps=int(limits.delay_steps),
        horizon_steps=int(limits.horizon_steps),
        **parsed,
    )


def _state_violation(q, dq, limits):
    if np.any(q < limits.q_lower) or np.any(q > limits.q_upper):
        return 'counter position left braking bounds'
    if np.any(np.abs(dq) > limits.max_velocity):
        return 'counter velocity left braking bounds'
    return None


def _validated_reconnect_limits(limits):
    if not isinstance(limits, ReconnectLimits):
        raise ValueError('limits must be ReconnectLimits')
    if (
        not np.isfinite(limits.dt) or limits.dt <= 0.0
        or not np.isfinite(limits.duration) or limits.duration <= 0.0
    ):
        raise ValueError('reconnect timing is invalid')
    values = {}
    for name in (
            'max_acceleration', 'max_acceleration_change', 'max_velocity',
            'max_torque_rate', 'q_lower', 'q_upper'):
        values[name] = _vector(getattr(limits, name), name)
    if any(np.any(values[name] <= 0.0) for name in (
            'max_acceleration', 'max_acceleration_change', 'max_velocity',
            'max_torque_rate')):
        raise ValueError('reconnect dynamic limits must be positive')
    if np.any(values['q_lower'] >= values['q_upper']):
        raise ValueError('reconnect position limits are invalid')
    return ReconnectLimits(
        dt=float(limits.dt),
        duration=float(limits.duration),
        **values,
    )


def _reconnect_violation(q, dq, acceleration, torque, limits):
    if np.any(q < limits.q_lower) or np.any(q > limits.q_upper):
        return 'reconnect position left bounds'
    if np.any(np.abs(dq) > limits.max_velocity):
        return 'reconnect velocity left bounds'
    if np.any(np.abs(acceleration) > limits.max_acceleration):
        return 'reconnect acceleration left bounds'
    if np.any(
            np.abs(np.diff(acceleration, axis=0))
            > limits.max_acceleration_change):
        return 'reconnect acceleration slew left bounds'
    torque_rate = np.abs(np.diff(torque, axis=0)) / limits.dt
    if np.any(torque_rate > limits.max_torque_rate):
        return 'reconnect torque rate left bounds'
    return None


def _result(valid, stopped, q, dq, acceleration, reason):
    return BrakingResult(
        valid=valid,
        stopped=bool(np.all(stopped)),
        q=np.asarray(q),
        dq=np.asarray(dq),
        acceleration=np.asarray(acceleration),
        invalid_reason=reason,
    )


def _vector(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (4,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite with shape (4,)')
    return np.copy(result)
