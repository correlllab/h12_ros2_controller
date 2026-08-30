from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ReactionSample:
    '''Store one filtered arm-reaction measurement'''

    valid: bool
    dt: float
    counter_h: np.ndarray
    moving_h: np.ndarray
    total_h: np.ndarray
    counter_hdot: np.ndarray
    moving_hdot: np.ndarray
    total_hdot: np.ndarray
    counter_acceleration: np.ndarray
    base_angular_acceleration: np.ndarray


@dataclass(frozen=True)
class ReactionSeedDiagnostic:
    '''Store one bounded reaction-seed calculation'''

    valid: bool
    desired_measured_rate: np.ndarray
    unbounded_acceleration: np.ndarray
    clipped_acceleration: np.ndarray
    saturation_mask: np.ndarray
    achieved_measured_rate: np.ndarray
    residual: np.ndarray
    unbounded_feasible: bool


class ReactionObserver:
    '''Estimate measured arm momentum and acceleration rates'''

    def __init__(self, filter_time_constant=0.06, max_dt=0.1):
        self.filter_time_constant = float(filter_time_constant)
        self.max_dt = float(max_dt)
        if self.filter_time_constant <= 0.0 or self.max_dt <= 0.0:
            raise ValueError('reaction observer timing must be positive')
        self.reset()

    def reset(self):
        '''Clear measurement history'''
        self._previous = None
        self._counter_hdot = np.zeros(2)
        self._moving_hdot = np.zeros(2)
        self._counter_acceleration = np.zeros(4)
        self._base_angular_acceleration = np.zeros(2)

    def update(self, timestamp, counter_dq, counter_h, moving_h, gyro_xy):
        '''Update finite-difference rates from one measured sample'''
        timestamp = float(timestamp)
        counter_dq = _vector(counter_dq, 4, 'counter_dq')
        counter_h = _vector(counter_h, 2, 'counter_h')
        moving_h = _vector(moving_h, 2, 'moving_h')
        gyro_xy = _vector(gyro_xy, 2, 'gyro_xy')
        total_h = counter_h + moving_h
        current = (timestamp, counter_dq, counter_h, moving_h, gyro_xy)
        if self._previous is None:
            self._previous = current
            return self._sample(False, np.nan, counter_h, moving_h)

        previous = self._previous
        dt = timestamp - previous[0]
        self._previous = current
        if not np.isfinite(dt) or dt <= 0.0 or dt > self.max_dt:
            self._counter_hdot[:] = 0.0
            self._moving_hdot[:] = 0.0
            self._counter_acceleration[:] = 0.0
            self._base_angular_acceleration[:] = 0.0
            return self._sample(False, dt, counter_h, moving_h)

        alpha = dt / (self.filter_time_constant + dt)
        self._counter_hdot = _low_pass(
            self._counter_hdot,
            (counter_h - previous[2]) / dt,
            alpha,
        )
        self._moving_hdot = _low_pass(
            self._moving_hdot,
            (moving_h - previous[3]) / dt,
            alpha,
        )
        self._counter_acceleration = _low_pass(
            self._counter_acceleration,
            (counter_dq - previous[1]) / dt,
            alpha,
        )
        self._base_angular_acceleration = _low_pass(
            self._base_angular_acceleration,
            (gyro_xy - previous[4]) / dt,
            alpha,
        )
        return ReactionSample(
            valid=True,
            dt=dt,
            counter_h=counter_h,
            moving_h=moving_h,
            total_h=total_h,
            counter_hdot=np.copy(self._counter_hdot),
            moving_hdot=np.copy(self._moving_hdot),
            total_hdot=np.copy(self._counter_hdot + self._moving_hdot),
            counter_acceleration=np.copy(self._counter_acceleration),
            base_angular_acceleration=np.copy(
                self._base_angular_acceleration,
            ),
        )

    def _sample(self, valid, dt, counter_h, moving_h):
        return ReactionSample(
            valid=valid,
            dt=dt,
            counter_h=counter_h,
            moving_h=moving_h,
            total_h=counter_h + moving_h,
            counter_hdot=np.copy(self._counter_hdot),
            moving_hdot=np.copy(self._moving_hdot),
            total_hdot=np.copy(self._counter_hdot + self._moving_hdot),
            counter_acceleration=np.copy(self._counter_acceleration),
            base_angular_acceleration=np.copy(
                self._base_angular_acceleration,
            ),
        )


def predict_frozen_momentum_rate(
        current_counter_map, current_moving_map,
        current_counter_dq, current_moving_dq,
        next_counter_map, next_moving_map,
        next_counter_dq, next_moving_dq, dt):
    '''Predict endpoint arm momentum rates from frozen maps'''
    current_counter_map = _matrix(
        current_counter_map, (2, 4), 'current_counter_map',
    )
    current_moving_map = _matrix(
        current_moving_map, (2, 7), 'current_moving_map',
    )
    next_counter_map = _matrix(
        next_counter_map, (2, 4), 'next_counter_map',
    )
    next_moving_map = _matrix(
        next_moving_map, (2, 7), 'next_moving_map',
    )
    current_counter_dq = _vector(
        current_counter_dq, 4, 'current_counter_dq',
    )
    current_moving_dq = _vector(
        current_moving_dq, 7, 'current_moving_dq',
    )
    next_counter_dq = _vector(next_counter_dq, 4, 'next_counter_dq')
    next_moving_dq = _vector(next_moving_dq, 7, 'next_moving_dq')
    dt = float(dt)
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError('prediction dt must be finite and positive')

    counter_rate = (
        next_counter_map @ next_counter_dq
        - current_counter_map @ current_counter_dq
    ) / dt
    moving_rate = (
        next_moving_map @ next_moving_dq
        - current_moving_map @ current_moving_dq
    ) / dt
    return counter_rate, moving_rate, counter_rate + moving_rate


def reaction_seed_diagnostic(
        current_counter_map, next_counter_map, current_counter_dq,
        desired_measured_rate, effectiveness, control_lower, control_upper, dt):
    '''Calculate a bounded local acceleration for a desired reaction'''
    current_counter_map = _matrix(
        current_counter_map, (2, 4), 'current_counter_map',
    )
    next_counter_map = _matrix(
        next_counter_map, (2, 4), 'next_counter_map',
    )
    current_counter_dq = _vector(
        current_counter_dq, 4, 'current_counter_dq',
    )
    desired_measured_rate = _vector(
        desired_measured_rate, 2, 'desired_measured_rate',
    )
    effectiveness = _vector(effectiveness, 2, 'effectiveness')
    control_lower = _vector(control_lower, 4, 'control_lower')
    control_upper = _vector(control_upper, 4, 'control_upper')
    dt = float(dt)
    if (
        not np.isfinite(dt) or dt <= 0.0
        or np.any(effectiveness <= 0.0)
        or np.any(control_lower > control_upper)
    ):
        raise ValueError('reaction seed inputs are infeasible')

    drift = (
        next_counter_map @ current_counter_dq
        - current_counter_map @ current_counter_dq
    ) / dt
    desired_model_rate = desired_measured_rate / effectiveness
    unbounded = np.linalg.lstsq(
        next_counter_map,
        desired_model_rate - drift,
        rcond=None,
    )[0]
    clipped = np.clip(unbounded, control_lower, control_upper)
    achieved_model_rate = next_counter_map @ clipped + drift
    achieved_measured_rate = effectiveness * achieved_model_rate
    saturation = np.isclose(clipped, control_lower) | np.isclose(
        clipped, control_upper,
    )
    return ReactionSeedDiagnostic(
        valid=bool(
            np.all(np.isfinite(unbounded))
            and np.all(np.isfinite(achieved_measured_rate))
        ),
        desired_measured_rate=desired_measured_rate,
        unbounded_acceleration=unbounded,
        clipped_acceleration=clipped,
        saturation_mask=saturation,
        achieved_measured_rate=achieved_measured_rate,
        residual=achieved_measured_rate - desired_measured_rate,
        unbounded_feasible=bool(
            np.all(unbounded >= control_lower)
            and np.all(unbounded <= control_upper)
        ),
    )


def _low_pass(previous, current, alpha):
    return previous + alpha * (current - previous)


def _vector(value, size, name):
    return _matrix(value, (size,), name)


def _matrix(value, shape, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite with shape {shape}')
    return np.copy(result)
