from dataclasses import dataclass

import numpy as np


_COUNTER_WIDTH = 4
_PLANAR_WIDTH = 2
_HORIZON = 2


@dataclass(frozen=True)
class U5State:
    '''Store the minimum residual-command realization state'''

    previous_realized_velocity: np.ndarray
    pending_request: np.ndarray

    def __post_init__(self):
        object.__setattr__(
            self,
            'previous_realized_velocity',
            _readonly_vector(
                self.previous_realized_velocity,
                _COUNTER_WIDTH,
                'previous_realized_velocity',
            ),
        )
        object.__setattr__(
            self,
            'pending_request',
            _readonly_vector(
                self.pending_request,
                _COUNTER_WIDTH,
                'pending_request',
            ),
        )

    @classmethod
    def zero(cls):
        '''Return a zero-carryover initial state'''
        return cls(np.zeros(_COUNTER_WIDTH), np.zeros(_COUNTER_WIDTH))


@dataclass(frozen=True)
class U5Prediction:
    '''Store one residual-command realization prediction'''

    realized_velocity: np.ndarray
    residual_momentum: np.ndarray
    velocity_jacobian: np.ndarray
    momentum_jacobian: np.ndarray
    final_state: U5State


@dataclass(frozen=True)
class U5Model:
    '''Map requested residual velocity to realized velocity and momentum'''

    delay: np.ndarray
    gain: np.ndarray
    carryover: np.ndarray

    def __post_init__(self):
        delay = np.asarray(self.delay)
        if (
            delay.shape != (_COUNTER_WIDTH,)
            or not np.issubdtype(delay.dtype, np.integer)
            or not np.all(np.isin(delay, (0, 1)))
        ):
            raise ValueError('delay must contain four integer zeros or ones')
        delay = np.array(delay, dtype=np.int64, copy=True)
        delay.setflags(write=False)
        object.__setattr__(self, 'delay', delay)
        object.__setattr__(
            self,
            'gain',
            _readonly_vector(self.gain, _COUNTER_WIDTH, 'gain'),
        )
        object.__setattr__(
            self,
            'carryover',
            _readonly_vector(
                self.carryover, _COUNTER_WIDTH, 'carryover',
            ),
        )

    def predict(self, requested_velocity, momentum_map, initial_state=None):
        '''Predict a residual-velocity sequence and its exact Jacobians'''
        requested = _sample_matrix(
            requested_velocity, _COUNTER_WIDTH, 'requested_velocity',
        )
        momentum_map = _finite_array(momentum_map, 'momentum_map')
        if momentum_map.shape != (_PLANAR_WIDTH, _COUNTER_WIDTH):
            raise ValueError('momentum_map must have shape (2, 4)')
        state = U5State.zero() if initial_state is None else initial_state
        if not isinstance(state, U5State):
            raise ValueError('initial_state must be a U5State')

        steps = requested.shape[0]
        realized = np.empty_like(requested)
        jacobian = np.zeros(
            (steps, _COUNTER_WIDTH, steps, _COUNTER_WIDTH),
        )
        previous = np.array(state.previous_realized_velocity, copy=True)
        previous_jacobian = np.zeros(
            (_COUNTER_WIDTH, steps, _COUNTER_WIDTH),
        )
        pending = np.array(state.pending_request, copy=True)
        gain_diagonal = np.diag(self.gain)

        for step in range(steps):
            effective = np.where(
                self.delay == 0,
                requested[step],
                pending,
            )
            current = self.carryover * previous + self.gain * effective
            current_jacobian = (
                self.carryover[:, None, None] * previous_jacobian
            )
            immediate = self.delay == 0
            current_jacobian[:, step, :] += (
                gain_diagonal * immediate[:, None]
            )
            if step > 0:
                delayed = self.delay == 1
                current_jacobian[:, step - 1, :] += (
                    gain_diagonal * delayed[:, None]
                )

            realized[step] = current
            jacobian[step] = current_jacobian
            previous = current
            previous_jacobian = current_jacobian
            pending = requested[step]

        momentum = realized @ momentum_map.T
        momentum_jacobian = np.einsum(
            'ij,tjsa->tisa', momentum_map, jacobian,
        )
        return U5Prediction(
            realized_velocity=realized,
            residual_momentum=momentum,
            velocity_jacobian=jacobian,
            momentum_jacobian=momentum_jacobian,
            final_state=U5State(previous, pending),
        )

    def predict_aligned(
            self, requested_velocity, previous_request,
            previous_realized_velocity):
        '''Predict independent aligned realization samples'''
        requested = _sample_matrix(
            requested_velocity, _COUNTER_WIDTH, 'requested_velocity',
        )
        pending = _matching_samples(
            previous_request, requested, 'previous_request',
        )
        previous = _matching_samples(
            previous_realized_velocity,
            requested,
            'previous_realized_velocity',
        )
        effective = np.where(self.delay == 0, requested, pending)
        return self.carryover * previous + self.gain * effective


@dataclass(frozen=True)
class ContextualU5Model:
    '''Map residual joint velocity through measured joint-state gains'''

    coefficients: np.ndarray
    context_center: np.ndarray
    context_scale: np.ndarray
    weak_direction_mask: np.ndarray

    def __post_init__(self):
        object.__setattr__(
            self,
            'coefficients',
            _readonly_matrix(self.coefficients, (4, 4), 'coefficients'),
        )
        object.__setattr__(
            self,
            'context_center',
            _readonly_matrix(
                self.context_center, (4, 3), 'context_center',
            ),
        )
        scale = _readonly_matrix(
            self.context_scale, (4, 3), 'context_scale',
        )
        if np.any(scale <= 0.0):
            raise ValueError('context_scale must be positive')
        object.__setattr__(self, 'context_scale', scale)
        mask = np.asarray(self.weak_direction_mask)
        if mask.shape != (4,) or mask.dtype != np.bool_:
            raise ValueError('weak_direction_mask must contain four booleans')
        mask = np.array(mask, copy=True)
        mask.setflags(write=False)
        object.__setattr__(self, 'weak_direction_mask', mask)

    def gains(self, counter_q, counter_dq, counter_side):
        '''Evaluate local retained-joint realization gains'''
        counter_q = _sample_matrix(counter_q, 4, 'counter_q')
        counter_dq = _matching_samples(counter_dq, counter_q, 'counter_dq')
        side = np.asarray(counter_side, dtype=np.float64)
        if side.shape != (len(counter_q),) or not np.all(np.isfinite(side)):
            raise ValueError('counter_side must align with requested samples')
        result = np.empty_like(counter_q)
        for axis in range(4):
            context = np.column_stack([
                counter_q[:, axis], counter_dq[:, axis], side,
            ])
            features = np.column_stack([
                np.ones(len(context)),
                (context - self.context_center[axis])
                / self.context_scale[axis],
            ])
            result[:, axis] = features @ self.coefficients[axis]
        result[:, self.weak_direction_mask] = 0.0
        return result

    def predict(self, requested, counter_q, counter_dq, counter_side):
        '''Predict one-tick delayed realized residual velocity'''
        requested = _sample_matrix(requested, 4, 'requested')
        gains = self.gains(counter_q, counter_dq, counter_side)
        if gains.shape != requested.shape:
            raise ValueError('state context must align with requested samples')
        return requested * gains


@dataclass(frozen=True)
class R5Model:
    '''Map realized residual momentum to H1/H2 angular-rate increments'''

    g0: np.ndarray
    g1: np.ndarray

    def __post_init__(self):
        object.__setattr__(
            self,
            'g0',
            _readonly_matrix(
                self.g0, (_PLANAR_WIDTH, _PLANAR_WIDTH), 'g0',
            ),
        )
        object.__setattr__(
            self,
            'g1',
            _readonly_matrix(
                self.g1, (_PLANAR_WIDTH, _PLANAR_WIDTH), 'g1',
            ),
        )

    def predict(self, realized_momentum):
        '''Predict incremental angular rate at H1 and H2'''
        momentum = _h2_samples(realized_momentum, 'realized_momentum')
        result = np.empty_like(momentum)
        result[..., 0, :] = momentum[..., 0, :] @ self.g0.T
        result[..., 1, :] = (
            momentum[..., 0, :] @ self.g1.T
            + momentum[..., 1, :] @ self.g0.T
        )
        return result

    def jacobian(self):
        '''Return the exact H2 angular-rate to momentum Jacobian'''
        result = np.zeros(
            (_HORIZON, _PLANAR_WIDTH, _HORIZON, _PLANAR_WIDTH),
        )
        result[0, :, 0, :] = self.g0
        result[1, :, 0, :] = self.g1
        result[1, :, 1, :] = self.g0
        return result


@dataclass(frozen=True)
class ContextualR5DiagonalModel:
    '''Map residual momentum through tilt/rate-conditioned diagonal gains'''

    coefficients: np.ndarray
    context_center: np.ndarray
    context_scale: np.ndarray

    def __post_init__(self):
        coefficients = _finite_array(self.coefficients, 'coefficients')
        if (
            coefficients.ndim != 2
            or coefficients.shape[0] != 2
            or coefficients.shape[1] < 5
        ):
            raise ValueError('coefficients must have shape (2, context+1)')
        coefficients = np.array(coefficients, copy=True)
        coefficients.setflags(write=False)
        object.__setattr__(
            self,
            'coefficients',
            coefficients,
        )
        context_width = coefficients.shape[1] - 1
        object.__setattr__(
            self,
            'context_center',
            _readonly_vector(
                self.context_center, context_width, 'context_center',
            ),
        )
        scale = _readonly_vector(
            self.context_scale, context_width, 'context_scale',
        )
        if np.any(scale <= 0.0):
            raise ValueError('context_scale must be positive')
        object.__setattr__(self, 'context_scale', scale)

    def gains(self, state_tilt, state_rate, state_height=None):
        '''Evaluate local roll/pitch gains from real-compatible context'''
        tilt = _sample_matrix(state_tilt, 2, 'state_tilt')
        rate = _matching_samples(state_rate, tilt, 'state_rate')
        context = np.column_stack([tilt, rate])
        if state_height is not None:
            height = np.asarray(state_height, dtype=np.float64)
            if height.shape != (len(context),) or not np.all(np.isfinite(height)):
                raise ValueError('state_height must align with context samples')
            context = np.column_stack([context, height])
        if context.shape[1] != len(self.context_center):
            raise ValueError('state context width does not match fitted model')
        features = np.column_stack([
            np.ones(len(context)),
            (context - self.context_center) / self.context_scale,
        ])
        return features @ self.coefficients.T

    def predict(
            self, realized_momentum, state_tilt, state_rate,
            state_height=None):
        '''Predict H2 angular-rate increments with local diagonal gains'''
        momentum = _h2_samples(realized_momentum, 'realized_momentum')
        if momentum.ndim != 3:
            raise ValueError('realized_momentum must have shape (samples, 2, 2)')
        gains = self.gains(state_tilt, state_rate, state_height)
        if gains.shape[0] != momentum.shape[0]:
            raise ValueError('state context must align with momentum samples')
        return momentum * gains[:, None, :]


@dataclass(frozen=True)
class ContextualR5Model:
    '''Map momentum through a compact context-conditioned full response matrix'''

    coefficients: np.ndarray
    context_center: np.ndarray
    context_scale: np.ndarray

    def __post_init__(self):
        coefficients = _finite_array(self.coefficients, 'coefficients')
        if coefficients.ndim != 3 or coefficients.shape[:2] != (2, 2):
            raise ValueError('coefficients must have shape (2, 2, context+1)')
        coefficients = np.array(coefficients, copy=True)
        coefficients.setflags(write=False)
        object.__setattr__(self, 'coefficients', coefficients)
        context_width = coefficients.shape[2] - 1
        object.__setattr__(
            self,
            'context_center',
            _readonly_vector(
                self.context_center, context_width, 'context_center',
            ),
        )
        scale = _readonly_vector(
            self.context_scale, context_width, 'context_scale',
        )
        if np.any(scale <= 0.0):
            raise ValueError('context_scale must be positive')
        object.__setattr__(self, 'context_scale', scale)

    def gains(self, context):
        '''Evaluate local full roll/pitch response matrices'''
        context = _sample_matrix(
            context, len(self.context_center), 'context',
        )
        features = np.column_stack([
            np.ones(len(context)),
            (context - self.context_center) / self.context_scale,
        ])
        return np.einsum('nf,oif->noi', features, self.coefficients)

    def predict(self, realized_momentum, context):
        '''Predict H2 response through local full roll/pitch gradients'''
        momentum = _h2_samples(realized_momentum, 'realized_momentum')
        if momentum.ndim != 3:
            raise ValueError('realized_momentum must have shape (samples, 2, 2)')
        gains = self.gains(context)
        if len(gains) != len(momentum):
            raise ValueError('context must align with momentum samples')
        return np.einsum('noi,nti->nto', gains, momentum)

@dataclass(frozen=True)
class ResidualResponsePrediction:
    '''Store one composed U5/R5 H2 residual prediction'''

    realized_velocity: np.ndarray
    residual_momentum: np.ndarray
    incremental_angular_rate: np.ndarray
    incremental_tilt: np.ndarray
    incremental_counter_position: np.ndarray
    velocity_jacobian: np.ndarray
    momentum_jacobian: np.ndarray
    angular_rate_jacobian: np.ndarray
    tilt_jacobian: np.ndarray
    counter_position_jacobian: np.ndarray
    final_u5_state: U5State


@dataclass(frozen=True)
class N5ValidityThresholds:
    '''Store calibrated nominal-phase validity limits'''

    max_abs_tilt: np.ndarray
    max_abs_rate: np.ndarray
    max_tilt_step: np.ndarray
    max_rate_step: np.ndarray
    max_integration_error: np.ndarray
    min_dt: float
    max_dt: float
    sign_deadband: np.ndarray

    def __post_init__(self):
        for name in (
            'max_abs_tilt', 'max_abs_rate', 'max_tilt_step',
            'max_rate_step', 'max_integration_error',
        ):
            value = _readonly_vector(
                getattr(self, name), _PLANAR_WIDTH, name,
            )
            if np.any(value <= 0.0):
                raise ValueError(f'{name} must be positive')
            object.__setattr__(self, name, value)
        deadband = _readonly_vector(
            self.sign_deadband, _PLANAR_WIDTH, 'sign_deadband',
        )
        if np.any(deadband < 0.0):
            raise ValueError('sign_deadband must be nonnegative')
        object.__setattr__(self, 'sign_deadband', deadband)
        min_dt = _positive_scalar(self.min_dt, 'min_dt')
        max_dt = _positive_scalar(self.max_dt, 'max_dt')
        if min_dt > max_dt:
            raise ValueError('min_dt must not exceed max_dt')
        object.__setattr__(self, 'min_dt', min_dt)
        object.__setattr__(self, 'max_dt', max_dt)


@dataclass(frozen=True)
class N5Prediction:
    '''Store one nominal H2 phase prediction and validity result'''

    tilt: np.ndarray
    angular_rate: np.ndarray
    tilt_sign: np.ndarray
    angular_rate_sign: np.ndarray
    diverging: np.ndarray
    peak_step: np.ndarray
    zero_crossing_step: np.ndarray
    valid: bool
    invalid_reasons: tuple


@dataclass(frozen=True)
class N5Model:
    '''Predict the minimum nominal H2 phase from two measured samples'''

    rate_trend_gain: np.ndarray
    validity: N5ValidityThresholds
    moving_momentum_gain: np.ndarray | None = None
    nominal_momentum_gain: np.ndarray | None = None

    def __post_init__(self):
        gain = _readonly_vector(
            self.rate_trend_gain, _PLANAR_WIDTH, 'rate_trend_gain',
        )
        if np.any(gain < 0.0) or np.any(gain > 1.0):
            raise ValueError('rate_trend_gain must lie in [0, 1]')
        if not isinstance(self.validity, N5ValidityThresholds):
            raise ValueError('validity must be N5ValidityThresholds')
        object.__setattr__(self, 'rate_trend_gain', gain)
        for name in ('moving_momentum_gain', 'nominal_momentum_gain'):
            value = getattr(self, name)
            value = (
                np.zeros((_PLANAR_WIDTH, _PLANAR_WIDTH))
                if value is None else value
            )
            object.__setattr__(
                self,
                name,
                _readonly_matrix(
                    value, (_PLANAR_WIDTH, _PLANAR_WIDTH), name,
                ),
            )

    def predict(
            self, current_tilt, previous_tilt,
            current_rate, previous_rate, dt,
            moving_momentum_change=None,
            nominal_momentum_change=None):
        '''Predict nominal tilt/rate and report calibrated phase validity'''
        current_tilt = _finite_vector(
            current_tilt, _PLANAR_WIDTH, 'current_tilt',
        )
        previous_tilt = _finite_vector(
            previous_tilt, _PLANAR_WIDTH, 'previous_tilt',
        )
        current_rate = _finite_vector(
            current_rate, _PLANAR_WIDTH, 'current_rate',
        )
        previous_rate = _finite_vector(
            previous_rate, _PLANAR_WIDTH, 'previous_rate',
        )
        dt = _positive_scalar(dt, 'dt')
        moving_change = _optional_h2_change(
            moving_momentum_change, 'moving_momentum_change',
        )
        nominal_change = _optional_h2_change(
            nominal_momentum_change, 'nominal_momentum_change',
        )

        rate = np.empty((_HORIZON, _PLANAR_WIDTH))
        measured_rate_delta = current_rate - previous_rate
        rate_delta = measured_rate_delta
        previous_prediction = current_rate
        for step in range(_HORIZON):
            current_prediction = (
                previous_prediction
                + self.rate_trend_gain * rate_delta
                + self.moving_momentum_gain @ moving_change[step]
                + self.nominal_momentum_gain @ nominal_change[step]
            )
            rate[step] = current_prediction
            rate_delta = current_prediction - previous_prediction
            previous_prediction = current_prediction
        tilt = _integrate_h2(current_tilt, current_rate, rate, dt)

        reasons = []
        validity = self.validity
        if dt < validity.min_dt or dt > validity.max_dt:
            reasons.append('dt')
        if np.any(np.abs(current_tilt) > validity.max_abs_tilt):
            reasons.append('tilt')
        if np.any(np.abs(previous_tilt) > validity.max_abs_tilt):
            reasons.append('previous_tilt')
        if np.any(np.abs(current_rate) > validity.max_abs_rate):
            reasons.append('rate')
        if np.any(np.abs(previous_rate) > validity.max_abs_rate):
            reasons.append('previous_rate')
        if np.any(
            np.abs(current_tilt - previous_tilt)
            > validity.max_tilt_step
        ):
            reasons.append('tilt_step')
        if np.any(np.abs(measured_rate_delta) > validity.max_rate_step):
            reasons.append('rate_step')
        integrated_step = 0.5 * dt * (previous_rate + current_rate)
        if np.any(
            np.abs(current_tilt - previous_tilt - integrated_step)
            > validity.max_integration_error
        ):
            reasons.append('integration')

        tilt_sign = _deadband_sign(tilt, validity.sign_deadband)
        rate_sign = _deadband_sign(rate, validity.sign_deadband)
        diverging = tilt * rate > 0.0
        peak_step = np.argmax(
            np.abs(np.vstack([current_tilt, tilt])), axis=0,
        )
        zero_crossing = _zero_crossings(
            np.vstack([current_tilt, tilt]), validity.sign_deadband,
        )
        return N5Prediction(
            tilt=tilt,
            angular_rate=rate,
            tilt_sign=tilt_sign,
            angular_rate_sign=rate_sign,
            diverging=diverging,
            peak_step=peak_step,
            zero_crossing_step=zero_crossing,
            valid=not reasons,
            invalid_reasons=tuple(reasons),
        )


@dataclass(frozen=True)
class ResponseValidationMetrics:
    '''Store sign, error, and baseline response metrics'''

    rmse: np.ndarray
    sign_accuracy: np.ndarray
    zero_baseline_improvement: np.ndarray
    reference_baseline_improvement: np.ndarray


@dataclass(frozen=True)
class DistinguishabilityThresholds:
    '''Store predeclared physical ranking thresholds'''

    noise_bound: float
    minimum_effect: float

    def __post_init__(self):
        for name in ('noise_bound', 'minimum_effect'):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and nonnegative')
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class ActionRankingMetrics:
    '''Store deterministic distinguishable-pair ranking metrics'''

    total_pairs: int
    distinguishable_pairs: int
    correct_pairs: int
    coverage: float
    accuracy: float
    distinguishable_mask: np.ndarray
    correct_mask: np.ndarray


def fit_u5(
        requested_velocity, previous_request, previous_realized_velocity,
        realized_velocity, delay=None, ridge=1e-8):
    '''Fit diagonal U5 gain/carryover and optional per-axis delay'''
    requested = _sample_matrix(
        requested_velocity, _COUNTER_WIDTH, 'requested_velocity',
    )
    pending = _matching_samples(
        previous_request, requested, 'previous_request',
    )
    previous = _matching_samples(
        previous_realized_velocity,
        requested,
        'previous_realized_velocity',
    )
    realized = _matching_samples(
        realized_velocity, requested, 'realized_velocity',
    )
    if requested.shape[0] < 2:
        raise ValueError('U5 fitting requires at least two samples')
    ridge = _nonnegative_scalar(ridge, 'ridge')

    if delay is None:
        delays = np.zeros(_COUNTER_WIDTH, dtype=np.int64)
        candidates = (0, 1)
    else:
        delays = _delay_vector(delay)
        candidates = None

    gain = np.empty(_COUNTER_WIDTH)
    carryover = np.empty(_COUNTER_WIDTH)
    for axis in range(_COUNTER_WIDTH):
        axis_candidates = candidates or (int(delays[axis]),)
        best = None
        for candidate in axis_candidates:
            effective = (
                requested[:, axis]
                if candidate == 0 else pending[:, axis]
            )
            features = np.column_stack([effective, previous[:, axis]])
            coefficients = _fit_zero_intercept(
                features, realized[:, axis, None], ridge,
            )[:, 0]
            residual = realized[:, axis] - features @ coefficients
            score = float(residual @ residual)
            result = (score, candidate, coefficients)
            if best is None or result[:2] < best[:2]:
                best = result
        delays[axis] = best[1]
        gain[axis], carryover[axis] = best[2]
    return U5Model(delays, gain, carryover)


def fit_contextual_u5(
        requested, realized, counter_q, counter_dq, counter_side,
        weak_direction_mask=None, ridge=1e-3):
    '''Fit one-tick contextual U5 gains from exact branch responses'''
    requested = _sample_matrix(requested, 4, 'requested')
    realized = _matching_samples(realized, requested, 'realized')
    counter_q = _matching_samples(counter_q, requested, 'counter_q')
    counter_dq = _matching_samples(counter_dq, requested, 'counter_dq')
    side = np.asarray(counter_side, dtype=np.float64)
    if side.shape != (len(requested),) or not np.all(np.isfinite(side)):
        raise ValueError('counter_side must align with requested samples')
    ridge = _nonnegative_scalar(ridge, 'ridge')
    if weak_direction_mask is None:
        weak_direction_mask = np.zeros(4, dtype=bool)
    mask = np.asarray(weak_direction_mask)
    if mask.shape != (4,) or mask.dtype != np.bool_:
        raise ValueError('weak_direction_mask must contain four booleans')
    coefficients = np.empty((4, 4))
    centers = np.empty((4, 3))
    scales = np.empty((4, 3))
    for axis in range(4):
        context = np.column_stack([
            counter_q[:, axis], counter_dq[:, axis], side,
        ])
        centers[axis] = np.mean(context, axis=0)
        scales[axis] = np.maximum(np.std(context, axis=0), 1e-6)
        features = np.column_stack([
            np.ones(len(context)),
            (context - centers[axis]) / scales[axis],
        ])
        design = requested[:, axis, None] * features
        coefficients[axis] = _fit_zero_intercept(
            design, realized[:, axis, None], ridge,
        )[:, 0]
    return ContextualU5Model(coefficients, centers, scales, mask)


def fit_r5(
        realized_momentum, incremental_angular_rate,
        ridge=1e-8, diagonal=False):
    '''Fit the zero-intercept delay-aware R5 G0/G1 FIR model'''
    momentum = _h2_samples(realized_momentum, 'realized_momentum')
    rate = _h2_samples(
        incremental_angular_rate, 'incremental_angular_rate',
    )
    if momentum.ndim != 3 or rate.ndim != 3:
        raise ValueError('R5 fitting arrays must have shape (samples, 2, 2)')
    if momentum.shape != rate.shape:
        raise ValueError('R5 fitting arrays must have matching shapes')
    if momentum.shape[0] < 2:
        raise ValueError('R5 fitting requires at least two samples')
    ridge = _nonnegative_scalar(ridge, 'ridge')
    if not isinstance(diagonal, bool):
        raise ValueError('diagonal must be boolean')

    if diagonal:
        g0 = np.zeros((_PLANAR_WIDTH, _PLANAR_WIDTH))
        g1 = np.zeros((_PLANAR_WIDTH, _PLANAR_WIDTH))
        for axis in range(_PLANAR_WIDTH):
            features = np.concatenate([
                np.column_stack([
                    momentum[:, 0, axis],
                    np.zeros(momentum.shape[0]),
                ]),
                np.column_stack([
                    momentum[:, 1, axis],
                    momentum[:, 0, axis],
                ]),
            ])
            targets = np.concatenate([
                rate[:, 0, axis], rate[:, 1, axis],
            ])
            coefficients = _fit_zero_intercept(
                features, targets[:, None], ridge,
            )[:, 0]
            g0[axis, axis], g1[axis, axis] = coefficients
        return R5Model(g0=g0, g1=g1)

    zeros = np.zeros_like(momentum[:, 0, :])
    features = np.concatenate([
        np.concatenate([momentum[:, 0, :], zeros], axis=1),
        np.concatenate(
            [momentum[:, 1, :], momentum[:, 0, :]], axis=1,
        ),
    ], axis=0)
    targets = np.concatenate([rate[:, 0, :], rate[:, 1, :]], axis=0)
    coefficients = _fit_zero_intercept(features, targets, ridge)
    return R5Model(
        g0=coefficients[:_PLANAR_WIDTH, :].T,
        g1=coefficients[_PLANAR_WIDTH:, :].T,
    )


def fit_n5_momentum_rate(
        previous_rate, current_rate, next_rate,
        moving_momentum_change, nominal_momentum_change,
        validity, ridge=1e-8):
    '''Fit one-step trend and known-momentum terms for minimum N5'''
    current = _sample_matrix(current_rate, _PLANAR_WIDTH, 'current_rate')
    previous = _matching_samples(previous_rate, current, 'previous_rate')
    following = _matching_samples(next_rate, current, 'next_rate')
    moving = _matching_samples(
        moving_momentum_change, current, 'moving_momentum_change',
    )
    nominal = _matching_samples(
        nominal_momentum_change, current, 'nominal_momentum_change',
    )
    if current.shape[0] < 6:
        raise ValueError('N5 fitting requires at least six samples')
    if not isinstance(validity, N5ValidityThresholds):
        raise ValueError('validity must be N5ValidityThresholds')
    ridge = _nonnegative_scalar(ridge, 'ridge')
    target = following - current
    moving_gain = np.empty((_PLANAR_WIDTH, _PLANAR_WIDTH))
    nominal_gain = np.empty((_PLANAR_WIDTH, _PLANAR_WIDTH))
    trend_gain = np.empty(_PLANAR_WIDTH)
    for axis in range(_PLANAR_WIDTH):
        features = np.column_stack([
            current[:, axis] - previous[:, axis],
            moving,
            nominal,
        ])
        coefficients = _fit_zero_intercept(
            features, target[:, axis, None], ridge,
        )[:, 0]
        trend_gain[axis] = np.clip(coefficients[0], 0.0, 1.0)
        moving_gain[axis] = coefficients[1:3]
        nominal_gain[axis] = coefficients[3:5]
    return N5Model(
        rate_trend_gain=trend_gain,
        validity=validity,
        moving_momentum_gain=moving_gain,
        nominal_momentum_gain=nominal_gain,
    )


def fit_contextual_r5_diagonal(
        realized_momentum, incremental_angular_rate,
        state_tilt, state_rate, state_height=None, ridge=1e-3):
    '''Fit minimum bilinear tilt/rate-conditioned diagonal R5 gains'''
    momentum = _h2_samples(realized_momentum, 'realized_momentum')
    response = _h2_samples(
        incremental_angular_rate, 'incremental_angular_rate',
    )
    if momentum.ndim != 3 or response.shape != momentum.shape:
        raise ValueError('R5 arrays must have matching shape (samples, 2, 2)')
    tilt = _sample_matrix(state_tilt, 2, 'state_tilt')
    rate = _matching_samples(state_rate, tilt, 'state_rate')
    if tilt.shape[0] != momentum.shape[0]:
        raise ValueError('state context must align with response samples')
    ridge = _nonnegative_scalar(ridge, 'ridge')
    context = np.column_stack([tilt, rate])
    if state_height is not None:
        height = np.asarray(state_height, dtype=np.float64)
        if height.shape != (len(context),) or not np.all(np.isfinite(height)):
            raise ValueError('state_height must align with response samples')
        context = np.column_stack([context, height])
    center = np.mean(context, axis=0)
    scale = np.maximum(np.std(context, axis=0), 1e-6)
    normalized = np.column_stack([
        np.ones(len(context)),
        (context - center) / scale,
    ])
    coefficients = np.empty((2, context.shape[1] + 1), dtype=np.float64)
    for axis in range(2):
        features = momentum[:, 1, axis, None] * normalized
        coefficients[axis] = _fit_zero_intercept(
            features,
            response[:, 1, axis, None],
            ridge,
        )[:, 0]
    return ContextualR5DiagonalModel(coefficients, center, scale)


def fit_contextual_r5(
        realized_momentum, incremental_angular_rate,
        context, ridge=1e-2):
    '''Fit compact bilinear full R5 gradients from explicit physical context'''
    momentum = _h2_samples(realized_momentum, 'realized_momentum')
    response = _h2_samples(
        incremental_angular_rate, 'incremental_angular_rate',
    )
    if momentum.ndim != 3 or response.shape != momentum.shape:
        raise ValueError('R5 arrays must have matching shape (samples, 2, 2)')
    context = _finite_array(context, 'context')
    if context.ndim != 2 or len(context) != len(momentum):
        raise ValueError('context must align with response samples')
    ridge = _nonnegative_scalar(ridge, 'ridge')
    center = np.mean(context, axis=0)
    scale = np.maximum(np.std(context, axis=0), 1e-6)
    features = np.column_stack([
        np.ones(len(context)),
        (context - center) / scale,
    ])
    design = np.concatenate([
        momentum[:, 1, axis, None] * features
        for axis in range(2)
    ], axis=1)
    fitted = _fit_zero_intercept(
        design, response[:, 1], ridge,
    )
    coefficients = fitted.reshape(2, features.shape[1], 2).transpose(2, 0, 1)
    return ContextualR5Model(coefficients, center, scale)


def predict_residual_response(
        u5_model, r5_model, requested_velocity, momentum_map, dt,
        initial_u5_state=None):
    '''Compose U5 and R5 with exact H2 trapezoidal derivatives'''
    if not isinstance(u5_model, U5Model):
        raise ValueError('u5_model must be a U5Model')
    if not isinstance(r5_model, R5Model):
        raise ValueError('r5_model must be an R5Model')
    dt = _positive_scalar(dt, 'dt')
    requested = _finite_array(requested_velocity, 'requested_velocity')
    if requested.shape != (_HORIZON, _COUNTER_WIDTH):
        raise ValueError('requested_velocity must have shape (2, 4)')
    state = U5State.zero() if initial_u5_state is None else initial_u5_state
    u5_prediction = u5_model.predict(requested, momentum_map, state)
    rate = r5_model.predict(u5_prediction.residual_momentum)
    rate_momentum_jacobian = r5_model.jacobian()
    rate_jacobian = np.einsum(
        'tihj,hjsa->tisa',
        rate_momentum_jacobian,
        u5_prediction.momentum_jacobian,
    )

    tilt = _integrate_h2(np.zeros(_PLANAR_WIDTH), np.zeros(2), rate, dt)
    tilt_jacobian = np.empty_like(rate_jacobian)
    tilt_jacobian[0] = 0.5 * dt * rate_jacobian[0]
    tilt_jacobian[1] = (
        tilt_jacobian[0]
        + 0.5 * dt * (rate_jacobian[0] + rate_jacobian[1])
    )

    position = _integrate_h2(
        np.zeros(_COUNTER_WIDTH),
        state.previous_realized_velocity,
        u5_prediction.realized_velocity,
        dt,
    )
    position_jacobian = np.empty_like(u5_prediction.velocity_jacobian)
    position_jacobian[0] = (
        0.5 * dt * u5_prediction.velocity_jacobian[0]
    )
    position_jacobian[1] = (
        position_jacobian[0]
        + 0.5 * dt * (
            u5_prediction.velocity_jacobian[0]
            + u5_prediction.velocity_jacobian[1]
        )
    )
    return ResidualResponsePrediction(
        realized_velocity=u5_prediction.realized_velocity,
        residual_momentum=u5_prediction.residual_momentum,
        incremental_angular_rate=rate,
        incremental_tilt=tilt,
        incremental_counter_position=position,
        velocity_jacobian=u5_prediction.velocity_jacobian,
        momentum_jacobian=u5_prediction.momentum_jacobian,
        angular_rate_jacobian=rate_jacobian,
        tilt_jacobian=tilt_jacobian,
        counter_position_jacobian=position_jacobian,
        final_u5_state=u5_prediction.final_state,
    )


def calibrate_n5_validity(
        current_tilt, previous_tilt, current_rate, previous_rate, dt,
        quantile=0.99, margin=1.25, sign_deadband=None):
    '''Calibrate N5 validity limits from aligned nominal samples'''
    current_tilt = _sample_matrix(
        current_tilt, _PLANAR_WIDTH, 'current_tilt',
    )
    previous_tilt = _matching_samples(
        previous_tilt, current_tilt, 'previous_tilt',
    )
    current_rate = _matching_samples(
        current_rate, current_tilt, 'current_rate',
    )
    previous_rate = _matching_samples(
        previous_rate, current_tilt, 'previous_rate',
    )
    if current_tilt.shape[0] < 2:
        raise ValueError('N5 calibration requires at least two samples')
    dt_values = np.asarray(dt, dtype=np.float64)
    if dt_values.ndim == 0:
        dt_values = np.full(current_tilt.shape[0], float(dt_values))
    if (
        dt_values.shape != (current_tilt.shape[0],)
        or not np.all(np.isfinite(dt_values))
        or np.any(dt_values <= 0.0)
    ):
        raise ValueError('dt must be positive and align with samples')
    quantile = float(quantile)
    margin = float(margin)
    if not np.isfinite(quantile) or not 0.0 < quantile <= 1.0:
        raise ValueError('quantile must lie in (0, 1]')
    if not np.isfinite(margin) or margin < 1.0:
        raise ValueError('margin must be finite and at least one')
    deadband = (
        np.zeros(_PLANAR_WIDTH)
        if sign_deadband is None
        else _finite_vector(
            sign_deadband, _PLANAR_WIDTH, 'sign_deadband',
        )
    )
    if np.any(deadband < 0.0):
        raise ValueError('sign_deadband must be nonnegative')

    integration_error = current_tilt - previous_tilt - (
        0.5 * dt_values[:, None] * (previous_rate + current_rate)
    )

    def limit(values):
        result = margin * np.quantile(np.abs(values), quantile, axis=0)
        return np.maximum(result, np.finfo(np.float64).eps)

    return N5ValidityThresholds(
        max_abs_tilt=limit(np.vstack([previous_tilt, current_tilt])),
        max_abs_rate=limit(np.vstack([previous_rate, current_rate])),
        max_tilt_step=limit(current_tilt - previous_tilt),
        max_rate_step=limit(current_rate - previous_rate),
        max_integration_error=limit(integration_error),
        min_dt=float(np.min(dt_values) / margin),
        max_dt=float(np.max(dt_values) * margin),
        sign_deadband=deadband,
    )


def response_validation_metrics(
        expected, predicted, reference_baseline=None, sign_threshold=0.0):
    '''Compute response sign and improvement over zero/reference baselines'''
    expected = _metric_samples(expected, 'expected')
    predicted = _metric_samples(predicted, 'predicted')
    if expected.shape != predicted.shape:
        raise ValueError('expected and predicted must have matching shapes')
    if reference_baseline is None:
        reference = np.zeros_like(expected)
    else:
        reference = _metric_samples(
            reference_baseline, 'reference_baseline',
        )
        if reference.shape != expected.shape:
            raise ValueError('reference_baseline must match expected')
    threshold = _nonnegative_scalar(sign_threshold, 'sign_threshold')
    model_mse = np.mean((predicted - expected) ** 2, axis=0)
    zero_mse = np.mean(expected ** 2, axis=0)
    reference_mse = np.mean((reference - expected) ** 2, axis=0)
    useful = np.abs(expected) > threshold
    correct = np.sign(predicted) == np.sign(expected)
    sign_accuracy = np.divide(
        np.sum(correct & useful, axis=0),
        np.sum(useful, axis=0),
        out=np.full(expected.shape[1], np.nan),
        where=np.sum(useful, axis=0) > 0,
    )
    return ResponseValidationMetrics(
        rmse=np.sqrt(model_mse),
        sign_accuracy=sign_accuracy,
        zero_baseline_improvement=_baseline_improvement(
            model_mse, zero_mse,
        ),
        reference_baseline_improvement=_baseline_improvement(
            model_mse, reference_mse,
        ),
    )


def n5_no_crossing_confidence(
        current_rate, previous_rate, error_bound, horizon_steps=2):
    '''Return axes whose measured phase cannot cross zero inside the horizon'''
    current = _finite_array(current_rate, 'current_rate')
    previous = _finite_array(previous_rate, 'previous_rate')
    if current.shape != previous.shape or current.shape[-1] != _PLANAR_WIDTH:
        raise ValueError('rate arrays must align and end with two axes')
    error = _finite_vector(error_bound, _PLANAR_WIDTH, 'error_bound')
    if np.any(error < 0.0):
        raise ValueError('error_bound must be nonnegative')
    if not isinstance(horizon_steps, int) or horizon_steps < 1:
        raise ValueError('horizon_steps must be a positive integer')
    margin = error + horizon_steps * np.abs(current - previous)
    return np.abs(current) > margin


def action_ranking_metrics(
        measured_scores, predicted_scores, measured_responses,
        thresholds, weights=None, confidence_radius=None):
    '''Evaluate rankings only on predeclared distinguishable comparisons'''
    if not isinstance(thresholds, DistinguishabilityThresholds):
        raise ValueError('thresholds must be DistinguishabilityThresholds')
    measured_scores = _finite_array(measured_scores, 'measured_scores')
    predicted_scores = _finite_array(predicted_scores, 'predicted_scores')
    responses = _finite_array(measured_responses, 'measured_responses')
    if (
        measured_scores.ndim != 2
        or measured_scores.shape[0] < 1
        or measured_scores.shape[1] < 2
    ):
        raise ValueError('scores must have shape (families, candidates>=2)')
    if predicted_scores.shape != measured_scores.shape:
        raise ValueError('score arrays must have matching shapes')
    if responses.shape[:2] != measured_scores.shape or responses.ndim < 3:
        raise ValueError('measured_responses must align with score candidates')
    responses = responses.reshape(*measured_scores.shape, -1)
    feature_count = responses.shape[-1]
    if weights is None:
        weights = np.ones(feature_count)
    else:
        weights = _finite_vector(weights, feature_count, 'weights')
        if np.any(weights < 0.0) or not np.any(weights > 0.0):
            raise ValueError('weights must be nonnegative and nonzero')
    if confidence_radius is None:
        confidence = np.zeros_like(measured_scores)
    else:
        confidence = _finite_array(
            confidence_radius, 'confidence_radius',
        )
        if confidence.shape != measured_scores.shape:
            raise ValueError('confidence_radius must match scores')
        if np.any(confidence < 0.0):
            raise ValueError('confidence_radius must be nonnegative')

    families, candidates = measured_scores.shape
    total_pairs = families * candidates * (candidates - 1) // 2
    distinguishable = np.zeros(total_pairs, dtype=bool)
    correct = np.zeros(total_pairs, dtype=bool)
    pair_index = 0
    for family in range(families):
        for first in range(candidates - 1):
            for second in range(first + 1, candidates):
                difference = responses[family, first] - responses[
                    family, second
                ]
                effect = np.sqrt(np.sum(weights * difference ** 2))
                lower_bound = effect - (
                    confidence[family, first]
                    + confidence[family, second]
                )
                measured_difference = (
                    measured_scores[family, first]
                    - measured_scores[family, second]
                )
                is_distinguishable = (
                    lower_bound > thresholds.noise_bound
                    and lower_bound > thresholds.minimum_effect
                    and measured_difference != 0.0
                )
                distinguishable[pair_index] = is_distinguishable
                if is_distinguishable:
                    predicted_difference = (
                        predicted_scores[family, first]
                        - predicted_scores[family, second]
                    )
                    correct[pair_index] = (
                        predicted_difference != 0.0
                        and np.signbit(predicted_difference)
                        == np.signbit(measured_difference)
                    )
                pair_index += 1
    distinguishable_count = int(np.sum(distinguishable))
    correct_count = int(np.sum(correct))
    accuracy = (
        correct_count / distinguishable_count
        if distinguishable_count else np.nan
    )
    return ActionRankingMetrics(
        total_pairs=total_pairs,
        distinguishable_pairs=distinguishable_count,
        correct_pairs=correct_count,
        coverage=distinguishable_count / total_pairs,
        accuracy=accuracy,
        distinguishable_mask=distinguishable,
        correct_mask=correct,
    )


def _integrate_h2(initial_value, initial_rate, future_rate, dt):
    result = np.empty_like(future_rate)
    result[0] = initial_value + 0.5 * dt * (
        initial_rate + future_rate[0]
    )
    result[1] = result[0] + 0.5 * dt * (
        future_rate[0] + future_rate[1]
    )
    return result


def _fit_zero_intercept(features, targets, ridge):
    regularization = ridge * np.eye(features.shape[1])
    return np.linalg.solve(
        features.T @ features + regularization,
        features.T @ targets,
    )


def _baseline_improvement(model_mse, baseline_mse):
    return 1.0 - model_mse / np.maximum(
        baseline_mse, np.finfo(np.float64).eps,
    )


def _zero_crossings(values, deadband):
    signs = _deadband_sign(values, deadband)
    result = np.full(_PLANAR_WIDTH, -1, dtype=np.int64)
    for axis in range(_PLANAR_WIDTH):
        if signs[0, axis] == 0:
            result[axis] = 0
            continue
        for step in range(1, values.shape[0]):
            if signs[step, axis] == 0 or (
                signs[step, axis] != signs[step - 1, axis]
            ):
                result[axis] = step
                break
    return result


def _deadband_sign(values, deadband):
    return np.where(
        np.abs(values) <= deadband,
        0,
        np.sign(values),
    ).astype(np.int8)


def _delay_vector(value):
    result = np.asarray(value)
    if (
        result.shape != (_COUNTER_WIDTH,)
        or not np.issubdtype(result.dtype, np.integer)
        or not np.all(np.isin(result, (0, 1)))
    ):
        raise ValueError('delay must contain four integer zeros or ones')
    return np.array(result, dtype=np.int64, copy=True)


def _matching_samples(value, reference, name):
    result = _sample_matrix(value, reference.shape[1], name)
    if result.shape != reference.shape:
        raise ValueError(f'{name} must match aligned sample shape')
    return result


def _sample_matrix(value, width, name):
    result = _finite_array(value, name)
    if result.ndim != 2 or result.shape[1] != width:
        raise ValueError(f'{name} must have shape (samples, {width})')
    return result


def _h2_samples(value, name):
    result = _finite_array(value, name)
    if result.ndim < 2 or result.shape[-2:] != (
        _HORIZON, _PLANAR_WIDTH,
    ):
        raise ValueError(f'{name} must end with shape (2, 2)')
    return result


def _optional_h2_change(value, name):
    if value is None:
        return np.zeros((_HORIZON, _PLANAR_WIDTH))
    result = _finite_array(value, name)
    if result.shape != (_HORIZON, _PLANAR_WIDTH):
        raise ValueError(f'{name} must have shape (2, 2)')
    return result


def _metric_samples(value, name):
    result = _finite_array(value, name)
    if result.ndim < 2:
        raise ValueError(f'{name} must include samples and response axes')
    return result.reshape(-1, result.shape[-1])


def _readonly_vector(value, width, name):
    result = _finite_vector(value, width, name)
    result = np.array(result, copy=True)
    result.setflags(write=False)
    return result


def _readonly_matrix(value, shape, name):
    result = _finite_array(value, name)
    if result.shape != shape:
        raise ValueError(f'{name} must have shape {shape}')
    result = np.array(result, copy=True)
    result.setflags(write=False)
    return result


def _finite_vector(value, width, name):
    result = _finite_array(value, name)
    if result.shape != (width,):
        raise ValueError(f'{name} must have shape ({width},)')
    return result


def _finite_array(value, name):
    result = np.asarray(value, dtype=np.float64)
    if not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite')
    return result


def _positive_scalar(value, name):
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return result


def _nonnegative_scalar(value, name):
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f'{name} must be finite and nonnegative')
    return result
