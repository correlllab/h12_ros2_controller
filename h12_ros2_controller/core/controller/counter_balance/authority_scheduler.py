from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AuthorityConfig:
    '''Configure one policy-blind adaptive-authority scheduler'''

    dt: float = 0.02
    momentum_scale: float = 1.0
    momentum_rate_scale: float = 1.0
    tilt_scale: float = 0.1
    angular_velocity_scale: float = 0.25
    divergence_weight: float = 1.0
    preview_entry: float = 0.5
    preview_full: float = 0.9
    response_entry: float = 0.5
    response_full: float = 0.9
    rise_time: float = 0.1
    decay_time: float = 0.3


@dataclass(frozen=True)
class AuthorityDecision:
    '''Store one scheduler decision and diagnostics'''

    authority: float
    target_authority: float
    effective_scale: float
    preview_risk: float
    response_risk: float
    feedforward_authority: float
    feedback_authority: float
    confirmation: float
    peak_index: int
    preview_valid: bool


class AdaptiveAuthorityScheduler:
    '''Schedule 3C authority from manipulation preview and measured response'''

    def __init__(self, config=None):
        self.config = config or AuthorityConfig()
        self._validate_config()
        self.reset()

    def reset(self):
        '''Clear episode authority and phase history'''
        self.authority = 0.0

    def evaluate(
            self, momentum, momentum_rate, tilt_error, angular_velocity,
            lifecycle_scale, preview_valid=True, feedback_enabled=True,
            confirmation_enabled=True):
        '''Evaluate one slew-limited adaptive authority decision'''
        tilt_error = _vector(tilt_error, 'tilt_error')
        angular_velocity = _vector(
            angular_velocity, 'angular_velocity',
        )
        response_risk = (
            np.linalg.norm(np.concatenate([
                tilt_error / self.config.tilt_scale,
                angular_velocity / self.config.angular_velocity_scale,
            ]))
            + self.config.divergence_weight
            * np.linalg.norm(np.maximum(
                tilt_error * angular_velocity, 0.0,
            ))
        )
        feedback = _smoothstep(
            response_risk,
            self.config.response_entry,
            self.config.response_full,
        )

        preview_risk = 0.0
        peak_index = 0
        feedforward = 0.0
        if preview_valid:
            momentum = _samples(momentum, 'momentum')
            momentum_rate = _samples(momentum_rate, 'momentum_rate')
            if len(momentum) != len(momentum_rate) + 1:
                raise ValueError('momentum preview sizes are inconsistent')
            rate_norm = np.linalg.norm(momentum_rate, axis=1)
            peak_index = int(np.argmax(rate_norm)) if len(rate_norm) else 0
            if len(momentum_rate):
                combined = np.sqrt(
                    np.linalg.norm(momentum[:-1], axis=1) ** 2
                    / self.config.momentum_scale ** 2
                    + rate_norm ** 2
                    / self.config.momentum_rate_scale ** 2
                )
                preview_risk = float(np.max(combined))
            feedforward = _smoothstep(
                preview_risk,
                self.config.preview_entry,
                self.config.preview_full,
            )

        confirmation = (
            1.0 if not confirmation_enabled or peak_index > 0 else feedback
        )
        target = max(
            confirmation * feedforward,
            feedback if feedback_enabled else 0.0,
        )
        rise = self.config.dt / self.config.rise_time
        decay = self.config.dt / self.config.decay_time
        self.authority = float(np.clip(
            target,
            self.authority - decay,
            self.authority + rise,
        ))
        lifecycle_scale = float(np.clip(lifecycle_scale, 0.0, 1.0))
        return AuthorityDecision(
            authority=self.authority,
            target_authority=float(target),
            effective_scale=self.authority * lifecycle_scale,
            preview_risk=preview_risk,
            response_risk=float(response_risk),
            feedforward_authority=float(feedforward),
            feedback_authority=float(feedback),
            confirmation=float(confirmation),
            peak_index=peak_index,
            preview_valid=bool(preview_valid),
        )

    def _validate_config(self):
        values = tuple(vars(self.config).values())
        if not all(np.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('authority scheduler values must be positive')
        if self.config.preview_full <= self.config.preview_entry:
            raise ValueError('preview full threshold must exceed entry')
        if self.config.response_full <= self.config.response_entry:
            raise ValueError('response full threshold must exceed entry')


def _smoothstep(value, entry, full):
    normalized = np.clip((value - entry) / (full - entry), 0.0, 1.0)
    return float(3.0 * normalized ** 2 - 2.0 * normalized ** 3)


def _vector(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (2,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite with shape (2,)')
    return result


def _samples(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 2 or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be a finite Nx2 matrix')
    return result
