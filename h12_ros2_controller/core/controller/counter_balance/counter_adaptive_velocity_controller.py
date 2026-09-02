import time

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.authority_scheduler import (
    AdaptiveAuthorityScheduler,
    AuthorityConfig,
)
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.utility.controller_config import load_controller_config


class CounterAdaptiveVelocityController(CounterDDPVelocityController):
    '''Schedule frozen 3C authority from preview and measured response'''

    def __init__(self, *args, config=None, **kwargs):
        resolved = load_controller_config() if config is None else config
        settings = self._adaptive_settings(
            resolved.get('adaptive_authority', {}),
        )
        super().__init__(*args, config=resolved, **kwargs)
        self.adaptive_shadow = settings['shadow']
        self.adaptive_mode = settings['mode']
        self.preview_steps = settings['preview_steps']
        self.max_preview_age = settings['max_preview_age']
        self.authority_scheduler = AdaptiveAuthorityScheduler(
            settings['scheduler'],
        )
        self.latest_authority_decision = None
        self.latest_adaptive_total_time = 0.0
        self.latest_preview_peak_momentum = 0.0
        self.latest_preview_peak_momentum_rate = 0.0
        self.latest_response_tilt_norm = 0.0
        self.latest_response_gyro_norm = 0.0
        self.latest_response_divergence_norm = 0.0
        self.adaptive_sequence = 0

    def _set_arm_ownership(self, moving_arm):
        super()._set_arm_ownership(moving_arm)
        if hasattr(self, 'authority_scheduler'):
            self.authority_scheduler.reset()

    def capture_reference(self):
        '''Capture 3C and scheduler references from a settled state'''
        self._require_arm_ownership()
        self.update_robot_model()
        motor_q, _ = self._measured_motor_state()
        support, com, _, _, _ = self._model_terms(motor_q)
        self._capture_reference(motor_q[self.arm_ids], support, com)
        if not self._reference_captured:
            raise RuntimeError('adaptive reference capture failed')
        self.authority_scheduler.reset()

    def control_preview_step(
            self, q_preview, dq_preview, lifecycle_scale,
            generated_at=None, sample_times=None):
        '''Apply one adaptive or shadow 3C command from a preview'''
        started = time.perf_counter()
        self.adaptive_sequence += 1
        q_preview = np.asarray(q_preview, dtype=np.float64)
        dq_preview = np.asarray(dq_preview, dtype=np.float64)
        if (
            q_preview.ndim != 2 or dq_preview.ndim != 2
            or q_preview.shape[1:] != (14,) or dq_preview.shape[1:] != (14,)
            or len(q_preview) < 1 or len(dq_preview) < 1
            or not np.all(np.isfinite(q_preview[0]))
            or not np.all(np.isfinite(dq_preview[0]))
        ):
            self.authority_scheduler.reset()
            raise ValueError('current adaptive manipulation sample is invalid')
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.authority_scheduler.reset()
            return super().control_configuration_step(
                q_preview[0], dq_preview[0], balance_scale=0.0,
            )
        preview_valid = False
        self.latest_preview_peak_momentum = 0.0
        self.latest_preview_peak_momentum_rate = 0.0
        self.latest_response_tilt_norm = 0.0
        self.latest_response_gyro_norm = 0.0
        self.latest_response_divergence_norm = 0.0
        try:
            q_preview, dq_preview = self._validated_preview(
                q_preview, dq_preview, generated_at, sample_times,
            )
            preview_valid = True
        except ValueError:
            self.authority_scheduler.reset()
            q_preview = np.repeat(
                q_preview[:1], self.preview_steps + 1, axis=0,
            )
            dq_preview = np.repeat(
                dq_preview[:1], self.preview_steps + 1, axis=0,
            )
        try:
            momentum, momentum_rate, tilt_error, gyro = (
                self._preview_signals(q_preview, dq_preview)
            )
        except Exception:
            self.authority_scheduler.reset()
            momentum = np.zeros((self.preview_steps + 1, 2))
            momentum_rate = np.zeros((self.preview_steps, 2))
            tilt_error = np.zeros(2)
            gyro = np.zeros(2)
            preview_valid = False
        decision = self.authority_scheduler.evaluate(
            momentum,
            momentum_rate,
            tilt_error,
            gyro,
            lifecycle_scale,
            preview_valid=preview_valid,
            feedback_enabled=self.adaptive_mode != 'preview_only',
            confirmation_enabled=self.adaptive_mode != 'preview_only',
        )
        self.latest_authority_decision = decision
        effective_scale = (
            float(np.clip(lifecycle_scale, 0.0, 1.0))
            if self.adaptive_shadow else decision.effective_scale
        )
        try:
            result = super().control_configuration_step(
                q_preview[0],
                dq_preview[0],
                balance_scale=effective_scale,
            )
            if self.latest_status not in ('solved', 'collision_backtracked'):
                self.authority_scheduler.reset()
            return result
        finally:
            self.latest_adaptive_total_time = time.perf_counter() - started

    def diagnostics(self):
        '''Return 3C and adaptive-authority diagnostics'''
        values = super().diagnostics()
        decision = self.latest_authority_decision
        values.update({
            'adaptive_shadow': bool(self.adaptive_shadow),
            'adaptive_sequence': int(self.adaptive_sequence),
            'adaptive_mode': self.adaptive_mode,
            'adaptive_available': decision is not None,
            'adaptive_authority': (
                float(decision.authority) if decision is not None else 0.0
            ),
            'adaptive_target_authority': (
                float(decision.target_authority)
                if decision is not None else 0.0
            ),
            'adaptive_effective_scale': (
                float(decision.effective_scale)
                if decision is not None else 0.0
            ),
            'adaptive_preview_risk': (
                float(decision.preview_risk) if decision is not None else 0.0
            ),
            'adaptive_response_risk': (
                float(decision.response_risk) if decision is not None else 0.0
            ),
            'adaptive_feedforward': (
                float(decision.feedforward_authority)
                if decision is not None else 0.0
            ),
            'adaptive_feedback': (
                float(decision.feedback_authority)
                if decision is not None else 0.0
            ),
            'adaptive_confirmation': (
                float(decision.confirmation) if decision is not None else 0.0
            ),
            'adaptive_peak_index': (
                int(decision.peak_index) if decision is not None else 0
            ),
            'adaptive_preview_valid': bool(
                decision.preview_valid if decision is not None else False
            ),
            'adaptive_preview_peak_momentum': float(
                self.latest_preview_peak_momentum
            ),
            'adaptive_preview_peak_momentum_rate': float(
                self.latest_preview_peak_momentum_rate
            ),
            'adaptive_response_tilt_norm': float(
                self.latest_response_tilt_norm
            ),
            'adaptive_response_gyro_norm': float(
                self.latest_response_gyro_norm
            ),
            'adaptive_response_divergence_norm': float(
                self.latest_response_divergence_norm
            ),
            'adaptive_total_time': float(self.latest_adaptive_total_time),
        })
        return values

    def _preview_signals(self, q_preview, dq_preview):
        self.update_robot_model()
        motor_q, _ = self._measured_motor_state()
        momentum = []
        torso_rotation = None
        for q_sample, dq_sample in zip(q_preview, dq_preview):
            candidate = np.copy(motor_q)
            candidate[self.moving_ids] = q_sample[self.moving_local]
            _, _, _, momentum_map, rotation = self._model_terms(candidate)
            moving_map = momentum_map[:2, self.moving_v_indices]
            momentum.append(
                moving_map @ dq_sample[self.moving_local]
            )
            if torso_rotation is None:
                torso_rotation = rotation
        momentum = np.asarray(momentum)
        momentum_rate = np.diff(momentum, axis=0) / self.dt
        tilt_error = (
            self._imu_tilt() - self.tilt_reference
            if self.tilt_reference is not None else np.zeros(2)
        )
        gyro, _ = self._torso_gyro(torso_rotation)
        self.latest_preview_peak_momentum = float(np.max(
            np.linalg.norm(momentum, axis=1),
        ))
        self.latest_preview_peak_momentum_rate = float(np.max(
            np.linalg.norm(momentum_rate, axis=1),
        ))
        self.latest_response_tilt_norm = float(np.linalg.norm(tilt_error))
        self.latest_response_gyro_norm = float(np.linalg.norm(gyro[:2]))
        self.latest_response_divergence_norm = float(np.linalg.norm(
            np.maximum(tilt_error * gyro[:2], 0.0),
        ))
        return momentum, momentum_rate, tilt_error, gyro[:2]

    def _validated_preview(
            self, q_preview, dq_preview, generated_at, sample_times):
        shape = (self.preview_steps + 1, 14)
        q_preview = np.asarray(q_preview, dtype=np.float64)
        dq_preview = np.asarray(dq_preview, dtype=np.float64)
        if (
            q_preview.shape != shape or dq_preview.shape != shape
            or not np.all(np.isfinite(q_preview[:, self.moving_local]))
            or not np.all(np.isfinite(dq_preview[:, self.moving_local]))
        ):
            raise ValueError(f'adaptive preview must be finite with shape {shape}')
        if generated_at is None or sample_times is None:
            raise ValueError('adaptive preview timestamps are required')
        sample_times = np.asarray(sample_times, dtype=np.float64)
        if (
            sample_times.shape != (self.preview_steps + 1,)
            or not np.all(np.isfinite(sample_times))
            or np.any(np.diff(sample_times) <= 0.0)
            or not np.allclose(np.diff(sample_times), self.dt, atol=1e-6)
            or not np.isclose(
                sample_times[0], generated_at, atol=1e-6, rtol=0.0,
            )
        ):
            raise ValueError('adaptive preview timestamps are invalid')
        age = time.monotonic() - float(generated_at)
        if age < -1e-3 or age > self.max_preview_age:
            raise ValueError('adaptive preview is stale')
        return np.copy(q_preview), np.copy(dq_preview)

    @staticmethod
    def _adaptive_settings(config):
        if not isinstance(config, dict):
            raise ValueError('adaptive_authority must be a mapping')
        shadow = config.get('shadow', True)
        if not isinstance(shadow, bool):
            raise ValueError('adaptive_authority.shadow must be a boolean')
        preview_steps = int(config.get('preview_steps', 10))
        if preview_steps < 1:
            raise ValueError('adaptive_authority.preview_steps must be positive')
        scheduler = AuthorityConfig(
            **(config.get('scheduler', {}) or {}),
        )
        mode = str(config.get('mode', 'combined'))
        if mode not in ('preview_only', 'combined'):
            raise ValueError('adaptive_authority.mode is invalid')
        return {
            'shadow': shadow,
            'mode': mode,
            'preview_steps': preview_steps,
            'max_preview_age': float(config.get('max_preview_age', 0.04)),
            'scheduler': scheduler,
        }
