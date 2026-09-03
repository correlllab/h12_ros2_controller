import time

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.utility.controller_config import load_controller_config


class CounterResidualProbeController(CounterDDPVelocityController):
    '''Collect bounded residual-command response around frozen 3C'''

    def __init__(self, *args, config=None, **kwargs):
        resolved = load_controller_config() if config is None else config
        settings = self._probe_settings(
            resolved.get('iteration5_residual_probe', {}),
        )
        super().__init__(*args, config=resolved, **kwargs)
        self.probe_shadow = settings['shadow']
        self.probe_mode = settings['mode']
        self.probe_first_step = settings['first_step']
        self.probe_spacing_steps = settings['spacing_steps']
        self.probe_pulse_steps = settings['pulse_steps']
        self.probe_seed = settings['seed']
        self.probe_sign_scale = settings['sign_scale']
        self.probe_realization_delay = settings['realization_delay']
        self.probe_realization_gain = settings['realization_gain']
        self.probe_momentum_damping = settings['momentum_damping']
        self.probe_max_residual_velocity = settings['max_residual_velocity']
        self.probe_sequence = self._probe_sequence(settings)
        self._probe_started = False
        self._probe_step = -1
        self._reset_probe_diagnostics()

    def _select_requested_counter_velocity(self, context, nominal):
        if context.balance_scale > 0.0:
            self._probe_started = True
        if self._probe_started:
            self._probe_step += 1
        schedule = self._scheduled_residual(self._probe_step, context)
        pulse_index = schedule['pulse_index']
        residual = schedule['residual']
        nominal_velocity = np.asarray(
            nominal.requested_counter_dq,
            dtype=np.float64,
        )
        residual = np.clip(
            residual,
            nominal.lower - nominal_velocity,
            nominal.upper - nominal_velocity,
        )
        selected = (
            nominal_velocity
            if self.probe_shadow
            else nominal_velocity + residual
        )
        self.latest_probe_step = self._probe_step
        self.latest_probe_pulse_index = pulse_index
        self.latest_probe_active = bool(np.any(residual != 0.0))
        self.latest_probe_nominal = np.copy(nominal_velocity)
        self.latest_probe_residual = np.copy(residual)
        self.latest_probe_combined = np.copy(selected)
        self.latest_probe_desired_momentum = np.copy(
            schedule['desired_momentum'],
        )
        self.latest_probe_expected_momentum = np.copy(
            schedule['expected_momentum'],
        )
        self.latest_probe_pulse_steps = schedule['pulse_steps']
        self.latest_probe_expected_response_step = (
            self._probe_step + self.probe_realization_delay
            if pulse_index >= 0 else -1
        )
        self.latest_probe_momentum_condition = schedule[
            'momentum_condition'
        ]
        self.latest_probe_momentum_map = np.copy(context.momentum_counter)
        self._capture_probe_observation(context, nominal_velocity)
        return selected

    def diagnostics(self):
        '''Return 3C and synchronized residual-probe diagnostics'''
        values = super().diagnostics()
        applied_residual = (
            self.latest_applied_counter_dq
            - self.latest_backtrack_scale * self.latest_probe_nominal
        )
        values.update({
            'probe_shadow': bool(self.probe_shadow),
            'probe_mode': self.probe_mode,
            'probe_step': int(self.latest_probe_step),
            'probe_pulse_index': int(self.latest_probe_pulse_index),
            'probe_active': bool(self.latest_probe_active),
            'probe_seed': int(self.probe_seed),
            'probe_sign_scale': float(self.probe_sign_scale),
            'probe_realization_delay': int(self.probe_realization_delay),
            'probe_nominal_dq': self.latest_probe_nominal.tolist(),
            'probe_residual_dq': self.latest_probe_residual.tolist(),
            'probe_combined_dq': self.latest_probe_combined.tolist(),
            'probe_applied_residual_dq': applied_residual.tolist(),
            'probe_desired_momentum': (
                self.latest_probe_desired_momentum.tolist()
            ),
            'probe_expected_momentum': (
                self.latest_probe_expected_momentum.tolist()
            ),
            'probe_pulse_steps': int(self.latest_probe_pulse_steps),
            'probe_expected_response_step': int(
                self.latest_probe_expected_response_step
            ),
            'probe_momentum_condition': (
                self.latest_probe_momentum_condition
            ),
            'probe_momentum_map': self.latest_probe_momentum_map.tolist(),
            'probe_tick': self.latest_probe_tick,
            'probe_unwrapped_tick': self.latest_probe_unwrapped_tick,
            'probe_sequence': self.latest_probe_sequence,
            'probe_sample_age': self.latest_probe_sample_age,
            'probe_tilt': self.latest_probe_tilt.tolist(),
            'probe_gyro': self.latest_probe_gyro.tolist(),
            'probe_counter_q': self.latest_probe_counter_q.tolist(),
            'probe_counter_dq': self.latest_probe_counter_dq.tolist(),
            'probe_counter_momentum': (
                self.latest_probe_counter_momentum.tolist()
            ),
            'probe_moving_momentum': (
                self.latest_probe_moving_momentum.tolist()
            ),
            'probe_nominal_counter_momentum': (
                self.latest_probe_nominal_counter_momentum.tolist()
            ),
        })
        return values

    def _scheduled_residual(self, step, context=None):
        residual = np.zeros(4, dtype=np.float64)
        result = {
            'pulse_index': -1,
            'residual': residual,
            'desired_momentum': np.zeros(2, dtype=np.float64),
            'expected_momentum': np.zeros(2, dtype=np.float64),
            'pulse_steps': 0,
            'momentum_condition': None,
        }
        if step < self.probe_first_step or not self.probe_sequence:
            return result
        relative = step - self.probe_first_step
        pulse_index = relative // self.probe_spacing_steps
        pulse_offset = relative % self.probe_spacing_steps
        if pulse_index >= len(self.probe_sequence):
            return result
        axis, amplitude, pulse_steps = self.probe_sequence[pulse_index]
        if pulse_offset >= pulse_steps:
            return result
        result['pulse_index'] = pulse_index
        result['pulse_steps'] = pulse_steps
        if self.probe_mode == 'joint':
            residual[axis] = amplitude
            return result
        if context is None:
            raise ValueError('momentum probe requires command context')
        desired = np.zeros(2, dtype=np.float64)
        desired[axis] = amplitude
        effective_map = (
            context.momentum_counter
            @ np.diag(self.probe_realization_gain)
        )
        gram = effective_map @ effective_map.T
        residual[:] = effective_map.T @ np.linalg.solve(
            gram + self.probe_momentum_damping * np.eye(2),
            desired,
        )
        residual[:] = np.clip(
            residual,
            -self.probe_max_residual_velocity,
            self.probe_max_residual_velocity,
        )
        result['desired_momentum'] = desired
        result['expected_momentum'] = effective_map @ residual
        singular_values = np.linalg.svd(effective_map, compute_uv=False)
        result['momentum_condition'] = float(
            singular_values[0] / max(
                singular_values[-1], np.finfo(np.float64).eps,
            )
        )
        return result

    def _capture_probe_observation(self, context, nominal_velocity):
        state = self.robot_model.state
        counter_dq = np.asarray(
            state.get('dq', np.zeros(27)),
            dtype=np.float64,
        ).reshape(-1)
        if counter_dq.shape != (27,) or not np.all(np.isfinite(counter_dq)):
            counter_dq = np.zeros(27, dtype=np.float64)
        measured_counter_dq = counter_dq[self.counter_ids]
        self.latest_probe_tick = self._optional_int(state.get('tick'))
        self.latest_probe_unwrapped_tick = self._optional_int(
            state.get('unwrapped_tick'),
        )
        self.latest_probe_sequence = self._optional_int(state.get('sequence'))
        arrival = state.get('arrival_monotonic')
        self.latest_probe_sample_age = (
            max(0.0, time.monotonic() - float(arrival))
            if arrival is not None and np.isfinite(arrival)
            else None
        )
        self.latest_probe_tilt = self._tilt_from_state(state)
        self.latest_probe_gyro = np.copy(context.gyro[:2])
        self.latest_probe_counter_q = np.copy(context.counter_q)
        self.latest_probe_counter_dq = np.copy(measured_counter_dq)
        self.latest_probe_counter_momentum = (
            context.momentum_counter @ measured_counter_dq
        )
        self.latest_probe_moving_momentum = (
            context.momentum_moving @ context.moving_dq
        )
        self.latest_probe_nominal_counter_momentum = (
            context.momentum_counter @ nominal_velocity
        )

    def _reset_probe_diagnostics(self):
        self.latest_probe_step = -1
        self.latest_probe_pulse_index = -1
        self.latest_probe_active = False
        self.latest_probe_nominal = np.zeros(4, dtype=np.float64)
        self.latest_probe_residual = np.zeros(4, dtype=np.float64)
        self.latest_probe_combined = np.zeros(4, dtype=np.float64)
        self.latest_probe_desired_momentum = np.zeros(2, dtype=np.float64)
        self.latest_probe_expected_momentum = np.zeros(2, dtype=np.float64)
        self.latest_probe_pulse_steps = 0
        self.latest_probe_expected_response_step = -1
        self.latest_probe_momentum_condition = None
        self.latest_probe_momentum_map = np.zeros((2, 4), dtype=np.float64)
        self.latest_probe_tick = None
        self.latest_probe_unwrapped_tick = None
        self.latest_probe_sequence = None
        self.latest_probe_sample_age = None
        self.latest_probe_tilt = np.zeros(2, dtype=np.float64)
        self.latest_probe_gyro = np.zeros(2, dtype=np.float64)
        self.latest_probe_counter_q = np.zeros(4, dtype=np.float64)
        self.latest_probe_counter_dq = np.zeros(4, dtype=np.float64)
        self.latest_probe_counter_momentum = np.zeros(2, dtype=np.float64)
        self.latest_probe_moving_momentum = np.zeros(2, dtype=np.float64)
        self.latest_probe_nominal_counter_momentum = np.zeros(
            2, dtype=np.float64,
        )

    @staticmethod
    def _tilt_from_state(state):
        imu = state.get('imu_state')
        quaternion = getattr(imu, 'quaternion', None)
        if quaternion is None:
            return np.zeros(2, dtype=np.float64)
        quaternion = np.asarray(quaternion, dtype=np.float64).reshape(-1)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            return np.zeros(2, dtype=np.float64)
        w, x, y, z = quaternion
        return np.array([
            np.arctan2(
                2.0 * (w * x + y * z),
                1.0 - 2.0 * (x * x + y * y),
            ),
            np.arcsin(np.clip(
                2.0 * (w * y - z * x),
                -1.0,
                1.0,
            )),
        ])

    @staticmethod
    def _optional_int(value):
        try:
            return int(value) if value is not None else None
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _probe_sequence(settings):
        mode = settings['mode']
        amplitudes = (
            settings['amplitudes']
            if mode == 'joint' else settings['momentum_amplitudes']
        )
        axes = settings['axes'] if mode == 'joint' else [0, 1]
        pulse_steps = (
            [settings['pulse_steps']]
            if mode == 'joint' else settings['pulse_step_options']
        )
        sequence = [
            (axis, settings['sign_scale'] * sign * amplitude, duration)
            for amplitude in amplitudes
            for sign in (-1.0, 1.0)
            for axis in axes
            for duration in pulse_steps
        ]
        if not sequence:
            return []
        rng = np.random.default_rng(settings['seed'])
        return [sequence[index] for index in rng.permutation(len(sequence))]

    @staticmethod
    def _probe_settings(config):
        if not isinstance(config, dict):
            raise ValueError('iteration5_residual_probe must be a mapping')
        shadow = config.get('shadow', True)
        if not isinstance(shadow, bool):
            raise ValueError('iteration5_residual_probe.shadow must be boolean')
        mode = config.get('mode', 'joint')
        if mode not in ('joint', 'momentum'):
            raise ValueError(
                'iteration5_residual_probe.mode must be joint or momentum'
            )
        values = {}
        for name, default, minimum in (
            ('first_step', 5, 0),
            ('spacing_steps', 4, 1),
            ('pulse_steps', 1, 1),
            ('seed', 0, 0),
        ):
            value = config.get(name, default)
            if not isinstance(value, int) or value < minimum:
                raise ValueError(
                    f'iteration5_residual_probe.{name} is invalid'
                )
            values[name] = value
        if values['pulse_steps'] > values['spacing_steps']:
            raise ValueError(
                'iteration5_residual_probe.pulse_steps exceeds spacing_steps'
            )
        amplitudes = np.asarray(
            config.get('amplitudes', [0.08, 0.16]),
            dtype=np.float64,
        ).reshape(-1)
        if (
            not np.all(np.isfinite(amplitudes))
            or np.any(amplitudes <= 0.0)
        ):
            raise ValueError(
                'iteration5_residual_probe.amplitudes must be positive'
            )
        sign_scale = float(config.get('sign_scale', 1.0))
        if not np.isfinite(sign_scale) or sign_scale == 0.0:
            raise ValueError(
                'iteration5_residual_probe.sign_scale must be finite and nonzero'
            )
        axes = config.get('axes', [0, 1, 2, 3])
        if (
            not isinstance(axes, list)
            or not axes
            or any(not isinstance(axis, int) or axis not in range(4)
                   for axis in axes)
            or len(set(axes)) != len(axes)
        ):
            raise ValueError(
                'iteration5_residual_probe.axes must be unique axes'
            )
        momentum_amplitudes = np.asarray(
            config.get('momentum_amplitudes', [0.05]),
            dtype=np.float64,
        ).reshape(-1)
        if (
            not np.all(np.isfinite(momentum_amplitudes))
            or np.any(momentum_amplitudes <= 0.0)
        ):
            raise ValueError(
                'iteration5_residual_probe.momentum_amplitudes must be positive'
            )
        pulse_step_options = config.get('pulse_step_options', [1, 2])
        if (
            not isinstance(pulse_step_options, list)
            or not pulse_step_options
            or any(not isinstance(value, int) or value < 1
                   for value in pulse_step_options)
            or max(pulse_step_options) > values['spacing_steps']
        ):
            raise ValueError(
                'iteration5_residual_probe.pulse_step_options are invalid'
            )
        realization_delay = config.get('realization_delay', 1)
        if realization_delay not in (0, 1):
            raise ValueError(
                'iteration5_residual_probe.realization_delay must be zero or one'
            )
        realization_gain = np.asarray(
            config.get('realization_gain', [0.16, 0.24, 0.19, 0.30]),
            dtype=np.float64,
        )
        if (
            realization_gain.shape != (4,)
            or not np.all(np.isfinite(realization_gain))
            or np.any(realization_gain <= 0.0)
        ):
            raise ValueError(
                'iteration5_residual_probe.realization_gain must be positive'
            )
        momentum_damping = float(config.get('momentum_damping', 1e-4))
        max_residual_velocity = float(
            config.get('max_residual_velocity', 0.6)
        )
        if not np.isfinite(momentum_damping) or momentum_damping <= 0.0:
            raise ValueError(
                'iteration5_residual_probe.momentum_damping must be positive'
            )
        if (
            not np.isfinite(max_residual_velocity)
            or max_residual_velocity <= 0.0
        ):
            raise ValueError(
                'iteration5_residual_probe.max_residual_velocity must be positive'
            )
        return {
            'shadow': shadow,
            'mode': mode,
            'amplitudes': amplitudes.tolist(),
            'axes': axes,
            'momentum_amplitudes': momentum_amplitudes.tolist(),
            'pulse_step_options': pulse_step_options,
            'realization_delay': realization_delay,
            'realization_gain': realization_gain,
            'momentum_damping': momentum_damping,
            'max_residual_velocity': max_residual_velocity,
            'sign_scale': sign_scale,
            **values,
        }
