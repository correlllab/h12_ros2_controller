from copy import deepcopy
from collections import deque

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller import (
    CounterDDPVelocityRobustController,
)


class CounterBalanceDiagnosticController(CounterDDPVelocityRobustController):
    '''Opt into reaction-target ablations without replacing the nominal finalizer

    CoM rows are model-frame xy velocities (m/s); angular momentum rows are
    centroidal model-frame xy (kg m^2/s), not ground reaction. Gyro is already
    rotated by the base controller into that same model frame (rad/s).
    Configuration and samples must be serialized by the idle parent adapter.
    No recovery, rate feedback, or learned-response actuation is implemented.
    '''

    DIAGNOSTIC_SUITE = True
    VARIANTS = (
        'c0', 'feedback_off', 'com_feedback_off', 'gyro_feedback_off',
        'ar_shadow', 'momentum_rate_shadow', 'ff_half',
    )
    RATE_DT_RANGE = (0.005, 0.2)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.configure_diagnostic_trial('c0')

    def configure_diagnostic_trial(self, variant, prediction_model=None):
        '''Reset an idle trial; the caller owns the trial lifecycle guard'''
        if getattr(self, '_diagnostic_in_step', False):
            raise RuntimeError('Cannot configure diagnostics during a control step')
        if not isinstance(variant, str) or variant not in self.VARIANTS:
            raise ValueError(f'Unknown diagnostic variant: {variant!r}')
        if prediction_model is not None:
            raise ValueError('No qualified prediction model contract is available')
        self.diagnostic_variant = variant
        self._diagnostic_sample = None
        self._diagnostic_record = {}
        self._diagnostic_previous = {}
        self._diagnostic_rates = {
            source: deque(maxlen=9) for source in ('measured', 'command')
        }
        self._diagnostic_in_step = False
        if getattr(self, 'moving_arm', None) in ('left', 'right'):
            self._set_arm_ownership(self.moving_arm)
        self._reset_diagnostics()
        self._nominal_observation = None
        self._latest_nominal_plan = None
        self._latest_nominal_context = None
        self._nominal_prepared_context = None
        self.latest_velocity_ocp_result = None
        self.latest_nominal_fallback_used = False
        self.latest_nominal_primary_result = None

    def set_diagnostic_sample(
            self, elapsed_s, move_duration_s, reference_preview=None):
        '''Supply execution-clock phase before exactly one configuration step'''
        if getattr(self, '_diagnostic_in_step', False):
            raise RuntimeError('Cannot replace a diagnostic sample during control')
        elapsed, duration = float(elapsed_s), float(move_duration_s)
        if not np.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError('elapsed_s must be finite and nonnegative')
        if not np.isfinite(duration) or duration <= 0.0:
            raise ValueError('move_duration_s must be finite and positive')
        # previews are data, never callbacks or an alternate control path
        def snapshot(value):
            if value is None or isinstance(value, (str, bool)):
                return value
            if isinstance(value, dict):
                if not all(isinstance(key, str) for key in value):
                    raise ValueError('reference_preview keys must be strings')
                return {key: snapshot(item) for key, item in value.items()}
            if isinstance(value, (list, tuple, np.ndarray)):
                return [snapshot(item) for item in value]
            if np.isscalar(value) and np.isfinite(float(value)):
                return float(value)
            raise ValueError('reference_preview must contain finite data only')

        preview = snapshot(reference_preview)
        self._diagnostic_sample = {
            'elapsed_s': elapsed,
            'move_duration_s': duration,
            'phase': 'move' if elapsed <= duration else 'post_move',
            'reference_preview': preview,
        }

    def control_configuration_step(
            self, moving_q_target_14, moving_dq_target_14,
            balance_scale=1.0):
        sample = self._diagnostic_sample
        self._diagnostic_sample = None
        self._diagnostic_record = {
            'variant': self.diagnostic_variant,
            'actuation': (
                self.diagnostic_variant if self.diagnostic_variant in (
                     'feedback_off', 'com_feedback_off', 'gyro_feedback_off', 'ff_half',
                ) else 'c0'
            ),
            'sample': deepcopy(sample),
            'phase_source': 'parent_execution_clock' if sample else 'unavailable',
            'signal_semantics': 'estimated_fixed_base_model_not_ground_reaction',
            'momentum_scope': 'two_arm_partial_centroidal_angular_xy_not_full_body',
            'frame': 'model_frame_xy_gyro_rotated_from_torso_by_base_controller',
            'units': {
                'com_rhs': 'm/s', 'momentum': 'kg*m^2/s',
                'momentum_rate': 'kg*m^2/s^2', 'gyro': 'rad/s',
            },
            'ddq_used': False,
            'rate_feedback_enabled': False,
            'ar_status': 'unavailable_no_qualified_prediction_model',
            'shadow_solve_status': 'not_computed',
            'overhead_policy': 'same_diagnostic_work_all_variants_no_extra_solves',
            'physical_effect_claim': False,
        }
        preview = sample.get('reference_preview') if sample else None
        if isinstance(preview, dict) and 'predicted_moving_dq' in preview:
            self._diagnostic_record['ar_status'] = 'shadow_prediction_only_no_actuation'
            self._diagnostic_record['ar_prediction_qualified'] = preview.get('prediction_qualified') is True
        self._diagnostic_in_step = True
        try:
            return super().control_configuration_step(
                moving_q_target_14, moving_dq_target_14,
                balance_scale=balance_scale,
            )
        finally:
            self._diagnostic_in_step = False
            if 'rates' not in self._diagnostic_record:
                self._diagnostic_previous.clear()
                for history in self._diagnostic_rates.values():
                    history.clear()

    def _reaction_target_offsets(
            self, com_moving, momentum_moving, moving_dq,
            com_error, gyro, balance_scale):
        com_offset, momentum_offset = super()._reaction_target_offsets(
            com_moving, momentum_moving, moving_dq,
            com_error, gyro, balance_scale,
        )
        com_feedback = -balance_scale * self.com_gain * com_error
        gyro_feedback = balance_scale * self.gyro_gain * gyro[:2]
        components = {
            'com_feedforward': balance_scale * (-com_moving @ moving_dq),
            'com_feedback': com_feedback,
            'momentum_feedforward': balance_scale * (
                -momentum_moving @ moving_dq
            ),
            'gyro_feedback': gyro_feedback,
        }
        if not all(np.all(np.isfinite(value)) for value in components.values()):
            raise ValueError('Diagnostic target components must be finite')
        variant = self.diagnostic_variant
        if variant in ('feedback_off', 'com_feedback_off'):
            com_offset = com_offset - com_feedback
        if variant in ('feedback_off', 'gyro_feedback_off'):
            momentum_offset = momentum_offset - gyro_feedback
        if variant == 'ff_half':
            feedforward_scale = 0.5
            # the inherited target already contains the full feedforward component
            com_offset += (feedforward_scale - 1.0) * components['com_feedforward']
            momentum_offset += (feedforward_scale - 1.0) * components['momentum_feedforward']
            self._diagnostic_record['feedforward_scale'] = feedforward_scale
        self._diagnostic_record['target_components'] = {
            key: value.tolist() for key, value in components.items()
        }
        sample = self._diagnostic_record.get('sample') or {}
        preview = sample.get('reference_preview') or {}
        prediction = preview.get('predicted_moving_dq')
        if prediction is not None:
            prediction = np.asarray(prediction, dtype=np.float64)
            if prediction.shape == (7,) and np.all(np.isfinite(prediction)):
                self._diagnostic_record['ar_shadow_targets'] = {
                    'com': (-balance_scale * (com_moving @ prediction) + com_feedback).tolist(),
                    'momentum': (-balance_scale * (momentum_moving @ prediction) + gyro_feedback).tolist(),
                    'used_for_actuation': False,
                }
        self._diagnostic_record['target_offsets'] = {
            'com': com_offset.tolist(), 'momentum': momentum_offset.tolist(),
        }
        return com_offset, momentum_offset

    def _finalize_counter_velocity(self, context, requested):
        result = super()._finalize_counter_velocity(context, requested)
        # diagnostics run after, and cannot modify, the inherited publication
        try:
            self._record_momentum_rates(context)
        except (ValueError, TypeError, IndexError, KeyError):
            self._diagnostic_record['rates'] = {'status': 'invalid_state'}
            self._diagnostic_previous.clear()
            for history in self._diagnostic_rates.values():
                history.clear()
        return result

    def _record_momentum_rates(self, context):
        state = self._control_observation_state()
        measured_dq = np.asarray(state['dq'], dtype=np.float64)
        measured = (
            context.momentum_moving @ measured_dq[self.moving_ids]
            + context.momentum_counter @ measured_dq[self.counter_ids]
        )
        command = (
            context.momentum_moving @ context.moving_dq
            + context.momentum_counter @ self.latest_applied_counter_dq
        )
        sample = self._diagnostic_record['sample']
        rates = {'status': 'diagnostic_only', 'sources': {}}
        for source, momentum, timestamp, clock in (
            ('measured', measured, state.get('arrival_monotonic'),
             'observation_arrival_monotonic_s'),
            ('command', command, sample['elapsed_s'] if sample else None,
             'parent_execution_elapsed_s'),
        ):
            valid = np.all(np.isfinite(momentum)) and (
                timestamp is not None and np.isfinite(float(timestamp))
            )
            previous = self._diagnostic_previous.pop(source, None)
            history = self._diagnostic_rates[source]
            record = {
                'clock': clock, 'timestamp_s': None,
                'momentum_xy': None, 'dt_s': None, 'valid': False,
                'raw_rate_xy': None, 'filtered_rate_xy': {},
            }
            if valid:
                timestamp = float(timestamp)
                record.update(timestamp_s=timestamp, momentum_xy=momentum.tolist())
                self._diagnostic_previous[source] = (timestamp, momentum.copy())
                if previous is not None:
                    dt = timestamp - previous[0]
                    record['dt_s'] = dt
                    if self.RATE_DT_RANGE[0] <= dt <= self.RATE_DT_RANGE[1]:
                        rate = (momentum - previous[1]) / dt
                        if np.all(np.isfinite(rate)):
                            history.append(rate)
                            record.update(valid=True, raw_rate_xy=rate.tolist())
            if not record['valid']:
                history.clear()
            for window in (3, 5, 9):
                record['filtered_rate_xy'][str(window)] = (
                    np.mean(list(history)[-window:], axis=0).tolist()
                    if len(history) >= window else None
                )
            rates['sources'][source] = record
        rates['filter'] = 'causal_mean_of_backward_differences_3_5_9_samples'
        rates['interpretation'] = (
            'model_frame_two_arm_xy_difference_not_inertial_momentum_rate; '
            'world_rotation_transport_and_remaining_body_terms_not_included'
        )
        rates['command_origin'] = (
            'moving_reference_dq_plus_finalized_counter_dq_not_delivery_confirmed'
        )
        rates['measured_clock_origin'] = (
            'local_observation_arrival_not_sensor_generation_time'
        )
        rates['valid_dt_range_s'] = list(self.RATE_DT_RANGE)
        self._diagnostic_record['rates'] = rates

    def diagnostics(self):
        values = super().diagnostics()
        values['counter_balance_diagnostic'] = deepcopy(self._diagnostic_record)
        return values
