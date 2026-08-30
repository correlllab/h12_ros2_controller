import time

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_balance_controller import (
    CounterBalanceController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_ocp import (
    CounterDDPOCP,
    FrozenBalanceKnot,
)
from h12_ros2_controller.core.controller.counter_balance.reaction_observer import (
    ReactionObserver,
    predict_frozen_momentum_rate,
    reaction_seed_diagnostic,
)
from h12_ros2_controller.utility.controller_config import load_controller_config


class CounterDDPController(CounterBalanceController):
    '''Finite-horizon DDP counter-arm controller'''

    def __init__(self, urdf_path: str, urdf_sphere_path: str,
                 srdf_sphere_path: str, moving_arm: str = None,
                 init: bool = True, handless: bool = False,
                 visualize: bool = False, config: dict = None):
        resolved_config = (
            load_controller_config() if config is None else config
        )
        settings = self._parse_ddp_config(
            resolved_config.get('counter_ddp', {}),
        )
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            moving_arm=moving_arm,
            init=init,
            handless=handless,
            visualize=visualize,
            config=resolved_config,
        )
        self.shadow = settings['shadow']
        self.horizon_steps = settings['horizon_steps']
        self.preview_steps = settings['preview_steps']
        self.max_acceleration = settings['max_acceleration']
        self.max_acceleration_change = settings['max_acceleration_change']
        self.max_velocity = settings['max_velocity']
        self.max_excursion = settings['max_excursion']
        self.gyro_rate_threshold = settings['gyro_rate_threshold']
        self.gyro_rate_full_scale = settings['gyro_rate_full_scale']
        self.moving_velocity_threshold = settings[
            'moving_velocity_threshold'
        ]
        self.moving_feedforward_authority = settings[
            'moving_feedforward_authority'
        ]
        self.stationary_gyro_feedback = settings[
            'stationary_gyro_feedback'
        ]
        self.stationary_feedback_max_authority = settings[
            'stationary_feedback_max_authority'
        ]
        self.moving_momentum_threshold = settings[
            'moving_momentum_threshold'
        ]
        self.moving_momentum_full_scale = settings[
            'moving_momentum_full_scale'
        ]
        self.moving_momentum_max_authority = settings[
            'moving_momentum_max_authority'
        ]
        self.reaction_feedforward_gain = settings[
            'reaction_feedforward_gain'
        ]
        self.reaction_effectiveness = settings['reaction_effectiveness']
        self.reaction_diagnostics_enabled = settings[
            'reaction_diagnostics_enabled'
        ]
        self.max_forecast_age = settings['max_forecast_age']
        self.max_frozen_map_displacement = settings[
            'max_frozen_map_displacement'
        ]
        self.metric_tolerance = settings['metric_tolerance']
        self.validation_steps = settings['validation_steps']
        self.timing_guard = settings['timing_guard']
        self.support_geometry = settings['support_geometry']
        self.ddp = CounterDDPOCP(
            dt=self.dt,
            horizon_steps=self.horizon_steps,
            max_iterations=settings['max_iterations'],
            weights=settings['weights'],
            gains=settings['gains'],
            scales=settings['scales'],
            initial_regularization=settings['initial_regularization'],
            minimum_cost_improvement=settings['minimum_cost_improvement'],
        )
        self.reaction_observer = ReactionObserver()
        self._previous_acceleration = np.zeros(4, dtype=np.float64)
        self._reset_ddp_diagnostics()

    def _set_arm_ownership(self, moving_arm):
        super()._set_arm_ownership(moving_arm)
        if hasattr(self, 'ddp'):
            self.ddp.reset()
        if hasattr(self, '_previous_acceleration'):
            self._previous_acceleration[:] = 0.0
        if hasattr(self, 'reaction_observer'):
            self.reaction_observer.reset()

    def capture_reference(self):
        '''Capture a settled counter posture and support-relative CoM'''
        self._require_arm_ownership()
        self.update_robot_model()
        motor_q, _ = self._measured_motor_state()
        support, com, _, _, _ = self._model_terms(motor_q)
        measured_arm_q = motor_q[self.arm_ids]
        self._capture_reference(measured_arm_q, support, com)
        if not self._reference_captured:
            raise RuntimeError('counter DDP reference capture failed')

    def control_horizon_step(self, moving_q_horizon, moving_dq_horizon,
                             moving_tau=None, sample_times=None,
                             generated_at=None, authority_scale=1.0,
                             forecast_source='explicit',
                             preview_q_horizon=None,
                             preview_dq_horizon=None):
        '''Solve and apply one receding-horizon counter-arm command'''
        started = time.perf_counter()
        self._require_arm_ownership()
        authority_scale = self._validated_balance_scale(authority_scale)
        self._frame_balance_scale = authority_scale
        self._reset_diagnostics(authority_scale)
        self._reset_ddp_diagnostics()
        self.latest_forecast_source = str(forecast_source)
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.latest_status = 'estopped'
            return np.zeros(14, dtype=np.float64)

        q_horizon, dq_horizon = self._validated_horizon(
            moving_q_horizon, moving_dq_horizon,
        )
        preview_q, preview_dq = q_horizon, dq_horizon
        if self.reaction_diagnostics_enabled:
            try:
                preview_q, preview_dq = self._validated_preview(
                    preview_q_horizon,
                    preview_dq_horizon,
                    q_horizon,
                    dq_horizon,
                )
                self.latest_preview_input_valid = True
            except Exception:
                self.latest_preview_input_valid = False
        tau_target = self._validated_tau(moving_tau)
        motor_q, motor_dq = self._measured_motor_state()
        measured_arm_q = np.copy(motor_q[self.arm_ids])
        arm_q, arm_dq, input_valid = self._moving_sample(
            q_horizon[0], dq_horizon[0], measured_arm_q,
        )
        if not input_valid:
            raise ValueError('moving-arm command must be finite')
        try:
            self._validate_forecast_time(sample_times, generated_at)
        except ValueError:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'invalid_forecast_time', started,
            )
        try:
            self.update_robot_model()
        except Exception:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'model_update_failure', started,
            )
        motor_q, motor_dq = self._measured_motor_state()
        state_sample_time = time.perf_counter()
        measured_arm_q = np.copy(motor_q[self.arm_ids])
        arm_q, arm_dq, input_valid = self._moving_sample(
            q_horizon[0], dq_horizon[0], measured_arm_q,
        )
        if not input_valid:
            raise ValueError('moving-arm command must be finite')

        counter_q = np.copy(measured_arm_q[self.counter_active_local])
        counter_dq = np.copy(motor_dq[self.counter_ids])
        try:
            support, com, _, momentum_map, rotation = self._model_terms(motor_q)
        except Exception:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'model_failure', started,
            )
        self.latest_support_valid = bool(support.valid)
        self.latest_support_error = support.invalid_reason
        self.latest_com = np.copy(com)
        if not self._reference_captured:
            self._capture_reference(measured_arm_q, support, com)
        if not support.valid or not self._reference_captured:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'invalid_support', started,
            )

        try:
            gyro, _ = self._torso_gyro(rotation)
            current_counter_map = np.copy(
                momentum_map[:2, self.counter_v_indices],
            )
            current_moving_map = np.copy(
                momentum_map[:2, self.moving_v_indices],
            )
            current_moving_dq = np.copy(
                dq_horizon[0, self.moving_local],
            )
            activation = self._activation(
                gyro[:2], dq_horizon, authority_scale,
                current_moving_map,
            )
        except Exception:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'activation_failure', started,
            )
        if self.reaction_diagnostics_enabled:
            try:
                self.latest_reaction_state_timestamp = state_sample_time
                self._record_response_diagnostics(gyro[:2])
                self._record_reaction_measurement(
                    state_sample_time,
                    counter_dq,
                    current_counter_map @ counter_dq,
                    current_moving_map @ motor_dq[self.moving_ids],
                    gyro[:2],
                )
                if self.latest_preview_input_valid:
                    self._record_disturbance_preview(
                        current_moving_map, preview_dq,
                    )
            except Exception:
                self.latest_reaction_measurement_valid = False
                self.latest_preview_input_valid = False
        self.latest_activation_scale = float(np.max(activation))
        self.latest_balance_scale = self.latest_activation_scale
        self.latest_axis_activation = np.copy(activation)
        if not np.any(activation > 0.0):
            return self._publish_inactive_sample(
                motor_q,
                q_horizon[0],
                dq_horizon[0],
                tau_target,
                started,
            )

        try:
            knots, _ = self._build_knots(
                motor_q, q_horizon, dq_horizon, authority_scale,
                activation=activation,
            )
            lower, upper = self._control_bounds(counter_q, counter_dq)
        except Exception:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'problem_build_failure', started,
            )
        if self.reaction_diagnostics_enabled:
            try:
                seed_lower, seed_upper = self._reaction_seed_bounds(
                    counter_q, counter_dq, lower[0], upper[0],
                )
                self._record_reaction_seed_diagnostic(
                    current_counter_map,
                    counter_dq,
                    knots[0],
                    seed_lower,
                    seed_upper,
                )
            except Exception:
                self.latest_reaction_seed_valid = False
        try:
            result = self.ddp.solve(
                np.concatenate([counter_q, counter_dq]),
                knots,
                lower,
                upper,
                self.q_counter_ref,
            )
        except Exception:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'solver_failure', started,
            )
        self._record_result(result)
        if not result.accepted:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                result.status, started,
            )
        if self.reaction_diagnostics_enabled:
            try:
                self._record_reaction_prediction(
                    current_counter_map,
                    current_moving_map,
                    counter_dq,
                    current_moving_dq,
                    knots[0],
                    result.xs[1, 4:],
                )
            except Exception:
                self.latest_reaction_prediction_valid = False

        valid, status, metric_error = self._validate_trajectory(
            motor_q, q_horizon, dq_horizon, knots, result.xs,
        )
        self.latest_metric_mismatch = metric_error
        if not valid:
            self.ddp.reset()
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target, status, started,
            )

        requested_acceleration = np.copy(result.us[0])
        requested_q = (
            counter_q + self.dt * counter_dq
            + 0.5 * self.dt ** 2 * requested_acceleration
        )
        requested_dq = counter_dq + self.dt * requested_acceleration
        self.latest_requested_counter_acceleration = requested_acceleration
        self.latest_requested_counter_dq = np.copy(requested_dq)
        backtracked, failure_status = self._backtrack_ddp_counter(
            motor_q,
            arm_q,
            arm_dq,
            counter_q,
            counter_dq,
            requested_q,
            requested_dq,
        )
        if backtracked is None:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                failure_status or 'counter_candidate_invalid', started,
            )
        candidate_q, command_dq, scale = backtracked
        applied_acceleration = (
            command_dq[self.counter_active_local] - counter_dq
        ) / self.dt
        self.latest_backtrack_scale = scale
        self.latest_collision_rejection = scale < 1.0
        self.latest_applied_counter_acceleration = applied_acceleration
        self.latest_applied_counter_dq = np.copy(
            command_dq[self.counter_active_local],
        )
        self._counter_q_command = np.copy(
            candidate_q[self.counter_active_local],
        )
        self._previous_acceleration = np.copy(applied_acceleration)

        if self.shadow:
            self.latest_status = f'shadow_{result.status}'
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                self.latest_status, started,
            )
        prepared_tau, torque_source, gravity_valid = self._prepare_arm_torque(
            motor_q, tau_target,
        )
        if not gravity_valid:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'gravity_failure', started,
                prepared_tau=(prepared_tau, torque_source, gravity_valid),
            )
        elapsed = time.perf_counter() - started
        if elapsed > self.timing_guard:
            self.latest_timing_overrun = True
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'timing_overrun', started,
                prepared_tau=(prepared_tau, torque_source, gravity_valid),
            )
        self.latest_status = (
            'collision_backtracked' if scale < 1.0 else result.status
        )
        if not self._publish_ddp_arm_command(
            candidate_q,
            command_dq,
            motor_q,
            tau_target,
            prepared_tau=(prepared_tau, torque_source, gravity_valid),
        ):
            self.latest_total_time = time.perf_counter() - started
            return np.zeros(14, dtype=np.float64)
        self.latest_total_time = time.perf_counter() - started
        if self.latest_total_time > self.timing_guard:
            self.latest_timing_overrun = True
            self.latest_status = 'published_timing_overrun'
        return command_dq

    def control_configuration_step(self, moving_q_target_14,
                                   moving_dq_target_14,
                                   balance_scale=1.0):
        '''Apply one sample using a bounded constant-velocity forecast'''
        q_target = self._as_arm_sample(
            moving_q_target_14, 'moving_q_target_14',
        )
        dq_target = self._as_arm_sample(
            moving_dq_target_14, 'moving_dq_target_14',
        )
        offsets = np.arange(self.horizon_steps + 1)[:, None] * self.dt
        q_horizon = q_target[None, :] + offsets * dq_target[None, :]
        dq_horizon = np.repeat(
            dq_target[None, :], self.horizon_steps + 1, axis=0,
        )
        preview_offsets = (
            np.arange(self.preview_steps + 1)[:, None] * self.dt
        )
        preview_q = q_target[None, :] + preview_offsets * dq_target[None, :]
        preview_dq = np.repeat(
            dq_target[None, :], self.preview_steps + 1, axis=0,
        )
        return self.control_horizon_step(
            q_horizon,
            dq_horizon,
            authority_scale=balance_scale,
            forecast_source='constant_velocity',
            preview_q_horizon=preview_q,
            preview_dq_horizon=preview_dq,
        )

    def _publish_position_command(self, q, dq, tau):
        '''Route inherited frame control without changing moving-arm semantics'''
        q = np.asarray(q, dtype=np.float64)
        dq = np.asarray(dq, dtype=np.float64)
        tau = np.asarray(tau, dtype=np.float64)
        moving_tau = tau[self.arm_ids] if tau.shape == (27,) else None
        q_target = q[self.arm_ids]
        dq_target = dq[self.arm_ids]
        forecast_dq = np.asarray(
            getattr(self, '_inherited_forecast_dq', dq_target),
            dtype=np.float64,
        )
        offsets = np.arange(self.horizon_steps + 1)[:, None] * self.dt
        q_horizon = q_target[None, :] + offsets * forecast_dq[None, :]
        dq_horizon = np.repeat(
            forecast_dq[None, :], self.horizon_steps + 1, axis=0,
        )
        q_horizon[0] = q_target
        dq_horizon[0] = dq_target
        preview_offsets = (
            np.arange(self.preview_steps + 1)[:, None] * self.dt
        )
        preview_q = q_target[None, :] + preview_offsets * forecast_dq[None, :]
        preview_dq = np.repeat(
            forecast_dq[None, :], self.preview_steps + 1, axis=0,
        )
        preview_q[0] = q_target
        preview_dq[0] = dq_target
        self.control_horizon_step(
            q_horizon,
            dq_horizon,
            moving_tau=moving_tau,
            authority_scale=self._frame_balance_scale,
            forecast_source='inherited_constant_velocity',
            preview_q_horizon=preview_q,
            preview_dq_horizon=preview_dq,
        )

    def _apply_velocity_command(self, vel):
        '''Preserve inherited publication while retaining IK forecast velocity'''
        vel = np.asarray(vel, dtype=np.float64)
        self._inherited_forecast_dq = np.copy(vel[self.arm_ids])
        try:
            return super()._apply_velocity_command(vel)
        finally:
            del self._inherited_forecast_dq

    def diagnostics(self):
        '''Return serializable DDP controller diagnostics'''
        values = super().diagnostics()
        values.update({
            'shadow': bool(self.shadow),
            'forecast_source': self.latest_forecast_source,
            'axis_activation': self.latest_axis_activation.tolist(),
            'gyro_rate': float(self.latest_gyro_rate),
            'moving_speed': float(self.latest_moving_speed),
            'response_tilt_error': self.latest_response_tilt_error.tolist(),
            'response_gyro': self.latest_response_gyro.tolist(),
            'response_divergence': self.latest_response_divergence.tolist(),
            'response_divergence_norm': float(
                self.latest_response_divergence_norm
            ),
            'moving_momentum_risk': float(self.latest_moving_momentum_risk),
            'moving_momentum_authority': float(
                self.latest_moving_momentum_authority
            ),
            'reaction_measurement_valid': bool(
                self.latest_reaction_measurement_valid
            ),
            'reaction_prediction_valid': bool(
                self.latest_reaction_prediction_valid
            ),
            'reaction_dt': self._finite_or_none(self.latest_reaction_dt),
            'measured_counter_momentum': (
                self.latest_measured_counter_momentum.tolist()
            ),
            'measured_moving_momentum': (
                self.latest_measured_moving_momentum.tolist()
            ),
            'measured_total_arm_momentum': (
                self.latest_measured_total_arm_momentum.tolist()
            ),
            'measured_counter_momentum_rate': (
                self.latest_measured_counter_momentum_rate.tolist()
            ),
            'measured_moving_momentum_rate': (
                self.latest_measured_moving_momentum_rate.tolist()
            ),
            'measured_total_arm_momentum_rate': (
                self.latest_measured_total_arm_momentum_rate.tolist()
            ),
            'measured_counter_acceleration': (
                self.latest_measured_counter_acceleration.tolist()
            ),
            'measured_base_angular_acceleration': (
                self.latest_measured_base_angular_acceleration.tolist()
            ),
            'predicted_counter_momentum_rate': (
                self.latest_predicted_counter_momentum_rate.tolist()
            ),
            'predicted_moving_momentum_rate': (
                self.latest_predicted_moving_momentum_rate.tolist()
            ),
            'predicted_total_arm_momentum_rate': (
                self.latest_predicted_total_arm_momentum_rate.tolist()
            ),
            'reaction_diagnostics_enabled': bool(
                self.reaction_diagnostics_enabled
            ),
            'reaction_state_timestamp': self._finite_or_none(
                self.latest_reaction_state_timestamp
            ),
            'reaction_command_timestamp': self._finite_or_none(
                self.latest_reaction_command_timestamp
            ),
            'preview_input_valid': bool(self.latest_preview_input_valid),
            'preview_configured_steps': int(self.preview_steps),
            'preview_steps': int(self.latest_preview_actual_steps),
            'preview_duration': float(self.latest_preview_actual_duration),
            'preview_moving_momentum': (
                self.latest_preview_moving_momentum.tolist()
            ),
            'preview_moving_momentum_rate': (
                self.latest_preview_moving_momentum_rate.tolist()
            ),
            'preview_peak_momentum_rate': float(
                self.latest_preview_peak_momentum_rate
            ),
            'preview_peak_momentum_change': float(
                self.latest_preview_peak_momentum_change
            ),
            'preview_discounted_rate_exposure': float(
                self.latest_preview_discounted_rate_exposure
            ),
            'preview_time_to_peak_rate': float(
                self.latest_preview_time_to_peak_rate
            ),
            'h3_peak_momentum_rate': float(
                self.latest_h3_peak_momentum_rate
            ),
            'h3_peak_momentum_change': float(
                self.latest_h3_peak_momentum_change
            ),
            'h3_discounted_rate_exposure': float(
                self.latest_h3_discounted_rate_exposure
            ),
            'h3_time_to_peak_rate': float(
                self.latest_h3_time_to_peak_rate
            ),
            'reaction_seed_valid': bool(self.latest_reaction_seed_valid),
            'reaction_seed_desired_rate': (
                self.latest_reaction_seed_desired_rate.tolist()
            ),
            'reaction_seed_unbounded_acceleration': (
                self.latest_reaction_seed_unbounded_acceleration.tolist()
            ),
            'reaction_seed_clipped_acceleration': (
                self.latest_reaction_seed_clipped_acceleration.tolist()
            ),
            'reaction_seed_saturation_mask': (
                self.latest_reaction_seed_saturation_mask.tolist()
            ),
            'reaction_seed_achieved_rate': (
                self.latest_reaction_seed_achieved_rate.tolist()
            ),
            'reaction_seed_residual': (
                self.latest_reaction_seed_residual.tolist()
            ),
            'reaction_seed_unbounded_feasible': bool(
                self.latest_reaction_seed_unbounded_feasible
            ),
            'requested_counter_acceleration': (
                self.latest_requested_counter_acceleration.tolist()
            ),
            'applied_counter_acceleration': (
                self.latest_applied_counter_acceleration.tolist()
            ),
            'solver_converged': bool(self.latest_solver_converged),
            'solver_iterations': int(self.latest_solver_iterations),
            'solver_stopping_criterion': float(
                self.latest_solver_stopping_criterion
            ),
            'seed_cost': self._finite_or_none(self.latest_seed_cost),
            'optimized_cost': self._finite_or_none(
                self.latest_optimized_cost,
            ),
            'solve_time': float(self.latest_solve_time),
            'total_time': float(self.latest_total_time),
            'timing_overrun': bool(self.latest_timing_overrun),
            'warm_started': bool(self.latest_warm_started),
            'metric_mismatch': self._finite_or_none(
                self.latest_metric_mismatch,
            ),
            'torque_source': self.latest_torque_source,
        })
        return values

    def _build_knots(self, motor_q, q_horizon, dq_horizon, authority_scale,
                     activation=None):
        raw = []
        com_target = None
        q_lower, q_upper = self._effective_counter_position_limits()
        velocity_limit = np.minimum(
            self._arm_velocity_limits()[self.counter_active_local],
            self.max_velocity,
        )
        for index in range(self.horizon_steps):
            endpoint = index + 1
            candidate_motor_q = np.copy(motor_q)
            candidate_motor_q[self.moving_ids] = q_horizon[
                endpoint, self.moving_local
            ]
            candidate_motor_q[self.counter_wrist_ids] = self.counter_wrist_ref
            support, com, com_jacobian, momentum_map, rotation = (
                self._model_terms(candidate_motor_q)
            )
            if not support.valid:
                raise ValueError('invalid predicted support')
            com_target = support.center + self.com_offset_ref
            gyro, _ = self._torso_gyro(rotation)
            moving_dq = dq_horizon[endpoint, self.moving_local]
            com_counter = np.copy(
                com_jacobian[:2, self.counter_v_indices],
            )
            com_moving = np.copy(
                com_jacobian[:2, self.moving_v_indices],
            )
            momentum_counter = np.copy(
                momentum_map[:2, self.counter_v_indices],
            )
            momentum_moving = np.copy(
                momentum_map[:2, self.moving_v_indices],
            )
            raw.append((
                np.copy(com[:2]),
                np.copy(com_target),
                com_counter,
                com_moving,
                momentum_counter,
                momentum_moving,
                np.copy(moving_dq),
                np.copy(gyro[:2]),
            ))
        if activation is None:
            activation = self._activation(
                raw[0][7], dq_horizon, authority_scale, raw[0][5],
            )
        knots = []
        counter_q = motor_q[self.counter_ids]
        for values in raw:
            knots.append(FrozenBalanceKnot(
                q_bar=np.copy(counter_q),
                q_ref=np.copy(self.q_counter_ref),
                com_bar=values[0],
                com_target=values[1],
                com_counter=values[2],
                com_moving=values[3],
                momentum_counter=values[4],
                momentum_moving=values[5],
                moving_dq=values[6],
                gyro_xy=values[7],
                com_gate=np.array([activation[1], activation[0]]),
                momentum_gate=np.copy(activation),
                q_lower=np.copy(q_lower),
                q_upper=np.copy(q_upper),
                velocity_limit=np.copy(velocity_limit),
            ))
        self.latest_com_target = np.copy(com_target)
        return knots, activation

    def _activation(self, gyro_xy, dq_horizon, authority_scale,
                    moving_momentum_map=None):
        gyro_rate = float(np.linalg.norm(gyro_xy))
        moving_speed = float(np.max(np.linalg.norm(
            dq_horizon[:, self.moving_local], axis=1,
        )))
        momentum_risk = 0.0
        momentum_authority = self.moving_feedforward_authority
        if (
            self.moving_momentum_threshold is not None
            and moving_momentum_map is not None
        ):
            moving_dq = dq_horizon[:, self.moving_local]
            momentum = moving_dq @ np.asarray(
                moving_momentum_map, dtype=np.float64,
            ).T
            momentum_risk = float(np.max(np.linalg.norm(momentum, axis=1)))
            risk_scale = np.clip(
                (momentum_risk - self.moving_momentum_threshold)
                / (
                    self.moving_momentum_full_scale
                    - self.moving_momentum_threshold
                ),
                0.0,
                1.0,
            )
            momentum_authority += (
                self.moving_momentum_max_authority
                - self.moving_feedforward_authority
            ) * risk_scale
        activation_scale = np.clip(
            (gyro_rate - self.gyro_rate_threshold)
            / (self.gyro_rate_full_scale - self.gyro_rate_threshold),
            0.0,
            1.0,
        )
        moving = moving_speed > self.moving_velocity_threshold
        if not moving and not self.stationary_gyro_feedback:
            activation_scale = 0.0
        elif not moving:
            activation_scale = min(
                activation_scale * authority_scale,
                self.stationary_feedback_max_authority,
            )
        elif moving:
            activation_scale = max(
                activation_scale,
                momentum_authority,
            )
            activation_scale *= authority_scale
        activation_scale = np.clip(activation_scale, 0.0, 1.0)
        activation = np.full(2, activation_scale, dtype=np.float64)
        self.latest_gyro_rate = gyro_rate
        self.latest_moving_speed = moving_speed
        self.latest_moving_momentum_risk = momentum_risk
        self.latest_moving_momentum_authority = momentum_authority
        return activation

    def _control_bounds(self, counter_q, counter_dq):
        lower = np.tile(-self.max_acceleration, (self.horizon_steps, 1))
        upper = np.tile(self.max_acceleration, (self.horizon_steps, 1))
        q_lower, q_upper = self._physical_counter_position_limits()
        velocity = np.minimum(
            self._arm_velocity_limits()[self.counter_active_local],
            self.max_velocity,
        )
        safety_lower = np.maximum.reduce([
            lower[0],
            (-velocity - counter_dq) / self.dt,
            2.0 * (
                q_lower - counter_q - self.dt * counter_dq
            ) / self.dt ** 2,
        ])
        safety_upper = np.minimum.reduce([
            upper[0],
            (velocity - counter_dq) / self.dt,
            2.0 * (
                q_upper - counter_q - self.dt * counter_dq
            ) / self.dt ** 2,
        ])
        if np.any(safety_lower > safety_upper):
            raise ValueError('first acceleration bounds are infeasible')
        first_lower = np.maximum(
            safety_lower,
            self._previous_acceleration - self.max_acceleration_change,
        )
        first_upper = np.minimum(
            safety_upper,
            self._previous_acceleration + self.max_acceleration_change,
        )
        infeasible_slew = first_lower > first_upper
        first_lower[infeasible_slew] = safety_lower[infeasible_slew]
        first_upper[infeasible_slew] = safety_upper[infeasible_slew]
        lower[0] = first_lower
        upper[0] = first_upper
        return lower, upper

    def _reaction_seed_bounds(
            self, counter_q, counter_dq, control_lower, control_upper):
        lower = np.copy(control_lower)
        upper = np.copy(control_upper)
        q_lower, q_upper = self._effective_counter_position_limits()
        lower = np.maximum(
            lower,
            2.0 * (
                q_lower - counter_q - self.dt * counter_dq
            ) / self.dt ** 2,
        )
        upper = np.minimum(
            upper,
            2.0 * (
                q_upper - counter_q - self.dt * counter_dq
            ) / self.dt ** 2,
        )
        if np.any(lower > upper):
            raise ValueError('reaction seed excursion bounds are infeasible')
        return lower, upper

    def _effective_counter_position_limits(self):
        lower, upper = self._physical_counter_position_limits()
        if self.q_counter_ref is not None:
            lower = np.maximum(lower, self.q_counter_ref - self.max_excursion)
            upper = np.minimum(upper, self.q_counter_ref + self.max_excursion)
        if np.any(lower >= upper):
            raise ValueError('counter position bounds are infeasible')
        return lower, upper

    def _physical_counter_position_limits(self):
        lower, upper = self._arm_position_limits()
        lower = lower[self.counter_active_local]
        upper = upper[self.counter_active_local]
        if np.any(lower >= upper):
            raise ValueError('counter physical position bounds are infeasible')
        return lower, upper

    def _validate_trajectory(self, motor_q, q_horizon, dq_horizon, knots, xs):
        q_lower, q_upper = self._physical_counter_position_limits()
        velocity = np.minimum(
            self._arm_velocity_limits()[self.counter_active_local],
            self.max_velocity,
        )
        max_metric_error = 0.0
        for index, (knot, state) in enumerate(zip(knots, xs[1:])):
            q_counter = state[:4]
            dq_counter = state[4:]
            if (
                np.any(q_counter < q_lower - 1e-8)
                or np.any(q_counter > q_upper + 1e-8)
                or np.any(np.abs(dq_counter) > velocity + 1e-8)
            ):
                return False, 'trajectory_out_of_bounds', max_metric_error
            if np.max(np.abs(q_counter - knot.q_bar)) > (
                    self.max_frozen_map_displacement):
                return False, 'trajectory_outside_trust_region', max_metric_error
            if index >= self.validation_steps:
                continue
            arm_q = np.copy(motor_q[self.arm_ids])
            arm_q[self.moving_local] = q_horizon[
                index + 1, self.moving_local
            ]
            arm_q[self.counter_active_local] = q_counter
            arm_q[self.counter_wrist_local] = self.counter_wrist_ref
            candidate = self._full_candidate(motor_q, arm_q)
            status = self._candidate_status(candidate, 'trajectory')
            if status is not None:
                return False, status, max_metric_error
            try:
                support, com, jacobian, momentum, rotation = self._model_terms(
                    candidate,
                )
                if not support.valid:
                    return False, 'trajectory_invalid_support', max_metric_error
                gyro, _ = self._torso_gyro(rotation)
                moving_dq = dq_horizon[index + 1, self.moving_local]
                exact_com = (
                    jacobian[:2, self.counter_v_indices] @ dq_counter
                    + jacobian[:2, self.moving_v_indices] @ moving_dq
                    + self.ddp.gains['com'] * (
                        com[:2] - support.center - self.com_offset_ref
                    )
                ) / self.ddp.scales['com_velocity']
                frozen_com = (
                    knot.com_counter @ dq_counter
                    + knot.com_moving @ moving_dq
                    + self.ddp.gains['com'] * (
                        knot.com_bar
                        + knot.com_counter @ (q_counter - knot.q_bar)
                        - knot.com_target
                    )
                ) / self.ddp.scales['com_velocity']
                exact_momentum = (
                    momentum[:2, self.counter_v_indices] @ dq_counter
                    + momentum[:2, self.moving_v_indices] @ moving_dq
                    - self.ddp.gains['gyro'] * gyro[:2]
                ) / self.ddp.scales['momentum']
                frozen_momentum = (
                    knot.momentum_counter @ dq_counter
                    + knot.momentum_moving @ moving_dq
                    - self.ddp.gains['gyro'] * knot.gyro_xy
                ) / self.ddp.scales['momentum']
                max_metric_error = max(
                    max_metric_error,
                    float(np.max(np.abs(exact_com - frozen_com))),
                    float(np.max(np.abs(
                        exact_momentum - frozen_momentum,
                    ))),
                )
            except Exception:
                return False, 'trajectory_model_failure', max_metric_error
        if max_metric_error > self.metric_tolerance:
            return False, 'trajectory_metric_mismatch', max_metric_error
        return True, 'valid', max_metric_error

    def _backtrack_ddp_counter(self, motor_q, arm_q, arm_dq, counter_q,
                               counter_dq, requested_q, requested_dq):
        q_delta = requested_q - counter_q
        excursion_lower, excursion_upper = (
            self._effective_counter_position_limits()
        )
        for scale in self.backtrack_scales:
            candidate_q = np.copy(arm_q)
            command_dq = np.copy(arm_dq)
            candidate_q[self.counter_active_local] = (
                counter_q + scale * q_delta
            )
            candidate_q[self.counter_wrist_local] = self.counter_wrist_ref
            command_dq[self.counter_active_local] = scale * requested_dq
            command_dq[self.counter_wrist_local] = 0.0
            candidate_counter = candidate_q[self.counter_active_local]
            inside = (
                (counter_q >= excursion_lower)
                & (counter_q <= excursion_upper)
            )
            valid_excursion = np.where(
                inside,
                (candidate_counter >= excursion_lower)
                & (candidate_counter <= excursion_upper),
                np.where(
                    counter_q > excursion_upper,
                    candidate_counter <= counter_q,
                    candidate_counter >= counter_q,
                ),
            )
            if not np.all(valid_excursion):
                continue
            status = self._candidate_status(
                self._full_candidate(motor_q, candidate_q),
                'counter_candidate',
            )
            if status is None:
                return (candidate_q, command_dq, scale), None
            if status == 'counter_candidate_check_failure':
                return None, status
        return None, 'counter_candidate_invalid'

    def _publish_ddp_hold(self, motor_q, arm_q, arm_dq, moving_tau, status,
                          started, prepared_tau=None):
        measured_arm_q = motor_q[self.arm_ids]
        arm_q[self.counter_active_local] = (
            measured_arm_q[self.counter_active_local]
        )
        arm_q[self.counter_wrist_local] = (
            self.counter_wrist_ref
            if self.counter_wrist_ref is not None
            else measured_arm_q[self.counter_wrist_local]
        )
        arm_dq[self.counter_local] = 0.0
        _, motor_dq = self._measured_motor_state()
        hold_acceleration = np.clip(
            -motor_dq[self.counter_ids] / self.dt,
            -self.max_acceleration,
            self.max_acceleration,
        )
        self.latest_applied_counter_acceleration = hold_acceleration
        self.latest_applied_counter_dq[:] = 0.0
        self._previous_acceleration = np.copy(hold_acceleration)
        self.latest_status = status
        self._publish_ddp_arm_command(
            arm_q,
            arm_dq,
            motor_q,
            moving_tau,
            prepared_tau=prepared_tau,
        )
        self.latest_total_time = time.perf_counter() - started
        if self.latest_total_time > self.timing_guard:
            self.latest_timing_overrun = True
            if status == 'inactive_hold':
                self.latest_status = 'inactive_timing_overrun'
        return arm_dq

    def _publish_inactive_sample(self, motor_q, arm_q, arm_dq, moving_tau,
                                 started):
        arm_q = np.copy(arm_q)
        arm_dq = np.copy(arm_dq)
        arm_q[self.counter_wrist_local] = self.counter_wrist_ref
        arm_dq[self.counter_wrist_local] = 0.0
        if not np.all(np.isfinite(arm_q)) or not np.all(np.isfinite(arm_dq)):
            return self._publish_ddp_hold(
                motor_q,
                arm_q,
                arm_dq,
                moving_tau,
                'inactive_invalid_sample',
                started,
            )
        self.latest_status = 'inactive_passthrough'
        self._publish_ddp_arm_command(arm_q, arm_dq, motor_q, moving_tau)
        self.latest_total_time = time.perf_counter() - started
        if self.latest_total_time > self.timing_guard:
            self.latest_timing_overrun = True
            self.latest_status = 'inactive_timing_overrun'
        return arm_dq

    def _publish_ddp_arm_command(self, arm_q, arm_dq, motor_q, moving_tau,
                                 prepared_tau=None):
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.latest_status = 'estopped'
            return False
        if prepared_tau is None:
            prepared_tau = self._prepare_arm_torque(motor_q, moving_tau)
        tau, source, gravity_valid = prepared_tau
        self.latest_torque_source = source
        if not gravity_valid:
            measured_arm_q = motor_q[self.arm_ids]
            arm_q = np.copy(arm_q)
            arm_dq = np.copy(arm_dq)
            arm_q[self.counter_local] = measured_arm_q[self.counter_local]
            arm_dq[self.counter_local] = 0.0
            self.latest_status = 'gravity_failure'
        if self.reaction_diagnostics_enabled:
            self.latest_reaction_command_timestamp = time.perf_counter()
        if not self._write_arm_command(arm_q, arm_dq, tau):
            return False
        self._record_moving_command_error(arm_q, arm_dq)
        self._record_applied_counter_dq()
        return True

    def _prepare_arm_torque(self, motor_q, moving_tau):
        try:
            gravity = np.asarray(
                self.robot_model.dynamics.get_gravity_compensation(motor_q),
                dtype=np.float64,
            )
            tau = np.copy(gravity[self.arm_ids])
            if tau.shape != (14,) or not np.all(np.isfinite(tau)):
                raise ValueError('nonfinite gravity compensation')
            source = 'gravity'
            gravity_valid = True
        except Exception:
            tau_cmd = np.asarray(
                getattr(self.low_cmd_handler, 'tau_cmd', np.zeros(27)),
                dtype=np.float64,
            )
            tau = (
                np.copy(tau_cmd[self.arm_ids])
                if tau_cmd.shape == (27,) and np.all(np.isfinite(tau_cmd))
                else np.zeros(14, dtype=np.float64)
            )
            source = 'previous'
            gravity_valid = False
        if moving_tau is not None:
            tau[self.moving_local] = moving_tau[self.moving_local]
            source = 'supplied'
        return tau, source, gravity_valid

    def _validated_horizon(self, q_horizon, dq_horizon):
        shape = (self.horizon_steps + 1, 14)
        q_horizon = np.asarray(q_horizon, dtype=np.float64)
        dq_horizon = np.asarray(dq_horizon, dtype=np.float64)
        if q_horizon.shape != shape or dq_horizon.shape != shape:
            raise ValueError(f'moving horizons must have shape {shape}')
        if (
            not np.all(np.isfinite(q_horizon[:, self.moving_local]))
            or not np.all(np.isfinite(dq_horizon[:, self.moving_local]))
        ):
            raise ValueError('moving-arm horizon must be finite')
        return np.copy(q_horizon), np.copy(dq_horizon)

    def _validated_preview(
            self, q_preview, dq_preview, q_horizon, dq_horizon):
        if q_preview is None and dq_preview is None:
            if self.preview_steps != self.horizon_steps:
                raise ValueError('configured preview horizon is missing')
            return np.copy(q_horizon), np.copy(dq_horizon)
        if q_preview is None or dq_preview is None:
            raise ValueError('preview position and velocity must be provided together')
        shape = (self.preview_steps + 1, 14)
        q_preview = np.asarray(q_preview, dtype=np.float64)
        dq_preview = np.asarray(dq_preview, dtype=np.float64)
        if q_preview.shape != shape or dq_preview.shape != shape:
            raise ValueError(f'moving preview must have shape {shape}')
        if (
            not np.all(np.isfinite(q_preview[:, self.moving_local]))
            or not np.all(np.isfinite(dq_preview[:, self.moving_local]))
        ):
            raise ValueError('moving-arm preview must be finite')
        if (
            not np.allclose(
                q_preview[:self.horizon_steps + 1, self.moving_local],
                q_horizon[:, self.moving_local],
                atol=1e-8,
            )
            or not np.allclose(
                dq_preview[:self.horizon_steps + 1, self.moving_local],
                dq_horizon[:, self.moving_local],
                atol=1e-8,
            )
        ):
            raise ValueError('moving preview must start with the OCP horizon')
        return np.copy(q_preview), np.copy(dq_preview)

    def _validate_forecast_time(self, sample_times, generated_at):
        if sample_times is None and generated_at is None:
            return
        if sample_times is None or generated_at is None:
            raise ValueError('sample_times and generated_at must be provided together')
        sample_times = np.asarray(sample_times, dtype=np.float64)
        if (
            sample_times.shape != (self.horizon_steps + 1,)
            or not np.all(np.isfinite(sample_times))
            or not np.isfinite(generated_at)
            or np.any(np.diff(sample_times) <= 0.0)
            or not np.allclose(np.diff(sample_times), self.dt, atol=1e-6)
            or not np.isclose(
                sample_times[0], generated_at, atol=1e-6, rtol=0.0,
            )
            or not np.isclose(
                sample_times[-1],
                generated_at + self.horizon_steps * self.dt,
                atol=1e-6,
                rtol=0.0,
            )
        ):
            raise ValueError('forecast timestamps are invalid')
        age = time.monotonic() - float(generated_at)
        self.latest_forecast_age = age
        if age < -1e-3 or age > self.max_forecast_age:
            raise ValueError('moving-arm forecast is stale')

    @staticmethod
    def _validated_tau(value):
        if value is None:
            return None
        value = np.asarray(value, dtype=np.float64)
        if value.shape != (14,) or not np.all(np.isfinite(value)):
            raise ValueError('moving_tau must be finite with shape (14,)')
        return np.copy(value)

    def _record_result(self, result):
        self.latest_solver_converged = result.converged
        self.latest_solver_iterations = result.iterations
        self.latest_solver_stopping_criterion = result.stopping_criterion
        self.latest_seed_cost = result.seed_cost
        self.latest_optimized_cost = result.optimized_cost
        self.latest_solve_time = result.solve_time
        self.latest_warm_started = result.warm_started

    def _record_reaction_measurement(
            self, timestamp, counter_dq, counter_h, moving_h, gyro_xy):
        sample = self.reaction_observer.update(
            timestamp, counter_dq, counter_h, moving_h, gyro_xy,
        )
        self.latest_reaction_measurement_valid = sample.valid
        self.latest_reaction_dt = sample.dt
        self.latest_measured_counter_momentum = sample.counter_h
        self.latest_measured_moving_momentum = sample.moving_h
        self.latest_measured_total_arm_momentum = sample.total_h
        self.latest_measured_counter_momentum_rate = sample.counter_hdot
        self.latest_measured_moving_momentum_rate = sample.moving_hdot
        self.latest_measured_total_arm_momentum_rate = sample.total_hdot
        self.latest_measured_counter_acceleration = (
            sample.counter_acceleration
        )
        self.latest_measured_base_angular_acceleration = (
            sample.base_angular_acceleration
        )

    def _record_response_diagnostics(self, gyro_xy):
        tilt_error = (
            self._imu_tilt() - self.tilt_reference
            if self.tilt_reference is not None else np.zeros(2)
        )
        gyro_xy = np.asarray(gyro_xy, dtype=np.float64)
        divergence = tilt_error * gyro_xy
        self.latest_response_tilt_error = tilt_error
        self.latest_response_gyro = gyro_xy
        self.latest_response_divergence = divergence
        self.latest_response_divergence_norm = float(
            np.linalg.norm(np.maximum(divergence, 0.0)),
        )

    def _record_reaction_prediction(
            self, current_counter_map, current_moving_map,
            current_counter_dq, current_moving_dq, knot, next_counter_dq):
        counter_rate, moving_rate, total_rate = (
            predict_frozen_momentum_rate(
                current_counter_map,
                current_moving_map,
                current_counter_dq,
                current_moving_dq,
                knot.momentum_counter,
                knot.momentum_moving,
                next_counter_dq,
                knot.moving_dq,
                self.dt,
            )
        )
        self.latest_predicted_counter_momentum_rate = counter_rate
        self.latest_predicted_moving_momentum_rate = moving_rate
        self.latest_predicted_total_arm_momentum_rate = total_rate
        self.latest_reaction_prediction_valid = True

    def _record_disturbance_preview(self, moving_map, preview_dq):
        moving_map = np.asarray(moving_map, dtype=np.float64)
        moving_dq = preview_dq[:, self.moving_local]
        momentum = moving_dq @ moving_map.T
        momentum_rate = np.diff(momentum, axis=0) / self.dt
        rate_norm = np.linalg.norm(momentum_rate, axis=1)
        momentum_change = momentum - momentum[0]
        change_norm = np.linalg.norm(momentum_change, axis=1)
        offsets = np.arange(1, len(momentum)) * self.dt
        discount = np.exp(-offsets / max(self.preview_steps * self.dt, self.dt))
        peak_index = int(np.argmax(rate_norm)) if len(rate_norm) else 0
        h3_count = min(self.horizon_steps + 1, len(momentum))
        h3_rate = momentum_rate[:max(0, h3_count - 1)]
        h3_change = momentum_change[:h3_count]
        h3_rate_norm = np.linalg.norm(h3_rate, axis=1)
        h3_offsets = np.arange(1, h3_count) * self.dt
        h3_discount = np.exp(
            -h3_offsets / max(self.horizon_steps * self.dt, self.dt),
        )
        h3_peak_index = int(np.argmax(h3_rate_norm)) if len(h3_rate_norm) else 0

        self.latest_preview_actual_steps = len(momentum) - 1
        self.latest_preview_actual_duration = (
            self.latest_preview_actual_steps * self.dt
        )
        self.latest_preview_moving_momentum = momentum
        self.latest_preview_moving_momentum_rate = momentum_rate
        self.latest_preview_peak_momentum_rate = (
            float(rate_norm[peak_index]) if len(rate_norm) else 0.0
        )
        self.latest_preview_peak_momentum_rate_vector = (
            np.copy(momentum_rate[peak_index])
            if len(momentum_rate) else np.zeros(2)
        )
        self.latest_preview_peak_momentum_change = float(np.max(change_norm))
        self.latest_preview_discounted_rate_exposure = float(
            np.sum(discount * rate_norm) * self.dt
        )
        self.latest_preview_time_to_peak_rate = (
            float((peak_index + 1) * self.dt) if len(rate_norm) else 0.0
        )
        self.latest_h3_peak_momentum_rate = (
            float(np.max(np.linalg.norm(h3_rate, axis=1)))
            if len(h3_rate) else 0.0
        )
        self.latest_h3_peak_momentum_change = (
            float(np.max(np.linalg.norm(h3_change, axis=1)))
            if len(h3_change) else 0.0
        )
        self.latest_h3_discounted_rate_exposure = float(
            np.sum(h3_discount * h3_rate_norm) * self.dt
        )
        self.latest_h3_time_to_peak_rate = (
            float((h3_peak_index + 1) * self.dt)
            if len(h3_rate_norm) else 0.0
        )

    def _record_reaction_seed_diagnostic(
            self, current_counter_map, current_counter_dq,
            knot, control_lower, control_upper):
        desired_rate = -self.reaction_feedforward_gain * (
            self.latest_preview_peak_momentum_rate_vector
        )
        result = reaction_seed_diagnostic(
            current_counter_map,
            knot.momentum_counter,
            current_counter_dq,
            desired_rate,
            self.reaction_effectiveness,
            control_lower,
            control_upper,
            self.dt,
        )
        self.latest_reaction_seed_valid = result.valid
        self.latest_reaction_seed_desired_rate = result.desired_measured_rate
        self.latest_reaction_seed_unbounded_acceleration = (
            result.unbounded_acceleration
        )
        self.latest_reaction_seed_clipped_acceleration = (
            result.clipped_acceleration
        )
        self.latest_reaction_seed_saturation_mask = result.saturation_mask
        self.latest_reaction_seed_achieved_rate = result.achieved_measured_rate
        self.latest_reaction_seed_residual = result.residual
        self.latest_reaction_seed_unbounded_feasible = (
            result.unbounded_feasible
        )

    def _reset_ddp_diagnostics(self):
        self.latest_axis_activation = np.zeros(2, dtype=np.float64)
        self.latest_gyro_rate = 0.0
        self.latest_moving_speed = 0.0
        self.latest_moving_momentum_risk = 0.0
        self.latest_moving_momentum_authority = 0.0
        self.latest_response_tilt_error = np.zeros(2)
        self.latest_response_gyro = np.zeros(2)
        self.latest_response_divergence = np.zeros(2)
        self.latest_response_divergence_norm = 0.0
        self.latest_reaction_measurement_valid = False
        self.latest_reaction_prediction_valid = False
        self.latest_reaction_dt = np.nan
        self.latest_measured_counter_momentum = np.zeros(2)
        self.latest_measured_moving_momentum = np.zeros(2)
        self.latest_measured_total_arm_momentum = np.zeros(2)
        self.latest_measured_counter_momentum_rate = np.zeros(2)
        self.latest_measured_moving_momentum_rate = np.zeros(2)
        self.latest_measured_total_arm_momentum_rate = np.zeros(2)
        self.latest_measured_counter_acceleration = np.zeros(4)
        self.latest_measured_base_angular_acceleration = np.zeros(2)
        self.latest_predicted_counter_momentum_rate = np.zeros(2)
        self.latest_predicted_moving_momentum_rate = np.zeros(2)
        self.latest_predicted_total_arm_momentum_rate = np.zeros(2)
        self.latest_preview_moving_momentum = np.zeros((0, 2))
        self.latest_preview_moving_momentum_rate = np.zeros((0, 2))
        self.latest_preview_peak_momentum_rate = 0.0
        self.latest_preview_peak_momentum_rate_vector = np.zeros(2)
        self.latest_preview_peak_momentum_change = 0.0
        self.latest_preview_discounted_rate_exposure = 0.0
        self.latest_preview_time_to_peak_rate = 0.0
        self.latest_preview_input_valid = False
        self.latest_preview_actual_steps = 0
        self.latest_preview_actual_duration = 0.0
        self.latest_h3_peak_momentum_rate = 0.0
        self.latest_h3_peak_momentum_change = 0.0
        self.latest_h3_discounted_rate_exposure = 0.0
        self.latest_h3_time_to_peak_rate = 0.0
        self.latest_reaction_seed_valid = False
        self.latest_reaction_seed_desired_rate = np.zeros(2)
        self.latest_reaction_seed_unbounded_acceleration = np.zeros(4)
        self.latest_reaction_seed_clipped_acceleration = np.zeros(4)
        self.latest_reaction_seed_saturation_mask = np.zeros(4, dtype=bool)
        self.latest_reaction_seed_achieved_rate = np.zeros(2)
        self.latest_reaction_seed_residual = np.zeros(2)
        self.latest_reaction_seed_unbounded_feasible = False
        self.latest_reaction_state_timestamp = np.nan
        self.latest_reaction_command_timestamp = np.nan
        self.latest_requested_counter_acceleration = np.zeros(4)
        self.latest_applied_counter_acceleration = np.zeros(4)
        self.latest_solver_converged = False
        self.latest_solver_iterations = 0
        self.latest_solver_stopping_criterion = np.nan
        self.latest_seed_cost = np.nan
        self.latest_optimized_cost = np.nan
        self.latest_solve_time = 0.0
        self.latest_total_time = 0.0
        self.latest_timing_overrun = False
        self.latest_warm_started = False
        self.latest_metric_mismatch = np.nan
        self.latest_forecast_source = 'none'
        self.latest_forecast_age = np.nan
        self.latest_torque_source = 'none'

    @staticmethod
    def _finite_or_none(value):
        return float(value) if np.isfinite(value) else None

    @classmethod
    def _parse_ddp_config(cls, cfg):
        if not isinstance(cfg, dict):
            raise ValueError('counter_ddp must be a mapping')
        horizon_steps = cls._positive_int(
            cfg.get('horizon_steps', 5), 'horizon_steps',
        )
        preview_steps = cls._positive_int(
            cfg.get('preview_steps', horizon_steps), 'preview_steps',
        )
        if preview_steps < horizon_steps:
            raise ValueError(
                'counter_ddp.preview_steps must cover horizon_steps',
            )
        max_iterations = cls._nonnegative_int(
            cfg.get('max_iterations', 2), 'max_iterations',
        )
        weights = cls._mapping(cfg, 'weights', {
            'com': 1.0,
            'momentum': 2.0,
            'posture': 0.02,
            'acceleration': 0.01,
            'velocity': 0.02,
            'limit': 10.0,
            'terminal_posture': 0.0,
            'terminal_velocity': 0.0,
        })
        gains = cls._mapping(cfg, 'gains', {'com': 2.0, 'gyro': 0.2})
        scales = cls._mapping(cfg, 'scales', {
            'com_velocity': 0.1,
            'momentum': 1.0,
            'posture': 1.0,
            'acceleration': 25.0,
            'velocity': 1.0,
        })
        for section_name, values, positive in (
            ('weights', weights, False),
            ('gains', gains, False),
            ('scales', scales, True),
        ):
            for name, value in values.items():
                values[name] = cls._ddp_scalar(
                    value, f'{section_name}.{name}', positive=positive,
                )
        activation = cfg.get('activation', {}) or {}
        if not isinstance(activation, dict):
            raise ValueError('counter_ddp.activation must be a mapping')
        reaction_diagnostics = cfg.get('reaction_diagnostics', {}) or {}
        if not isinstance(reaction_diagnostics, dict):
            raise ValueError('counter_ddp.reaction_diagnostics must be a mapping')
        gyro_rate_threshold = cls._ddp_scalar(
            activation.get('gyro_rate_threshold', 0.08),
            'activation.gyro_rate_threshold',
            positive=False,
        )
        gyro_rate_full_scale = cls._ddp_scalar(
            activation.get('gyro_rate_full_scale', 0.12),
            'activation.gyro_rate_full_scale',
            positive=True,
        )
        if gyro_rate_full_scale <= gyro_rate_threshold:
            raise ValueError(
                'counter_ddp gyro rate full scale must exceed threshold',
            )
        moving_momentum_threshold = activation.get(
            'moving_momentum_threshold'
        )
        moving_momentum_full_scale = activation.get(
            'moving_momentum_full_scale'
        )
        if moving_momentum_threshold is None:
            if moving_momentum_full_scale is not None:
                raise ValueError(
                    'counter_ddp moving momentum threshold is required',
                )
        else:
            moving_momentum_threshold = cls._ddp_scalar(
                moving_momentum_threshold,
                'activation.moving_momentum_threshold',
                positive=False,
            )
            moving_momentum_full_scale = cls._ddp_scalar(
                moving_momentum_full_scale,
                'activation.moving_momentum_full_scale',
                positive=True,
            )
            if moving_momentum_full_scale <= moving_momentum_threshold:
                raise ValueError(
                    'counter_ddp moving momentum full scale must exceed threshold',
                )
        moving_feedforward_authority = cls._unit_scalar(
            activation.get('moving_feedforward_authority', 0.0),
            'activation.moving_feedforward_authority',
        )
        moving_momentum_max_authority = cls._unit_scalar(
            activation.get('moving_momentum_max_authority', 1.0),
            'activation.moving_momentum_max_authority',
        )
        if moving_momentum_max_authority < moving_feedforward_authority:
            raise ValueError(
                'counter_ddp moving momentum authority must not reduce feedforward',
            )
        shadow = cfg.get('shadow', True)
        if not isinstance(shadow, bool):
            raise ValueError('counter_ddp.shadow must be a boolean')
        support = cfg.get('support_geometry', {}) or {}
        if not isinstance(support, dict):
            raise ValueError('counter_ddp.support_geometry must be a mapping')
        support_geometry = {
            name: cls._ddp_scalar(
                support.get(name, default),
                f'support_geometry.{name}',
                positive=True,
            )
            for name, default in (
                ('front', 0.174),
                ('rear', 0.086),
                ('half_width', 0.043),
                ('max_yaw_divergence', np.deg2rad(20.0)),
            )
        }
        return {
            'shadow': shadow,
            'horizon_steps': horizon_steps,
            'preview_steps': preview_steps,
            'max_iterations': max_iterations,
            'weights': weights,
            'gains': gains,
            'scales': scales,
            'max_velocity': cls._ddp_vector(
                cfg.get('max_velocity', [2.6, 3.2, 2.6, 1.5]),
                'max_velocity', allow_zero=False,
            ),
            'max_acceleration': cls._ddp_vector(
                cfg.get('max_acceleration', 25.0),
                'max_acceleration', allow_zero=False,
            ),
            'max_acceleration_change': cls._ddp_vector(
                cfg.get('max_acceleration_change', 5.0),
                'max_acceleration_change', allow_zero=False,
            ),
            'max_excursion': cls._ddp_vector(
                cfg.get('max_excursion', [0.35, 0.28, 0.20, 0.28]),
                'max_excursion', allow_zero=False,
            ),
            'gyro_rate_threshold': gyro_rate_threshold,
            'gyro_rate_full_scale': gyro_rate_full_scale,
            'moving_velocity_threshold': cls._ddp_scalar(
                activation.get('moving_velocity_threshold', 1e-3),
                'activation.moving_velocity_threshold',
                positive=True,
            ),
            'moving_feedforward_authority': moving_feedforward_authority,
            'moving_momentum_threshold': moving_momentum_threshold,
            'moving_momentum_full_scale': moving_momentum_full_scale,
            'moving_momentum_max_authority': moving_momentum_max_authority,
            'reaction_feedforward_gain': cls._ddp_scalar(
                reaction_diagnostics.get('feedforward_gain', 0.3),
                'reaction_diagnostics.feedforward_gain',
                positive=False,
            ),
            'reaction_effectiveness': cls._axis_vector(
                reaction_diagnostics.get('effectiveness', [0.105, 0.115]),
                'reaction_diagnostics.effectiveness',
                allow_zero=False,
            ),
            'reaction_diagnostics_enabled': cls._bool_value(
                reaction_diagnostics.get('enabled', False),
                'reaction_diagnostics.enabled',
            ),
            'stationary_gyro_feedback': cls._bool_value(
                activation.get('stationary_gyro_feedback', False),
                'activation.stationary_gyro_feedback',
            ),
            'stationary_feedback_max_authority': cls._unit_scalar(
                activation.get('stationary_feedback_max_authority', 1.0),
                'activation.stationary_feedback_max_authority',
            ),
            'max_forecast_age': cls._ddp_scalar(
                cfg.get('max_forecast_age', 0.1),
                'max_forecast_age', positive=True,
            ),
            'max_frozen_map_displacement': cls._ddp_scalar(
                cfg.get('max_frozen_map_displacement', 0.1),
                'max_frozen_map_displacement', positive=True,
            ),
            'metric_tolerance': cls._ddp_scalar(
                cfg.get('metric_tolerance', 0.25),
                'metric_tolerance', positive=True,
            ),
            'validation_steps': min(
                horizon_steps,
                cls._nonnegative_int(
                    cfg.get('validation_steps', 1), 'validation_steps',
                ),
            ),
            'timing_guard': cls._ddp_scalar(
                cfg.get('timing_guard', 0.015),
                'timing_guard', positive=True,
            ),
            'initial_regularization': cls._ddp_scalar(
                cfg.get('initial_regularization', 1e-6),
                'initial_regularization', positive=True,
            ),
            'minimum_cost_improvement': cls._ddp_scalar(
                cfg.get('minimum_cost_improvement', 0.0),
                'minimum_cost_improvement', positive=False,
            ),
            'support_geometry': support_geometry,
        }

    @staticmethod
    def _mapping(cfg, name, defaults):
        values = cfg.get(name, {}) or {}
        if not isinstance(values, dict):
            raise ValueError(f'counter_ddp.{name} must be a mapping')
        return {**defaults, **values}

    @staticmethod
    def _ddp_scalar(value, name, positive):
        try:
            value = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f'counter_ddp.{name} must be finite') from error
        if not np.isfinite(value) or value < 0.0 or (positive and value == 0.0):
            requirement = 'positive' if positive else 'nonnegative'
            raise ValueError(f'counter_ddp.{name} must be finite and {requirement}')
        return value

    @classmethod
    def _unit_scalar(cls, value, name):
        value = cls._ddp_scalar(value, name, positive=False)
        if value > 1.0:
            raise ValueError(f'counter_ddp.{name} must not exceed one')
        return value

    @staticmethod
    def _bool_value(value, name):
        if not isinstance(value, bool):
            raise ValueError(f'counter_ddp.{name} must be a boolean')
        return value

    @classmethod
    def _ddp_vector(cls, value, name, allow_zero):
        result = np.asarray(value, dtype=np.float64)
        size = 4
        if result.ndim == 0:
            result = np.full(size, float(result))
        expected = (size,)
        if (
            result.shape != expected
            or not np.all(np.isfinite(result))
            or np.any(result < 0.0)
            or (not allow_zero and np.any(result == 0.0))
        ):
            raise ValueError(
                f'counter_ddp.{name} must contain {expected[0]} '
                'finite values',
            )
        return np.copy(result)

    @staticmethod
    def _axis_vector(value, name, allow_zero):
        result = np.asarray(value, dtype=np.float64)
        if (
            result.shape != (2,)
            or not np.all(np.isfinite(result))
            or np.any(result < 0.0)
            or (not allow_zero and np.any(result == 0.0))
        ):
            raise ValueError(
                f'counter_ddp.{name} must contain two finite values',
            )
        return np.copy(result)

    @staticmethod
    def _positive_int(value, name):
        value = int(value)
        if value <= 0:
            raise ValueError(f'counter_ddp.{name} must be positive')
        return value

    @staticmethod
    def _nonnegative_int(value, name):
        value = int(value)
        if value < 0:
            raise ValueError(f'counter_ddp.{name} must be nonnegative')
        return value
