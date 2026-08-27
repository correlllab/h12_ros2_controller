import time

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_balance_controller import (
    CounterBalanceController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_ocp import (
    CounterDDPOCP,
    FrozenBalanceKnot,
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
        self.max_acceleration = settings['max_acceleration']
        self.max_acceleration_change = settings['max_acceleration_change']
        self.max_velocity = settings['max_velocity']
        self.max_excursion = settings['max_excursion']
        self.gyro_rate_threshold = settings['gyro_rate_threshold']
        self.gyro_rate_full_scale = settings['gyro_rate_full_scale']
        self.moving_velocity_threshold = settings[
            'moving_velocity_threshold'
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
        self._previous_acceleration = np.zeros(4, dtype=np.float64)
        self._reset_ddp_diagnostics()

    def _set_arm_ownership(self, moving_arm):
        super()._set_arm_ownership(moving_arm)
        if hasattr(self, 'ddp'):
            self.ddp.reset()
        if hasattr(self, '_previous_acceleration'):
            self._previous_acceleration[:] = 0.0

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
                             forecast_source='explicit'):
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
        measured_arm_q = np.copy(motor_q[self.arm_ids])
        arm_q, arm_dq, input_valid = self._moving_sample(
            q_horizon[0], dq_horizon[0], measured_arm_q,
        )
        if not input_valid:
            raise ValueError('moving-arm command must be finite')

        counter_q = np.copy(measured_arm_q[self.counter_active_local])
        counter_dq = np.copy(motor_dq[self.counter_ids])
        try:
            support, com, _, _, _ = self._model_terms(motor_q)
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
            knots, activation = self._build_knots(
                motor_q, q_horizon, dq_horizon, authority_scale,
            )
            lower, upper = self._control_bounds(counter_q, counter_dq)
        except Exception:
            return self._publish_ddp_hold(
                motor_q, arm_q, arm_dq, tau_target,
                'problem_build_failure', started,
            )
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
        return self.control_horizon_step(
            q_horizon,
            dq_horizon,
            authority_scale=balance_scale,
            forecast_source='constant_velocity',
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
        self.control_horizon_step(
            q_horizon,
            dq_horizon,
            moving_tau=moving_tau,
            authority_scale=self._frame_balance_scale,
            forecast_source='inherited_constant_velocity',
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

    def _build_knots(self, motor_q, q_horizon, dq_horizon, authority_scale):
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
        gyro_rate = float(np.linalg.norm(raw[0][7]))
        moving_speed = float(np.max(np.linalg.norm(
            dq_horizon[:, self.moving_local], axis=1,
        )))
        activation_scale = np.clip(
            (gyro_rate - self.gyro_rate_threshold)
            / (self.gyro_rate_full_scale - self.gyro_rate_threshold),
            0.0,
            1.0,
        )
        if moving_speed <= self.moving_velocity_threshold:
            activation_scale = 0.0
        activation_scale *= authority_scale
        activation = np.full(2, activation_scale, dtype=np.float64)
        self.latest_gyro_rate = gyro_rate
        self.latest_moving_speed = moving_speed
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

    def _reset_ddp_diagnostics(self):
        self.latest_axis_activation = np.zeros(2, dtype=np.float64)
        self.latest_gyro_rate = 0.0
        self.latest_moving_speed = 0.0
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
