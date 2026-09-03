import time

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_ocp import (
    RATE,
    TILT,
    ResidualH2Context,
    ResidualH2OCP,
)
from h12_ros2_controller.core.controller.counter_balance.residual_response_model import (
    n5_no_crossing_confidence,
)
from h12_ros2_controller.core.controller.counter_balance.verified_response_parameters import (
    verified_h2_models,
)
from h12_ros2_controller.utility.controller_config import load_controller_config


class CounterResidualH2Controller(CounterDDPVelocityController):
    '''Run verified H2 residual MPC while publishing frozen 3C'''

    def __init__(self, *args, config=None, **kwargs):
        resolved = load_controller_config() if config is None else config
        settings = self._h2_settings(resolved.get('iteration5_h2', {}))
        super().__init__(*args, config=resolved, **kwargs)
        self.h2_models = verified_h2_models()
        self.h2_ocp = ResidualH2OCP(
            dt=self.dt,
            max_iterations=settings['max_iterations'],
            weights=settings['weights'],
        )
        self.h2_trust_velocity = settings['trust_velocity']
        self.h2_context_limit = settings['context_limit']
        self.h2_shadow = settings['shadow']
        self._h2_previous_tilt = None
        self._h2_previous_rate = None
        self._h2_previous_moving_momentum = None
        self._h2_previous_nominal_momentum = None
        self._h2_sequence = 0
        self._h2_pending_residual = np.zeros(4)
        self._h2_nominal_for_finalize = np.zeros(4)
        self._reset_h2_diagnostics()

    def control_configuration_step(
            self, moving_q_target_14, moving_dq_target_14,
            balance_scale=1.0):
        '''Run one shadow H2 and publish the frozen 3C command'''
        self._reset_h2_diagnostics()
        return super().control_configuration_step(
            moving_q_target_14,
            moving_dq_target_14,
            balance_scale=balance_scale,
        )

    def _select_requested_counter_velocity(self, context, nominal):
        started = time.perf_counter()
        self._h2_sequence += 1
        self.latest_h2_sequence = self._h2_sequence
        try:
            residual = self._run_h2(context, nominal)
        except Exception as error:
            self.latest_h2_status = 'model_failure'
            self.latest_h2_error = f'{type(error).__name__}: {error}'
            self.h2_ocp.reset()
            residual = np.zeros(4)
        finally:
            self.latest_h2_total_time = time.perf_counter() - started
        selected = (
            nominal.requested_counter_dq
            if self.h2_shadow
            else nominal.requested_counter_dq + residual
        )
        self._h2_pending_residual = (
            np.zeros(4) if self.h2_shadow else np.copy(residual)
        )
        self._h2_nominal_for_finalize = np.copy(nominal.requested_counter_dq)
        return selected

    def _finalize_counter_velocity(self, context, requested):
        command = super()._finalize_counter_velocity(context, requested)
        if self.h2_shadow:
            self._h2_pending_residual[:] = 0.0
        else:
            self._h2_pending_residual = (
                self.latest_applied_counter_dq
                - self.latest_backtrack_scale * self._h2_nominal_for_finalize
            )
            self._h2_pending_residual[2] = 0.0
        return command

    def _publish_counter_hold(self, *args, **kwargs):
        if hasattr(self, '_h2_pending_residual'):
            self._h2_pending_residual[:] = 0.0
        return super()._publish_counter_hold(*args, **kwargs)

    def _run_h2(self, context, nominal):
        state = self.robot_model.state
        counter_dq = np.asarray(
            state.get('dq', np.zeros(27)), dtype=np.float64,
        )
        if counter_dq.shape != (27,) or not np.all(np.isfinite(counter_dq)):
            raise ValueError('counter velocity observation is invalid')
        counter_dq = counter_dq[self.counter_ids]
        tilt = self._tilt_from_state(state)
        rate = np.asarray(context.gyro[:2], dtype=np.float64)
        side = 1.0 if self.moving_arm == 'left' else -1.0
        canonical_sign = np.array(
            [-1.0, 1.0] if self.moving_arm == 'left' else [1.0, 1.0],
        )
        canonical_tilt = canonical_sign * tilt
        canonical_rate = canonical_sign * rate
        momentum_map = canonical_sign[:, None] * context.momentum_counter
        moving_momentum = canonical_sign * (
            context.momentum_moving @ context.moving_dq
        )
        nominal_momentum = momentum_map @ nominal.requested_counter_dq
        self.latest_h2_current_tilt = np.copy(canonical_tilt)
        self.latest_h2_current_rate = np.copy(canonical_rate)

        if self._h2_previous_rate is None:
            nominal_tilt = np.tile(canonical_tilt, (2, 1))
            nominal_rate = np.tile(canonical_rate, (2, 1))
            confidence = np.zeros(2, dtype=bool)
            n5_valid = False
        else:
            moving_change = moving_momentum - self._h2_previous_moving_momentum
            nominal_change = (
                nominal_momentum - self._h2_previous_nominal_momentum
            )
            prediction = self.h2_models.n5.predict(
                canonical_tilt,
                self._h2_previous_tilt,
                canonical_rate,
                self._h2_previous_rate,
                self.dt,
                moving_momentum_change=np.vstack([
                    moving_change, moving_change,
                ]),
                nominal_momentum_change=np.vstack([
                    nominal_change, nominal_change,
                ]),
            )
            nominal_tilt = prediction.tilt
            nominal_rate = prediction.angular_rate
            confidence = n5_no_crossing_confidence(
                canonical_rate,
                self._h2_previous_rate,
                self.h2_models.n5_error_bound,
                horizon_steps=2,
            )
            n5_valid = prediction.valid

        u5_gain = self.h2_models.u5.gains(
            context.counter_q[None, :],
            counter_dq[None, :],
            np.array([side]),
        )[0]
        r5_context = np.concatenate([
            canonical_tilt, canonical_rate, [side],
        ])
        r5_gain = self.h2_models.r5.gains(r5_context[None, :])[0]
        context_valid = self._context_valid(
            context.counter_q, counter_dq, side, r5_context,
        )
        model_valid = bool(
            n5_valid
            and context_valid
            and np.any(confidence)
            and np.all(u5_gain[[0, 1, 3]] > 0.0)
        )
        lower = np.maximum(
            -self.h2_trust_velocity,
            nominal.lower - nominal.requested_counter_dq,
        )
        upper = np.minimum(
            self.h2_trust_velocity,
            nominal.upper - nominal.requested_counter_dq,
        )
        lower[2] = 0.0
        upper[2] = 0.0
        if not model_valid:
            lower[:] = 0.0
            upper[:] = 0.0
        h2_context = ResidualH2Context(
            nominal_tilt=nominal_tilt,
            nominal_rate=nominal_rate,
            u5_gain=u5_gain,
            r5_gain=r5_gain,
            momentum_map=momentum_map,
            confidence=confidence,
        )
        result = self.h2_ocp.solve(
            h2_context,
            lower,
            upper,
            pending=self._h2_pending_residual,
        )
        self.latest_h2_status = result.status
        self.latest_h2_accepted = result.accepted
        self.latest_h2_confidence = np.copy(confidence)
        self.latest_h2_n5_valid = bool(n5_valid)
        self.latest_h2_context_valid = bool(context_valid)
        self.latest_h2_model_valid = model_valid
        self.latest_h2_nominal_tilt = np.copy(nominal_tilt)
        self.latest_h2_nominal_rate = np.copy(nominal_rate)
        self.latest_h2_u5_gain = np.copy(u5_gain)
        self.latest_h2_r5_gain = np.copy(r5_gain)
        self.latest_h2_residual = np.copy(result.residual)
        self.latest_h2_solve_time = result.solve_time
        self.latest_h2_iterations = result.iterations
        self.latest_h2_stopping_criterion = result.stopping_criterion
        self.latest_h2_seed_cost = result.seed_cost
        self.latest_h2_optimized_cost = result.optimized_cost
        self.latest_h2_warm_started = result.warm_started
        if result.accepted and len(result.xs) == 3:
            self.latest_h2_incremental_tilt = np.asarray([
                result.xs[1, TILT], result.xs[2, TILT],
            ])
            self.latest_h2_incremental_rate = np.asarray([
                result.xs[1, RATE], result.xs[2, RATE],
            ])
            self.latest_h2_predicted_tilt = (
                nominal_tilt + self.latest_h2_incremental_tilt
            )
            self.latest_h2_predicted_rate = (
                nominal_rate + self.latest_h2_incremental_rate
            )
        self.latest_h2_decision = self._decision(
            nominal.requested_counter_dq,
            result.residual,
            model_valid,
        )
        self._h2_previous_tilt = np.copy(canonical_tilt)
        self._h2_previous_rate = np.copy(canonical_rate)
        self._h2_previous_moving_momentum = np.copy(moving_momentum)
        self._h2_previous_nominal_momentum = np.copy(nominal_momentum)
        return result.residual if model_valid and result.accepted else np.zeros(4)

    def _context_valid(self, counter_q, counter_dq, side, r5_context):
        u5 = self.h2_models.u5
        for axis in (0, 1, 3):
            values = np.array([counter_q[axis], counter_dq[axis], side])
            normalized = (
                values - u5.context_center[axis]
            ) / u5.context_scale[axis]
            if np.any(np.abs(normalized) > self.h2_context_limit):
                return False
        normalized = (
            r5_context - self.h2_models.r5.context_center
        ) / self.h2_models.r5.context_scale
        return bool(np.all(np.abs(normalized) <= self.h2_context_limit))

    @staticmethod
    def _decision(nominal, residual, valid):
        if not valid or np.linalg.norm(residual) <= 1e-8:
            return 'abstain'
        projection = float(residual @ nominal)
        if projection >= 0.0:
            return 'continue'
        combined = nominal + residual
        if np.any(
            (np.sign(combined) != np.sign(nominal))
            & (np.abs(nominal) > 1e-6)
        ):
            return 'reverse'
        return 'brake'

    @staticmethod
    def _tilt_from_state(state):
        imu = state.get('imu_state')
        quaternion = getattr(imu, 'quaternion', None)
        quaternion = np.asarray(quaternion, dtype=np.float64).reshape(-1)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError('IMU quaternion is invalid')
        w, x, y, z = quaternion
        return np.array([
            np.arctan2(
                2.0 * (w * x + y * z),
                1.0 - 2.0 * (x * x + y * y),
            ),
            np.arcsin(np.clip(
                2.0 * (w * y - z * x), -1.0, 1.0,
            )),
        ])

    def diagnostics(self):
        '''Return frozen 3C and H2 shadow diagnostics'''
        values = super().diagnostics()
        values.update({
            'h2_shadow': bool(self.h2_shadow),
            'h2_sequence': int(self.latest_h2_sequence),
            'h2_status': self.latest_h2_status,
            'h2_accepted': bool(self.latest_h2_accepted),
            'h2_error': self.latest_h2_error,
            'h2_confidence': self.latest_h2_confidence.tolist(),
            'h2_n5_valid': bool(self.latest_h2_n5_valid),
            'h2_context_valid': bool(self.latest_h2_context_valid),
            'h2_model_valid': bool(self.latest_h2_model_valid),
            'h2_current_tilt': self.latest_h2_current_tilt.tolist(),
            'h2_current_rate': self.latest_h2_current_rate.tolist(),
            'h2_nominal_tilt': self.latest_h2_nominal_tilt.tolist(),
            'h2_nominal_rate': self.latest_h2_nominal_rate.tolist(),
            'h2_incremental_tilt': (
                self.latest_h2_incremental_tilt.tolist()
            ),
            'h2_incremental_rate': (
                self.latest_h2_incremental_rate.tolist()
            ),
            'h2_predicted_tilt': self.latest_h2_predicted_tilt.tolist(),
            'h2_predicted_rate': self.latest_h2_predicted_rate.tolist(),
            'h2_u5_gain': self.latest_h2_u5_gain.tolist(),
            'h2_r5_gain': self.latest_h2_r5_gain.tolist(),
            'h2_residual': self.latest_h2_residual.tolist(),
            'h2_pending_residual': self._h2_pending_residual.tolist(),
            'h2_decision': self.latest_h2_decision,
            'h2_solve_time': float(self.latest_h2_solve_time),
            'h2_total_time': float(self.latest_h2_total_time),
            'h2_iterations': int(self.latest_h2_iterations),
            'h2_stopping_criterion': float(
                self.latest_h2_stopping_criterion
            ),
            'h2_seed_cost': float(self.latest_h2_seed_cost),
            'h2_optimized_cost': float(self.latest_h2_optimized_cost),
            'h2_warm_started': bool(self.latest_h2_warm_started),
        })
        return values

    def _reset_h2_diagnostics(self):
        self.latest_h2_sequence = self._h2_sequence
        self.latest_h2_status = 'not_run'
        self.latest_h2_accepted = False
        self.latest_h2_error = None
        self.latest_h2_confidence = np.zeros(2, dtype=bool)
        self.latest_h2_n5_valid = False
        self.latest_h2_context_valid = False
        self.latest_h2_model_valid = False
        self.latest_h2_current_tilt = np.zeros(2)
        self.latest_h2_current_rate = np.zeros(2)
        self.latest_h2_nominal_tilt = np.zeros((2, 2))
        self.latest_h2_nominal_rate = np.zeros((2, 2))
        self.latest_h2_incremental_tilt = np.zeros((2, 2))
        self.latest_h2_incremental_rate = np.zeros((2, 2))
        self.latest_h2_predicted_tilt = np.zeros((2, 2))
        self.latest_h2_predicted_rate = np.zeros((2, 2))
        self.latest_h2_u5_gain = np.zeros(4)
        self.latest_h2_r5_gain = np.zeros((2, 2))
        self.latest_h2_residual = np.zeros(4)
        self.latest_h2_decision = 'abstain'
        self.latest_h2_solve_time = 0.0
        self.latest_h2_total_time = 0.0
        self.latest_h2_iterations = 0
        self.latest_h2_stopping_criterion = np.nan
        self.latest_h2_seed_cost = np.nan
        self.latest_h2_optimized_cost = np.nan
        self.latest_h2_warm_started = False

    @staticmethod
    def _h2_settings(config):
        if not isinstance(config, dict):
            raise ValueError('iteration5_h2 must be a mapping')
        shadow = config.get('shadow', True)
        if not isinstance(shadow, bool):
            raise ValueError('iteration5_h2.shadow must be boolean')
        max_iterations = config.get('max_iterations', 1)
        if not isinstance(max_iterations, int) or max_iterations < 0:
            raise ValueError('iteration5_h2.max_iterations is invalid')
        trust = np.asarray(
            config.get('trust_velocity', [0.1, 0.1, 0.0, 0.1]),
            dtype=np.float64,
        )
        if (
            trust.shape != (4,)
            or not np.all(np.isfinite(trust))
            or np.any(trust < 0.0)
            or trust[2] != 0.0
        ):
            raise ValueError('iteration5_h2.trust_velocity is invalid')
        context_limit = float(config.get('context_limit', 3.0))
        if not np.isfinite(context_limit) or context_limit <= 0.0:
            raise ValueError('iteration5_h2.context_limit must be positive')
        weights = {
            'action': 1.0,
            'change': 0.5,
            'tilt': 2.0,
            'rate': 1.0,
            'divergence': 2.0,
            'reserve': 0.1,
            **config.get('weights', {}),
        }
        if not np.all(np.isfinite(list(weights.values()))) or any(
            value < 0.0 for value in weights.values()
        ):
            raise ValueError('iteration5_h2.weights are invalid')
        return {
            'shadow': shadow,
            'max_iterations': max_iterations,
            'trust_velocity': trust,
            'context_limit': context_limit,
            'weights': weights,
        }
