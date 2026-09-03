import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_controller import (
    CounterResidualH2Controller,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_ocp import (
    RATE,
    TILT,
    ResidualH2Context,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h3_ocp import (
    ResidualH3OCP,
)
from h12_ros2_controller.core.controller.counter_balance.residual_response_model import (
    n5_no_crossing_confidence,
)
from h12_ros2_controller.core.controller.counter_balance.verified_response_parameters import (
    verified_h3_models,
)
from h12_ros2_controller.utility.controller_config import load_controller_config


class CounterResidualH3Controller(CounterResidualH2Controller):
    '''Run verified 60 ms H3 residual MPC through the H2 controller contract'''

    def __init__(self, *args, config=None, **kwargs):
        resolved = load_controller_config() if config is None else config
        settings = self._h2_settings(resolved.get('iteration5b_h3', {}))
        CounterDDPVelocityController.__init__(
            self, *args, config=resolved, **kwargs,
        )
        self.h2_models = verified_h3_models()
        self.h2_ocp = ResidualH3OCP(
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
            nominal_tilt = np.tile(canonical_tilt, (3, 1))
            nominal_rate = np.tile(canonical_rate, (3, 1))
            confidence = np.zeros(2, dtype=bool)
            n5_valid = False
        else:
            moving_change = moving_momentum - self._h2_previous_moving_momentum
            nominal_change = (
                nominal_momentum - self._h2_previous_nominal_momentum
            )
            nominal_tilt, nominal_rate = self._predict_n5_h3(
                canonical_tilt,
                self._h2_previous_tilt,
                canonical_rate,
                self._h2_previous_rate,
                moving_change,
                nominal_change,
            )
            confidence = n5_no_crossing_confidence(
                canonical_rate,
                self._h2_previous_rate,
                self.h2_models.n5_error_bound,
                horizon_steps=3,
            )
            n5_valid = self._h3_n5_valid(
                canonical_tilt,
                self._h2_previous_tilt,
                canonical_rate,
                self._h2_previous_rate,
            )

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
        h3_context = ResidualH2Context(
            nominal_tilt=nominal_tilt[1:],
            nominal_rate=nominal_rate[1:],
            u5_gain=u5_gain,
            r5_gain=r5_gain,
            momentum_map=momentum_map,
            confidence=confidence,
        )
        result = self.h2_ocp.solve(
            h3_context,
            self.h2_models.u5_carryover,
            lower,
            upper,
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
        self.latest_h3_controls = (
            np.copy(result.us) if result.accepted else np.zeros((3, 4))
        )
        if result.accepted and len(result.xs) == 4:
            self.latest_h2_incremental_tilt = np.asarray([
                result.xs[index, TILT] for index in range(1, 4)
            ])
            self.latest_h2_incremental_rate = np.asarray([
                result.xs[index, RATE] for index in range(1, 4)
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

    def _predict_n5_h3(
            self, tilt, previous_tilt, rate, previous_rate,
            moving_change, nominal_change):
        model = self.h2_models.n5
        rates = np.empty((3, 2))
        tilts = np.empty((3, 2))
        rate_delta = rate - previous_rate
        current_rate = np.copy(rate)
        current_tilt = np.copy(tilt)
        for step in range(3):
            next_rate = (
                current_rate
                + model.rate_trend_gain * rate_delta
                + model.moving_momentum_gain @ moving_change
                + model.nominal_momentum_gain @ nominal_change
            )
            next_tilt = current_tilt + 0.5 * self.dt * (
                current_rate + next_rate
            )
            rates[step] = next_rate
            tilts[step] = next_tilt
            rate_delta = next_rate - current_rate
            current_rate = next_rate
            current_tilt = next_tilt
        return tilts, rates

    def _h3_n5_valid(self, tilt, previous_tilt, rate, previous_rate):
        validity = self.h2_models.n5.validity
        integration_error = np.abs(
            (tilt - previous_tilt) - 0.5 * self.dt * (rate + previous_rate)
        )
        return bool(
            validity.min_dt <= self.dt <= validity.max_dt
            and np.all(np.abs(tilt) <= validity.max_abs_tilt)
            and np.all(np.abs(rate) <= validity.max_abs_rate)
            and np.all(np.abs(tilt - previous_tilt) <= validity.max_tilt_step)
            and np.all(np.abs(rate - previous_rate) <= validity.max_rate_step)
            and np.all(integration_error <= validity.max_integration_error)
        )

    def diagnostics(self):
        '''Return 3C and H3-specific shadow/active diagnostics'''
        values = super().diagnostics()
        for key in list(values):
            if key.startswith('h2_'):
                values['h3_' + key[3:]] = values.pop(key)
        values['h3_horizon_steps'] = 3
        values['h3_u5_carryover'] = self.h2_models.u5_carryover.tolist()
        values['h3_controls'] = self.latest_h3_controls.tolist()
        return values

    def _reset_h2_diagnostics(self):
        super()._reset_h2_diagnostics()
        self.latest_h3_controls = np.zeros((3, 4))
