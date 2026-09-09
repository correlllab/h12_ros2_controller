import time
from copy import deepcopy
from dataclasses import asdict

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.core.controller.counter_balance.scipy_nominal_planner import (
    ScipyNominalConfig,
    solve_scipy_nominal,
)


class ScipyNominalMixin:
    '''Share a nonpublishing SciPy nominal solve and one diagnostic commit'''

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.latest_nominal_fallback_used = False
        self.latest_nominal_primary_result = None
        self._nominal_observation = None
        self._latest_nominal_plan = None
        self._latest_nominal_context = None
        self._nominal_prepared_context = None

    def _create_velocity_ocp(self):
        return None

    def control_configuration_step(
            self, moving_q_target_14, moving_dq_target_14,
            balance_scale=1.0):
        started = time.perf_counter()
        self.latest_nominal_fallback_used = False
        self.latest_nominal_primary_result = None
        self._nominal_observation = None
        self._latest_nominal_plan = None
        self._latest_nominal_context = None
        self._nominal_prepared_context = None
        try:
            return super().control_configuration_step(
                moving_q_target_14, moving_dq_target_14,
                balance_scale=balance_scale,
            )
        finally:
            self._nominal_observation = None
            self.latest_velocity_total_time = time.perf_counter() - started

    def _capture_control_observation(self):
        state = self.robot_model.state
        self._nominal_observation = deepcopy({
            key: state[key] for key in (
                'q', 'dq', 'imu_state', 'sequence', 'time_stamp',
                'arrival_monotonic', 'tick', 'unwrapped_tick', 'tick_valid',
                'received',
            ) if key in state
        })

    def _control_observation_state(self):
        snapshot = getattr(self, '_nominal_observation', None)
        if snapshot is None:
            return super()._control_observation_state()
        return snapshot

    def _isolated_velocity_solve(
            self, com_counter, momentum_counter, com_rhs, momentum_rhs,
            posture_target, lower, upper, balance_scale):
        return solve_scipy_nominal(
            com_counter, momentum_counter, com_rhs, momentum_rhs,
            posture_target, lower, upper, balance_scale,
            ScipyNominalConfig(
                com_weight=self.com_weight,
                momentum_weight=self.momentum_weight,
                posture_weight=self.posture_weight,
                damping=self.damping,
                com_velocity_scale=self.com_velocity_scale,
                momentum_scale=self.momentum_scale,
                posture_velocity_scale=self.posture_velocity_scale,
            ),
        )

    def _commit_nominal_plan(self, nominal):
        super()._commit_nominal_plan(nominal)
        self._latest_nominal_plan = nominal
        result = nominal.solve_diagnostics
        self.latest_nominal_fallback_used = bool(result.retry_used)
        self.latest_nominal_primary_result = (
            result.attempts[0] if result.attempts else None
        )

    def _select_requested_counter_velocity(self, context, nominal):
        self._latest_nominal_context = context
        # freeze before the finalizer can mutate context arrays or publisher state
        def serializable(value):
            if hasattr(value, 'tolist'):
                return value.tolist()
            if isinstance(value, dict):
                return {key: serializable(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [serializable(item) for item in value]
            return value

        observation = deepcopy(self._nominal_observation or {})
        imu = observation.pop('imu_state', None)
        if imu is not None:
            observation['imu_state'] = {
                name: getattr(imu, name) for name in (
                    'quaternion', 'gyroscope', 'accelerometer', 'rpy',
                ) if hasattr(imu, name)
            }
        self._nominal_prepared_context = serializable({
            'origin': 'captured_pre_finalization',
            'context': asdict(context),
            'observation': observation,
            'references': {
                name: deepcopy(getattr(self, name)) for name in (
                    'q_counter_ref', 'counter_wrist_ref', 'com_offset_ref',
                    'tilt_reference', '_reference_captured',
                )
            },
            'publication': {
                'dt': self.dt,
                'moving_arm': self.moving_arm,
                'backtrack_scales': self.backtrack_scales,
                'limits': deepcopy(self.config.get('limits', {})),
                'dq_lim': self.dq_lim,
                **{name: deepcopy(getattr(self.low_cmd_handler, name, None))
                   for name in ('q_cmd', 'dq_cmd', 'tau_cmd', '_estopped')},
            },
        })
        return super()._select_requested_counter_velocity(context, nominal)

    def diagnostics(self):
        values = super().diagnostics()
        result = self.latest_velocity_ocp_result
        values.update({
            'nominal_backend': result.backend if result is not None else None,
            'nominal_retry_used': bool(result is not None and result.retry_used),
            'nominal_fallback_used': bool(self.latest_nominal_fallback_used),
            'nominal_primary_accepted': bool(
                result is not None and result.primary_accepted
            ),
            'nominal_status': result.status if result is not None else 'not_run',
        })
        if self._nominal_prepared_context is not None:
            values['nominal_prepared_context'] = deepcopy(
                self._nominal_prepared_context,
            )
        nominal = self._latest_nominal_plan
        if nominal is not None and nominal.objective_matrix is not None:
            # retain the exact solved problem for offline backend comparisons
            values.update({
                'nominal_problem_matrix': nominal.objective_matrix.tolist(),
                'nominal_problem_target': nominal.objective_target.tolist(),
                'nominal_problem_lower': nominal.lower.tolist(),
                'nominal_problem_upper': nominal.upper.tolist(),
                'nominal_problem_requested': nominal.requested_counter_dq.tolist(),
                'nominal_problem_balance_scale': float(self.latest_balance_scale),
                'nominal_problem_counter_q': [0.0] * 4,
                'nominal_problem_counter_q_origin': 'solver_dummy_state_not_physical',
            })
            context = self._latest_nominal_context
            if context is not None:
                terms = {
                    'com_counter': context.com_counter.tolist(),
                    'momentum_counter': context.momentum_counter.tolist(),
                    'com_rhs': nominal.com_rhs.tolist(),
                    'momentum_rhs': nominal.momentum_rhs.tolist(),
                    'posture_target': nominal.posture_target.tolist(),
                    'balance_scale': float(context.balance_scale),
                }
                for name in (
                    'com_weight', 'momentum_weight', 'posture_weight', 'damping',
                    'com_velocity_scale', 'momentum_scale', 'posture_velocity_scale',
                ):
                    terms[name] = float(getattr(self, name))
                values['nominal_problem_terms'] = terms
        return values


class CounterDDPVelocityRobustController(
        ScipyNominalMixin, CounterDDPVelocityController):
    '''Run the shared SciPy nominal controller without H2 models or OCP'''
