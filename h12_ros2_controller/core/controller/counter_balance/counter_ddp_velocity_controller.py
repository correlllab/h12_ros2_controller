import time

from h12_ros2_controller.core.controller.counter_balance.counter_balance_controller import (
    CounterBalanceController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_velocity_ocp import (
    CounterVelocityOCP,
)
from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    Frozen3CVelocitySolve,
)
from h12_ros2_controller.core.controller.counter_balance.objective import (
    CounterVelocityBoundsError,
    bounded_velocity_problem,
)


class CounterDDPVelocityController(CounterBalanceController):
    '''Iteration 2 behavior with a one-step Crocoddyl velocity solve'''

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.velocity_ocp = CounterVelocityOCP(dt=self.dt)
        self.latest_velocity_ocp_result = None
        self.latest_velocity_total_time = 0.0

    def control_configuration_step(
            self, moving_q_target_14, moving_dq_target_14,
            balance_scale=1.0):
        '''Apply one direct parity command and record total controller time'''
        started = time.perf_counter()
        self.latest_velocity_ocp_result = None
        try:
            return super().control_configuration_step(
                moving_q_target_14,
                moving_dq_target_14,
                balance_scale=balance_scale,
            )
        finally:
            self.latest_velocity_total_time = time.perf_counter() - started

    def _solve_bounded_velocity(
            self, com_counter, momentum_counter, com_rhs, momentum_rhs,
            posture_target, lower, upper, balance_scale=1.0):
        solved = self._isolated_velocity_solve(
            com_counter,
            momentum_counter,
            com_rhs,
            momentum_rhs,
            posture_target,
            lower,
            upper,
            balance_scale,
        )
        self.latest_velocity_ocp_result = solved.diagnostics
        if not solved.accepted:
            raise RuntimeError('Crocoddyl velocity solve returned invalid output')
        return solved.requested_counter_dq

    def _isolated_velocity_solve(
            self, com_counter, momentum_counter, com_rhs, momentum_rhs,
            posture_target, lower, upper, balance_scale):
        if any(lower[index] > upper[index] for index in range(4)):
            raise CounterVelocityBoundsError(
                'Counter velocity bounds are empty',
            )
        matrix, target = bounded_velocity_problem(
            com_counter,
            momentum_counter,
            com_rhs,
            momentum_rhs,
            posture_target,
            balance_scale,
            self.com_weight,
            self.momentum_weight,
            self.posture_weight,
            self.damping,
            self.com_velocity_scale,
            self.momentum_scale,
            self.posture_velocity_scale,
        )
        result = self.velocity_ocp.solve(
            [0.0, 0.0, 0.0, 0.0],
            matrix,
            target,
            lower,
            upper,
        )
        return Frozen3CVelocitySolve(
            requested_counter_dq=result.velocity,
            accepted=result.accepted,
            diagnostics=result,
            objective_matrix=matrix,
            objective_target=target,
        )

    def _commit_nominal_plan(self, nominal):
        self.latest_velocity_ocp_result = nominal.solve_diagnostics

    def diagnostics(self):
        '''Return reactive and Crocoddyl parity diagnostics'''
        values = super().diagnostics()
        result = self.latest_velocity_ocp_result
        values.update({
            'velocity_ocp_available': result is not None,
            'velocity_ocp_converged': bool(
                result.converged if result is not None else False
            ),
            'velocity_ocp_cost': (
                float(result.cost) if result is not None else None
            ),
            'velocity_ocp_solve_time': (
                float(result.solve_time) if result is not None else 0.0
            ),
            'velocity_ocp_iterations': (
                int(result.iterations) if result is not None else 0
            ),
            'velocity_ocp_stopping_criterion': (
                float(result.stopping_criterion)
                if result is not None else None
            ),
            'velocity_ocp_kkt_violation': (
                float(result.kkt_violation) if result is not None else None
            ),
            'velocity_ocp_regularization': (
                float(result.regularization) if result is not None else None
            ),
            'velocity_ocp_boxqp_polished': bool(
                result.boxqp_polished if result is not None else False
            ),
            'velocity_total_time': float(getattr(
                self, 'latest_velocity_total_time', 0.0,
            )),
        })
        return values
