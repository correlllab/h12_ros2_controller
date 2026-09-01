from h12_ros2_controller.core.controller.counter_balance.counter_balance_controller import (
    CounterBalanceController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_velocity_ocp import (
    CounterVelocityOCP,
)
from h12_ros2_controller.core.controller.counter_balance.objective import (
    bounded_velocity_problem,
)


class CounterDDPVelocityController(CounterBalanceController):
    '''Iteration 2 behavior with a one-step Crocoddyl velocity solve'''

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.velocity_ocp = CounterVelocityOCP(dt=self.dt)
        self.latest_velocity_ocp_result = None

    def _solve_bounded_velocity(
            self, com_counter, momentum_counter, com_rhs, momentum_rhs,
            posture_target, lower, upper, balance_scale=1.0):
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
        self.latest_velocity_ocp_result = result
        if not result.accepted:
            raise RuntimeError('Crocoddyl velocity solve returned invalid output')
        return result.velocity

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
        })
        return values
