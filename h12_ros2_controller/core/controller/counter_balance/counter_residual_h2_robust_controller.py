from h12_ros2_controller.core.controller.counter_balance.counter_balance_controller import (
    CounterBalanceController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_controller import (
    CounterResidualH2Controller,
)
from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    Frozen3CVelocitySolve,
)


class CounterResidualH2RobustController(CounterResidualH2Controller):
    '''Use SciPy nominal fallback only after rejected Crocoddyl output'''

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.latest_nominal_fallback_used = False
        self.latest_nominal_primary_result = None

    def control_configuration_step(
            self, moving_q_target_14, moving_dq_target_14,
            balance_scale=1.0):
        '''Run robust H2 while preserving accepted frozen-H2 behavior'''
        self.latest_nominal_fallback_used = False
        self.latest_nominal_primary_result = None
        return super().control_configuration_step(
            moving_q_target_14,
            moving_dq_target_14,
            balance_scale=balance_scale,
        )

    def _isolated_velocity_solve(
            self, com_counter, momentum_counter, com_rhs, momentum_rhs,
            posture_target, lower, upper, balance_scale):
        primary = CounterDDPVelocityController._isolated_velocity_solve(
            self,
            com_counter,
            momentum_counter,
            com_rhs,
            momentum_rhs,
            posture_target,
            lower,
            upper,
            balance_scale,
        )
        self.latest_nominal_primary_result = primary.diagnostics
        if primary.accepted:
            return primary
        requested = CounterBalanceController._solve_bounded_velocity(
            self,
            com_counter,
            momentum_counter,
            com_rhs,
            momentum_rhs,
            posture_target,
            lower,
            upper,
            balance_scale=balance_scale,
        )
        self.latest_nominal_fallback_used = True
        return Frozen3CVelocitySolve(
            requested_counter_dq=requested,
            accepted=True,
            diagnostics=primary.diagnostics,
            objective_matrix=primary.objective_matrix,
            objective_target=primary.objective_target,
        )

    def diagnostics(self):
        '''Return H2 and nominal-fallback diagnostics'''
        values = super().diagnostics()
        values.update({
            'nominal_fallback_used': bool(self.latest_nominal_fallback_used),
            'nominal_primary_accepted': bool(
                self.latest_nominal_primary_result is not None
                and self.latest_nominal_primary_result.accepted
            ),
        })
        return values
