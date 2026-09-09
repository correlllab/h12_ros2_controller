import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller import (
    ScipyNominalMixin,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_controller import (
    CounterResidualH2Controller,
)


class CounterResidualH2RobustController(
        ScipyNominalMixin, CounterResidualH2Controller):
    '''Apply verified H2 residuals to the shared SciPy nominal plan'''

    def _run_h2(self, context, nominal):
        residual = np.asarray(super()._run_h2(context, nominal), dtype=np.float64)
        lower = np.maximum(
            -self.h2_trust_velocity,
            nominal.lower - nominal.requested_counter_dq,
        )
        upper = np.minimum(
            self.h2_trust_velocity,
            nominal.upper - nominal.requested_counter_dq,
        )
        if not self.latest_h2_accepted:
            raise ValueError('H2 residual is rejected or invalid')
        for value in (self.latest_h2_residual, residual):
            value = np.asarray(value, dtype=np.float64)
            if (
                value.shape != (4,)
                or not np.all(np.isfinite(value))
                or np.any(value < lower)
                or np.any(value > upper)
                or value[2] != 0.0
            ):
                raise ValueError('H2 residual is rejected or invalid')
        return residual

    def _reset_h2_solver(self):
        self.latest_h2_accepted = False
        self.latest_h2_residual = np.zeros(4)
        self.latest_h2_decision = 'abstain'
        try:
            super()._reset_h2_solver()
        except Exception as error:
            self.latest_h2_error += f'; reset {type(error).__name__}: {error}'
