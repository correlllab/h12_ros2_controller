import numpy as np

from h12_ros2_controller.core.controller.counter_balance.diagnostic_controller import (
    CounterBalanceDiagnosticController,
)
from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    Frozen3CPlannerConfig, plan_frozen_3c_velocity,
)
from h12_ros2_controller.core.controller.counter_balance.scipy_nominal_planner import (
    ScipyNominalConfig, solve_scipy_nominal,
)


class SimCrossAxisController(CounterBalanceDiagnosticController):
    '''Evaluate the 7d perpendicular CoM penalty in an isolated simulation route'''

    VARIANTS = ('c0', 'sim_cross_axis')
    DIRECTION_FLOOR_M_S = 0.001

    def _plan_counter_velocity(self, inputs):
        if self.diagnostic_variant == 'c0':
            return super()._plan_counter_velocity(inputs)

        # both maps use the C0 fixed-base model xy plane; no world or torso rotation
        moving = inputs.com_moving @ inputs.moving_dq
        speed = np.linalg.norm(moving)
        if not np.isfinite(speed):
            raise ValueError('Invalid planned moving-arm CoM velocity')
        direction = moving / max(self.DIRECTION_FLOOR_M_S, speed)
        perpendicular = np.array([-direction[1], direction[0]])
        weight = inputs.balance_scale * self.com_weight
        row = None
        if weight > 0 and np.any(perpendicular):
            # use the same operation order and single fade as the existing C0 CoM block
            com_rows = np.sqrt(weight) * (inputs.com_counter / self.com_velocity_scale)
            projected = perpendicular @ com_rows
            if np.any(projected):
                row = projected
        self._diagnostic_record['cross_axis_row'] = {
            'active': row is not None,
            'frame': 'fixed_base_model_xy',
            'moving_reference_com_xy_m_s': moving.tolist(),
            'perpendicular_xy': perpendicular.tolist(),
            'direction_floor_m_s': self.DIRECTION_FLOOR_M_S,
            'weighted_row': None if row is None else row.tolist(),
            'target': 0.0,
        }
        if row is None:
            # omit a zero row entirely so that the original solve stays bitwise identical
            return super()._plan_counter_velocity(inputs)

        config = ScipyNominalConfig(**{
            name: getattr(self, name) for name in ScipyNominalConfig.__dataclass_fields__
        })
        return plan_frozen_3c_velocity(
            inputs, Frozen3CPlannerConfig(self.com_gain, self.gyro_gain, self.posture_gain),
            lambda *args: solve_scipy_nominal(*args, config, weighted_row=(row, 0.0)),
        )

    def diagnostics(self):
        values = super().diagnostics()
        record = values['counter_balance_diagnostic']
        if record.get('variant') == 'sim_cross_axis':
            # identify the finalized publication, not an attempted solve or DDS receipt
            record['actuation'] = (
                'sim_cross_axis' if 'cross_axis_row' in record and values['published']
                and values['status'] in ('solved', 'collision_backtracked') else 'not_delivered'
            )
        return values
