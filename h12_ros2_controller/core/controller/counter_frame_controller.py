import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_goto_controller import (
    CounterGotoController,
    hold,
)


class CounterFrameController(CounterGotoController):
    '''Use the robust 3C counter core for single-arm frame action goals'''

    def __init__(self, *args, **kwargs):
        # staged initialization has no moving-arm ownership and commands both arms
        super().__init__(*args, plain_goto=True, **kwargs)

    def prepare_frame_goal(self, frame_names):
        if len(frame_names) != 1:
            raise ValueError('Counter mode requires exactly one moving-arm frame goal')
        moving_arm = self._arm_for_frame(frame_names[0])
        if moving_arm != self.moving_arm:
            self._set_arm_ownership(moving_arm)
        self.plain_goto = True
        self.balanced_live = True
        return f'balanced moving={self.moving_arm}, counter={self.counter_arm}'

    def prepare_named_goal(self):
        self.plain_goto = True
        self.balanced_live = False
        self.clear_frame_tasks()
        if self.moving_arm is not None:
            # the next balanced goal captures references from the measured posture
            self._set_arm_ownership(self.moving_arm)
        return 'plain named configuration (both arms)'

    def execute_path(self, path, validate=True, cancel_check=None):
        if not self.balanced_live:
            return super().execute_path(path, validate=validate, cancel_check=cancel_check)
        self._planned_previous = np.asarray(path[0], dtype=float).copy()
        self.planned_balance = True
        try:
            # retain the frame controller's provenance, timing and cancel guards
            return super().execute_path(path, validate=validate, cancel_check=cancel_check)
        finally:
            self.planned_balance = False
            hold(self)

    def _publish_position_command(self, q, dq, tau):
        if self.planned_balance:
            reduced = np.asarray(q)[self.robot_model.reduced_mask]
            velocity = (reduced - self._planned_previous) / self.dt
            self.control_configuration_step(reduced, velocity, balance_scale=1.0)
            self._planned_previous = reduced.copy()
            return
        return super()._publish_position_command(q, dq, tau)

    def finish_goal(self):
        # retain the counter pose and references between segmented frame goals
        hold(self)
