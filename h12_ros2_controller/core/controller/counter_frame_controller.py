import numpy as np

from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS
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

    def capture_reference(self):
        '''Capture settled counter references without computing or publishing control'''
        self.check_state_fresh()
        self.update_robot_model()
        motor_q, _ = self._measured_motor_state()
        support, com, _, _, _ = self._model_terms(motor_q)
        self._capture_reference(motor_q[self.arm_ids], support, com)
        if not self._reference_captured:
            raise RuntimeError('Comparison counter reference capture failed')

    def execute_path(self, path, validate=True, cancel_check=None, validation=None):
        if getattr(self, '_comparison_mode', None) is not None:
            self._planned_previous = np.asarray(path[0], dtype=float).copy()
            self._comparison_playback = True
            try:
                return UpperController.execute_path(
                    self, path, validate=validate, cancel_check=cancel_check,
                    validation=validation,
                )
            finally:
                self._comparison_playback = False
        if validation is not None:
            raise ValueError('Saved reference validation requires comparison mode')
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
        mode = getattr(self, '_comparison_mode', None)
        if mode is not None:
            nominal = tuple(np.asarray(value, dtype=float).copy() for value in (q, dq, tau))
            # nominal motor torque includes the same existing clip as the Upper writer
            nominal[2][:] = self.low_cmd_handler._clip_tau_cmd(nominal[2])
            if mode == 'counter' and self._comparison_playback:
                reduced = nominal[0][self.robot_model.reduced_mask]
                velocity = (reduced - self._planned_previous) / self.dt
                self._comparison_pending = nominal
                self._comparison_velocity = velocity
                self._comparison_written = False
                try:
                    self.control_configuration_step(reduced, velocity, balance_scale=1.0)
                    if not self._comparison_written:
                        raise RuntimeError('Counter did not emit a comparison command')
                finally:
                    self._comparison_pending = None
                self._planned_previous = reduced.copy()
                return
            combined = tuple(value.copy() for value in nominal)
            if mode == 'counter':
                # the defined I-stage holds the last counter posture, not a return
                combined[0][self.counter_arm_ids] = self.low_cmd_handler.q_cmd[self.counter_arm_ids]
                combined[1][self.counter_arm_ids] = 0.0
                combined[2][self.counter_arm_ids] = self.low_cmd_handler.tau_cmd[self.counter_arm_ids]
            return self._publish_comparison_command(nominal, combined)
        if self.planned_balance:
            reduced = np.asarray(q)[self.robot_model.reduced_mask]
            velocity = (reduced - self._planned_previous) / self.dt
            self.control_configuration_step(reduced, velocity, balance_scale=1.0)
            self._planned_previous = reduced.copy()
            return
        return super()._publish_position_command(q, dq, tau)

    def _write_arm_command(self, arm_q, arm_dq, tau):
        pending = getattr(self, '_comparison_pending', None)
        if pending is None:
            return super()._write_arm_command(arm_q, arm_dq, tau)
        if self.latest_status not in ('solved', 'collision_backtracked'):
            raise RuntimeError(f'Counter comparison failed: {self.latest_status}')
        if (not np.array_equal(arm_q[self.moving_local], pending[0][self.moving_ids]) or
                not np.array_equal(arm_dq[self.moving_local],
                                   self._comparison_velocity[self.moving_local])):
            raise ValueError('Counter changed the moving-arm reference; holding')
        combined = tuple(value.copy() for value in pending)
        for destination, source in zip(combined, (arm_q, arm_dq, tau)):
            destination[self.counter_arm_ids] = np.asarray(source)[self.counter_local]
        self._publish_comparison_command(pending, combined)
        self._comparison_written = True
        self.latest_published = True
        return True

    def _publish_comparison_command(self, nominal, combined):
        check = getattr(self, '_comparison_check', None)
        if check is not None:
            check()
        q, dq, tau = combined
        # validate the torso goal that Upper publishes, not the measured torso
        torso = BODY_JOINTS.index('torso_joint')
        q[torso] = self.low_cmd_handler.q_cmd[torso]
        if not all(np.isfinite(value).all() for value in combined):
            raise ValueError('Nonfinite combined comparison command; holding')
        if any(not np.array_equal(value[self.moving_ids], reference[self.moving_ids])
               for value, reference in zip(combined, nominal)):
            raise ValueError('Combined moving-arm command differs from nominal; holding')
        self._validate_waypoint(q, q[self.robot_model.reduced_mask])
        limits = self.low_cmd_handler._q_clip_limits
        if limits is not None and np.any(
                (q[self.upper_ids] < limits[self.upper_ids, 0]) |
                (q[self.upper_ids] > limits[self.upper_ids, 1])):
            raise ValueError('Combined comparison command would be clipped; holding')
        if self._comparison_playback:
            config = self.planner.planner_config
            rates = np.abs((q[self.arm_ids] -
                            self.low_cmd_handler.q_cmd[self.arm_ids]) / self.dt)
            limit = np.minimum(config.joint_velocity_limit,
                               self.robot_model.model_body_reduced.velocityLimit)
            if np.any(rates > limit + 1e-6):
                raise ValueError('Combined comparison command exceeds joint rates; holding')
        UpperController._publish_position_command(self, q, dq, tau)
        emitted = tuple(np.asarray(getattr(self.low_cmd_handler, name))[self.moving_ids]
                        for name in ('q_cmd', 'dq_cmd', 'tau_cmd'))
        if not all(np.isfinite(value).all() for value in emitted):
            raise ValueError('Nonfinite emitted moving-arm command; holding')
        # compare exact publisher-buffer values, not float64 solver intermediates
        expected = tuple(value[self.moving_ids].astype(actual.dtype, copy=False)
                         for value, actual in zip(nominal, emitted))
        deviations = [float(np.max(np.abs(actual - wanted)))
                      for actual, wanted in zip(emitted, expected)]
        self._comparison_max_deviation = np.maximum(
            self._comparison_max_deviation, deviations,
        )
        if self._comparison_audit is not None:
            self._comparison_audit({
                'type': 'command', 'moving_nominal': [value.tolist() for value in expected],
                'moving_emitted': [value.tolist() for value in emitted],
                'moving_max_deviation': deviations,
                'counter_status': self.latest_status if self._comparison_mode == 'counter' else None,
                'counter_backtrack_scale': (float(self.latest_backtrack_scale)
                                            if self._comparison_mode == 'counter' else None),
                'nominal_retry_used': bool(getattr(self, 'latest_nominal_fallback_used', False))
                if self._comparison_mode == 'counter' else False,
            })
        if any(deviations):
            raise ValueError('Emitted moving-arm command differs from nominal; holding')

    def _steady_state_control_ids(self):
        if getattr(self, '_comparison_mode', None) is not None:
            return self.moving_ids
        return super()._steady_state_control_ids()

    def finish_goal(self):
        # retain the counter pose and references between segmented frame goals
        hold(self)
