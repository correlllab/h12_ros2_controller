import numpy as np

from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.utility.joint_definition import UPPER_BODY_INDEX
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller import (
    CounterDDPVelocityRobustController,
)


def fresh_state(controller):
    q = np.array(controller.robot_model.state['q'], dtype=float, copy=True)
    if q.shape != (27,) or not np.isfinite(q).all():
        raise ValueError('Measured motor positions must be finite')
    return q


def valid_arm_command(controller, arm_q):
    candidate = fresh_state(controller)
    candidate[controller.arm_ids] = arm_q
    return (controller.robot_model.check_within_limits_reduced(arm_q)
            and controller.robot_model.check_collision_free(
                candidate, min_distance=controller.d_min,
            ))


def hold(controller):
    if getattr(controller.low_cmd_handler, '_estopped', False):
        return
    ids = UPPER_BODY_INDEX
    controller.low_cmd_handler.set_joint_commands(
        q=controller.low_cmd_handler.q_cmd[ids],
        dq=np.zeros(len(ids)),
        tau=controller.low_cmd_handler.tau_cmd[ids],
        joint_ids=ids,
    )


class CounterGotoController(CounterDDPVelocityRobustController):
    '''Route plain setup, balanced motion and moving-arm steady-state hold'''

    def __init__(self, *args, plain_goto=False, **kwargs):
        self.plain_goto = plain_goto
        self.balanced_live = False
        self.planned_balance = False
        super().__init__(*args, **kwargs)

    def _apply_velocity_command(self, velocity):
        if not self.balanced_live:
            return super()._apply_velocity_command(velocity)
        self.ik_solver.integrate(velocity)
        self.control_configuration_step(
            self.ik_solver.q[self.arm_ids], velocity[self.arm_ids], balance_scale=1.0,
        )
        self.update_robot_model()
        return velocity

    def _write_arm_command(self, arm_q, arm_dq, tau):
        if self.planned_balance:
            if not valid_arm_command(self, arm_q):
                raise ValueError('Combined planned/counter-arm command is invalid; holding')
        # retain velocity for balance prediction, not moving-arm motor feedforward
        arm_dq = np.copy(arm_dq)
        arm_dq[self.moving_local] = 0.0
        return super()._write_arm_command(arm_q, arm_dq, tau)

    def _record_moving_command_error(self, arm_q, arm_dq):
        motor_dq = np.copy(arm_dq)
        motor_dq[self.moving_local] = 0.0
        super()._record_moving_command_error(arm_q, motor_dq)

    def _publish_position_command(self, q, dq, tau):
        if self.plain_goto:
            if self.balanced_live:
                # the I-stage holds the last counter command while correcting the moving arm
                q_hold = self.low_cmd_handler.q_cmd.copy()
                dq_hold = np.zeros_like(q_hold)
                tau_hold = self.low_cmd_handler.tau_cmd.copy()
                q_hold[self.moving_ids] = q[self.moving_ids]
                tau_hold[self.moving_ids] = tau[self.moving_ids]
                return UpperController._publish_position_command(
                    self, q_hold, dq_hold, tau_hold,
                )
            return UpperController._publish_position_command(self, q, dq, tau)
        return super()._publish_position_command(q, dq, tau)

    def _steady_state_control_ids(self):
        if self.plain_goto:
            if self.balanced_live:
                return self.moving_ids
            return self.upper_ids
        return super()._steady_state_control_ids()
