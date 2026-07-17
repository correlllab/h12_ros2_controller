import logging
import time

import numpy as np
import pinocchio as pin
from scipy.optimize import lsq_linear

from h12_ros2_controller.utility.joint_definition import (
    LEFT_ARM_INDEX,
    RIGHT_ARM_INDEX,
)

LOGGER = logging.getLogger(__name__)


def format_vector(value):
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    return '[' + ' '.join(f'{entry:+.2f}' for entry in arr) + ']'


def format_vector_map(values):
    if not values:
        return '{}'
    entries = [
        f'{name}:{format_vector(value)}'
        for name, value in values.items()
    ]
    return '{' + ', '.join(entries) + '}'


def format_status_map(statuses):
    if not statuses:
        return '{}'
    entries = [
        f'{name}:{status}'
        for name, status in statuses.items()
    ]
    return '{' + ', '.join(entries) + '}'


class BalanceActuator:
    def __init__(self, robot_model, dt, config):
        zmp_cfg = config.get('zmp', {})
        ddp_cfg = zmp_cfg.get('ddp', {})
        blending_cfg = zmp_cfg.get('blending', {})
        failure_cfg = zmp_cfg.get('solver_failure', {})
        execution_cfg = zmp_cfg.get('execution', {})
        response_cfg = zmp_cfg.get('response', {})
        controller_cfg = config.get('controller', {})

        self.robot_model = robot_model
        self.config = config
        self.dt = float(dt)
        self.return_threshold = float(
            ddp_cfg.get(
                'return_threshold',
                controller_cfg.get('threshold_joint', 1e-3),
            )
        )
        self.return_max_steps = max(
            1,
            int(float(ddp_cfg.get('return_timeout', 0.4)) / self.dt),
        )
        self.max_return_velocity = float(ddp_cfg.get('max_velocity', 6.0))
        self.execution_mode = execution_cfg.get('mode', 'ddp')
        self.direct_damping = float(execution_cfg.get('direct_damping', 0.05))
        self.direct_solver = execution_cfg.get('direct_solver', 'reduced_dls')
        self.direct_max_velocity = float(
            execution_cfg.get('direct_max_velocity', 6.0)
        )
        self.direct_max_steps = max(
            1,
            int(float(execution_cfg.get('direct_duration', 0.2)) / self.dt),
        )
        self.direct_cooldown_steps = max(
            0,
            int(float(execution_cfg.get('direct_cooldown', 0.2)) / self.dt),
        )
        self.direct_hold_after_burst = bool(
            execution_cfg.get('direct_hold_after_burst', False)
        )
        self.direct_joint_velocity = min(
            self.direct_max_velocity,
            float(controller_cfg.get('dq_lim', self.direct_max_velocity)),
        )
        self.direct_position_margin = float(
            execution_cfg.get('direct_position_margin', 0.05)
        )
        self.response_mode = response_cfg.get('mode', 'legacy_burst')
        self.reject_max_steps = max(
            1,
            int(float(response_cfg.get('reject_max_duration', 0.6)) / self.dt),
        )
        self.arrest_steps = max(
            1,
            int(float(response_cfg.get('arrest_duration', 0.12)) / self.dt),
        )
        self.recovery_min_steps = max(
            0,
            int(float(response_cfg.get('recovery_min_duration', 0.3)) / self.dt),
        )
        self.recovery_settle_steps = max(
            1,
            int(float(response_cfg.get('recovery_settle_duration', 0.3)) / self.dt),
        )
        self.recovery_zmp_threshold = float(
            response_cfg.get('recovery_zmp_residual', 0.003)
        )
        self.recovery_zmp_velocity_threshold = float(
            response_cfg.get('recovery_zmp_velocity', 0.0015)
        )
        self.recovery_angular_velocity_threshold = float(
            response_cfg.get('recovery_angular_velocity', 0.08)
        )
        self.min_replan_ticks = max(
            1,
            int(
                np.ceil(
                    float(ddp_cfg.get('min_replan_interval', 0.08)) / self.dt
                )
            ),
        )
        self.interrupt_ticks = max(
            1,
            int(blending_cfg.get('interrupt_ticks', 3)),
        )
        self.allow_single_arm_success = bool(
            failure_cfg.get('allow_single_arm_success', True)
        )
        self.accept_best_effort = bool(
            failure_cfg.get('accept_best_effort', True)
        )
        self.min_alignment = float(failure_cfg.get('min_alignment', 0.2))
        self.min_useful_momentum = float(
            failure_cfg.get('min_useful_momentum', 0.1)
        )
        self.suppressed_warning_interval = float(
            failure_cfg.get('suppressed_warning_interval', 1.0)
        )

        self.behaviors = {}
        self.state = 'idle'
        self.plans = {}
        self.plan_index = 0
        self.return_targets = {}
        self.return_index = 0
        self.ticks_since_plan = self.min_replan_ticks
        self.last_return_command = None
        self.blend_reference = None
        self.blend_index = 0

        self.latest_target_momentum = np.zeros(3, dtype=np.float64)
        self.latest_arm_targets = {}
        self.latest_per_arm_achieved = {}
        self.latest_combined_achieved = np.zeros(3, dtype=np.float64)
        self.latest_solver_statuses = {}
        self.latest_plan_duration = 0.0
        self.latest_response_status = 'idle'
        self.latest_response_summary = ''
        self.latest_best_effort_used = False
        self.latest_raw_plan_command_norm = 0.0
        self.latest_raw_command = np.zeros(
            self.robot_model.model_body.nv,
            dtype=np.float64,
        )
        self.latest_plan_lengths = {}
        self.latest_plan_phase_lengths = {}
        self.latest_executed_plan_length = 0
        self.direct_targets = {}
        self.direct_index = 0
        self.direct_cooldown_index = 0
        self.burst_id = 0
        self.burst_entering = False
        self.arrest_index = 0
        self.recovery_index = 0
        self.recovery_settle_index = 0
        self.last_reject_command = np.zeros(
            self.robot_model.model_body.nv,
            dtype=np.float64,
        )
        self._last_suppressed_warning_time = -np.inf

        if self.execution_mode not in ('ddp', 'direct'):
            raise ValueError(
                f'unsupported zmp.execution.mode: {self.execution_mode}'
            )
        if self.direct_solver not in ('reduced_dls', 'full_body_bounded'):
            raise ValueError(
                f'unsupported zmp.execution.direct_solver: {self.direct_solver}'
            )

    def can_start_response(self, perturbation_state, target_momentum):
        if not perturbation_state.active:
            return False
        if np.linalg.norm(target_momentum) <= 1e-9:
            return False
        if self.ticks_since_plan < self.min_replan_ticks:
            return False
        return self.state in ('idle', 'returning')

    def reset_response(self):
        '''Clear active response state before a new balance experiment'''
        self.state = 'idle'
        self.return_targets = {}
        self.return_index = 0
        self.direct_targets = {}
        self.direct_index = 0
        self.direct_cooldown_index = 0
        self.burst_id = 0
        self.burst_entering = False
        self.arrest_index = 0
        self.recovery_index = 0
        self.recovery_settle_index = 0
        self.last_reject_command[:] = 0.0
        self.latest_response_status = 'idle'
        self.latest_response_summary = ''
        self.latest_target_momentum = np.zeros(3, dtype=np.float64)
        self.latest_arm_targets = {}

    def maybe_start_response(self, arm_targets, perturbation_state):
        if not arm_targets:
            return False
        was_returning = self.state == 'returning'
        blend_reference = None
        if was_returning:
            blend_reference = self._return_velocity_command()

        started = self._start_response(arm_targets, perturbation_state)
        if started and was_returning:
            self.blend_reference = blend_reference
            self.blend_index = 0
        return started

    def update_direct_response(
            self,
            arm_targets,
            perturbation_state,
            balance_state=None):
        '''Update the closed-loop momentum response target'''
        self.burst_entering = False
        if self.execution_mode != 'direct':
            return

        if self.response_mode == 'state_machine':
            self._update_state_machine(
                arm_targets,
                perturbation_state,
                balance_state,
            )
            return

        if self.direct_cooldown_index > 0:
            return

        if perturbation_state.active and arm_targets:
            if self.state not in ('direct_impulse', 'holding'):
                if not self.return_targets:
                    self.return_targets = {
                        target.arm: self._current_arm_q(target.arm)
                        for target in arm_targets
                    }
                self.return_index = 0
                self.direct_index = 0
                self.burst_id += 1
                self.burst_entering = True
            if self.state != 'holding':
                self.direct_targets = {
                    target.arm: np.copy(target.target_momentum)
                    for target in arm_targets
                }
                self.state = 'direct_impulse'
            self.latest_target_momentum = self._combined_achieved(
                self.direct_targets
            )
            self.latest_arm_targets = dict(self.direct_targets)
            self.latest_response_status = self.state
            self.latest_response_summary = (
                'ZMP response direct '
                f'target={format_vector(self.latest_target_momentum)} '
                f'per_arm={format_vector_map(self.direct_targets)}'
            )
            return

        if self.state in ('direct_impulse', 'holding'):
            self.direct_targets = {}
            self.state = 'returning'
            self.return_index = 0
            self.latest_response_status = 'returning'

    def step(self):
        '''Return one full-body velocity command for active response state'''
        self.ticks_since_plan += 1
        self.direct_cooldown_index = max(
            0,
            self.direct_cooldown_index - 1,
        )
        if self.state == 'direct_impulse':
            return self._execute_direct_step()
        if self.state == 'rejecting':
            return self._execute_reject_step()
        if self.state == 'arresting':
            return self._execute_arrest_step()
        if self.state == 'recovery_wait':
            command = np.zeros(
                self.robot_model.model_body.nv,
                dtype=np.float64,
            )
            self._set_latest_raw_command(command)
            return command
        if self.state == 'holding':
            command = np.zeros(
                self.robot_model.model_body.nv,
                dtype=np.float64,
            )
            self._set_latest_raw_command(command)
            return command
        if self.state == 'executing_impulse':
            return self._execute_plan_step()
        if self.state == 'returning':
            return self._execute_return_step()
        command = np.zeros(self.robot_model.model_body.nv, dtype=np.float64)
        self._set_latest_raw_command(command)
        return command

    def _update_state_machine(
            self,
            arm_targets,
            perturbation_state,
            balance_state):
        new_episode = bool(perturbation_state.entering)
        if new_episode and self.state in ('idle', 'returning', 'recovery_wait'):
            self._start_reject(arm_targets)
            return

        if self.state == 'idle':
            if perturbation_state.active and arm_targets:
                self._start_reject(arm_targets)
            return

        if self.state == 'rejecting' and arm_targets:
            self.direct_targets = {
                target.arm: np.copy(target.target_momentum)
                for target in arm_targets
            }
            self.latest_target_momentum = self._combined_achieved(
                self.direct_targets
            )
            self.latest_arm_targets = dict(self.direct_targets)
            return

        if self.state == 'recovery_wait':
            self.recovery_index += 1
            if self._balance_quiet(balance_state) and not perturbation_state.active:
                self.recovery_settle_index += 1
            else:
                self.recovery_settle_index = 0
            if (
                    self.recovery_index >= self.recovery_min_steps
                    and self.recovery_settle_index
                    >= self.recovery_settle_steps):
                self.state = 'returning'
                self.return_index = 0
                self.latest_response_status = 'returning'

    def _start_reject(self, arm_targets):
        if not arm_targets:
            return
        if not self.return_targets:
            self.return_targets = {
                target.arm: self._current_arm_q(target.arm)
                for target in arm_targets
            }
        self.direct_targets = {
            target.arm: np.copy(target.target_momentum)
            for target in arm_targets
        }
        self.direct_index = 0
        self.arrest_index = 0
        self.recovery_index = 0
        self.recovery_settle_index = 0
        self.burst_id += 1
        self.burst_entering = True
        self.state = 'rejecting'
        self.latest_response_status = 'rejecting'
        self.latest_target_momentum = self._combined_achieved(
            self.direct_targets
        )
        self.latest_arm_targets = dict(self.direct_targets)

    def _execute_reject_step(self):
        if self.direct_index >= self.reject_max_steps:
            self.state = 'arresting'
            self.arrest_index = 0
            self.direct_targets = {}
            self.last_reject_command = np.copy(self.latest_raw_command)
            self.latest_response_status = 'arresting'
            return self._execute_arrest_step()
        if self.direct_solver == 'full_body_bounded':
            command = self._execute_full_body_direct_step()
        else:
            command = self._execute_direct_step()
        self.last_reject_command = np.copy(command)
        return command

    def _execute_arrest_step(self):
        if self.arrest_index >= self.arrest_steps:
            self.state = 'recovery_wait'
            self.recovery_index = 0
            self.recovery_settle_index = 0
            self.latest_response_status = 'recovery_wait'
            command = np.zeros(
                self.robot_model.model_body.nv,
                dtype=np.float64,
            )
            self._set_latest_raw_command(command)
            return command
        scale = 1.0 - float(self.arrest_index + 1) / self.arrest_steps
        command = scale * self.last_reject_command
        self.arrest_index += 1
        self._set_latest_raw_command(command)
        return command

    def _balance_quiet(self, balance_state):
        if balance_state is None:
            return False
        return (
            np.linalg.norm(balance_state.zmp_residual)
            <= self.recovery_zmp_threshold
            and np.linalg.norm(balance_state.zmp_residual_velocity)
            <= self.recovery_zmp_velocity_threshold
            and np.linalg.norm(balance_state.angular_velocity[:2])
            <= self.recovery_angular_velocity_threshold
        )

    def _execute_direct_step(self):
        if self.direct_index >= self.direct_max_steps:
            self.direct_targets = {}
            if self.direct_hold_after_burst:
                self.state = 'holding'
                self.latest_response_status = 'holding'
                command = np.zeros(
                    self.robot_model.model_body.nv,
                    dtype=np.float64,
                )
                self._set_latest_raw_command(command)
                return command
            self.direct_cooldown_index = self.direct_cooldown_steps
            self.state = 'returning'
            self.return_index = 0
            self.latest_response_status = 'returning'
            return self._execute_return_step()

        if self.direct_solver == 'full_body_bounded':
            return self._execute_full_body_direct_step()

        command = np.zeros(
            self.robot_model.model_body.nv,
            dtype=np.float64,
        )
        achieved = {}
        for arm, target in self.direct_targets.items():
            behavior = self._behavior(arm)
            q = behavior.current_arm_q()
            data = behavior.arm_model.createData()
            centroidal_map = pin.computeCentroidalMap(
                behavior.arm_model,
                data,
                q,
            )
            angular_map = np.asarray(
                centroidal_map[3:6, :],
                dtype=np.float64,
            )
            regularized = (
                angular_map @ angular_map.T
                + self.direct_damping ** 2 * np.eye(3)
            )
            velocity = angular_map.T @ np.linalg.solve(regularized, target)
            velocity = np.clip(
                velocity,
                -self.direct_max_velocity,
                self.direct_max_velocity,
            )
            command[behavior.arm_ids] = velocity
            achieved[arm] = angular_map @ velocity

        self.direct_index += 1
        self.latest_per_arm_achieved = achieved
        self.latest_combined_achieved = self._combined_achieved(achieved)
        self._set_latest_raw_command(command)
        return command

    def _execute_full_body_direct_step(self):
        command = np.zeros(
            self.robot_model.model_body.nv,
            dtype=np.float64,
        )
        arm_ids = []
        if 'left' in self.direct_targets:
            arm_ids.extend(LEFT_ARM_INDEX)
        if 'right' in self.direct_targets:
            arm_ids.extend(RIGHT_ARM_INDEX)
        if not arm_ids:
            self._set_latest_raw_command(command)
            return command

        target = self._combined_achieved(self.direct_targets)
        q = np.asarray(self.robot_model.state['q'], dtype=np.float64)
        angular_map = self.robot_model.get_angular_centroidal_momentum_matrix(
            q,
            arm_ids,
        )
        lower, upper = self._direct_velocity_bounds(q, arm_ids)
        free = upper - lower > 1e-9
        velocity = np.zeros(len(arm_ids), dtype=np.float64)
        status = 'blocked'
        start = time.monotonic()
        if np.any(free):
            free_map = angular_map[:, free]
            augmented_map = np.vstack((
                free_map,
                self.direct_damping * np.eye(np.count_nonzero(free)),
            ))
            augmented_target = np.concatenate((
                target,
                np.zeros(np.count_nonzero(free), dtype=np.float64),
            ))
            result = lsq_linear(
                augmented_map,
                augmented_target,
                bounds=(lower[free], upper[free]),
                tol=1e-7,
                lsmr_tol='auto',
                max_iter=30,
            )
            velocity[free] = result.x
            status = 'success' if result.success else f'failed:{result.status}'

        command[arm_ids] = velocity
        achieved = {}
        offset = 0
        for arm, ids in (
                ('left', LEFT_ARM_INDEX),
                ('right', RIGHT_ARM_INDEX)):
            if arm not in self.direct_targets:
                continue
            count = len(ids)
            achieved[arm] = (
                angular_map[:, offset:offset + count]
                @ velocity[offset:offset + count]
            )
            offset += count

        self.direct_index += 1
        self.latest_per_arm_achieved = achieved
        self.latest_combined_achieved = angular_map @ velocity
        self.latest_solver_statuses = {'full_body': status}
        self.latest_plan_duration = time.monotonic() - start
        self.latest_response_summary = (
            'ZMP response full_body_bounded '
            f'target={format_vector(target)} '
            f'achieved={format_vector(self.latest_combined_achieved)} '
            f'status={status}'
        )
        self._set_latest_raw_command(command)
        return command

    def _direct_velocity_bounds(self, q, arm_ids):
        model = self.robot_model.model_body
        velocity_limit = np.asarray(model.velocityLimit, dtype=np.float64)[arm_ids]
        finite_velocity = np.where(
            np.isfinite(velocity_limit) & (velocity_limit > 0.0),
            velocity_limit,
            self.direct_joint_velocity,
        )
        maximum = np.minimum(finite_velocity, self.direct_joint_velocity)
        lower = -maximum
        upper = maximum
        q_arm = q[arm_ids]
        position_lower = np.asarray(model.lowerPositionLimit)[arm_ids]
        position_upper = np.asarray(model.upperPositionLimit)[arm_ids]
        lower = np.maximum(
            lower,
            (position_lower + self.direct_position_margin - q_arm) / self.dt,
        )
        upper = np.minimum(
            upper,
            (position_upper - self.direct_position_margin - q_arm) / self.dt,
        )
        return lower, upper

    def _start_response(self, arm_targets, perturbation_state):
        start = time.monotonic()
        statuses = {}
        plans = {}
        achieved = {}
        return_targets = {}
        per_arm_targets = {
            arm_target.arm: np.copy(arm_target.target_momentum)
            for arm_target in arm_targets
        }
        target_momentum = self._combined_achieved(per_arm_targets)
        self.latest_target_momentum = np.copy(target_momentum)
        self.latest_arm_targets = per_arm_targets
        self.burst_entering = False

        for arm_target in arm_targets:
            arm = arm_target.arm
            target = np.asarray(arm_target.target_momentum, dtype=np.float64)
            try:
                behavior = self._behavior(arm)
                return_targets[arm] = self._current_arm_q(arm)
                plan = behavior.solve(target, q_ref=return_targets[arm])
            except Exception as err:
                statuses[arm] = 'exception'
                LOGGER.debug(
                    'ZMP balance solve exception for %s: %s',
                    arm,
                    err,
                )
                continue

            accepted, status, peak_momentum = self._evaluate_plan(
                plan,
                target,
            )
            statuses[arm] = status
            if not accepted:
                continue
            plans[arm] = plan
            achieved[arm] = peak_momentum

        if not plans:
            plan_duration = time.monotonic() - start
            summary = self._response_summary(
                'suppressed',
                target_momentum,
                per_arm_targets,
                statuses,
                {},
                np.zeros(3, dtype=np.float64),
                plan_duration,
            )
            self._clear_plan_debug(statuses, plan_duration, summary)
            self._log_suppressed(summary)
            return False

        if (
            len(plans) == 1
            and len(arm_targets) > 1
            and not self.allow_single_arm_success
        ):
            plan_duration = time.monotonic() - start
            summary = self._response_summary(
                'suppressed_single_arm',
                target_momentum,
                per_arm_targets,
                statuses,
                {},
                np.zeros(3, dtype=np.float64),
                plan_duration,
            )
            self._clear_plan_debug(statuses, plan_duration, summary)
            self._log_suppressed(summary)
            return False

        self.plans = plans
        self.return_targets = {
            arm: return_targets[arm]
            for arm in plans
        }
        self.plan_index = 0
        self.return_index = 0
        self.state = 'executing_impulse'
        self.burst_id += 1
        self.burst_entering = True
        self.ticks_since_plan = 0
        self.latest_per_arm_achieved = achieved
        self.latest_combined_achieved = self._combined_achieved(achieved)
        self.latest_solver_statuses = statuses
        self.latest_plan_lengths = {
            arm: len(plan.us)
            for arm, plan in plans.items()
        }
        self.latest_plan_phase_lengths = {
            arm: getattr(plan, 'phase_lengths', None)
            for arm, plan in plans.items()
        }
        self.latest_executed_plan_length = max(
            self.latest_plan_lengths.values()
        )
        self.latest_plan_duration = time.monotonic() - start
        self.latest_best_effort_used = any(
            status == 'best_effort'
            for status in statuses.values()
        )
        self.latest_response_status = (
            'best_effort'
            if self.latest_best_effort_used
            else 'solved'
        )
        self.latest_response_summary = self._response_summary(
            self.latest_response_status,
            self.latest_target_momentum,
            self.latest_arm_targets,
            self.latest_solver_statuses,
            self.latest_per_arm_achieved,
            self.latest_combined_achieved,
            self.latest_plan_duration,
        )
        LOGGER.info(self.latest_response_summary)
        return True

    def _evaluate_plan(self, plan, target):
        us = np.asarray(plan.us, dtype=np.float64)
        if len(us) == 0:
            return False, 'empty', np.zeros(3, dtype=np.float64)
        if not np.isfinite(us).all():
            return False, 'nonfinite_velocity', np.zeros(3, dtype=np.float64)

        peak_momentum = np.asarray(plan.peak_momentum, dtype=np.float64)
        if peak_momentum.shape != (3,) or not np.isfinite(peak_momentum).all():
            return False, 'nonfinite_momentum', np.zeros(3, dtype=np.float64)
        if np.linalg.norm(peak_momentum) <= 1e-9:
            return False, 'not_useful', np.copy(peak_momentum)

        alignment = self._momentum_alignment(peak_momentum, target)
        if alignment < self.min_alignment:
            if alignment < 0.0:
                return False, 'opposite', np.copy(peak_momentum)
            return False, 'misaligned', np.copy(peak_momentum)

        useful = self._useful_momentum(peak_momentum, target)
        if useful < self.min_useful_momentum:
            return False, 'not_useful', np.copy(peak_momentum)

        if plan.solved:
            return True, 'solved', np.copy(peak_momentum)
        if self.accept_best_effort:
            return True, 'best_effort', np.copy(peak_momentum)
        return False, 'unconverged', np.copy(peak_momentum)

    def _clear_plan_debug(self, statuses, plan_duration, summary):
        self.latest_per_arm_achieved = {}
        self.latest_combined_achieved = np.zeros(3, dtype=np.float64)
        self.latest_solver_statuses = statuses
        self.latest_plan_duration = float(plan_duration)
        self.latest_response_status = 'suppressed'
        self.latest_response_summary = summary
        self.latest_best_effort_used = False
        self.latest_plan_lengths = {}
        self.latest_plan_phase_lengths = {}
        self.latest_executed_plan_length = 0

    def _log_suppressed(self, summary):
        now = time.monotonic()
        elapsed = now - self._last_suppressed_warning_time
        if elapsed < self.suppressed_warning_interval:
            return
        self._last_suppressed_warning_time = now
        LOGGER.warning(summary)

    @staticmethod
    def _response_summary(status,
                          target_momentum,
                          per_arm_targets,
                          statuses,
                          achieved,
                          combined_achieved,
                          plan_duration):
        return (
            f'ZMP response {status} '
            f'target={format_vector(target_momentum)} '
            f'per_arm={format_vector_map(per_arm_targets)} '
            f'status={format_status_map(statuses)} '
            f'achieved={format_vector_map(achieved)} '
            f'combined={format_vector(combined_achieved)} '
            f'duration={plan_duration:.3f}s'
        )

    @staticmethod
    def _momentum_alignment(achieved, target):
        achieved_norm = float(np.linalg.norm(achieved))
        target_norm = float(np.linalg.norm(target))
        if achieved_norm <= 1e-9 or target_norm <= 1e-9:
            return -np.inf
        return float(np.dot(achieved, target) / (achieved_norm * target_norm))

    @staticmethod
    def _useful_momentum(achieved, target):
        target_norm = float(np.linalg.norm(target))
        if target_norm <= 1e-9:
            return 0.0
        direction = target / target_norm
        return float(max(0.0, np.dot(achieved, direction)))

    def _execute_plan_step(self):
        nv = self.robot_model.model_body.nv
        if self._plan_complete():
            self.state = 'returning'
            return self._execute_return_step()

        command = np.zeros(nv, dtype=np.float64)
        for plan in self.plans.values():
            if self.plan_index < len(plan.us):
                command += plan.body_velocity_at(self.plan_index, nv)
        self.plan_index += 1
        command = self._blend_command(command)
        self._set_latest_raw_command(command)
        return command

    def _execute_return_step(self):
        if self._return_complete():
            self.state = 'idle'
            self.return_targets = {}
            self.last_return_command = None
            self.latest_response_status = 'idle'
            self.latest_response_summary = ''
            self.latest_target_momentum = np.zeros(3, dtype=np.float64)
            self.latest_arm_targets = {}
            command = np.zeros(
                self.robot_model.model_body.nv,
                dtype=np.float64,
            )
            self._set_latest_raw_command(command)
            return command

        command = self._return_velocity_command()
        self.return_index += 1
        self.last_return_command = np.copy(command)
        self._set_latest_raw_command(command)
        return command

    def _set_latest_raw_command(self, command):
        self.latest_raw_command = np.copy(command)
        self.latest_raw_plan_command_norm = float(np.linalg.norm(command))

    def _return_velocity_command(self):
        command = np.zeros(self.robot_model.model_body.nv, dtype=np.float64)
        remaining_time = max(
            self.dt,
            (self.return_max_steps - self.return_index) * self.dt,
        )
        for arm, q_target in self.return_targets.items():
            behavior = self._behavior(arm)
            q_current = behavior.current_arm_q()
            velocity = pin.difference(
                behavior.arm_model,
                q_current,
                q_target,
            ) / remaining_time
            velocity = np.clip(
                velocity,
                -self.max_return_velocity,
                self.max_return_velocity,
            )
            command[behavior.arm_ids] = velocity
        return command

    def _blend_command(self, command):
        if self.blend_reference is None:
            return command
        beta = float(self.blend_index + 1) / float(self.interrupt_ticks)
        beta = np.clip(beta, 0.0, 1.0)
        blended = (1.0 - beta) * self.blend_reference + beta * command
        self.blend_index += 1
        if self.blend_index >= self.interrupt_ticks:
            self.blend_reference = None
            self.blend_index = 0
        return blended

    def _plan_complete(self):
        if not self.plans:
            return True
        return all(
            self.plan_index >= len(plan.us)
            for plan in self.plans.values()
        )

    def _return_complete(self):
        if not self.return_targets:
            return True
        if self.return_index >= self.return_max_steps:
            return True
        errors = []
        for arm, q_target in self.return_targets.items():
            behavior = self._behavior(arm)
            q_current = behavior.current_arm_q()
            errors.append(
                np.max(
                    np.abs(
                        pin.difference(
                            behavior.arm_model,
                            q_current,
                            q_target,
                        )
                    )
                )
            )
        return max(errors) < self.return_threshold

    def _behavior(self, arm):
        if arm not in self.behaviors:
            self.behaviors[arm] = self.robot_model.dynamics.create_momentum_ddp(
                dt=self.dt,
                config=self._behavior_config(arm),
                arm=arm,
            )
        return self.behaviors[arm]

    def _behavior_config(self, arm):
        ddp_cfg = self._ddp_config()
        ddp_cfg['arm'] = arm
        ddp_cfg['dt'] = self.dt
        ddp_cfg['enabled'] = True
        return {'momentum_ddp': ddp_cfg}

    def _ddp_config(self):
        ddp = self.config.get('zmp', {}).get('ddp', {})
        return dict(ddp)

    def _current_arm_q(self, arm):
        return np.copy(self._behavior(arm).current_arm_q())

    @staticmethod
    def _combined_achieved(achieved):
        total = np.zeros(3, dtype=np.float64)
        for momentum in achieved.values():
            total += momentum
        return total
