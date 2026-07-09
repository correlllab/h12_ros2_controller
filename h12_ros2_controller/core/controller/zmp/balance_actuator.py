import logging
import time

import numpy as np
import pinocchio as pin

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
        self._last_suppressed_warning_time = -np.inf

    def can_start_response(self, perturbation_state, target_momentum):
        if not perturbation_state.active:
            return False
        if np.linalg.norm(target_momentum) <= 1e-9:
            return False
        if self.ticks_since_plan < self.min_replan_ticks:
            return False
        return self.state in ('idle', 'returning')

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

    def step(self):
        '''Return one full-body velocity command for active response state'''
        self.ticks_since_plan += 1
        if self.state == 'executing_impulse':
            return self._execute_plan_step()
        if self.state == 'returning':
            return self._execute_return_step()
        command = np.zeros(self.robot_model.model_body.nv, dtype=np.float64)
        self._set_latest_raw_command(command)
        return command

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
