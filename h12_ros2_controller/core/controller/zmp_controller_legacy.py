import numpy as np

from h12_ros2_controller.core.controller.upper_controller import (
    UpperController,
)
from h12_ros2_controller.core.controller.zmp_legacy import (
    BalanceActuator,
    BalanceObserver,
    MomentumAllocator,
    MomentumTargetEstimator,
    PerturbationDetector,
)
from h12_ros2_controller.utility.joint_definition import (
    LEFT_ARM_INDEX,
    RIGHT_ARM_INDEX,
)


class ZmpController(UpperController):
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 init: bool = True,
                 handless: bool = False,
                 visualize: bool = False,
                 config: dict = None):
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            init=init,
            handless=handless,
            visualize=visualize,
            config=config,
        )
        self.zmp_config = self.config.get('zmp', {})
        self.zmp_enabled = bool(self.zmp_config.get('enabled', False))
        self.reflex_output_enabled = self.zmp_enabled

        self.observer = BalanceObserver(self.robot_model, self.dt, self.config)
        self.detector = PerturbationDetector(self.config)
        self.target_estimator = MomentumTargetEstimator(
            self.robot_model,
            self.config,
        )
        self.allocator = MomentumAllocator(self.config)
        self.actuator = BalanceActuator(
            self.robot_model,
            self.dt,
            self.config,
        )

        self.latest_balance_state = None
        self.latest_perturbation_state = None
        self.latest_target_momentum = np.zeros(3, dtype=np.float64)
        self.latest_raw_target_momentum = np.zeros(3, dtype=np.float64)
        self.latest_arm_targets = {}
        self.latest_arm_achieved_momentum = {}
        self.latest_combined_achieved_momentum = np.zeros(3, dtype=np.float64)
        self.latest_post_limit_momentum = np.zeros(3, dtype=np.float64)
        self.latest_pre_limit_momentum = np.zeros(3, dtype=np.float64)
        self.latest_measured_arm_momentum = np.zeros(3, dtype=np.float64)
        self.latest_solver_statuses = {}
        self.latest_plan_duration = 0.0
        self.latest_response_status = 'idle'
        self.latest_response_summary = ''
        self.latest_best_effort_used = False
        execution_cfg = self.zmp_config.get('execution', {})
        self.limit_ddp_velocity = bool(
            execution_cfg.get('limit_ddp_velocity', False)
        )
        self.latest_raw_command_norm = 0.0
        self.latest_applied_command_norm = 0.0
        self.latest_raw_max_arm_velocity = {'left': 0.0, 'right': 0.0}
        self.latest_applied_max_arm_velocity = {'left': 0.0, 'right': 0.0}
        self.latest_integrated_q_delta = {'left': 0.0, 'right': 0.0}
        self.latest_actuator_state = self.actuator.state
        self.latest_actuator_plan_index = self.actuator.plan_index

    def control_step(self, com=False):
        '''Run one ZMP balance control tick'''
        del com
        if not self.zmp_enabled:
            self.update_robot_model()
            return np.zeros(self.robot_model.model_body.nv, dtype=np.float64)

        self.update_robot_model()
        self.update_ik_solver()
        self._update_balance_state()
        raw_command = self.actuator.step()
        if self.limit_ddp_velocity:
            applied_command = self._limit_joint_vel(raw_command)
        else:
            applied_command = raw_command
        self._update_execution_diagnostics(raw_command, applied_command)
        if np.linalg.norm(applied_command) <= 1e-12:
            return applied_command
        return self._apply_velocity_command(applied_command)

    def control_step_reduced(self, com=False):
        '''Run one ZMP balance control tick for reduced-control callers'''
        return self.control_step(com=com)

    def set_reflex_output_enabled(self, enabled):
        '''Enable or suppress reflex actuation while keeping observation active'''
        self.reflex_output_enabled = bool(enabled)
        if not self.reflex_output_enabled:
            self.actuator.reset_response()

    def reset_balance_reference(self):
        '''Reset quiet calibration before a new balance experiment'''
        self.observer.reset_calibration()
        self.detector.reset()
        self.actuator.reset_response()
        self.latest_perturbation_state = None
        self.latest_target_momentum = np.zeros(3, dtype=np.float64)
        self.latest_raw_target_momentum = np.zeros(3, dtype=np.float64)
        self.latest_arm_targets = {}
        self.latest_arm_achieved_momentum = {}
        self.latest_combined_achieved_momentum = np.zeros(3, dtype=np.float64)
        self.latest_pre_limit_momentum = np.zeros(3, dtype=np.float64)
        self.latest_post_limit_momentum = np.zeros(3, dtype=np.float64)
        self.latest_measured_arm_momentum = np.zeros(3, dtype=np.float64)
        self.latest_raw_command_norm = 0.0
        self.latest_applied_command_norm = 0.0
        self.latest_response_status = 'idle'
        self.latest_response_summary = ''

    def _update_balance_state(self):
        freeze_reference = self._reference_should_freeze()
        balance_state = self.observer.observe(
            freeze_center_reference=freeze_reference,
        )
        perturbation_state = self.detector.update(balance_state)
        target_momentum = self.target_estimator.estimate(
            balance_state,
            perturbation_state,
        )

        active_arms = self._balance_arms()
        arm_targets = self.allocator.allocate(target_momentum, active_arms)
        if self.reflex_output_enabled and self.actuator.execution_mode == 'direct':
            self.actuator.update_direct_response(
                arm_targets,
                perturbation_state,
                balance_state,
            )
        elif self.reflex_output_enabled and self.actuator.can_start_response(
                perturbation_state,
                target_momentum):
            self.actuator.maybe_start_response(
                arm_targets,
                perturbation_state,
            )

        self.latest_balance_state = balance_state
        self.latest_perturbation_state = perturbation_state
        self.latest_target_momentum = np.copy(target_momentum)
        self.latest_raw_target_momentum = np.copy(
            self.target_estimator.latest_raw_target
        )
        self.latest_arm_targets = {
            target.arm: np.copy(target.target_momentum)
            for target in arm_targets
        }
        self.latest_arm_achieved_momentum = {
            arm: np.copy(momentum)
            for arm, momentum in self.actuator.latest_per_arm_achieved.items()
        }
        self.latest_combined_achieved_momentum = np.copy(
            self.actuator.latest_combined_achieved
        )
        self.latest_solver_statuses = dict(
            self.actuator.latest_solver_statuses
        )
        self.latest_plan_duration = float(self.actuator.latest_plan_duration)
        self.latest_response_status = getattr(
            self.actuator,
            'latest_response_status',
            getattr(self, 'latest_response_status', 'idle'),
        )
        self.latest_response_summary = getattr(
            self.actuator,
            'latest_response_summary',
            getattr(self, 'latest_response_summary', ''),
        )
        self.latest_best_effort_used = self.actuator.latest_best_effort_used

    def _update_execution_diagnostics(self, raw_command, applied_command):
        self.latest_raw_command_norm = float(np.linalg.norm(raw_command))
        self.latest_applied_command_norm = float(
            np.linalg.norm(applied_command)
        )
        self.latest_raw_max_arm_velocity = {
            'left': self._max_abs(raw_command, LEFT_ARM_INDEX),
            'right': self._max_abs(raw_command, RIGHT_ARM_INDEX),
        }
        self.latest_applied_max_arm_velocity = {
            'left': self._max_abs(applied_command, LEFT_ARM_INDEX),
            'right': self._max_abs(applied_command, RIGHT_ARM_INDEX),
        }
        self.latest_integrated_q_delta = {
            'left': self.dt * self._max_abs(applied_command, LEFT_ARM_INDEX),
            'right': self.dt * self._max_abs(applied_command, RIGHT_ARM_INDEX),
        }
        self.latest_actuator_state = self.actuator.state
        self.latest_actuator_plan_index = self.actuator.plan_index
        self.latest_response_status = getattr(
            self.actuator,
            'latest_response_status',
            getattr(self, 'latest_response_status', 'idle'),
        )
        self.latest_response_summary = getattr(
            self.actuator,
            'latest_response_summary',
            getattr(self, 'latest_response_summary', ''),
        )
        self.latest_pre_limit_momentum = self._command_momentum(raw_command)
        self.latest_post_limit_momentum = self._command_momentum(applied_command)
        state = getattr(self.robot_model, 'state', {})
        self.latest_measured_arm_momentum = self._command_momentum(
            state.get(
                'dq',
                np.zeros_like(applied_command),
            ),
        )

    @staticmethod
    def _max_abs(values, indices):
        values = np.asarray(values, dtype=np.float64)
        if values.size == 0:
            return 0.0
        return float(np.max(np.abs(values[indices])))

    def _command_momentum(self, velocity):
        get_momentum = getattr(
            self.robot_model,
            'get_angular_centroidal_momentum',
            None,
        )
        if get_momentum is None:
            return np.zeros(3, dtype=np.float64)
        joint_ids = list(LEFT_ARM_INDEX) + list(RIGHT_ARM_INDEX)
        state = getattr(self.robot_model, 'state', {})
        return np.asarray(
            get_momentum(state['q'], velocity, joint_ids),
            dtype=np.float64,
        )

    def _reference_should_freeze(self):
        if self.latest_perturbation_state is None:
            return bool(self.detector.entering)
        return bool(
            self.latest_perturbation_state.active
            or self.latest_perturbation_state.raw_perturbed
            or self.detector.entering
            or self.actuator.state != 'idle'
        )

    def _balance_arms(self):
        mode = self.zmp_config.get('mode', 'both_arm_balance')
        if mode == 'both_arm_balance':
            return ['left', 'right']
        if mode == 'single_arm_task_balance':
            return [self.zmp_config.get('assist_arm', 'right')]
        raise ValueError(f'unsupported zmp.mode: {mode}')


__all__ = ['ZmpController']
