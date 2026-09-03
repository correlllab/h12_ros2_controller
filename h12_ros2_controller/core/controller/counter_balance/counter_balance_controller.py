import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    CounterCommandContext,
    Frozen3CNominalInput,
    Frozen3CPlannerConfig,
    Frozen3CVelocitySolve,
    plan_frozen_3c_velocity,
)
from h12_ros2_controller.core.controller.counter_balance.objective import (
    CounterVelocityBoundsError,
    reaction_targets,
    solve_bounded_velocity,
)
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.support_region import support_rectangle
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import (
    BODY_JOINTS,
    LEFT_ARM_INDEX,
    RIGHT_ARM_INDEX,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS


class CounterBalanceController(FrameController):
    '''Reactive counter-arm controller for moving-arm trajectories'''

    def __init__(self, urdf_path: str, urdf_sphere_path: str,
                 srdf_sphere_path: str, moving_arm: str = None,
                 init: bool = True, handless: bool = False,
                 visualize: bool = False, config: dict = None):
        if moving_arm not in (None, 'left', 'right'):
            raise ValueError('moving_arm must be "left" or "right"')
        resolved_config = (
            load_controller_config() if config is None else config
        )
        self._load_reactive_config(
            resolved_config.get('reactive_counter_balance', {})
        )
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            init=init,
            handless=handless,
            visualize=visualize,
            config=resolved_config,
        )

        self.arm_ids = list(LEFT_ARM_INDEX) + list(RIGHT_ARM_INDEX)
        self.motor_q_indices, self.motor_v_indices = self._body_indices()
        self.moving_arm = None
        self.counter_arm = None
        self.moving_ids = []
        self.counter_arm_ids = []
        self.counter_ids = []
        self.counter_wrist_ids = []
        self.moving_local = np.array([], dtype=int)
        self.counter_local = np.array([], dtype=int)
        self.counter_active_local = np.array([], dtype=int)
        self.counter_wrist_local = np.array([], dtype=int)
        self.moving_v_indices = np.array([], dtype=int)
        self.counter_v_indices = np.array([], dtype=int)
        if moving_arm is not None:
            self._set_arm_ownership(moving_arm)
        self._reactive_data = self.robot_model.model_body.createData()
        self.backtrack_scales = (1.0, 0.5, 0.25, 0.125, 0.0)
        self._reference_captured = False
        self.q_counter_ref = None
        self.counter_wrist_ref = None
        self.com_offset_ref = None
        self.tilt_reference = None
        self._latched_activation = 0.0
        self._counter_q_command = None
        self._frame_balance_scale = 1.0
        self._reset_diagnostics()

    def set_frame_target(self, frame_name, target, task_name='moving_arm_task'):
        '''Configure one moving-arm frame target and infer arm ownership'''
        moving_arm = self._arm_for_frame(frame_name)
        self.clear_frame_tasks()
        self.add_frame_task(task_name, frame_name, np.asarray(target))
        self._set_arm_ownership(moving_arm)

    def set_named_target(self, config_name, tolerance=1e-6):
        '''Configure one single-arm named target and infer arm ownership'''
        if config_name not in NAMED_CONFIGS:
            raise ValueError(f'Unknown named configuration: {config_name}')
        target = np.asarray(NAMED_CONFIGS[config_name], dtype=np.float64)
        reference = np.asarray(
            NAMED_CONFIGS['home'],
            dtype=np.float64,
        )
        changed = (
            np.max(np.abs(target[:7] - reference[:7])) > tolerance,
            np.max(np.abs(target[7:] - reference[7:])) > tolerance,
        )
        if changed == (True, False):
            moving_arm = 'left'
        elif changed == (False, True):
            moving_arm = 'right'
        else:
            raise ValueError(
                'named target must change exactly one moving arm',
            )
        self._set_arm_ownership(moving_arm)
        self.set_reduced_configuration_target(target)
        return target

    def _arm_for_frame(self, frame_name):
        support = self._reduced_support_mask(frame_name)
        left = bool(np.any(support[:7]))
        right = bool(np.any(support[7:14]))
        if left == right:
            raise ValueError('frame must depend on exactly one moving arm')
        return 'left' if left else 'right'

    def _set_arm_ownership(self, moving_arm):
        if moving_arm not in ('left', 'right'):
            raise ValueError('moving_arm must be "left" or "right"')
        self.moving_arm = moving_arm
        self.counter_arm = 'right' if moving_arm == 'left' else 'left'
        self.moving_ids = list(
            LEFT_ARM_INDEX if moving_arm == 'left' else RIGHT_ARM_INDEX
        )
        self.counter_arm_ids = list(
            RIGHT_ARM_INDEX if moving_arm == 'left' else LEFT_ARM_INDEX
        )
        self.counter_ids = self.counter_arm_ids[:4]
        self.counter_wrist_ids = self.counter_arm_ids[4:]
        self.moving_local = self._arm_local_indices(self.moving_ids)
        self.counter_local = self._arm_local_indices(self.counter_arm_ids)
        self.counter_active_local = self.counter_local[:4]
        self.counter_wrist_local = self.counter_local[4:]
        self.moving_v_indices = self.motor_v_indices[self.moving_ids]
        self.counter_v_indices = self.motor_v_indices[self.counter_ids]
        self._reference_captured = False
        self.q_counter_ref = None
        self.counter_wrist_ref = None
        self.com_offset_ref = None
        self.tilt_reference = None
        self._latched_activation = 0.0
        self._counter_q_command = None
        self._frame_balance_scale = 1.0

    def control_configuration_step(self, moving_q_target_14,
                                   moving_dq_target_14,
                                   balance_scale=1.0):
        '''Apply one 50 Hz 14-arm configuration trajectory sample'''
        self._require_arm_ownership()
        balance_scale = self._validated_balance_scale(balance_scale)
        self._frame_balance_scale = balance_scale
        self._reset_diagnostics(balance_scale)
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.latest_status = 'estopped'
            return np.zeros(14, dtype=np.float64)

        q_target = self._as_arm_sample(
            moving_q_target_14,
            'moving_q_target_14',
        )
        dq_target = self._as_arm_sample(
            moving_dq_target_14,
            'moving_dq_target_14',
        )
        motor_q, _ = self._measured_motor_state()
        measured_arm_q = np.copy(motor_q[self.arm_ids])
        arm_q, arm_dq, input_valid = self._moving_sample(
            q_target,
            dq_target,
            measured_arm_q,
        )
        if not input_valid:
            raise ValueError('moving-arm command must be finite')
        try:
            self.update_robot_model()
        except Exception:
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'model_update_failure',
            )
        motor_q, _ = self._measured_motor_state()
        measured_arm_q = np.copy(motor_q[self.arm_ids])
        arm_q, arm_dq, input_valid = self._moving_sample(
            q_target,
            dq_target,
            measured_arm_q,
        )
        if not input_valid:
            raise ValueError('moving-arm command must be finite')

        counter_q = np.copy(measured_arm_q[self.counter_active_local])
        if self.counter_wrist_ref is not None:
            arm_q[self.counter_wrist_local] = self.counter_wrist_ref
        try:
            support, com, com_jacobian, momentum_map, torso_rotation = (
                self._model_terms(motor_q)
            )
        except Exception:
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'model_failure',
            )

        self.latest_support_valid = bool(support.valid)
        self.latest_support_error = support.invalid_reason
        self.latest_com = np.copy(com)
        if not self._reference_captured:
            self._capture_reference(measured_arm_q, support, com)
        if not support.valid or not self._reference_captured:
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'invalid_support',
            )

        activation_scale = self._balance_activation()
        balance_scale *= activation_scale
        self.latest_activation_scale = activation_scale
        self.latest_balance_scale = balance_scale

        com_target = support.center + self.com_offset_ref
        com_error = np.asarray(com[:2] - com_target, dtype=np.float64)
        gyro, gyro_available = self._torso_gyro(torso_rotation)
        self.latest_com_target = np.copy(com_target)
        self.latest_com_error = np.copy(com_error)
        self.latest_gyro_available = gyro_available
        moving_dq = arm_dq[self.moving_local]
        com_moving = com_jacobian[:2, self.moving_v_indices]
        com_counter = com_jacobian[:2, self.counter_v_indices]
        momentum_moving = momentum_map[:2, self.moving_v_indices]
        momentum_counter = momentum_map[:2, self.counter_v_indices]
        com_rhs_offset, momentum_rhs_offset = self._reaction_target_offsets(
            com_moving,
            momentum_moving,
            moving_dq,
            com_error,
            gyro,
            balance_scale,
        )
        lower, upper = self._counter_velocity_bounds(counter_q)
        lower, upper = self._counter_excursion_bounds(
            counter_q,
            lower,
            upper,
        )
        planner_input = Frozen3CNominalInput(
            moving_dq=np.copy(moving_dq),
            counter_q=np.copy(counter_q),
            counter_q_ref=np.copy(self.q_counter_ref),
            com_error=np.copy(com_error),
            gyro=np.copy(gyro),
            com_moving=np.copy(com_moving),
            com_counter=np.copy(com_counter),
            momentum_moving=np.copy(momentum_moving),
            momentum_counter=np.copy(momentum_counter),
            balance_scale=balance_scale,
            lower=np.copy(lower),
            upper=np.copy(upper),
            com_rhs_offset=np.copy(com_rhs_offset),
            momentum_rhs_offset=np.copy(momentum_rhs_offset),
        )
        try:
            nominal = self._plan_counter_velocity(planner_input)
        except CounterVelocityBoundsError:
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'counter_bounds_infeasible',
            )
        except Exception:
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'solver_failure',
            )
        self._commit_nominal_plan(nominal)
        if not nominal.accepted:
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'solver_failure',
            )
        context = CounterCommandContext(
            motor_q=np.copy(motor_q),
            arm_q=np.copy(arm_q),
            arm_dq=np.copy(arm_dq),
            counter_q=np.copy(counter_q),
            moving_dq=np.copy(moving_dq),
            com_error=np.copy(com_error),
            gyro=np.copy(gyro),
            com_moving=np.copy(com_moving),
            com_counter=np.copy(com_counter),
            momentum_moving=np.copy(momentum_moving),
            momentum_counter=np.copy(momentum_counter),
            balance_scale=balance_scale,
        )
        requested = self._select_requested_counter_velocity(context, nominal)
        requested = np.asarray(requested, dtype=np.float64)
        if requested.shape != (4,) or not np.all(np.isfinite(requested)):
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'nonfinite_solution',
            )
        return self._finalize_counter_velocity(context, requested)

    def _reaction_target_offsets(
            self, com_moving, momentum_moving, moving_dq,
            com_error, gyro, balance_scale):
        return (
            np.zeros(2, dtype=np.float64),
            np.zeros(2, dtype=np.float64),
        )

    def _plan_counter_velocity(self, inputs):
        return plan_frozen_3c_velocity(
            inputs,
            Frozen3CPlannerConfig(
                com_gain=self.com_gain,
                gyro_gain=self.gyro_gain,
                posture_gain=self.posture_gain,
            ),
            self._isolated_velocity_solve,
        )

    def _isolated_velocity_solve(
            self, com_counter, momentum_counter, com_rhs, momentum_rhs,
            posture_target, lower, upper, balance_scale):
        requested = self._solve_bounded_velocity(
            com_counter,
            momentum_counter,
            com_rhs,
            momentum_rhs,
            posture_target,
            lower,
            upper,
            balance_scale=balance_scale,
        )
        return Frozen3CVelocitySolve(
            requested_counter_dq=requested,
            accepted=True,
        )

    def _commit_nominal_plan(self, nominal):
        pass

    def _select_requested_counter_velocity(self, context, nominal):
        return nominal.requested_counter_dq

    def _finalize_counter_velocity(self, context, requested):
        motor_q = context.motor_q
        arm_q = context.arm_q
        arm_dq = context.arm_dq
        counter_q = context.counter_q

        self.latest_requested_counter_dq = np.copy(requested)
        result, failure_status = self._backtrack_counter(
            motor_q, arm_q, arm_dq, counter_q, requested,
        )
        if result is None:
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                failure_status or 'counter_candidate_invalid',
            )

        candidate_q, command_dq, scale = result
        applied = scale * requested
        self._counter_q_command = np.copy(
            candidate_q[self.counter_active_local]
        )
        self.latest_applied_counter_dq = np.copy(applied)
        self.latest_backtrack_scale = scale
        self.latest_collision_rejection = scale < 1.0
        self.latest_status = (
            'collision_backtracked' if scale < 1.0 else 'solved'
        )
        self._set_predicted_residuals(
            context.com_moving,
            context.com_counter,
            context.momentum_moving,
            context.momentum_counter,
            context.moving_dq,
            applied,
            context.com_error,
            context.gyro,
            context.balance_scale,
        )
        if not self._publish_arm_command(candidate_q, command_dq, motor_q):
            return np.zeros(14, dtype=np.float64)
        return command_dq

    def _publish_position_command(self, q, dq, tau):
        '''Route inherited frame tracking through the counter-arm overlay'''
        q = np.asarray(q, dtype=np.float64)
        dq = np.asarray(dq, dtype=np.float64)
        self.control_configuration_step(
            q[self.arm_ids],
            dq[self.arm_ids],
            balance_scale=self._frame_balance_scale,
        )

    def _steady_state_control_ids(self):
        '''Use only moving-arm joints for I-stage clipping and convergence'''
        self._require_arm_ownership()
        return self.moving_ids

    def _require_arm_ownership(self):
        if self.moving_arm not in ('left', 'right'):
            raise RuntimeError('moving arm is not configured')

    def diagnostics(self):
        '''Return serializable reactive controller diagnostics'''
        return {
            'status': self.latest_status,
            'moving_arm': self.moving_arm,
            'counter_arm': self.counter_arm,
            'balance_scale': float(self.latest_balance_scale),
            'activation_scale': float(self.latest_activation_scale),
            'requested_counter_dq': (
                self.latest_requested_counter_dq.tolist()
            ),
            'applied_counter_dq': self.latest_applied_counter_dq.tolist(),
            'predicted_com_residual': (
                self.latest_predicted_com_residual.tolist()
                if self.latest_predicted_com_residual is not None else None
            ),
            'predicted_momentum_residual': (
                self.latest_predicted_momentum_residual.tolist()
                if self.latest_predicted_momentum_residual is not None else None
            ),
            'com_velocity_scale': float(self.com_velocity_scale),
            'momentum_scale': float(self.momentum_scale),
            'posture_velocity_scale': float(self.posture_velocity_scale),
            'backtrack_scale': float(self.latest_backtrack_scale),
            'clipped': bool(self.latest_clipped),
            'collision_rejection': bool(self.latest_collision_rejection),
            'moving_position_command_error': float(
                self.latest_moving_position_command_error
            ),
            'moving_velocity_command_error': float(
                self.latest_moving_velocity_command_error
            ),
            'support_valid': bool(self.latest_support_valid),
            'support_error': self.latest_support_error or None,
            'com': (
                self.latest_com.tolist()
                if self.latest_com is not None else None
            ),
            'com_target': (
                self.latest_com_target.tolist()
                if self.latest_com_target is not None else None
            ),
            'com_error': (
                self.latest_com_error.tolist()
                if self.latest_com_error is not None else None
            ),
            'gyro_available': bool(self.latest_gyro_available),
            'estopped': bool(
                getattr(self.low_cmd_handler, '_estopped', False)
            ),
            'published': bool(self.latest_published),
        }

    def _load_reactive_config(self, cfg):
        if not isinstance(cfg, dict):
            raise ValueError('reactive_counter_balance must be a mapping')
        weights = cfg.get('weights', {})
        gains = cfg.get('gains', {})
        if not isinstance(weights, dict):
            raise ValueError('reactive_counter_balance.weights must be a mapping')
        if not isinstance(gains, dict):
            raise ValueError('reactive_counter_balance.gains must be a mapping')
        self.com_weight = self._finite_scalar(
            weights.get('com', 1.0), 'weights.com', minimum=0.0
        )
        self.momentum_weight = self._finite_scalar(
            weights.get('momentum', 1.0),
            'weights.momentum',
            minimum=0.0,
        )
        self.posture_weight = self._finite_scalar(
            weights.get('posture', 0.05),
            'weights.posture',
            minimum=0.0,
        )
        self.com_gain = self._finite_scalar(
            gains.get('com', 2.0), 'gains.com', minimum=0.0
        )
        self.gyro_gain = self._finite_scalar(
            gains.get('gyro', 0.1), 'gains.gyro', minimum=0.0
        )
        self.posture_gain = self._finite_scalar(
            gains.get('posture', 0.5),
            'gains.posture',
            minimum=0.0,
        )
        self.damping = self._positive_scalar(
            cfg.get('damping', 1e-4),
            'damping',
        )
        self.com_velocity_scale = self._positive_scalar(
            cfg.get('com_velocity_scale', 0.1),
            'com_velocity_scale',
        )
        self.momentum_scale = self._positive_scalar(
            cfg.get('momentum_scale', 1.0),
            'momentum_scale',
        )
        self.posture_velocity_scale = self._positive_scalar(
            cfg.get('posture_velocity_scale', 1.0),
            'posture_velocity_scale',
        )
        activation = cfg.get('activation', {})
        if not isinstance(activation, dict):
            raise ValueError(
                'reactive_counter_balance.activation must be a mapping'
            )
        self.activation_tilt_threshold = self._finite_scalar(
            activation.get('tilt_threshold', 0.0),
            'activation.tilt_threshold',
            minimum=0.0,
        )
        self.activation_tilt_full_scale = self._finite_scalar(
            activation.get(
                'tilt_full_scale', self.activation_tilt_threshold,
            ),
            'activation.tilt_full_scale',
            minimum=self.activation_tilt_threshold,
        )
        if (
            self.activation_tilt_threshold > 0.0
            and self.activation_tilt_full_scale
            <= self.activation_tilt_threshold
        ):
            raise ValueError(
                'reactive_counter_balance.activation.tilt_full_scale must '
                'exceed tilt_threshold'
            )
        self.activation_latch = bool(activation.get('latch', False))
        support = cfg.get('support_geometry', {})
        if not isinstance(support, dict):
            raise ValueError(
                'reactive_counter_balance.support_geometry must be a mapping'
            )
        self.support_geometry = {
            name: self._positive_scalar(
                support.get(name, default),
                f'support_geometry.{name}',
            )
            for name, default in (
                ('front', 0.174),
                ('rear', 0.086),
                ('half_width', 0.043),
                ('max_yaw_divergence', np.deg2rad(20.0)),
            )
        }
        max_velocity = np.asarray(
            cfg.get('max_velocity', 1.0),
            dtype=np.float64,
        )
        if max_velocity.ndim == 0:
            max_velocity = np.full(4, float(max_velocity))
        max_velocity = max_velocity.reshape(-1)
        if (
            max_velocity.shape != (4,)
            or not np.all(np.isfinite(max_velocity))
            or np.any(max_velocity <= 0.0)
        ):
            raise ValueError(
                'reactive_counter_balance.max_velocity must contain four '
                'finite positive values'
            )
        self.max_velocity = max_velocity
        max_excursion = np.asarray(
            cfg.get('max_excursion', np.inf),
            dtype=np.float64,
        )
        if max_excursion.ndim == 0:
            max_excursion = np.full(4, float(max_excursion))
        max_excursion = max_excursion.reshape(-1)
        if (
            max_excursion.shape != (4,)
            or np.any(np.isnan(max_excursion))
            or np.any(max_excursion <= 0.0)
        ):
            raise ValueError(
                'reactive_counter_balance.max_excursion must contain four '
                'positive values'
            )
        self.max_excursion = max_excursion

    @classmethod
    def _positive_scalar(cls, value, name):
        value = cls._finite_scalar(value, name, minimum=0.0)
        if value == 0.0:
            raise ValueError(
                f'reactive_counter_balance.{name} must be positive'
            )
        return value

    @staticmethod
    def _finite_scalar(value, name, minimum):
        try:
            value = float(value)
        except (TypeError, ValueError) as err:
            raise ValueError(
                f'reactive_counter_balance.{name} must be a finite scalar'
            ) from err
        if not np.isfinite(value) or value < minimum:
            raise ValueError(
                f'reactive_counter_balance.{name} must be finite and '
                f'at least {minimum}'
            )
        return value

    @staticmethod
    def _validated_balance_scale(value):
        try:
            value = float(value)
        except (TypeError, ValueError) as err:
            raise ValueError('balance_scale must be a finite scalar') from err
        if not np.isfinite(value) or value < 0.0:
            raise ValueError('balance_scale must be finite and nonnegative')
        return value

    @staticmethod
    def _as_arm_sample(value, name):
        value = np.asarray(value, dtype=np.float64)
        if value.shape != (14,):
            raise ValueError(f'{name} must have shape (14,)')
        return value

    def _arm_local_indices(self, motor_ids):
        local = {motor_id: index for index, motor_id in enumerate(self.arm_ids)}
        return np.asarray([local[motor_id] for motor_id in motor_ids], dtype=int)

    def _body_indices(self):
        model = self.robot_model.model_body
        q_indices = np.empty(len(BODY_JOINTS), dtype=int)
        v_indices = np.empty(len(BODY_JOINTS), dtype=int)
        for motor_id, joint_name in enumerate(BODY_JOINTS):
            joint_id = model.getJointId(joint_name)
            joint = model.joints[joint_id]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError('Body motor joints must be one degree of freedom')
            q_indices[motor_id] = joint.idx_q
            v_indices[motor_id] = joint.idx_v
        if (
            len(np.unique(q_indices)) != len(BODY_JOINTS)
            or len(np.unique(v_indices)) != len(BODY_JOINTS)
        ):
            raise ValueError('Body motor joint mappings must be unique')
        return q_indices, v_indices

    def _measured_motor_state(self):
        state = self.robot_model.state
        q = np.asarray(state.get('q'), dtype=np.float64)
        dq = np.asarray(state.get('dq'), dtype=np.float64)
        fallback_q = np.asarray(self.low_cmd_handler.q_cmd, dtype=np.float64)
        if q.shape != (27,):
            q = fallback_q
        if fallback_q.shape != (27,):
            fallback_q = np.zeros(27, dtype=np.float64)
        if dq.shape != (27,):
            dq = np.zeros(27, dtype=np.float64)
        q = np.where(np.isfinite(q), q, fallback_q)
        q = np.where(np.isfinite(q), q, 0.0)
        dq = np.where(np.isfinite(dq), dq, 0.0)
        return np.copy(q), np.copy(dq)

    def _moving_sample(self, q_target, dq_target, measured_arm_q):
        arm_q = np.copy(measured_arm_q)
        arm_dq = np.zeros(14, dtype=np.float64)
        q_values = q_target[self.moving_local]
        dq_values = dq_target[self.moving_local]
        valid = bool(
            np.all(np.isfinite(q_values))
            and np.all(np.isfinite(dq_values))
        )
        if valid:
            arm_q[self.moving_local] = q_values
            arm_dq[self.moving_local] = dq_values
        return arm_q, arm_dq, valid

    def _capture_reference(self, measured_arm_q, support, com):
        self.q_counter_ref = np.copy(
            measured_arm_q[self.counter_active_local]
        )
        self.counter_wrist_ref = np.copy(
            measured_arm_q[self.counter_wrist_local]
        )
        self._counter_q_command = np.copy(self.q_counter_ref)
        if (
            support.valid
            and com.shape[0] >= 2
            and np.all(np.isfinite(com))
        ):
            self.com_offset_ref = np.copy(com[:2] - support.center)
            self.tilt_reference = self._imu_tilt()
            self._reference_captured = True

    def _body_q(self, motor_q):
        model = self.robot_model.model_body
        body_q = np.zeros(model.nq, dtype=np.float64)
        body_q[self.motor_q_indices] = motor_q
        return body_q

    def _model_terms(self, motor_q):
        model = self.robot_model.model_body
        data = self._reactive_data
        body_q = self._body_q(motor_q)
        pin.forwardKinematics(model, data, body_q)
        pin.updateFramePlacements(model, data)
        support = self._support_rectangle(data)
        torso_id = model.getFrameId('torso_link')
        torso_rotation = np.asarray(data.oMf[torso_id].rotation).copy()
        com = np.asarray(pin.centerOfMass(model, data, body_q))
        com_jacobian = np.asarray(
            pin.jacobianCenterOfMass(model, data, body_q)
        )
        pin.computeCentroidalMap(model, data, body_q)
        momentum_map = np.asarray(data.Ag[3:, :])
        if (
            com.shape[0] < 3
            or com_jacobian.shape != (3, model.nv)
            or momentum_map.shape != (3, model.nv)
            or torso_rotation.shape != (3, 3)
            or not np.all(np.isfinite(com[:3]))
            or not np.all(np.isfinite(com_jacobian))
            or not np.all(np.isfinite(momentum_map))
            or not np.all(np.isfinite(torso_rotation))
        ):
            raise ValueError('Live body model returned nonfinite dynamics')
        return (
            support,
            com[:3],
            com_jacobian,
            momentum_map,
            torso_rotation,
        )

    def _support_rectangle(self, data):
        model = self.robot_model.model_body
        left_id = model.getFrameId('left_ankle_roll_link')
        right_id = model.getFrameId('right_ankle_roll_link')
        return support_rectangle(
            data.oMf[left_id].homogeneous,
            data.oMf[right_id].homogeneous,
            **self.support_geometry,
        )

    def _torso_gyro(self, torso_rotation):
        state = self.robot_model.state
        sources = (state.get('imu_state'), self.robot_model)
        for source in sources:
            if source is None:
                continue
            for name in (
                'get_gyroscope', 'gyroscope', 'angular_velocity', 'gyro',
            ):
                value = getattr(source, name, None)
                if value is None:
                    continue
                try:
                    value = value() if callable(value) else value
                    value = np.asarray(value, dtype=np.float64).reshape(-1)
                except Exception:
                    continue
                if value.size >= 3 and np.all(np.isfinite(value[:3])):
                    rotation = np.asarray(torso_rotation, dtype=np.float64)
                    if rotation.shape != (3, 3) or not np.all(
                        np.isfinite(rotation)
                    ):
                        break
                    return rotation @ value[:3], True
        return np.zeros(3, dtype=np.float64), False

    def _imu_tilt(self):
        imu_state = self.robot_model.state.get('imu_state')
        quaternion = getattr(imu_state, 'quaternion', None)
        if quaternion is None:
            return np.zeros(2, dtype=np.float64)
        quaternion = np.asarray(quaternion, dtype=np.float64).reshape(-1)
        if quaternion.size != 4 or not np.all(np.isfinite(quaternion)):
            return np.zeros(2, dtype=np.float64)
        w, x, y, z = quaternion
        roll = np.arctan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )
        pitch = np.arcsin(np.clip(
            2.0 * (w * y - z * x), -1.0, 1.0,
        ))
        return np.array([roll, pitch], dtype=np.float64)

    def _balance_activation(self):
        threshold = self.activation_tilt_threshold
        if threshold <= 0.0:
            return 1.0
        if self.tilt_reference is None:
            return 0.0
        tilt_error = float(np.linalg.norm(
            self._imu_tilt() - self.tilt_reference,
        ))
        span = self.activation_tilt_full_scale - threshold
        scale = float(np.clip((tilt_error - threshold) / span, 0.0, 1.0))
        if self.activation_latch:
            self._latched_activation = max(self._latched_activation, scale)
            return self._latched_activation
        return scale

    def _reaction_targets(self, com_moving, momentum_moving, moving_dq,
                          com_error, gyro, balance_scale):
        return reaction_targets(
            com_moving,
            momentum_moving,
            moving_dq,
            com_error,
            gyro,
            balance_scale,
            self.com_gain,
            self.gyro_gain,
        )

    def _solve_bounded_velocity(self, com_counter, momentum_counter,
                                com_rhs, momentum_rhs, posture_target,
                                lower, upper, balance_scale=1.0):
        return solve_bounded_velocity(
            com_counter,
            momentum_counter,
            com_rhs,
            momentum_rhs,
            posture_target,
            lower,
            upper,
            balance_scale,
            self.com_weight,
            self.momentum_weight,
            self.posture_weight,
            self.damping,
            self.com_velocity_scale,
            self.momentum_scale,
            self.posture_velocity_scale,
        )

    def _counter_velocity_bounds(self, counter_q):
        lower, upper = self._arm_position_limits()
        velocity = self._arm_velocity_limits()[self.counter_active_local]
        velocity = np.minimum(velocity, self.max_velocity)
        q_lower = lower[self.counter_active_local]
        q_upper = upper[self.counter_active_local]
        return (
            np.maximum(-velocity, (q_lower - counter_q) / self.dt),
            np.minimum(velocity, (q_upper - counter_q) / self.dt),
        )

    def _counter_excursion_bounds(self, counter_q, lower, upper):
        if self.q_counter_ref is None:
            return lower, upper
        q_lower = self.q_counter_ref - self.max_excursion
        q_upper = self.q_counter_ref + self.max_excursion
        return (
            np.maximum(lower, (q_lower - counter_q) / self.dt),
            np.minimum(upper, (q_upper - counter_q) / self.dt),
        )

    def _arm_position_limits(self):
        model = self.robot_model.model_body
        q_indices = self.motor_q_indices[self.arm_ids]
        lower = np.asarray(
            model.lowerPositionLimit,
            dtype=np.float64,
        )[q_indices].copy()
        upper = np.asarray(
            model.upperPositionLimit,
            dtype=np.float64,
        )[q_indices].copy()
        publisher = self.config.get('limits', {}).get('q_clip_limits')
        if publisher is not None:
            publisher = np.asarray(publisher, dtype=np.float64)[self.arm_ids]
            lower = np.maximum(lower, publisher[:, 0])
            upper = np.minimum(upper, publisher[:, 1])
        if (
            not np.all(np.isfinite(lower))
            or not np.all(np.isfinite(upper))
            or np.any(lower >= upper)
        ):
            raise ValueError('Effective arm position limits are invalid')
        return lower, upper

    def _arm_velocity_limits(self):
        model = self.robot_model.model_body
        v_indices = self.motor_v_indices[self.arm_ids]
        velocity = np.asarray(
            model.velocityLimit,
            dtype=np.float64,
        )[v_indices].copy()
        velocity = np.minimum(velocity, self.dq_lim)
        publisher = self.config.get('limits', {}).get('dq_clip_limits')
        if publisher is not None:
            publisher = np.asarray(publisher, dtype=np.float64)[self.arm_ids]
            velocity = np.minimum(velocity, publisher)
        if not np.all(np.isfinite(velocity)) or np.any(velocity <= 0.0):
            raise ValueError('Effective arm velocity limits are invalid')
        return velocity

    def _full_candidate(self, measured_motor_q, arm_q):
        candidate = np.copy(measured_motor_q)
        candidate[self.arm_ids] = arm_q
        return candidate

    def _candidate_status(self, candidate, prefix):
        candidate = np.asarray(candidate, dtype=np.float64)
        if candidate.shape != (27,) or not np.all(np.isfinite(candidate)):
            return f'{prefix}_nonfinite'
        try:
            within_limits = self.robot_model.check_within_limits(candidate)
        except Exception:
            return f'{prefix}_check_failure'
        if not within_limits:
            return f'{prefix}_out_of_limits'
        try:
            collision_free = self.robot_model.check_collision_free(candidate)
        except Exception:
            return f'{prefix}_check_failure'
        if not collision_free:
            return f'{prefix}_collision'
        return None

    def _backtrack_counter(self, motor_q, arm_q, arm_dq,
                           counter_q, requested):
        q_delta = self.dt * requested
        for scale in self.backtrack_scales:
            candidate_q = np.copy(arm_q)
            command_dq = np.copy(arm_dq)
            candidate_q[self.counter_active_local] = counter_q + scale * q_delta
            candidate_q[self.counter_wrist_local] = self.counter_wrist_ref
            command_dq[self.counter_active_local] = scale * requested
            command_dq[self.counter_wrist_local] = 0.0
            full_candidate = self._full_candidate(motor_q, candidate_q)
            status = self._candidate_status(
                full_candidate,
                prefix='counter_candidate',
            )
            if status is None:
                return (candidate_q, command_dq, scale), None
            if status == 'counter_candidate_check_failure':
                return None, status
        return None, 'counter_candidate_invalid'

    def _publish_counter_hold(self, motor_q, arm_q, arm_dq, status):
        measured_arm_q = motor_q[self.arm_ids]
        arm_q[self.counter_active_local] = (
            measured_arm_q[self.counter_active_local]
        )
        if self.counter_wrist_ref is not None:
            arm_q[self.counter_wrist_local] = self.counter_wrist_ref
        else:
            arm_q[self.counter_wrist_local] = (
                measured_arm_q[self.counter_wrist_local]
            )
        arm_dq[self.counter_local] = 0.0
        self._counter_q_command = np.copy(
            arm_q[self.counter_active_local]
        )
        self.latest_status = status
        if not self._publish_arm_command(arm_q, arm_dq, motor_q):
            return np.zeros(14, dtype=np.float64)
        return arm_dq

    def _publish_arm_command(self, arm_q, arm_dq, measured_motor_q):
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.latest_status = 'estopped'
            return False
        try:
            gravity = self.robot_model.dynamics.get_gravity_compensation(
                measured_motor_q
            )
            tau = np.asarray(gravity, dtype=np.float64)[self.arm_ids]
            if tau.shape != (14,) or not np.all(np.isfinite(tau)):
                raise ValueError('Nonfinite gravity compensation')
        except Exception:
            self.latest_status = 'gravity_failure'
            self.latest_applied_counter_dq[:] = 0.0
            self.latest_predicted_com_residual = None
            self.latest_predicted_momentum_residual = None
            measured_arm_q = measured_motor_q[self.arm_ids]
            arm_q = np.copy(arm_q)
            arm_dq = np.copy(arm_dq)
            arm_q[self.counter_local] = measured_arm_q[self.counter_local]
            arm_dq[self.counter_local] = 0.0
            return self._publish_without_gravity(arm_q, arm_dq)
        if not self._write_arm_command(arm_q, arm_dq, tau):
            return False
        self._record_moving_command_error(arm_q, arm_dq)
        self._record_applied_counter_dq()
        return True

    def _publish_without_gravity(self, arm_q, arm_dq):
        tau_cmd = getattr(self.low_cmd_handler, 'tau_cmd', None)
        tau = np.zeros(14, dtype=np.float64)
        if tau_cmd is not None:
            tau_cmd = np.asarray(tau_cmd, dtype=np.float64)
            if tau_cmd.shape == (27,) and np.all(np.isfinite(tau_cmd)):
                tau = np.copy(tau_cmd[self.arm_ids])
        published = self._write_arm_command(arm_q, arm_dq, tau)
        if published:
            self._record_moving_command_error(arm_q, arm_dq)
        return published

    def _write_arm_command(self, arm_q, arm_dq, tau):
        try:
            self.low_cmd_handler.set_joint_commands(
                q=np.asarray(arm_q, dtype=np.float64),
                dq=np.asarray(arm_dq, dtype=np.float64),
                tau=np.asarray(tau, dtype=np.float64),
                joint_ids=self.arm_ids,
            )
        except Exception:
            if getattr(self.low_cmd_handler, '_estopped', False):
                self.latest_status = 'estopped'
                return False
            raise
        self.latest_published = True
        return True

    def _record_applied_counter_dq(self):
        applied = getattr(self.low_cmd_handler, 'dq_cmd', None)
        if applied is None:
            return
        applied = np.asarray(applied, dtype=np.float64)
        if applied.shape == (27,) and np.all(np.isfinite(applied)):
            self.latest_applied_counter_dq = np.copy(
                applied[self.counter_ids]
            )

    def _record_moving_command_error(self, arm_q, arm_dq):
        q_cmd = np.asarray(
            getattr(self.low_cmd_handler, 'q_cmd', []),
            dtype=np.float64,
        )
        dq_cmd = np.asarray(
            getattr(self.low_cmd_handler, 'dq_cmd', []),
            dtype=np.float64,
        )
        if q_cmd.shape == (27,):
            self.latest_moving_position_command_error = float(np.max(np.abs(
                q_cmd[self.moving_ids] - arm_q[self.moving_local]
            )))
        if dq_cmd.shape == (27,):
            self.latest_moving_velocity_command_error = float(np.max(np.abs(
                dq_cmd[self.moving_ids] - arm_dq[self.moving_local]
            )))

    def _set_predicted_residuals(self, com_moving, com_counter,
                                 momentum_moving, momentum_counter,
                                 moving_dq, applied, com_error, gyro,
                                 balance_scale):
        self.latest_predicted_com_residual = (
            com_counter @ applied
            + balance_scale * (
                com_moving @ moving_dq + self.com_gain * com_error
            )
        ) / self.com_velocity_scale
        self.latest_predicted_momentum_residual = (
            momentum_counter @ applied
            + balance_scale * (
                momentum_moving @ moving_dq - self.gyro_gain * gyro[:2]
            )
        ) / self.momentum_scale

    def _reset_diagnostics(self, balance_scale=1.0):
        self.latest_status = 'idle'
        self.latest_balance_scale = float(balance_scale)
        self.latest_activation_scale = 1.0
        self.latest_requested_counter_dq = np.zeros(4, dtype=np.float64)
        self.latest_applied_counter_dq = np.zeros(4, dtype=np.float64)
        self.latest_predicted_com_residual = None
        self.latest_predicted_momentum_residual = None
        self.latest_backtrack_scale = 0.0
        self.latest_clipped = False
        self.latest_collision_rejection = False
        self.latest_moving_position_command_error = 0.0
        self.latest_moving_velocity_command_error = 0.0
        self.latest_support_valid = False
        self.latest_support_error = ''
        self.latest_com = None
        self.latest_com_target = None
        self.latest_com_error = None
        self.latest_gyro_available = False
        self.latest_published = False


ReactiveCounterBalanceController = CounterBalanceController
