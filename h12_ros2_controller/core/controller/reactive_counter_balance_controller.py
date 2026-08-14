import numpy as np
import pinocchio as pin
from scipy.optimize import lsq_linear

from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.core.support_region import support_rectangle
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import (
    BODY_JOINTS,
    LEFT_ARM_INDEX,
    RIGHT_ARM_INDEX,
)


class ReactiveCounterBalanceController(UpperController):
    '''Reactive opposite-arm controller for configuration trajectories'''

    def __init__(self, urdf_path: str, urdf_sphere_path: str,
                 srdf_sphere_path: str, moving_arm: str,
                 init: bool = True, handless: bool = False,
                 visualize: bool = False, config: dict = None):
        if moving_arm not in ('left', 'right'):
            raise ValueError('moving_arm must be "left" or "right"')
        resolved_config = (
            load_controller_config() if config is None else config
        )
        self._load_reactive_config(
            resolved_config.get('reactive_counter_balance', {})
        )
        self.moving_arm = moving_arm
        self.counter_arm = 'right' if moving_arm == 'left' else 'left'
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

        self.motor_q_indices, self.motor_v_indices = self._body_indices()
        self.moving_v_indices = self.motor_v_indices[self.moving_ids]
        self.counter_v_indices = self.motor_v_indices[self.counter_ids]
        self._reactive_data = self.robot_model.model_body.createData()
        self.backtrack_scales = (1.0, 0.5, 0.25, 0.125, 0.0)
        self._reference_captured = False
        self.q_counter_ref = None
        self.counter_wrist_ref = None
        self.com_offset_ref = None
        self._counter_q_command = None
        self._reset_diagnostics()

    def control_configuration_step(self, moving_q_target_14,
                                   moving_dq_target_14,
                                   balance_scale=1.0):
        '''Apply one 50 Hz 14-arm configuration trajectory sample'''
        balance_scale = self._validated_balance_scale(balance_scale)
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
        try:
            self.update_robot_model()
        except Exception:
            motor_q, _ = self._measured_motor_state()
            return self._publish_measured_hold(
                motor_q,
                'model_update_failure',
            )

        motor_q, _ = self._measured_motor_state()
        measured_arm_q = np.copy(motor_q[self.arm_ids])
        arm_q, arm_dq, input_valid = self._bounded_moving_sample(
            q_target,
            dq_target,
            measured_arm_q,
        )
        if not input_valid:
            return self._publish_measured_hold(motor_q, 'invalid_input')

        counter_q = np.copy(measured_arm_q[self.counter_active_local])
        if self.counter_wrist_ref is not None:
            arm_q[self.counter_wrist_local] = self.counter_wrist_ref
        moving_candidate = self._full_candidate(motor_q, arm_q)
        moving_status = self._candidate_status(
            moving_candidate,
            prefix='moving_candidate',
        )
        if moving_status is not None:
            return self._publish_measured_hold(motor_q, moving_status)

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
        com_rhs, momentum_rhs = self._reaction_targets(
            com_moving,
            momentum_moving,
            moving_dq,
            com_error,
            gyro,
            balance_scale,
        )

        posture_target = -self.posture_gain * (
            counter_q - self.q_counter_ref
        )
        lower, upper = self._counter_velocity_bounds(counter_q)
        lower, upper = self._counter_excursion_bounds(
            counter_q,
            lower,
            upper,
        )
        try:
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
        except Exception:
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'solver_failure',
            )
        if requested.shape != (4,) or not np.all(np.isfinite(requested)):
            return self._publish_counter_hold(
                motor_q,
                arm_q,
                arm_dq,
                'nonfinite_solution',
            )

        self.latest_requested_counter_dq = np.copy(requested)
        result, failure_status = self._backtrack_counter(
            motor_q,
            arm_q,
            arm_dq,
            counter_q,
            requested,
        )
        if result is None:
            return self._publish_measured_hold(
                motor_q,
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
            com_moving,
            com_counter,
            momentum_moving,
            momentum_counter,
            moving_dq,
            applied,
            com_error,
            gyro,
            balance_scale,
        )
        if not self._publish_arm_command(candidate_q, command_dq, motor_q):
            return np.zeros(14, dtype=np.float64)
        return command_dq

    def diagnostics(self):
        '''Return serializable reactive controller diagnostics'''
        return {
            'status': self.latest_status,
            'moving_arm': self.moving_arm,
            'counter_arm': self.counter_arm,
            'balance_scale': float(self.latest_balance_scale),
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

    def _bounded_moving_sample(self, q_target, dq_target, measured_arm_q):
        arm_q = np.copy(measured_arm_q)
        arm_dq = np.zeros(14, dtype=np.float64)
        q_values = q_target[self.moving_local]
        dq_values = dq_target[self.moving_local]
        valid = bool(
            np.all(np.isfinite(q_values))
            and np.all(np.isfinite(dq_values))
        )
        if valid:
            lower, upper = self._arm_position_limits()
            velocity = self._arm_velocity_limits()
            bounded_q = np.clip(
                q_values,
                lower[self.moving_local],
                upper[self.moving_local],
            )
            bounded_dq = np.clip(
                dq_values,
                -velocity[self.moving_local],
                velocity[self.moving_local],
            )
            self.latest_clipped = bool(
                not np.array_equal(bounded_q, q_values)
                or not np.array_equal(bounded_dq, dq_values)
            )
            arm_q[self.moving_local] = bounded_q
            arm_dq[self.moving_local] = bounded_dq
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

    def _reaction_targets(self, com_moving, momentum_moving, moving_dq,
                          com_error, gyro, balance_scale):
        com_rhs = balance_scale * (
            -com_moving @ moving_dq - self.com_gain * com_error
        )
        momentum_rhs = balance_scale * (
            -momentum_moving @ moving_dq + self.gyro_gain * gyro[:2]
        )
        return com_rhs, momentum_rhs

    def _solve_bounded_velocity(self, com_counter, momentum_counter,
                                com_rhs, momentum_rhs, posture_target,
                                lower, upper, balance_scale=1.0):
        blocks = []
        targets = []
        terms = (
            (
                balance_scale * self.com_weight,
                com_counter / self.com_velocity_scale,
                com_rhs / self.com_velocity_scale,
            ),
            (
                balance_scale * self.momentum_weight,
                momentum_counter / self.momentum_scale,
                momentum_rhs / self.momentum_scale,
            ),
            (
                self.posture_weight,
                np.eye(4) / self.posture_velocity_scale,
                posture_target / self.posture_velocity_scale,
            ),
            (
                self.damping,
                np.eye(4) / self.posture_velocity_scale,
                np.zeros(4),
            ),
        )
        for weight, matrix, target in terms:
            if weight > 0.0:
                root = np.sqrt(weight)
                blocks.append(root * np.asarray(matrix, dtype=np.float64))
                targets.append(root * np.asarray(target, dtype=np.float64))
        matrix = np.vstack(blocks)
        target = np.concatenate(targets)
        values = (matrix, target, lower, upper)
        if not all(np.all(np.isfinite(value)) for value in values):
            raise ValueError('Least-squares input is nonfinite')
        if np.any(lower > upper):
            raise ValueError('Counter velocity bounds are empty')
        result = lsq_linear(matrix, target, bounds=(lower, upper))
        if not result.success:
            raise RuntimeError('Bounded least-squares solve failed')
        return np.asarray(result.x, dtype=np.float64)

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

    def _publish_measured_hold(self, motor_q, status):
        arm_q = np.copy(motor_q[self.arm_ids])
        arm_dq = np.zeros(14, dtype=np.float64)
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
            self._publish_hold_without_gravity(measured_motor_q)
            return False
        if not self._write_arm_command(arm_q, arm_dq, tau):
            return False
        self._record_applied_counter_dq()
        return True

    def _publish_hold_without_gravity(self, measured_motor_q):
        tau_cmd = getattr(self.low_cmd_handler, 'tau_cmd', None)
        tau = np.zeros(14, dtype=np.float64)
        if tau_cmd is not None:
            tau_cmd = np.asarray(tau_cmd, dtype=np.float64)
            if tau_cmd.shape == (27,) and np.all(np.isfinite(tau_cmd)):
                tau = np.copy(tau_cmd[self.arm_ids])
        arm_q = np.copy(measured_motor_q[self.arm_ids])
        arm_dq = np.zeros(14, dtype=np.float64)
        return self._write_arm_command(arm_q, arm_dq, tau)

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
        self.latest_requested_counter_dq = np.zeros(4, dtype=np.float64)
        self.latest_applied_counter_dq = np.zeros(4, dtype=np.float64)
        self.latest_predicted_com_residual = None
        self.latest_predicted_momentum_residual = None
        self.latest_backtrack_scale = 0.0
        self.latest_clipped = False
        self.latest_collision_rejection = False
        self.latest_support_valid = False
        self.latest_support_error = ''
        self.latest_com = None
        self.latest_com_target = None
        self.latest_com_error = None
        self.latest_gyro_available = False
        self.latest_published = False
