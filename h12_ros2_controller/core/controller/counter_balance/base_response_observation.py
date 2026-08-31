from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class CounterStateObservation:
    '''Store one timestamped measured counter state'''

    monotonic_time: float
    sequence: int
    tick: int
    q_counter: np.ndarray
    dq_counter: np.ndarray
    age: float
    valid: bool
    invalid_reason: str | None = None


@dataclass(frozen=True)
class BaseObservation:
    '''Store one real-compatible planar base observation'''

    monotonic_time: float
    orientation_error_xy: np.ndarray
    angular_velocity_xy: np.ndarray
    com_position_xy: np.ndarray
    com_velocity_xy: np.ndarray
    base_height: float
    base_vertical_velocity: float
    age: float
    valid: bool
    invalid_reason: str | None = None
    frame: str = 'support'


@dataclass(frozen=True)
class SupportObservation:
    '''Store support and foot kinematics in canonical coordinates'''

    monotonic_time: float
    left_position: np.ndarray
    right_position: np.ndarray
    left_rotation: np.ndarray
    right_rotation: np.ndarray
    left_twist: np.ndarray
    right_twist: np.ndarray
    support_origin: np.ndarray
    support_rotation: np.ndarray
    support_twist: np.ndarray
    age: float
    valid: bool
    invalid_reason: str | None = None
    frame: str = 'model_root'
    twist_order: str = 'linear_angular'


@dataclass(frozen=True)
class SupportValidityObservation:
    '''Store stationary double-support validity evidence'''

    monotonic_time: float
    double_support_valid: bool
    confidence: float
    age: float
    metrics: np.ndarray
    peak_normalized_residual: float
    invalid_reason: str | None = None


@dataclass(frozen=True)
class ManipulationDisturbanceObservation:
    '''Store real-compatible manipulation momentum disturbance'''

    monotonic_time: float
    momentum_xy: np.ndarray
    momentum_rate_xy: np.ndarray
    valid: bool
    invalid_reason: str | None = None


@dataclass(frozen=True)
class BaseResponseObservationBundle:
    '''Bundle one synchronized Iteration 4 observation sample'''

    counter: CounterStateObservation
    base: BaseObservation | None
    support: SupportObservation | None
    support_validity: SupportValidityObservation | None
    disturbance: ManipulationDisturbanceObservation | None


@dataclass(frozen=True)
class SupportValidityThresholds:
    '''Configure provisional stationary-support validity thresholds'''

    foot_height: float = 0.03
    foot_orientation: float = 0.18
    linear_velocity: float = 0.14
    angular_velocity: float = 0.65
    relative_translation: float = 0.09
    relative_rotation: float = 0.06
    exit_scale: float = 1.5
    enter_samples: int = 5
    exit_samples: int = 2
    max_age: float = 0.04


class SupportKinematicsObserver:
    '''Derive support-relative base quantities from proprioception'''

    def __init__(self, filter_time_constant=0.06, max_dt=0.1):
        self.filter_time_constant = float(filter_time_constant)
        self.max_dt = float(max_dt)
        self.reset()

    def reset(self):
        '''Clear support and derivative history'''
        self._previous = None
        self._reference_rotation = None
        self._angular_velocity = np.zeros(2)
        self._com_velocity = np.zeros(3)
        self._height_velocity = 0.0

    def update(
            self, timestamp, left_position, right_position,
            left_rotation, right_rotation, left_twist, right_twist,
            com_position, base_position, base_rotation, age=0.0):
        '''Build one support and base observation from a single snapshot'''
        timestamp = float(timestamp)
        left_position = _strict(left_position, (3,), 'left_position')
        right_position = _strict(right_position, (3,), 'right_position')
        left_rotation = _strict(left_rotation, (3, 3), 'left_rotation')
        right_rotation = _strict(right_rotation, (3, 3), 'right_rotation')
        left_twist = _strict(left_twist, (6,), 'left_twist')
        right_twist = _strict(right_twist, (6,), 'right_twist')
        com_position = _strict(com_position, (3,), 'com_position')
        base_position = _strict(base_position, (3,), 'base_position')
        base_rotation = _strict(base_rotation, (3, 3), 'base_rotation')

        origin = 0.5 * (left_position + right_position)
        z_axis = _unit(left_rotation[:, 2] + right_rotation[:, 2], 'support z')
        x_raw = left_rotation[:, 0] + right_rotation[:, 0]
        x_axis = _unit(x_raw - z_axis * np.dot(z_axis, x_raw), 'support x')
        y_axis = _unit(np.cross(z_axis, x_axis), 'support y')
        x_axis = _unit(np.cross(y_axis, z_axis), 'support x')
        support_rotation = np.column_stack([x_axis, y_axis, z_axis])
        support_twist_world = 0.5 * (left_twist + right_twist)
        support_twist = np.concatenate([
            support_rotation.T @ support_twist_world[:3],
            support_rotation.T @ support_twist_world[3:],
        ])

        com_support = support_rotation.T @ (com_position - origin)
        base_support = support_rotation.T @ (base_position - origin)
        relative_rotation = support_rotation.T @ base_rotation
        if self._reference_rotation is None:
            self._reference_rotation = np.copy(relative_rotation)
        orientation_error = pin.log3(
            self._reference_rotation.T @ relative_rotation,
        )[:2]
        valid_rate = False
        alpha = 0.0
        if self._previous is not None:
            dt = timestamp - self._previous[0]
            valid_rate = np.isfinite(dt) and 0.0 < dt <= self.max_dt
            if valid_rate:
                alpha = dt / (self.filter_time_constant + dt)
                raw_angular_velocity = (
                    orientation_error - self._previous[1]
                ) / dt
                raw_com_velocity = (com_support - self._previous[2]) / dt
                raw_height_velocity = (
                    base_support[2] - self._previous[3]
                ) / dt
                self._angular_velocity += alpha * (
                    raw_angular_velocity - self._angular_velocity
                )
                self._com_velocity += alpha * (
                    raw_com_velocity - self._com_velocity
                )
                self._height_velocity += alpha * (
                    raw_height_velocity - self._height_velocity
                )
        if not valid_rate:
            self._angular_velocity[:] = 0.0
            self._com_velocity[:] = 0.0
            self._height_velocity = 0.0
        self._previous = (
            timestamp,
            np.copy(orientation_error),
            np.copy(com_support),
            float(base_support[2]),
        )

        support = SupportObservation(
            monotonic_time=timestamp,
            left_position=_readonly(left_position),
            right_position=_readonly(right_position),
            left_rotation=_readonly(left_rotation),
            right_rotation=_readonly(right_rotation),
            left_twist=_readonly(left_twist),
            right_twist=_readonly(right_twist),
            support_origin=_readonly(origin),
            support_rotation=_readonly(support_rotation),
            support_twist=_readonly(support_twist),
            age=float(age),
            valid=True,
        )
        base = BaseObservation(
            monotonic_time=timestamp,
            orientation_error_xy=_readonly(orientation_error),
            angular_velocity_xy=_readonly(self._angular_velocity),
            com_position_xy=_readonly(com_support[:2]),
            com_velocity_xy=_readonly(self._com_velocity[:2]),
            base_height=float(base_support[2]),
            base_vertical_velocity=float(self._height_velocity),
            age=float(age),
            valid=valid_rate,
            invalid_reason=(
                None if valid_rate else 'support derivative history unavailable'
            ),
        )
        return base, support


class SupportValidityObserver:
    '''Track stationary double support from foot kinematics'''

    def __init__(self, thresholds=None):
        self.thresholds = thresholds or SupportValidityThresholds()
        self.reset()

    def reset(self):
        '''Clear settled reference and hysteresis state'''
        self._reference = None
        self._valid = False
        self._good_samples = 0
        self._bad_samples = 0

    @property
    def reference_captured(self):
        return self._reference is not None

    def capture_reference(self, support):
        '''Capture settled foot poses for support validation'''
        if not support.valid:
            raise ValueError('support observation is invalid')
        self._reference = (
            np.copy(support.left_position),
            np.copy(support.right_position),
            np.copy(support.left_rotation),
            np.copy(support.right_rotation),
        )
        self._valid = False
        self._good_samples = 0
        self._bad_samples = 0

    def update(self, support, now):
        '''Update hysteretic stationary-support validity'''
        age = float(now) - float(support.monotonic_time)
        if self._reference is None:
            return self._invalid(support.monotonic_time, age, 'reference unavailable')
        if not support.valid or not np.isfinite(age) or age < 0.0:
            return self._invalid(support.monotonic_time, age, 'support observation invalid')
        if age > self.thresholds.max_age:
            return self._invalid(support.monotonic_time, age, 'support observation stale')

        metrics, residuals = self._residuals(support)
        peak = max(residuals)
        enter_good = peak <= 1.0
        exit_bad = peak > self.thresholds.exit_scale
        self._good_samples = self._good_samples + 1 if enter_good else 0
        self._bad_samples = self._bad_samples + 1 if exit_bad else 0
        if not self._valid and self._good_samples >= self.thresholds.enter_samples:
            self._valid = True
            self._bad_samples = 0
        elif self._valid and self._bad_samples >= self.thresholds.exit_samples:
            self._valid = False
            self._good_samples = 0
        confidence = float(np.clip(
            1.0 - peak / self.thresholds.exit_scale, 0.0, 1.0,
        ))
        return SupportValidityObservation(
            monotonic_time=support.monotonic_time,
            double_support_valid=self._valid,
            confidence=confidence,
            age=age,
            metrics=_readonly(metrics),
            peak_normalized_residual=float(peak),
            invalid_reason=None if self._valid else 'stationary support not established',
        )

    def _residuals(self, support):
        left_p, right_p, left_r, right_r = self._reference
        relative_ref = left_r.T @ right_r
        relative = support.left_rotation.T @ support.right_rotation
        metrics = np.array((
            abs(support.left_position[2] - left_p[2]),
            abs(support.right_position[2] - right_p[2]),
            np.linalg.norm(pin.log3(left_r.T @ support.left_rotation)),
            np.linalg.norm(pin.log3(right_r.T @ support.right_rotation)),
            max(
                np.linalg.norm(support.left_twist[:3]),
                np.linalg.norm(support.right_twist[:3]),
            ),
            max(
                np.linalg.norm(support.left_twist[3:]),
                np.linalg.norm(support.right_twist[3:]),
            ),
            np.linalg.norm(
                (support.right_position - support.left_position)
                - (right_p - left_p),
            ),
            np.linalg.norm(pin.log3(relative_ref.T @ relative)),
        ))
        scales = np.array((
            self.thresholds.foot_height,
            self.thresholds.foot_height,
            self.thresholds.foot_orientation,
            self.thresholds.foot_orientation,
            self.thresholds.linear_velocity,
            self.thresholds.angular_velocity,
            self.thresholds.relative_translation,
            self.thresholds.relative_rotation,
        ))
        return metrics, metrics / scales

    def _invalid(self, timestamp, age, reason):
        self._valid = False
        self._good_samples = 0
        self._bad_samples = 0
        return SupportValidityObservation(
            monotonic_time=float(timestamp),
            double_support_valid=False,
            confidence=0.0,
            age=float(age),
            metrics=_readonly(np.zeros(8)),
            peak_normalized_residual=np.inf,
            invalid_reason=reason,
        )


class BaseResponseObservationPipeline:
    '''Derive synchronized real-compatible observations from LowState'''

    def __init__(
            self, robot_model, counter_ids, moving_ids=None,
            support_observer=None, validity_observer=None,
            max_age=0.04):
        self.robot_model = robot_model
        self.counter_ids = np.asarray(counter_ids, dtype=np.int64)
        self.moving_ids = (
            None if moving_ids is None
            else np.asarray(moving_ids, dtype=np.int64)
        )
        self.support_observer = support_observer or SupportKinematicsObserver()
        self.validity_observer = validity_observer or SupportValidityObserver()
        self.max_age = float(max_age)
        self._previous_disturbance = None
        self._disturbance_rate = np.zeros(2)

    @property
    def reference_captured(self):
        return self.validity_observer.reference_captured

    def capture_reference(self, state, now):
        '''Capture settled support and base references from one snapshot'''
        self.support_observer.reset()
        self.validity_observer.reset()
        self._previous_disturbance = None
        self._disturbance_rate[:] = 0.0
        bundle = self.update(state, now)
        if bundle.support is None or not bundle.support.valid:
            raise ValueError('could not capture support reference')
        self.validity_observer.capture_reference(bundle.support)

    def update(self, state, now):
        '''Build one bundle from one immutable measured snapshot'''
        counter = counter_state_observation(
            state, self.counter_ids, now, self.max_age,
        )
        if not counter.valid:
            return BaseResponseObservationBundle(
                counter, None, None, None, None,
            )
        try:
            quaternion = _strict(
                state['imu_state'].quaternion,
                (4,),
                'imu quaternion',
            )
            quaternion_norm = np.linalg.norm(quaternion)
            if not 0.5 <= quaternion_norm <= 1.5:
                raise ValueError('imu quaternion norm is invalid')
            q_full, dq_full = self.robot_model.update_kinematics_from_snapshot(
                state,
            )
            model = self.robot_model.model
            data = self.robot_model.data
            left_id = model.getFrameId('left_ankle_roll_link')
            right_id = model.getFrameId('right_ankle_roll_link')
            left_pose = data.oMf[left_id]
            right_pose = data.oMf[right_id]
            left_velocity = pin.getFrameVelocity(
                model, data, left_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            right_velocity = pin.getFrameVelocity(
                model, data, right_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            com = pin.centerOfMass(model, data, q_full, dq_full)
            base_rotation = pin.Quaternion(q_full[3:7]).matrix()
            base, support = self.support_observer.update(
                counter.monotonic_time,
                left_pose.translation,
                right_pose.translation,
                left_pose.rotation,
                right_pose.rotation,
                np.concatenate([left_velocity.linear, left_velocity.angular]),
                np.concatenate([right_velocity.linear, right_velocity.angular]),
                com,
                q_full[:3],
                base_rotation,
                age=counter.age,
            )
            validity = self.validity_observer.update(support, now)
            disturbance = self._disturbance(
                counter.monotonic_time,
                state,
                q_full,
                support.support_rotation,
            )
            return BaseResponseObservationBundle(
                counter, base, support, validity, disturbance,
            )
        except (ValueError, KeyError, IndexError, RuntimeError):
            return BaseResponseObservationBundle(
                counter, None, None, None, None,
            )

    def _disturbance(self, timestamp, state, q_full, support_rotation):
        if self.moving_ids is None or self.moving_ids.shape != (7,):
            return None
        pin.computeCentroidalMap(
            self.robot_model.model,
            self.robot_model.data,
            q_full,
        )
        angular_map = self.robot_model.data.Ag[3:, 6:]
        dq = np.asarray(state['dq'], dtype=np.float64)
        momentum_world = angular_map[:, self.moving_ids] @ dq[self.moving_ids]
        momentum = (support_rotation.T @ momentum_world)[:2]
        valid = False
        if self._previous_disturbance is not None:
            dt = timestamp - self._previous_disturbance[0]
            valid = np.isfinite(dt) and 0.0 < dt <= self.support_observer.max_dt
            if valid:
                alpha = dt / (self.support_observer.filter_time_constant + dt)
                raw_rate = (momentum - self._previous_disturbance[1]) / dt
                self._disturbance_rate += alpha * (
                    raw_rate - self._disturbance_rate
                )
        if not valid:
            self._disturbance_rate[:] = 0.0
        self._previous_disturbance = (timestamp, np.copy(momentum))
        return ManipulationDisturbanceObservation(
            monotonic_time=timestamp,
            momentum_xy=_readonly(momentum),
            momentum_rate_xy=_readonly(self._disturbance_rate),
            valid=valid,
            invalid_reason=None if valid else 'disturbance history unavailable',
        )


def counter_state_observation(state, counter_ids, now, max_age):
    '''Build strict measured counter state without command fallback'''
    timestamp = float(state.get('arrival_monotonic', 0.0))
    age = float(now) - timestamp
    reason = None
    q = np.asarray(state.get('q', []), dtype=np.float64)
    dq = np.asarray(state.get('dq', []), dtype=np.float64)
    ids = np.asarray(counter_ids, dtype=np.int64)
    if not bool(state.get('received', False)):
        reason = 'no low-state sample received'
    elif not bool(state.get('tick_valid', False)):
        reason = 'low-state tick is invalid'
    elif not np.isfinite(age) or age < 0.0 or age > float(max_age):
        reason = 'low-state sample is stale'
    elif (
        q.ndim != 1 or dq.ndim != 1
        or ids.shape != (4,)
        or np.any(ids < 0)
        or np.any(ids >= len(q))
        or np.any(ids >= len(dq))
    ):
        reason = 'counter state shape is invalid'
    else:
        q = q[ids]
        dq = dq[ids]
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
            reason = 'counter state is nonfinite'

    if reason is not None:
        q = np.zeros(4)
        dq = np.zeros(4)
    return CounterStateObservation(
        monotonic_time=timestamp,
        sequence=int(state.get('sequence', 0)),
        tick=int(state.get('tick', 0)),
        q_counter=_readonly(q),
        dq_counter=_readonly(dq),
        age=age,
        valid=reason is None,
        invalid_reason=reason,
    )


def _readonly(value):
    result = np.array(value, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result


def _strict(value, shape, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite with shape {shape}')
    return np.copy(result)


def _unit(value, name):
    norm = np.linalg.norm(value)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError(f'{name} is degenerate')
    return value / norm
