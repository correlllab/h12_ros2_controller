from dataclasses import dataclass

import numpy as np


@dataclass
class BalanceState:
    zmp: np.ndarray
    zmp_target: np.ndarray
    zmp_error: np.ndarray
    support_margin: float
    com_xy: np.ndarray
    center_reference: np.ndarray
    center_shift: np.ndarray
    com_velocity: np.ndarray
    angular_velocity: np.ndarray
    angular_acceleration: np.ndarray
    force_proxy: float


class BalanceObserver:
    def __init__(self, robot_model, dt, config):
        zmp_cfg = config.get('zmp', {})
        observer_cfg = zmp_cfg.get('observer', {})

        self.robot_model = robot_model
        self.dt = float(dt)
        self.center_reference_alpha = float(
            observer_cfg.get('center_reference_alpha', 0.02)
        )
        self.angular_acceleration_alpha = float(
            observer_cfg.get('angular_acceleration_alpha', 0.35)
        )
        self.com_acceleration_alpha = float(
            observer_cfg.get('com_acceleration_alpha', 0.20)
        )
        self.support_offset = self._as_vec2(
            observer_cfg.get('support_offset', [0.0, 0.0])
        )
        self.support_half_extents = self._as_vec2(
            observer_cfg.get('support_half_extents', [0.12, 0.06])
        )

        self._center_reference = None
        self._last_angular_velocity = None
        self._angular_acceleration = np.zeros(3, dtype=np.float64)
        self._last_com_velocity = None
        self._com_acceleration = np.zeros(2, dtype=np.float64)

    def observe(self, freeze_center_reference=False):
        '''Compute current balance metrics from RobotModel state'''
        dynamics = self.robot_model.dynamics
        zmp = np.asarray(dynamics.get_zmp(), dtype=np.float64)[:2]
        zmp_target = self._support_target()
        zmp_error = zmp - zmp_target
        support_margin = self.compute_support_margin(zmp, zmp_target)

        com_xy = np.asarray(dynamics.get_com(), dtype=np.float64)[:2]
        self._update_center_reference(com_xy, freeze_center_reference)
        center_shift = com_xy - self._center_reference

        com_velocity = np.asarray(
            dynamics.get_com_velocity(),
            dtype=np.float64,
        )[:2]
        self._update_com_acceleration(com_velocity)

        angular_velocity = self._imu_angular_velocity()
        self._update_angular_acceleration(angular_velocity)
        force_proxy = self._force_proxy()

        return BalanceState(
            zmp=np.copy(zmp),
            zmp_target=np.copy(zmp_target),
            zmp_error=np.copy(zmp_error),
            support_margin=float(support_margin),
            com_xy=np.copy(com_xy),
            center_reference=np.copy(self._center_reference),
            center_shift=np.copy(center_shift),
            com_velocity=np.copy(com_velocity),
            angular_velocity=np.copy(angular_velocity),
            angular_acceleration=np.copy(self._angular_acceleration),
            force_proxy=float(force_proxy),
        )

    def compute_support_margin(self, zmp, zmp_target):
        '''Return rectangular support margin around the support target'''
        delta = np.abs(np.asarray(zmp, dtype=np.float64) - zmp_target)
        margins = self.support_half_extents - delta
        return float(np.min(margins))

    def _support_target(self):
        q = self.robot_model.state['q']
        left = self.robot_model.get_frame_position(
            'left_ankle_roll_link',
            q,
        )[:2]
        right = self.robot_model.get_frame_position(
            'right_ankle_roll_link',
            q,
        )[:2]
        return 0.5 * (left + right) + self.support_offset

    def _update_center_reference(self, com_xy, freeze):
        if self._center_reference is None:
            self._center_reference = np.copy(com_xy)
            return
        if freeze:
            return
        alpha = np.clip(self.center_reference_alpha, 0.0, 1.0)
        self._center_reference = (
            (1.0 - alpha) * self._center_reference
            + alpha * com_xy
        )

    def _update_angular_acceleration(self, angular_velocity):
        if self._last_angular_velocity is None:
            raw = np.zeros(3, dtype=np.float64)
        else:
            raw = (angular_velocity - self._last_angular_velocity) / self.dt
        alpha = np.clip(self.angular_acceleration_alpha, 0.0, 1.0)
        self._angular_acceleration = (
            (1.0 - alpha) * self._angular_acceleration
            + alpha * raw
        )
        self._last_angular_velocity = np.copy(angular_velocity)

    def _update_com_acceleration(self, com_velocity):
        if self._last_com_velocity is None:
            raw = np.zeros(2, dtype=np.float64)
        else:
            raw = (com_velocity - self._last_com_velocity) / self.dt
        alpha = np.clip(self.com_acceleration_alpha, 0.0, 1.0)
        self._com_acceleration = (
            (1.0 - alpha) * self._com_acceleration
            + alpha * raw
        )
        self._last_com_velocity = np.copy(com_velocity)

    def _force_proxy(self):
        mass = float(getattr(self.robot_model, 'total_mass', 0.0))
        return mass * float(np.linalg.norm(self._com_acceleration))

    def _imu_angular_velocity(self):
        imu_state = self.robot_model.state.get('imu_state')
        if imu_state is None:
            return np.zeros(3, dtype=np.float64)
        for name in ('gyroscope', 'angular_velocity', 'gyro'):
            value = getattr(imu_state, name, None)
            if value is not None:
                value = np.asarray(value, dtype=np.float64).reshape(-1)
                if value.size >= 3 and np.isfinite(value[:3]).all():
                    return np.copy(value[:3])
        return np.zeros(3, dtype=np.float64)

    @staticmethod
    def _as_vec2(value):
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if arr.size != 2:
            raise ValueError('expected a two-element vector')
        return arr
