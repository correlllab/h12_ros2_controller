import numpy as np


class MomentumTargetEstimator:
    def __init__(self, robot_model, config):
        zmp_cfg = config.get('zmp', {})
        gains = zmp_cfg.get('gains', {})
        target = zmp_cfg.get('target', {})

        self.robot_model = robot_model
        self.k_zmp = self._as_vec2(gains.get('zmp', [1.0, 1.0]))
        self.k_center = self._as_vec2(gains.get('center', [0.5, 0.5]))
        self.k_com_velocity = self._as_vec2(
            gains.get('com_velocity', [0.4, 0.4])
        )
        self.k_angular_acceleration = self._as_vec2(
            gains.get('angular_acceleration', [0.0, 0.0])
        )
        self.response_time = float(target.get('response_time', 0.08))
        self.min_momentum_norm = float(target.get('min_momentum_norm', 0.0))
        self.max_momentum = self._as_vec3(
            target.get('max_momentum', [1.2, 1.2, 0.0])
        )

    def estimate(self, balance_state, perturbation_state):
        '''Return clipped whole-body angular momentum target'''
        if not perturbation_state.active:
            return np.zeros(3, dtype=np.float64)

        correction = (
            self.k_zmp * balance_state.zmp_error
            + self.k_center * balance_state.center_shift
            + self.k_com_velocity * balance_state.com_velocity
            + self.k_angular_acceleration
            * balance_state.angular_acceleration[:2]
        )
        mass = float(getattr(self.robot_model, 'total_mass', 0.0))
        mg = mass * 9.81
        momentum_rate = np.array(
            [-mg * correction[1], mg * correction[0], 0.0],
            dtype=np.float64,
        )
        target = self.response_time * momentum_rate
        target = self._apply_min_momentum(target)
        return np.clip(target, -self.max_momentum, self.max_momentum)

    def _apply_min_momentum(self, target):
        norm = float(np.linalg.norm(target))
        if norm <= 1e-9 or norm >= self.min_momentum_norm:
            return target
        return target * (self.min_momentum_norm / norm)

    @staticmethod
    def _as_vec2(value):
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if arr.size != 2:
            raise ValueError('expected a two-element vector')
        return arr

    @staticmethod
    def _as_vec3(value):
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if arr.size != 3:
            raise ValueError('expected a three-element vector')
        return arr
