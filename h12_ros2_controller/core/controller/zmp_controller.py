import numpy as np

from h12_ros2_controller.core.controller.momentum_controller import MomentumController


class ZmpController(MomentumController):
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 handless: bool=False,
                 visualize: bool=False,
                 config: dict=None):
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            handless=handless,
            visualize=visualize,
            config=config,
        )
        zmp_cfg = self.config.get('zmp', {})
        self.zmp_enabled = bool(zmp_cfg.get('enabled', False))
        self.zmp_response_time = float(zmp_cfg.get('response_time', 0.25))
        self.zmp_min_error = float(zmp_cfg.get('min_error', 0.005))
        self.zmp_support_offset = self._as_xy(
            zmp_cfg.get('support_offset', [0.0, 0.0]),
            'support_offset',
        )
        self.zmp_k = self._as_xy(zmp_cfg.get('k_zmp', [0.5, 0.5]), 'k_zmp')
        self.zmp_k_com_velocity = self._as_xy(
            zmp_cfg.get('k_com_velocity', [0.0, 0.0]),
            'k_com_velocity',
        )
        self.zmp_max_momentum = self._as_xyz(
            zmp_cfg.get('max_momentum', [4.0, 4.0, 0.0]),
            'max_momentum',
        )
        self.latest_zmp = None
        self.latest_zmp_target = None
        self.latest_zmp_error = None
        self.latest_momentum_target = None

    def estimate_zmp_target(self):
        '''Estimate angular momentum target from current ZMP error'''
        self.update_robot_model()
        zmp = self.robot_model.get_zmp()
        zmp_target = self._support_center() + self.zmp_support_offset
        zmp_error = zmp[:2] - zmp_target
        com_velocity = self.robot_model.get_com_velocity()[:2]

        # map horizontal zmp correction into roll/pitch angular impulse
        correction = self.zmp_k * zmp_error
        correction += self.zmp_k_com_velocity * com_velocity
        fz = self.robot_model.total_mass * 9.81
        h_target = self.zmp_response_time * np.array(
            [-fz * correction[1], fz * correction[0], 0.0],
            dtype=np.float64,
        )
        h_target = np.clip(
            h_target,
            -self.zmp_max_momentum,
            self.zmp_max_momentum,
        )

        self.latest_zmp = np.copy(zmp)
        self.latest_zmp_target = np.copy(zmp_target)
        self.latest_zmp_error = np.copy(zmp_error)
        self.latest_momentum_target = np.copy(h_target)
        return h_target

    def plan_zmp(self, force=False):
        '''Plan one momentum trajectory from current ZMP error'''
        if not self.zmp_enabled and not force:
            return None

        target_momentum = self.estimate_zmp_target()
        if not force and np.linalg.norm(self.latest_zmp_error) < self.zmp_min_error:
            return None
        return self.plan_momentum(target_momentum)

    def execute_zmp_step(self):
        '''Continuously monitor ZMP and execute one control step'''
        if self.plan_done:
            self.plan_zmp()
        return self.execute_plan_step()

    def _support_center(self):
        left_foot = self.robot_model.get_frame_position('left_ankle_roll_link')
        right_foot = self.robot_model.get_frame_position('right_ankle_roll_link')
        return 0.5 * (left_foot[:2] + right_foot[:2])

    @staticmethod
    def _as_xy(value, name):
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (2,):
            raise ValueError(f'zmp.{name} must have length 2')
        return vector

    @staticmethod
    def _as_xyz(value, name):
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (3,):
            raise ValueError(f'zmp.{name} must have length 3')
        return vector
