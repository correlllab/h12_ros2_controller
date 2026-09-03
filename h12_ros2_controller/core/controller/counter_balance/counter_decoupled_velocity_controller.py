import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.utility.controller_config import load_controller_config


class CounterDecoupledVelocityController(CounterDDPVelocityController):
    '''Add bounded tilt feedback without scaling frozen 3C feedforward'''

    def __init__(self, *args, config=None, **kwargs):
        resolved = load_controller_config() if config is None else config
        settings = self._feedback_settings(
            resolved.get('decoupled_feedback', {}),
        )
        super().__init__(*args, config=resolved, **kwargs)
        self.tilt_gain = settings['tilt_gain']
        self.max_tilt_error = settings['max_tilt_error']
        self.latest_tilt_error = np.zeros(2, dtype=np.float64)
        self.latest_clipped_tilt_error = np.zeros(2, dtype=np.float64)
        self.latest_tilt_feedback = np.zeros(2, dtype=np.float64)
        self.latest_tilt_feedback_available = False
        self.latest_momentum_feedforward = np.zeros(2, dtype=np.float64)
        self.latest_gyro_feedback = np.zeros(2, dtype=np.float64)
        self.latest_positive_divergence = np.zeros(2, dtype=np.float64)

    def control_configuration_step(
            self, moving_q_target_14, moving_dq_target_14,
            balance_scale=1.0):
        '''Apply one decoupled feedforward-feedback command'''
        self.latest_tilt_error = np.zeros(2, dtype=np.float64)
        self.latest_clipped_tilt_error = np.zeros(2, dtype=np.float64)
        self.latest_tilt_feedback = np.zeros(2, dtype=np.float64)
        self.latest_tilt_feedback_available = False
        self.latest_momentum_feedforward = np.zeros(2, dtype=np.float64)
        self.latest_gyro_feedback = np.zeros(2, dtype=np.float64)
        self.latest_positive_divergence = np.zeros(2, dtype=np.float64)
        return super().control_configuration_step(
            moving_q_target_14,
            moving_dq_target_14,
            balance_scale=balance_scale,
        )

    def _reaction_targets(
            self, com_moving, momentum_moving, moving_dq,
            com_error, gyro, balance_scale):
        com_rhs, momentum_rhs = super()._reaction_targets(
            com_moving,
            momentum_moving,
            moving_dq,
            com_error,
            gyro,
            balance_scale,
        )
        com_offset, momentum_offset = self._reaction_target_offsets(
            com_moving,
            momentum_moving,
            moving_dq,
            com_error,
            gyro,
            balance_scale,
        )
        return com_rhs + com_offset, momentum_rhs + momentum_offset

    def _reaction_target_offsets(
            self, com_moving, momentum_moving, moving_dq,
            com_error, gyro, balance_scale):
        tilt_error, available = self._measured_tilt_error()
        clipped = np.clip(
            tilt_error,
            -self.max_tilt_error,
            self.max_tilt_error,
        )
        feedback = balance_scale * self.tilt_gain * clipped
        self.latest_tilt_error = tilt_error
        self.latest_clipped_tilt_error = clipped
        self.latest_tilt_feedback = feedback
        self.latest_tilt_feedback_available = available
        self.latest_momentum_feedforward = (
            -balance_scale * momentum_moving @ moving_dq
        )
        self.latest_gyro_feedback = (
            balance_scale * self.gyro_gain * gyro[:2]
        )
        self.latest_positive_divergence = np.maximum(
            tilt_error * gyro[:2],
            0.0,
        )
        return np.zeros(2, dtype=np.float64), feedback

    def _measured_tilt_error(self):
        reference = np.asarray(self.tilt_reference, dtype=np.float64)
        imu_state = self.robot_model.state.get('imu_state')
        quaternion = getattr(imu_state, 'quaternion', None)
        if quaternion is None:
            return np.zeros(2, dtype=np.float64), False
        quaternion = np.asarray(quaternion, dtype=np.float64).reshape(-1)
        norm = np.linalg.norm(quaternion)
        if (
            reference.shape != (2,)
            or not np.all(np.isfinite(reference))
            or quaternion.shape != (4,)
            or not np.all(np.isfinite(quaternion))
            or not 0.5 <= norm <= 1.5
        ):
            return np.zeros(2, dtype=np.float64), False
        tilt_error = self._imu_tilt() - reference
        if not np.all(np.isfinite(tilt_error)):
            return np.zeros(2, dtype=np.float64), False
        return tilt_error, True

    def diagnostics(self):
        '''Return frozen 3C and decoupled feedback diagnostics'''
        values = super().diagnostics()
        values.update({
            'decoupled_tilt_gain': float(self.tilt_gain),
            'decoupled_max_tilt_error': float(self.max_tilt_error),
            'decoupled_tilt_feedback_available': bool(
                self.latest_tilt_feedback_available
            ),
            'decoupled_tilt_error': self.latest_tilt_error.tolist(),
            'decoupled_clipped_tilt_error': (
                self.latest_clipped_tilt_error.tolist()
            ),
            'decoupled_tilt_feedback': self.latest_tilt_feedback.tolist(),
            'decoupled_momentum_feedforward': (
                self.latest_momentum_feedforward.tolist()
            ),
            'decoupled_gyro_feedback': self.latest_gyro_feedback.tolist(),
            'decoupled_positive_divergence': (
                self.latest_positive_divergence.tolist()
            ),
        })
        return values

    @staticmethod
    def _feedback_settings(config):
        if not isinstance(config, dict):
            raise ValueError('decoupled_feedback must be a mapping')
        try:
            tilt_gain = float(config.get('tilt_gain', 0.0))
            max_tilt_error = float(config.get('max_tilt_error', 0.1))
        except (TypeError, ValueError) as error:
            raise ValueError(
                'decoupled feedback values must be finite scalars'
            ) from error
        if not np.isfinite(tilt_gain) or tilt_gain < 0.0:
            raise ValueError(
                'decoupled_feedback.tilt_gain must be finite and nonnegative'
            )
        if not np.isfinite(max_tilt_error) or max_tilt_error <= 0.0:
            raise ValueError(
                'decoupled_feedback.max_tilt_error must be finite and positive'
            )
        return {
            'tilt_gain': tilt_gain,
            'max_tilt_error': max_tilt_error,
        }
