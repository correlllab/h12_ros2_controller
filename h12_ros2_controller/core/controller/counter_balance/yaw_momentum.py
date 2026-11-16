'''Vertical-axis (yaw) momentum objective for the counter arm

The base 3C objective cancels the moving arm's roll/pitch centroidal angular
momentum with the counter arm (rows 0-1 of the centroidal momentum map). It
leaves the yaw row free, so the counter arm's own motion can exchange angular
momentum about the vertical with the body, which turns the torso. This opt-in
row asks the counter arm to cancel the arms' yaw momentum as well:

    A_z,counter dq_counter = s (-A_z,moving dq_moving + gyro_gain * omega_z)

weighted like the momentum rows (balance scale s times weight, normalized by
scale). With weight 0 nothing is added.
'''
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class YawMomentumConfig:
    '''Store the opt-in yaw-momentum weight, normalization and yaw-rate gain'''

    weight: float = 0.0
    scale: float | None = None      # defaults to the momentum rows' scale
    gyro_gain: float = 0.0


def parse_yaw_momentum_config(cfg):
    '''Validate the optional reactive_counter_balance.yaw_momentum mapping'''
    if cfg is None:
        return YawMomentumConfig()
    if not isinstance(cfg, dict):
        raise ValueError('reactive_counter_balance.yaw_momentum must be a mapping')
    weight = float(cfg.get('weight', 0.0))
    scale = cfg.get('scale')
    scale = None if scale is None else float(scale)
    gyro_gain = float(cfg.get('gyro_gain', 0.0))
    if not np.isfinite(weight) or weight < 0.0:
        raise ValueError('yaw_momentum.weight must be finite and nonnegative')
    if scale is not None and (not np.isfinite(scale) or scale <= 0.0):
        raise ValueError('yaw_momentum.scale must be finite and positive')
    if not np.isfinite(gyro_gain):
        raise ValueError('yaw_momentum.gyro_gain must be finite')
    return YawMomentumConfig(weight=weight, scale=scale, gyro_gain=gyro_gain)


def yaw_momentum_row(yaw_counter, yaw_moving, moving_dq, gyro_z,
                     balance_scale, config, default_scale):
    '''Return the weighted (1, 4) row and (1,) target to append to the objective'''
    yaw_counter = np.asarray(yaw_counter, dtype=np.float64).reshape(1, -1)
    yaw_moving = np.asarray(yaw_moving, dtype=np.float64).reshape(-1)
    moving_dq = np.asarray(moving_dq, dtype=np.float64).reshape(-1)
    scale = config.scale if config.scale is not None else float(default_scale)
    rhs = balance_scale * (-yaw_moving @ moving_dq + config.gyro_gain * float(gyro_z))
    root = np.sqrt(balance_scale * config.weight) / scale
    row, target = root * yaw_counter, np.array([root * rhs])
    if row.shape != (1, 4) or not (np.all(np.isfinite(row)) and np.all(np.isfinite(target))):
        raise ValueError('yaw momentum row is invalid')
    return row, target
