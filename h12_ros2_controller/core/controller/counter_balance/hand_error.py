'''Hand-error weighting for the counter-arm momentum rows

Under a rigid tilt about the ankle pivot, residual roll/pitch angular momentum
h rotates the base at I_rp^-1 h, which moves the moving hand at
omega x r = -[r]x omega. The hand-error objective ||M h||^2 with
M = -[r]x[:, :2] I_rp^-1 is quadratic in the same momentum residual as the
existing momentum term, so both fold into one weighted pair of rows.
'''
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HandErrorConfig:
    '''Store the opt-in hand-error weight and its velocity normalization'''

    weight: float = 0.0
    velocity_scale: float = 0.1
    frame: str | None = None


def parse_hand_error_config(cfg):
    '''Validate the optional reactive_counter_balance.hand_error mapping'''
    if cfg is None:
        return HandErrorConfig()
    if not isinstance(cfg, dict):
        raise ValueError('reactive_counter_balance.hand_error must be a mapping')
    weight = float(cfg.get('weight', 0.0))
    scale = float(cfg.get('velocity_scale', 0.1))
    frame = cfg.get('frame')
    if not np.isfinite(weight) or weight < 0.0:
        raise ValueError('hand_error.weight must be finite and nonnegative')
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError('hand_error.velocity_scale must be finite and positive')
    if frame is not None and not isinstance(frame, str):
        raise ValueError('hand_error.frame must be a frame name')
    return HandErrorConfig(weight=weight, velocity_scale=scale, frame=frame)


def skew(vector):
    '''Return the cross-product matrix of a three-vector'''
    x, y, z = np.asarray(vector, dtype=np.float64)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def pivot_inertia(com_inertia, mass, com, pivot):
    '''Shift a CoM-frame rotational inertia to the pivot by parallel axes'''
    offset = np.asarray(com, dtype=np.float64) - np.asarray(pivot, dtype=np.float64)
    return (np.asarray(com_inertia, dtype=np.float64)
            + mass * (offset @ offset * np.eye(3) - np.outer(offset, offset)))


def hand_momentum_map(hand, pivot, com_inertia, mass, com):
    '''Map roll/pitch momentum residual to hand velocity, shape (3, 2)'''
    inertia = pivot_inertia(com_inertia, mass, com, pivot)[:2, :2]
    lever = np.asarray(hand, dtype=np.float64) - np.asarray(pivot, dtype=np.float64)
    hand_map = -skew(lever)[:, :2] @ np.linalg.inv(inertia)
    if hand_map.shape != (3, 2) or not np.all(np.isfinite(hand_map)):
        raise ValueError('hand momentum map is nonfinite')
    return hand_map


def fold_hand_error(momentum_counter, momentum_rhs, hand_map,
                    momentum_weight, momentum_scale, config):
    '''Fold momentum and hand-error weights into one pair of momentum rows

    Returns rows A' and target b' with ||A' x - b'||^2 equal to
    momentum_weight ||(A x - b) / momentum_scale||^2
    + weight ||M (A x - b) / velocity_scale||^2, to be solved with unit
    momentum weight and scale. The caller's balance scale still applies.
    '''
    quadratic = (momentum_weight / momentum_scale ** 2) * np.eye(2)
    quadratic = quadratic + (config.weight / config.velocity_scale ** 2) * (
        hand_map.T @ hand_map
    )
    values, vectors = np.linalg.eigh(0.5 * (quadratic + quadratic.T))
    root = np.sqrt(np.clip(values, 0.0, None))[:, None] * vectors.T
    rows = root @ np.asarray(momentum_counter, dtype=np.float64)
    target = root @ np.asarray(momentum_rhs, dtype=np.float64)
    if not (np.all(np.isfinite(rows)) and np.all(np.isfinite(target))):
        raise ValueError('folded hand-error rows are nonfinite')
    return rows, target
