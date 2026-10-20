import numpy as np
from scipy.optimize import lsq_linear


class CounterVelocityBoundsError(ValueError):
    '''Report an empty safe counter-velocity interval'''


def reaction_targets(com_moving, momentum_moving, moving_dq, com_error,
                     gyro, balance_scale, com_gain, gyro_gain):
    '''Build CoM and momentum targets for the counter arm'''
    com_rhs = balance_scale * (
        -com_moving @ moving_dq - com_gain * com_error
    )
    momentum_rhs = balance_scale * (
        -momentum_moving @ moving_dq + gyro_gain * gyro[:2]
    )
    return com_rhs, momentum_rhs


def solve_bounded_velocity(com_counter, momentum_counter, com_rhs,
                           momentum_rhs, posture_target, lower, upper,
                           balance_scale, com_weight, momentum_weight,
                           posture_weight, damping, com_velocity_scale,
                           momentum_scale, posture_velocity_scale):
    '''Solve the normalized bounded counter-arm least-squares objective'''
    matrix, target = bounded_velocity_problem(
        com_counter,
        momentum_counter,
        com_rhs,
        momentum_rhs,
        posture_target,
        balance_scale,
        com_weight,
        momentum_weight,
        posture_weight,
        damping,
        com_velocity_scale,
        momentum_scale,
        posture_velocity_scale,
    )
    values = (matrix, target, lower, upper)
    if not all(np.all(np.isfinite(value)) for value in values):
        raise ValueError('Least-squares input is nonfinite')
    if np.any(lower > upper):
        raise CounterVelocityBoundsError('Counter velocity bounds are empty')
    result = lsq_linear(matrix, target, bounds=(lower, upper))
    if not result.success:
        raise RuntimeError('Bounded least-squares solve failed')
    return np.asarray(result.x, dtype=np.float64)


def bounded_velocity_problem(
        com_counter, momentum_counter, com_rhs, momentum_rhs,
        posture_target, balance_scale, com_weight, momentum_weight,
        posture_weight, damping, com_velocity_scale, momentum_scale,
        posture_velocity_scale):
    '''Build the normalized Iteration 2 least-squares matrix and target'''
    blocks = []
    targets = []
    terms = (
        (
            balance_scale * com_weight,
            com_counter / com_velocity_scale,
            com_rhs / com_velocity_scale,
        ),
        (
            balance_scale * momentum_weight,
            momentum_counter / momentum_scale,
            momentum_rhs / momentum_scale,
        ),
        (
            posture_weight,
            np.eye(4) / posture_velocity_scale,
            posture_target / posture_velocity_scale,
        ),
        (
            damping,
            np.eye(4) / posture_velocity_scale,
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
    return matrix, target
