from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from h12_ros2_controller.core.controller.counter_balance.objective import (
    reaction_targets,
)


@dataclass(frozen=True)
class Frozen3CPlannerConfig:
    '''Store the frozen mathematical 3C target gains'''

    com_gain: float
    gyro_gain: float
    posture_gain: float


@dataclass(frozen=True)
class Frozen3CNominalInput:
    '''Store explicit inputs to one non-publishing 3C plan'''

    moving_dq: np.ndarray
    counter_q: np.ndarray
    counter_q_ref: np.ndarray
    com_error: np.ndarray
    gyro: np.ndarray
    com_moving: np.ndarray
    com_counter: np.ndarray
    momentum_moving: np.ndarray
    momentum_counter: np.ndarray
    balance_scale: float
    lower: np.ndarray
    upper: np.ndarray
    com_rhs_offset: np.ndarray
    momentum_rhs_offset: np.ndarray


@dataclass(frozen=True)
class Frozen3CVelocitySolve:
    '''Store one isolated velocity solve result'''

    requested_counter_dq: np.ndarray
    accepted: bool
    diagnostics: Any = None
    objective_matrix: np.ndarray | None = None
    objective_target: np.ndarray | None = None


@dataclass(frozen=True)
class Frozen3CNominalPlan:
    '''Store one complete non-publishing frozen 3C velocity plan'''

    requested_counter_dq: np.ndarray
    accepted: bool
    com_rhs: np.ndarray
    momentum_rhs: np.ndarray
    posture_target: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    solve_diagnostics: Any = None
    objective_matrix: np.ndarray | None = None
    objective_target: np.ndarray | None = None


@dataclass(frozen=True)
class CounterCommandContext:
    '''Store prepared state needed to finalize one counter command'''

    motor_q: np.ndarray
    arm_q: np.ndarray
    arm_dq: np.ndarray
    counter_q: np.ndarray
    moving_dq: np.ndarray
    com_error: np.ndarray
    gyro: np.ndarray
    com_moving: np.ndarray
    com_counter: np.ndarray
    momentum_moving: np.ndarray
    momentum_counter: np.ndarray
    balance_scale: float


def plan_frozen_3c_velocity(
        inputs: Frozen3CNominalInput,
        config: Frozen3CPlannerConfig,
        solve_velocity: Callable[..., Frozen3CVelocitySolve],
) -> Frozen3CNominalPlan:
    '''Compute one frozen 3C velocity plan without publishing or committing'''
    com_rhs, momentum_rhs = reaction_targets(
        inputs.com_moving,
        inputs.momentum_moving,
        inputs.moving_dq,
        inputs.com_error,
        inputs.gyro,
        inputs.balance_scale,
        config.com_gain,
        config.gyro_gain,
    )
    com_rhs = com_rhs + inputs.com_rhs_offset
    momentum_rhs = momentum_rhs + inputs.momentum_rhs_offset
    posture_target = -config.posture_gain * (
        inputs.counter_q - inputs.counter_q_ref
    )
    solved = solve_velocity(
        inputs.com_counter,
        inputs.momentum_counter,
        com_rhs,
        momentum_rhs,
        posture_target,
        inputs.lower,
        inputs.upper,
        inputs.balance_scale,
    )
    return Frozen3CNominalPlan(
        requested_counter_dq=_readonly(solved.requested_counter_dq),
        accepted=bool(solved.accepted),
        com_rhs=_readonly(com_rhs),
        momentum_rhs=_readonly(momentum_rhs),
        posture_target=_readonly(posture_target),
        lower=_readonly(inputs.lower),
        upper=_readonly(inputs.upper),
        solve_diagnostics=solved.diagnostics,
        objective_matrix=_optional_readonly(solved.objective_matrix),
        objective_target=_optional_readonly(solved.objective_target),
    )


def _readonly(value):
    array = np.asarray(value, dtype=np.float64).copy()
    array.setflags(write=False)
    return array


def _optional_readonly(value):
    return None if value is None else _readonly(value)
