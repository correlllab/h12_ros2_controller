import time
from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import lsq_linear

from h12_ros2_controller.core.controller.counter_balance.objective import (
    CounterVelocityBoundsError,
    bounded_velocity_problem,
)
from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    Frozen3CVelocitySolve,
)


BVLS_TOL = 1e-10
BVLS_MAX_ITER = 100
TRF_TOL = 1e-10
TRF_MAX_ITER = 100
KKT_TOLERANCE = 5e-4
BOUND_ACTIVITY_TOLERANCE = 1e-10
OBJECTIVE_ATOL = 1e-12
OBJECTIVE_RTOL = 1e-10


@dataclass(frozen=True)
class ScipyNominalConfig:
    '''Store weights and normalization scales for the frozen objective'''

    com_weight: float
    momentum_weight: float
    posture_weight: float
    damping: float
    com_velocity_scale: float
    momentum_scale: float
    posture_velocity_scale: float


@dataclass(frozen=True)
class ScipySolveAttempt:
    '''Store independently validated diagnostics for one backend attempt'''

    backend: str
    accepted: bool
    converged: bool
    cost: float
    kkt_violation: float
    iterations: int
    solve_time: float
    status: str


@dataclass(frozen=True)
class ScipyVelocitySolve:
    '''Store immutable velocity diagnostics compatible with velocity DDP'''

    velocity: np.ndarray
    accepted: bool = False
    converged: bool = False
    cost: float = np.inf
    solve_time: float = 0.0
    iterations: int = 0
    stopping_criterion: float = np.inf
    kkt_violation: float = np.inf
    regularization: float = 0.0
    boxqp_polished: bool = False
    backend: str = 'none'
    retry_used: bool = False
    primary_accepted: bool = False
    status: str = 'invalid_input'
    attempts: tuple[ScipySolveAttempt, ...] = ()


def _readonly(value):
    return np.frombuffer(np.asarray(value, dtype=np.float64).tobytes(),
                         dtype=np.float64).reshape(np.shape(value))


def _validate(matrix, target, lower, upper, velocity, baseline_cost):
    '''Validate the original objective and box without solver metadata'''
    if velocity.shape != (4,) or not np.all(np.isfinite(velocity)):
        return False, np.inf, np.inf, 'invalid_candidate'
    if np.any(velocity < lower) or np.any(velocity > upper):
        return False, np.inf, np.inf, 'out_of_bounds'
    with np.errstate(over='ignore', invalid='ignore'):
        residual = matrix @ velocity - target
        cost = 0.5 * float(residual @ residual)
        gradient = matrix.T @ residual
        # cap activity tolerance so narrow, nonzero intervals stay free
        activity = np.minimum(
            BOUND_ACTIVITY_TOLERANCE, upper * 0.25 - lower * 0.25,
        )
        violation = np.abs(gradient)
        at_lower = velocity - lower <= activity
        at_upper = upper - velocity <= activity
        violation[at_lower] = np.maximum(-gradient[at_lower], 0.0)
        violation[at_upper] = np.maximum(gradient[at_upper], 0.0)
        violation[lower == upper] = 0.0
        kkt = float(np.max(violation))
    if not (np.isfinite(cost) and np.all(np.isfinite(gradient))
            and np.isfinite(baseline_cost) and np.isfinite(kkt)):
        return False, cost, kkt, 'nonfinite_objective'
    if cost > baseline_cost + OBJECTIVE_ATOL + OBJECTIVE_RTOL * baseline_cost:
        return False, cost, kkt, 'objective_regression'
    if kkt > KKT_TOLERANCE:
        return False, cost, kkt, 'poor_kkt'
    return True, cost, kkt, 'accepted'


def solve_bounded_least_squares(matrix, target, lower, upper) -> ScipyVelocitySolve:
    '''Solve a four-column box least-squares problem with one optional retry'''
    started = time.perf_counter()
    rejected = ScipyVelocitySolve(velocity=_readonly(np.zeros(4)))
    try:
        matrix, target, lower, upper = (
            np.array(value, dtype=np.float64, copy=True)
            for value in (matrix, target, lower, upper)
        )
        if (matrix.ndim != 2 or matrix.shape[1] != 4
                or matrix.shape[0] == 0
                or target.shape != (matrix.shape[0],)
                or lower.shape != (4,) or upper.shape != (4,)
                or not all(np.all(np.isfinite(value)) for value in (
                    matrix, target, lower, upper,
                ))):
            raise ValueError('Invalid least-squares input')
    except (ValueError, TypeError, OverflowError):
        return replace(rejected, solve_time=time.perf_counter() - started)
    if np.any(lower > upper):
        raise CounterVelocityBoundsError('Counter velocity bounds are empty')
    baseline = np.minimum(np.maximum(np.zeros(4), lower), upper)
    with np.errstate(over='ignore', invalid='ignore'):
        residual = matrix @ baseline - target
        baseline_cost = 0.5 * float(residual @ residual)
    fixed = lower == upper
    free = ~fixed
    with np.errstate(over='ignore', invalid='ignore'):
        reduced_target = target - matrix[:, fixed] @ lower[fixed]
    if not np.isfinite(baseline_cost) or not np.all(np.isfinite(reduced_target)):
        return replace(rejected, velocity=_readonly(baseline),
                       solve_time=time.perf_counter() - started)
    attempts = []
    methods = ('bvls', 'trf') if np.any(free) else ('fixed',)
    for backend in methods:
        attempt_started = time.perf_counter()
        converged = False
        iterations = 0
        velocity = baseline.copy()
        try:
            if backend != 'fixed':
                result = lsq_linear(
                    matrix[:, free].copy(), reduced_target.copy(),
                    bounds=(lower[free].copy(), upper[free].copy()),
                    method=backend, lsq_solver='exact',
                    tol=BVLS_TOL if backend == 'bvls' else TRF_TOL,
                    max_iter=(BVLS_MAX_ITER if backend == 'bvls'
                              else TRF_MAX_ITER),
                )
                candidate = np.asarray(result.x, dtype=np.float64)
                if candidate.shape != (int(np.sum(free)),):
                    raise ValueError('Invalid candidate shape')
                velocity[free] = candidate
                # metadata is advisory only and cannot veto a valid candidate
                try:
                    converged = bool(getattr(result, 'success', False))
                    iterations = max(0, int(getattr(result, 'nit', 0)))
                except (TypeError, ValueError, OverflowError):
                    pass
            accepted, cost, kkt, status = _validate(
                matrix, target, lower, upper, velocity, baseline_cost,
            )
            if backend == 'fixed':
                converged = accepted
        except Exception as error:
            accepted, cost, kkt = False, np.inf, np.inf
            status = f'exception:{type(error).__name__}'
        attempts.append(ScipySolveAttempt(
            backend, accepted, converged, cost, kkt, iterations,
            time.perf_counter() - attempt_started, status,
        ))
        if accepted:
            break
    return ScipyVelocitySolve(
        velocity=_readonly(velocity if accepted else baseline),
        accepted=accepted, converged=converged, cost=cost,
        solve_time=time.perf_counter() - started,
        iterations=sum(attempt.iterations for attempt in attempts),
        stopping_criterion=kkt, kkt_violation=kkt, backend=backend,
        retry_used=len(attempts) == 2,
        primary_accepted=attempts[0].accepted, status=status,
        attempts=tuple(attempts),
    )


def solve_scipy_nominal(com_counter, momentum_counter, com_rhs, momentum_rhs,
                        posture_target, lower, upper, balance_scale,
                        config: ScipyNominalConfig) -> Frozen3CVelocitySolve:
    '''Assemble the exact frozen affine objective and solve without publishing'''
    started = time.perf_counter()
    matrix = target = None
    try:
        arrays = tuple(np.array(value, dtype=np.float64, copy=True) for value in (
            com_counter, momentum_counter, com_rhs, momentum_rhs, posture_target,
        ))
        shapes = ((2, 4), (2, 4), (2,), (2,), (4,))
        weights = np.asarray((config.com_weight, config.momentum_weight,
                              config.posture_weight, config.damping),
                             dtype=np.float64)
        scales = np.asarray((config.com_velocity_scale, config.momentum_scale,
                             config.posture_velocity_scale), dtype=np.float64)
        balance = np.asarray(balance_scale, dtype=np.float64)
        if (any(value.shape != shape or not np.all(np.isfinite(value))
                for value, shape in zip(arrays, shapes))
                or weights.shape != (4,) or scales.shape != (3,)
                or balance.shape != () or not np.isfinite(balance)
                or balance < 0.0 or not np.all(np.isfinite(weights))
                or not np.all(np.isfinite(scales))
                or np.any(weights < 0.0) or np.any(scales <= 0.0)):
            raise ValueError('Invalid nominal input')
        with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
            matrix, target = bounded_velocity_problem(
                *arrays, float(balance), *weights, *scales,
            )
    except (ValueError, TypeError, OverflowError, AttributeError):
        result = ScipyVelocitySolve(velocity=_readonly(np.zeros(4)))
    else:
        result = solve_bounded_least_squares(matrix, target, lower, upper)
    result = replace(result, solve_time=time.perf_counter() - started)
    return Frozen3CVelocitySolve(
        requested_counter_dq=result.velocity, accepted=result.accepted,
        diagnostics=result,
        objective_matrix=None if matrix is None else _readonly(matrix),
        objective_target=None if target is None else _readonly(target),
    )
