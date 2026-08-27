import time
from dataclasses import dataclass

import crocoddyl
import numpy as np


NQ = 4
NX = 2 * NQ


@dataclass(frozen=True)
class FrozenBalanceKnot:
    '''Store one endpoint approximation of the balance metrics'''

    q_bar: np.ndarray
    q_ref: np.ndarray
    com_bar: np.ndarray
    com_target: np.ndarray
    com_counter: np.ndarray
    com_moving: np.ndarray
    momentum_counter: np.ndarray
    momentum_moving: np.ndarray
    moving_dq: np.ndarray
    gyro_xy: np.ndarray
    com_gate: np.ndarray
    momentum_gate: np.ndarray
    q_lower: np.ndarray
    q_upper: np.ndarray
    velocity_limit: np.ndarray


@dataclass
class CounterOCPResult:
    '''Return one finite-horizon solve and its diagnostics'''

    accepted: bool
    converged: bool
    status: str
    xs: np.ndarray
    us: np.ndarray
    seed_cost: float
    optimized_cost: float
    solve_time: float
    iterations: int
    stopping_criterion: float
    warm_started: bool


class CounterBalanceActionModel(crocoddyl.ActionModelAbstract):
    '''Linear counter-arm dynamics with frozen balance maps'''

    def __init__(self, dt, weights, gains, scales):
        self.dt = float(dt)
        self.weights = dict(weights)
        self.gains = dict(gains)
        self.scales = dict(scales)
        self.knot = None
        self._fx = np.block([
            [np.eye(NQ), self.dt * np.eye(NQ)],
            [np.zeros((NQ, NQ)), np.eye(NQ)],
        ])
        self._fu = np.vstack([
            0.5 * self.dt ** 2 * np.eye(NQ),
            self.dt * np.eye(NQ),
        ])
        super().__init__(crocoddyl.StateVector(NX), NQ)

    def update(self, knot, control_lower, control_upper):
        '''Update one knot and its acceleration bounds'''
        self.knot = _validated_knot(knot)
        self.u_lb = _vector(control_lower, NQ, 'control_lower')
        self.u_ub = _vector(control_upper, NQ, 'control_upper')
        if np.any(self.u_lb > self.u_ub):
            raise ValueError('control bounds are infeasible')

    def calc(self, data, x, u=None):
        x = _vector(x, NX, 'state')
        u = _vector(u, NQ, 'control')
        data.xnext[:] = self._fx @ x + self._fu @ u
        residuals = self._residuals(data.xnext, u)
        data.cost = sum(
            0.5 * weight * float(residual @ residual)
            for weight, residual, _ in residuals
        )

    def calcDiff(self, data, x, u=None):
        x = _vector(x, NX, 'state')
        u = _vector(u, NQ, 'control')
        xnext = self._fx @ x + self._fu @ u
        residuals = self._residuals(xnext, u)
        ly = np.zeros(NX, dtype=np.float64)
        lu_direct = np.zeros(NQ, dtype=np.float64)
        lyy = np.zeros((NX, NX), dtype=np.float64)
        luu_direct = np.zeros((NQ, NQ), dtype=np.float64)
        for weight, residual, (jy, ju) in residuals:
            ly += weight * jy.T @ residual
            lyy += weight * jy.T @ jy
            if ju is not None:
                lu_direct += weight * ju.T @ residual
                luu_direct += weight * ju.T @ ju

        data.Fx[:] = self._fx
        data.Fu[:] = self._fu
        data.Lx[:] = self._fx.T @ ly
        data.Lu[:] = self._fu.T @ ly + lu_direct
        data.Lxx[:] = self._fx.T @ lyy @ self._fx
        data.Lxu[:] = self._fx.T @ lyy @ self._fu
        data.Luu[:] = self._fu.T @ lyy @ self._fu + luu_direct

    def _residuals(self, xnext, u):
        if self.knot is None:
            raise RuntimeError('action model knot is not configured')
        q = xnext[:NQ]
        dq = xnext[NQ:]
        knot = self.knot
        com_scale = self.scales['com_velocity']
        momentum_scale = self.scales['momentum']
        posture_scale = self.scales['posture']
        acceleration_scale = self.scales['acceleration']

        com_gate = np.sqrt(knot.com_gate)
        momentum_gate = np.sqrt(knot.momentum_gate)
        com_jy = np.hstack([
            self.gains['com'] * knot.com_counter,
            knot.com_counter,
        ]) / com_scale
        com_jy = com_gate[:, None] * com_jy
        com_residual = (
            knot.com_counter @ dq
            + knot.com_moving @ knot.moving_dq
            + self.gains['com'] * (
                knot.com_bar
                + knot.com_counter @ (q - knot.q_bar)
                - knot.com_target
            )
        ) / com_scale
        com_residual *= com_gate

        momentum_jy = np.hstack([
            np.zeros((2, NQ), dtype=np.float64),
            knot.momentum_counter,
        ]) / momentum_scale
        momentum_jy = momentum_gate[:, None] * momentum_jy
        momentum_residual = (
            knot.momentum_counter @ dq
            + knot.momentum_moving @ knot.moving_dq
            - self.gains['gyro'] * knot.gyro_xy
        ) / momentum_scale
        momentum_residual *= momentum_gate

        posture_jy = np.hstack([
            np.eye(NQ), np.zeros((NQ, NQ), dtype=np.float64),
        ]) / posture_scale
        posture_residual = (q - knot.q_ref) / posture_scale
        acceleration_ju = np.eye(NQ) / acceleration_scale
        acceleration_residual = u / acceleration_scale
        velocity_jy = np.hstack([
            np.zeros((NQ, NQ), dtype=np.float64),
            np.diag(1.0 / knot.velocity_limit),
        ])
        velocity_residual = dq / knot.velocity_limit
        span = knot.q_upper - knot.q_lower
        margin = 0.1 * span
        soft_lower = knot.q_lower + margin
        soft_upper = knot.q_upper - margin
        limit_residual = np.zeros(NQ, dtype=np.float64)
        limit_derivative = np.zeros(NQ, dtype=np.float64)
        below = q < soft_lower
        above = q > soft_upper
        limit_residual[below] = (q[below] - soft_lower[below]) / margin[below]
        limit_residual[above] = (q[above] - soft_upper[above]) / margin[above]
        limit_derivative[below | above] = 1.0 / margin[below | above]
        limit_jy = np.hstack([
            np.diag(limit_derivative),
            np.zeros((NQ, NQ), dtype=np.float64),
        ])
        return (
            (self.weights['com'], com_residual, (com_jy, None)),
            (
                self.weights['momentum'],
                momentum_residual,
                (momentum_jy, None),
            ),
            (
                self.weights['posture'],
                posture_residual,
                (posture_jy, None),
            ),
            (
                self.weights['acceleration'],
                acceleration_residual,
                (np.zeros((NQ, NX)), acceleration_ju),
            ),
            (
                self.weights['velocity'],
                velocity_residual,
                (velocity_jy, None),
            ),
            (
                self.weights['limit'],
                limit_residual,
                (limit_jy, None),
            ),
        )


class CounterTerminalActionModel(crocoddyl.ActionModelAbstract):
    '''State-only terminal regularization for the counter arm'''

    def __init__(self, scales, weights):
        self.scales = dict(scales)
        self.weights = dict(weights)
        self.q_ref = np.zeros(NQ, dtype=np.float64)
        super().__init__(crocoddyl.StateVector(NX), 0)

    def update(self, q_ref):
        '''Update the terminal posture reference'''
        self.q_ref = _vector(q_ref, NQ, 'q_ref')

    def calc(self, data, x, u=None):
        x = _vector(x, NX, 'state')
        data.xnext[:] = x
        q_residual = (x[:NQ] - self.q_ref) / self.scales['posture']
        dq_residual = x[NQ:] / self.scales['velocity']
        data.cost = (
            0.5 * self.weights['terminal_posture']
            * float(q_residual @ q_residual)
            + 0.5 * self.weights['terminal_velocity']
            * float(dq_residual @ dq_residual)
        )

    def calcDiff(self, data, x, u=None):
        x = _vector(x, NX, 'state')
        data.Fx[:] = np.eye(NX)
        data.Lx[:] = 0.0
        data.Lxx[:] = 0.0
        q_scale = self.scales['posture']
        dq_scale = self.scales['velocity']
        data.Lx[:NQ] = (
            self.weights['terminal_posture']
            * (x[:NQ] - self.q_ref) / q_scale ** 2
        )
        data.Lx[NQ:] = (
            self.weights['terminal_velocity'] * x[NQ:] / dq_scale ** 2
        )
        data.Lxx[:NQ, :NQ] = (
            self.weights['terminal_posture'] / q_scale ** 2 * np.eye(NQ)
        )
        data.Lxx[NQ:, NQ:] = (
            self.weights['terminal_velocity'] / dq_scale ** 2 * np.eye(NQ)
        )


class CounterDDPOCP:
    '''Warm-started bounded finite-horizon counter-arm OCP'''

    def __init__(self, dt=0.02, horizon_steps=5, max_iterations=2,
                 weights=None, gains=None, scales=None,
                 initial_regularization=1e-6,
                 minimum_cost_improvement=0.0):
        self.dt = float(dt)
        self.horizon_steps = int(horizon_steps)
        self.max_iterations = int(max_iterations)
        self.initial_regularization = float(initial_regularization)
        self.minimum_cost_improvement = float(minimum_cost_improvement)
        self.weights = {
            'com': 1.0,
            'momentum': 2.0,
            'posture': 0.02,
            'acceleration': 0.01,
            'velocity': 0.02,
            'limit': 10.0,
            'terminal_posture': 0.0,
            'terminal_velocity': 0.0,
            **(weights or {}),
        }
        self.gains = {'com': 2.0, 'gyro': 0.2, **(gains or {})}
        self.scales = {
            'com_velocity': 0.1,
            'momentum': 1.0,
            'posture': 1.0,
            'acceleration': 25.0,
            'velocity': 1.0,
            **(scales or {}),
        }
        self._validate_config()
        self.running_models = [
            CounterBalanceActionModel(
                self.dt, self.weights, self.gains, self.scales,
            )
            for _ in range(self.horizon_steps)
        ]
        self.terminal_model = CounterTerminalActionModel(
            self.scales, self.weights,
        )
        self.problem = crocoddyl.ShootingProblem(
            np.zeros(NX), self.running_models, self.terminal_model,
        )
        self.solver = crocoddyl.SolverBoxFDDP(self.problem)
        self._last_us = None

    def reset(self):
        '''Clear the shifted warm start'''
        self._last_us = None

    def solve(self, x0, knots, control_lower, control_upper, q_ref):
        '''Solve one bounded finite-horizon problem'''
        started = time.perf_counter()
        x0 = _vector(x0, NX, 'x0')
        lower = _matrix(
            control_lower, (self.horizon_steps, NQ), 'control_lower',
        )
        upper = _matrix(
            control_upper, (self.horizon_steps, NQ), 'control_upper',
        )
        if len(knots) != self.horizon_steps:
            raise ValueError('knots must match horizon_steps')
        self.problem.x0 = x0
        for model, knot, lb, ub in zip(
                self.running_models, knots, lower, upper):
            model.update(knot, lb, ub)
        self.terminal_model.update(q_ref)

        warm_started = self._last_us is not None
        us = self._seed_controls(lower, upper)
        seed_us = [np.copy(value) for value in us]
        xs = list(self.problem.rollout(seed_us))
        seed_cost = float(self.problem.calc(xs, seed_us))
        try:
            converged = bool(self.solver.solve(
                xs,
                seed_us,
                self.max_iterations,
                True,
                self.initial_regularization,
            ))
        except Exception:
            return self._failure('solver_failure', started, warm_started)

        solved_xs = np.asarray(self.solver.xs, dtype=np.float64)
        solved_us = np.asarray(self.solver.us, dtype=np.float64)
        optimized_cost = float(self.solver.cost)
        valid = (
            solved_xs.shape == (self.horizon_steps + 1, NX)
            and solved_us.shape == (self.horizon_steps, NQ)
            and np.all(np.isfinite(solved_xs))
            and np.all(np.isfinite(solved_us))
            and np.isfinite(optimized_cost)
            and np.all(solved_us >= lower - 1e-8)
            and np.all(solved_us <= upper + 1e-8)
        )
        if not valid:
            return self._failure('invalid_solution', started, warm_started)
        improvement = seed_cost - optimized_cost
        if improvement + 1e-12 < self.minimum_cost_improvement:
            return self._result(
                False, converged, 'no_cost_improvement', solved_xs,
                solved_us, seed_cost, optimized_cost, started, warm_started,
            )
        self._last_us = np.copy(solved_us)
        status = 'solved' if converged else 'best_effort'
        return self._result(
            True, converged, status, solved_xs, solved_us, seed_cost,
            optimized_cost, started, warm_started,
        )

    def _seed_controls(self, lower, upper):
        if self._last_us is None:
            seed = np.zeros((self.horizon_steps, NQ), dtype=np.float64)
        else:
            seed = np.vstack([self._last_us[1:], self._last_us[-1]])
        return np.clip(seed, lower, upper)

    def _failure(self, status, started, warm_started):
        return self._result(
            False,
            False,
            status,
            np.empty((0, NX), dtype=np.float64),
            np.empty((0, NQ), dtype=np.float64),
            np.nan,
            np.nan,
            started,
            warm_started,
        )

    def _result(self, accepted, converged, status, xs, us, seed_cost,
                optimized_cost, started, warm_started):
        return CounterOCPResult(
            accepted=accepted,
            converged=converged,
            status=status,
            xs=xs,
            us=us,
            seed_cost=seed_cost,
            optimized_cost=optimized_cost,
            solve_time=time.perf_counter() - started,
            iterations=int(getattr(self.solver, 'iter', 0)),
            stopping_criterion=float(getattr(self.solver, 'stop', np.nan)),
            warm_started=warm_started,
        )

    def _validate_config(self):
        if not np.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError('counter_ddp.dt must be finite and positive')
        if self.horizon_steps < 1:
            raise ValueError('counter_ddp.horizon_steps must be positive')
        if self.max_iterations < 0:
            raise ValueError('counter_ddp.max_iterations must be nonnegative')
        values = [
            self.initial_regularization,
            self.minimum_cost_improvement,
            *self.weights.values(),
            *self.gains.values(),
            *self.scales.values(),
        ]
        if not np.all(np.isfinite(values)) or np.any(np.asarray(values) < 0.0):
            raise ValueError('counter_ddp parameters must be finite and nonnegative')
        if self.initial_regularization <= 0.0:
            raise ValueError('counter_ddp.initial_regularization must be positive')
        if any(self.scales[name] <= 0.0 for name in self.scales):
            raise ValueError('counter_ddp scales must be positive')


def _validated_knot(knot):
    if not isinstance(knot, FrozenBalanceKnot):
        raise ValueError('knot must be FrozenBalanceKnot')
    values = {
        'q_bar': (knot.q_bar, (NQ,)),
        'q_ref': (knot.q_ref, (NQ,)),
        'com_bar': (knot.com_bar, (2,)),
        'com_target': (knot.com_target, (2,)),
        'com_counter': (knot.com_counter, (2, NQ)),
        'com_moving': (knot.com_moving, (2, 7)),
        'momentum_counter': (knot.momentum_counter, (2, NQ)),
        'momentum_moving': (knot.momentum_moving, (2, 7)),
        'moving_dq': (knot.moving_dq, (7,)),
        'gyro_xy': (knot.gyro_xy, (2,)),
        'com_gate': (knot.com_gate, (2,)),
        'momentum_gate': (knot.momentum_gate, (2,)),
        'q_lower': (knot.q_lower, (NQ,)),
        'q_upper': (knot.q_upper, (NQ,)),
        'velocity_limit': (knot.velocity_limit, (NQ,)),
    }
    normalized = {}
    for name, (value, shape) in values.items():
        normalized[name] = _matrix(value, shape, name)
    for name in ('com_gate', 'momentum_gate'):
        if np.any(normalized[name] < 0.0) or np.any(normalized[name] > 1.0):
            raise ValueError(f'{name} must be in [0, 1]')
    if np.any(normalized['q_lower'] >= normalized['q_upper']):
        raise ValueError('knot position limits are infeasible')
    if np.any(normalized['velocity_limit'] <= 0.0):
        raise ValueError('knot velocity limits must be positive')
    return FrozenBalanceKnot(**normalized)


def _vector(value, size, name):
    return _matrix(value, (size,), name)


def _matrix(value, shape, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite with shape {shape}')
    return np.copy(result)
