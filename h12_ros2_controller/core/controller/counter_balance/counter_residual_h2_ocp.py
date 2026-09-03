import time
from dataclasses import dataclass

import crocoddyl
import numpy as np


NQ = 4
PLANAR = 2
NX = 12
TILT = slice(0, 2)
RATE = slice(2, 4)
COUNTER_Q = slice(4, 8)
PENDING = slice(8, 12)


@dataclass(frozen=True)
class ResidualH2Context:
    '''Store one fixed local H2 response linearization'''

    nominal_tilt: np.ndarray
    nominal_rate: np.ndarray
    u5_gain: np.ndarray
    r5_gain: np.ndarray
    momentum_map: np.ndarray
    confidence: np.ndarray


@dataclass(frozen=True)
class ResidualH2Result:
    '''Store one H2 residual solve result'''

    accepted: bool
    converged: bool
    status: str
    residual: np.ndarray
    xs: np.ndarray
    us: np.ndarray
    seed_cost: float
    optimized_cost: float
    solve_time: float
    iterations: int
    stopping_criterion: float
    warm_started: bool


class ResidualH2ActionModel(crocoddyl.ActionModelAbstract):
    '''Propagate one delay-aware residual H2 knot'''

    def __init__(self, dt, stage, action_weight, change_weight):
        self.dt = float(dt)
        self.stage = int(stage)
        self.action_weight = float(action_weight)
        self.change_weight = float(change_weight)
        self.context = None
        self._fx = np.eye(NX)
        self._fu = np.zeros((NX, NQ))
        super().__init__(crocoddyl.StateVector(NX), NQ)

    def update(self, context, lower, upper):
        '''Update one local model and residual bounds'''
        self.context = _validated_context(context)
        lower = _vector(lower, NQ, 'lower')
        upper = _vector(upper, NQ, 'upper')
        if np.any(lower > upper):
            raise ValueError('residual bounds are infeasible')
        fixed = np.isclose(lower, upper, atol=1e-12)
        solve_lower = np.copy(lower)
        solve_upper = np.copy(upper)
        solve_lower[fixed] -= 1e-10
        solve_upper[fixed] += 1e-10
        self.u_lb = solve_lower
        self.u_ub = solve_upper
        self._update_dynamics()

    def calc(self, data, x, u=None):
        x = _vector(x, NX, 'x')
        u = _vector(u, NQ, 'u')
        data.xnext[:] = self._dynamics(x, u)
        action_scale = 0.1
        change = u - x[PENDING]
        data.cost = 0.5 * (
            self.action_weight * float(u @ u) / action_scale ** 2
            + self.change_weight * float(change @ change) / action_scale ** 2
        )

    def calcDiff(self, data, x, u=None):
        x = _vector(x, NX, 'x')
        u = _vector(u, NQ, 'u')
        action_scale = 0.1
        scale2 = action_scale ** 2
        change = u - x[PENDING]
        data.Fx[:] = self._fx
        data.Fu[:] = self._fu
        data.Lx[:] = 0.0
        data.Lu[:] = (
            self.action_weight * u + self.change_weight * change
        ) / scale2
        data.Lx[PENDING] = -self.change_weight * change / scale2
        data.Lxx[:] = 0.0
        data.Lxx[PENDING, PENDING] = (
            self.change_weight / scale2 * np.eye(NQ)
        )
        data.Lxu[:] = 0.0
        data.Lxu[PENDING, :] = (
            -self.change_weight / scale2 * np.eye(NQ)
        )
        data.Luu[:] = (
            (self.action_weight + self.change_weight)
            / scale2 * np.eye(NQ)
        )

    def _dynamics(self, x, u):
        result = np.copy(x)
        if self.stage == 1:
            realized = self.context.u5_gain * x[PENDING]
            momentum = self.context.momentum_map @ realized
            rate_delta = (
                self.context.confidence.astype(np.float64)
                * (self.context.r5_gain @ momentum)
            )
            result[TILT] += 0.5 * self.dt * rate_delta
            result[RATE] += rate_delta
            result[COUNTER_Q] += self.dt * realized
        result[PENDING] = u
        return result

    def _update_dynamics(self):
        self._fx[:] = np.eye(NX)
        self._fx[PENDING, :] = 0.0
        self._fu[:] = 0.0
        self._fu[PENDING, :] = np.eye(NQ)
        if self.stage != 1:
            return
        realization = np.diag(self.context.u5_gain)
        response = (
            np.diag(self.context.confidence.astype(np.float64))
            @ self.context.r5_gain
            @ self.context.momentum_map
            @ realization
        )
        self._fx[TILT, PENDING] = 0.5 * self.dt * response
        self._fx[RATE, PENDING] = response
        self._fx[COUNTER_Q, PENDING] = self.dt * realization


class ResidualH2TerminalModel(crocoddyl.ActionModelAbstract):
    '''Penalize terminal absolute response and consumed counter reserve'''

    def __init__(self, weights):
        self.weights = dict(weights)
        self.context = None
        super().__init__(crocoddyl.StateVector(NX), 0)

    def update(self, context):
        '''Update terminal nominal response'''
        self.context = _validated_context(context)

    def calc(self, data, x, u=None):
        x = _vector(x, NX, 'x')
        data.xnext[:] = x
        tilt = self.context.nominal_tilt[1] + x[TILT]
        rate = self.context.nominal_rate[1] + x[RATE]
        divergence = np.maximum(tilt * rate, 0.0)
        confidence = self.context.confidence.astype(np.float64)
        data.cost = 0.5 * (
            self.weights['tilt'] * float((confidence * tilt) @ tilt)
            / 0.05 ** 2
            + self.weights['rate'] * float((confidence * rate) @ rate)
            / 0.2 ** 2
            + self.weights['divergence']
            * float((confidence * divergence) @ divergence) / 0.01 ** 2
            + self.weights['reserve']
            * float(x[COUNTER_Q] @ x[COUNTER_Q]) / 0.1 ** 2
        )

    def calcDiff(self, data, x, u=None):
        x = _vector(x, NX, 'x')
        tilt = self.context.nominal_tilt[1] + x[TILT]
        rate = self.context.nominal_rate[1] + x[RATE]
        active = tilt * rate > 0.0
        divergence = np.where(active, tilt * rate, 0.0)
        confidence = self.context.confidence.astype(np.float64)
        data.Fx[:] = np.eye(NX)
        data.Lx[:] = 0.0
        data.Lxx[:] = 0.0
        data.Lx[TILT] = (
            confidence * self.weights['tilt'] * tilt / 0.05 ** 2
        )
        data.Lx[RATE] = (
            confidence * self.weights['rate'] * rate / 0.2 ** 2
        )
        data.Lxx[TILT, TILT] = (
            self.weights['tilt'] / 0.05 ** 2 * np.diag(confidence)
        )
        data.Lxx[RATE, RATE] = (
            self.weights['rate'] / 0.2 ** 2 * np.diag(confidence)
        )
        for axis in range(PLANAR):
            if not active[axis] or not self.context.confidence[axis]:
                continue
            weight = self.weights['divergence'] / 0.01 ** 2
            data.Lx[axis] += weight * divergence[axis] * rate[axis]
            data.Lx[2 + axis] += weight * divergence[axis] * tilt[axis]
            jacobian = np.array([rate[axis], tilt[axis]])
            hessian = weight * np.outer(jacobian, jacobian)
            indices = np.array([axis, 2 + axis])
            data.Lxx[np.ix_(indices, indices)] += hessian
        data.Lx[COUNTER_Q] = (
            self.weights['reserve'] * x[COUNTER_Q] / 0.1 ** 2
        )
        data.Lxx[COUNTER_Q, COUNTER_Q] = (
            self.weights['reserve'] / 0.1 ** 2 * np.eye(NQ)
        )


class ResidualH2OCP:
    '''Solve one fixed-size two-knot Crocoddyl residual problem'''

    def __init__(self, dt=0.02, max_iterations=1, weights=None):
        self.dt = float(dt)
        self.max_iterations = int(max_iterations)
        self.weights = {
            'action': 1.0,
            'change': 0.5,
            'tilt': 2.0,
            'rate': 1.0,
            'divergence': 2.0,
            'reserve': 0.1,
            **(weights or {}),
        }
        self.running_models = [
            ResidualH2ActionModel(
                self.dt,
                stage,
                self.weights['action'],
                self.weights['change'],
            )
            for stage in range(2)
        ]
        self.terminal_model = ResidualH2TerminalModel(self.weights)
        self.problem = crocoddyl.ShootingProblem(
            np.zeros(NX), self.running_models, self.terminal_model,
        )
        self.solver = crocoddyl.SolverBoxFDDP(self.problem)
        self.solver.th_stop = 1e-9
        self._last_us = None

    def reset(self):
        '''Clear shifted residual warm start'''
        self._last_us = None

    def solve(self, context, lower, upper, pending=None):
        '''Solve H2 and return one bounded residual sequence'''
        started = time.perf_counter()
        context = _validated_context(context)
        lower = _vector(lower, NQ, 'lower')
        upper = _vector(upper, NQ, 'upper')
        if np.any(lower > upper):
            raise ValueError('residual bounds are infeasible')
        x0 = np.zeros(NX, dtype=np.float64)
        if pending is not None:
            x0[PENDING] = _vector(pending, NQ, 'pending')
        self.problem.x0 = x0
        for model in self.running_models:
            model.update(context, lower, upper)
        self.terminal_model.update(context)
        warm_started = self._last_us is not None
        if warm_started:
            controls = [
                np.clip(self._last_us[1], lower, upper),
                np.clip(self._last_us[1], lower, upper),
            ]
        else:
            controls = [np.zeros(NQ), np.zeros(NQ)]
        states = list(self.problem.rollout(controls))
        seed_cost = float(self.problem.calc(states, controls))
        try:
            converged = bool(self.solver.solve(
                states, controls, self.max_iterations, True, 1e-6,
            ))
        except Exception:
            return self._failure('solver_failure', started, warm_started)
        xs = np.asarray(self.solver.xs, dtype=np.float64)
        us = np.asarray(self.solver.us, dtype=np.float64)
        valid = bool(
            xs.shape == (3, NX)
            and us.shape == (2, NQ)
            and np.all(np.isfinite(xs))
            and np.all(np.isfinite(us))
            and np.all(us >= lower - 1e-8)
            and np.all(us <= upper + 1e-8)
            and np.all(np.abs(us[:, 2]) <= 1e-8)
        )
        if not valid:
            return self._failure('invalid_solution', started, warm_started)
        optimized_cost = float(self.solver.cost)
        self._last_us = np.copy(us)
        return ResidualH2Result(
            accepted=True,
            converged=converged,
            status='solved' if converged else 'best_effort',
            residual=np.clip(us[0], lower, upper),
            xs=xs,
            us=us,
            seed_cost=seed_cost,
            optimized_cost=optimized_cost,
            solve_time=time.perf_counter() - started,
            iterations=int(self.solver.iter),
            stopping_criterion=float(self.solver.stop),
            warm_started=warm_started,
        )

    def _failure(self, status, started, warm_started):
        return ResidualH2Result(
            accepted=False,
            converged=False,
            status=status,
            residual=np.zeros(NQ),
            xs=np.empty((0, NX)),
            us=np.empty((0, NQ)),
            seed_cost=np.nan,
            optimized_cost=np.nan,
            solve_time=time.perf_counter() - started,
            iterations=0,
            stopping_criterion=np.nan,
            warm_started=warm_started,
        )


def _validated_context(context):
    if not isinstance(context, ResidualH2Context):
        raise ValueError('context must be ResidualH2Context')
    values = {
        'nominal_tilt': (context.nominal_tilt, (2, 2)),
        'nominal_rate': (context.nominal_rate, (2, 2)),
        'u5_gain': (context.u5_gain, (4,)),
        'r5_gain': (context.r5_gain, (2, 2)),
        'momentum_map': (context.momentum_map, (2, 4)),
    }
    normalized = {
        name: _matrix(value, shape, name)
        for name, (value, shape) in values.items()
    }
    confidence = np.asarray(context.confidence)
    if confidence.shape != (2,) or confidence.dtype != np.bool_:
        raise ValueError('confidence must contain two booleans')
    confidence = np.copy(confidence)
    confidence.setflags(write=False)
    return ResidualH2Context(confidence=confidence, **normalized)


def _vector(value, size, name):
    return _matrix(value, (size,), name)


def _matrix(value, shape, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be finite with shape {shape}')
    return np.copy(result)
