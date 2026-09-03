import time
from dataclasses import dataclass

import crocoddyl
import numpy as np

from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_ocp import (
    COUNTER_Q,
    NQ,
    NX,
    PENDING,
    PLANAR,
    RATE,
    TILT,
    ResidualH2Context,
    ResidualH2TerminalModel,
)


@dataclass(frozen=True)
class ResidualH3Result:
    '''Store one H3 residual solve result'''

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


class ResidualH3ActionModel(crocoddyl.ActionModelAbstract):
    '''Propagate one realized-velocity H3 residual knot'''

    def __init__(self, dt, stage, action_weight, change_weight):
        self.dt = float(dt)
        self.stage = int(stage)
        self.action_weight = float(action_weight)
        self.change_weight = float(change_weight)
        self.context = None
        self.carryover = np.zeros(NQ)
        self._fx = np.eye(NX)
        self._fu = np.zeros((NX, NQ))
        super().__init__(crocoddyl.StateVector(NX), NQ)

    def update(self, context, carryover, lower, upper):
        '''Update one fixed local H3 knot'''
        if not isinstance(context, ResidualH2Context):
            raise ValueError('context must be ResidualH2Context')
        self.context = context
        self.carryover = _vector(carryover, NQ, 'carryover')
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
        change = u - x[PENDING]
        data.cost = 0.5 * (
            self.action_weight * float(u @ u) / 0.1 ** 2
            + self.change_weight * float(change @ change) / 0.1 ** 2
        )

    def calcDiff(self, data, x, u=None):
        x = _vector(x, NX, 'x')
        u = _vector(u, NQ, 'u')
        change = u - x[PENDING]
        data.Fx[:] = self._fx
        data.Fu[:] = self._fu
        data.Lx[:] = 0.0
        data.Lx[PENDING] = -self.change_weight * change / 0.1 ** 2
        data.Lu[:] = (
            self.action_weight * u + self.change_weight * change
        ) / 0.1 ** 2
        data.Lxx[:] = 0.0
        data.Lxx[PENDING, PENDING] = (
            self.change_weight / 0.1 ** 2 * np.eye(NQ)
        )
        data.Lxu[:] = 0.0
        data.Lxu[PENDING, :] = (
            -self.change_weight / 0.1 ** 2 * np.eye(NQ)
        )
        data.Luu[:] = (
            (self.action_weight + self.change_weight)
            / 0.1 ** 2 * np.eye(NQ)
        )

    def _response(self, realized):
        momentum = self.context.momentum_map @ realized
        return (
            self.context.confidence.astype(np.float64)
            * (self.context.r5_gain @ momentum)
        )

    def _dynamics(self, x, u):
        result = np.copy(x)
        gain = self.context.u5_gain
        if self.stage == 0:
            result[PENDING] = gain * u
            return result
        realized = x[PENDING]
        response = self._response(realized)
        result[TILT] += 0.5 * self.dt * (x[RATE] + response)
        result[RATE] = response
        result[COUNTER_Q] += self.dt * realized
        result[PENDING] = self.carryover * realized + gain * u
        return result

    def _update_dynamics(self):
        gain = np.diag(self.context.u5_gain)
        self._fx[:] = np.eye(NX)
        self._fu[:] = 0.0
        if self.stage == 0:
            self._fx[PENDING, :] = 0.0
            self._fu[PENDING, :] = gain
            return
        response = (
            np.diag(self.context.confidence.astype(np.float64))
            @ self.context.r5_gain
            @ self.context.momentum_map
        )
        self._fx[TILT, RATE] = 0.5 * self.dt * np.eye(PLANAR)
        self._fx[TILT, PENDING] = 0.5 * self.dt * response
        self._fx[RATE, :] = 0.0
        self._fx[RATE, PENDING] = response
        self._fx[COUNTER_Q, PENDING] = self.dt * np.eye(NQ)
        self._fx[PENDING, :] = 0.0
        self._fx[PENDING, PENDING] = np.diag(self.carryover)
        self._fu[PENDING, :] = gain


class ResidualH3OCP:
    '''Solve one preallocated three-knot Crocoddyl residual problem'''

    def __init__(self, dt=0.02, max_iterations=1, weights=None):
        self.dt = float(dt)
        self.max_iterations = int(max_iterations)
        self.weights = {
            'action': 0.01,
            'change': 0.005,
            'tilt': 2.0,
            'rate': 1.0,
            'divergence': 2.0,
            'reserve': 0.1,
            **(weights or {}),
        }
        self.running_models = [
            ResidualH3ActionModel(
                self.dt, stage,
                self.weights['action'],
                0.0 if stage == 2 else self.weights['change'],
            )
            for stage in range(3)
        ]
        self.terminal_model = ResidualH2TerminalModel(self.weights)
        self.problem = crocoddyl.ShootingProblem(
            np.zeros(NX), self.running_models, self.terminal_model,
        )
        self.solver = crocoddyl.SolverBoxFDDP(self.problem)
        self.solver.th_stop = 1e-9
        self._last_us = None

    def reset(self):
        '''Clear shifted H3 warm start'''
        self._last_us = None

    def solve(self, context, carryover, lower, upper, pending=None):
        '''Solve H3 and return the first residual action'''
        started = time.perf_counter()
        lower = _vector(lower, NQ, 'lower')
        upper = _vector(upper, NQ, 'upper')
        carryover = _vector(carryover, NQ, 'carryover')
        x0 = np.zeros(NX)
        if pending is not None:
            x0[PENDING] = _vector(pending, NQ, 'pending')
        self.problem.x0 = x0
        for model in self.running_models:
            model.update(context, carryover, lower, upper)
        self.terminal_model.update(context)
        warm_started = self._last_us is not None
        if warm_started:
            controls = [
                np.clip(self._last_us[1], lower, upper),
                np.clip(self._last_us[2], lower, upper),
                np.clip(self._last_us[2], lower, upper),
            ]
        else:
            controls = [np.zeros(NQ) for _ in range(3)]
        states = list(self.problem.rollout(controls))
        seed_cost = float(self.problem.calc(states, controls))
        try:
            converged = bool(self.solver.solve(
                states, controls, self.max_iterations, True, 1e-6,
            ))
        except Exception:
            return self._failure('solver_failure', started, warm_started)
        xs = np.asarray(self.solver.xs)
        us = np.asarray(self.solver.us)
        valid = bool(
            xs.shape == (4, NX)
            and us.shape == (3, NQ)
            and np.all(np.isfinite(xs))
            and np.all(np.isfinite(us))
            and np.all(us >= lower - 1e-8)
            and np.all(us <= upper + 1e-8)
            and np.all(np.abs(us[:, 2]) <= 1e-8)
        )
        if not valid:
            return self._failure('invalid_solution', started, warm_started)
        self._last_us = np.copy(us)
        return ResidualH3Result(
            accepted=True,
            converged=converged,
            status='solved' if converged else 'best_effort',
            residual=np.clip(us[0], lower, upper),
            xs=xs,
            us=us,
            seed_cost=seed_cost,
            optimized_cost=float(self.solver.cost),
            solve_time=time.perf_counter() - started,
            iterations=int(self.solver.iter),
            stopping_criterion=float(self.solver.stop),
            warm_started=warm_started,
        )

    def _failure(self, status, started, warm_started):
        return ResidualH3Result(
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


def _vector(value, size, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {size} finite values')
    return np.copy(result)
