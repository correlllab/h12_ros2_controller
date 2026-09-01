import time
from dataclasses import dataclass

import crocoddyl
import numpy as np


NQ = 4


@dataclass(frozen=True)
class VelocityOCPResult:
    '''Store one bounded velocity solve result'''

    accepted: bool
    converged: bool
    velocity: np.ndarray
    cost: float
    solve_time: float
    iterations: int
    stopping_criterion: float


class VelocityActionModel(crocoddyl.ActionModelAbstract):
    '''One-step velocity dynamics with Iteration 2 quadratic cost'''

    def __init__(self, dt):
        self.dt = float(dt)
        self.matrix = np.zeros((1, NQ))
        self.target = np.zeros(1)
        super().__init__(crocoddyl.StateVector(NQ), NQ)

    def update(self, matrix, target, lower, upper):
        '''Update the least-squares cost and velocity bounds'''
        matrix = np.asarray(matrix, dtype=np.float64)
        target = np.asarray(target, dtype=np.float64)
        lower = np.asarray(lower, dtype=np.float64)
        upper = np.asarray(upper, dtype=np.float64)
        if (
            matrix.ndim != 2 or matrix.shape[1] != NQ
            or target.shape != (matrix.shape[0],)
            or lower.shape != (NQ,) or upper.shape != (NQ,)
            or not all(np.all(np.isfinite(value)) for value in (
                matrix, target, lower, upper,
            ))
            or np.any(lower > upper)
        ):
            raise ValueError('velocity OCP input is invalid')
        self.matrix = np.copy(matrix)
        self.target = np.copy(target)
        self.u_lb = np.copy(lower)
        self.u_ub = np.copy(upper)

    def calc(self, data, x, u=None):
        u = np.asarray(u, dtype=np.float64)
        data.xnext[:] = x + self.dt * u
        residual = self.matrix @ u - self.target
        data.cost = 0.5 * float(residual @ residual)

    def calcDiff(self, data, x, u=None):
        u = np.asarray(u, dtype=np.float64)
        residual = self.matrix @ u - self.target
        data.Fx[:] = np.eye(NQ)
        data.Fu[:] = self.dt * np.eye(NQ)
        data.Lx[:] = 0.0
        data.Lu[:] = self.matrix.T @ residual
        data.Lxx[:] = 0.0
        data.Lxu[:] = 0.0
        data.Luu[:] = self.matrix.T @ self.matrix


class VelocityTerminalModel(crocoddyl.ActionModelAbstract):
    '''Zero-cost terminal model for one-step parity'''

    def __init__(self):
        super().__init__(crocoddyl.StateVector(NQ), 0)

    def calc(self, data, x, u=None):
        data.xnext[:] = x
        data.cost = 0.0

    def calcDiff(self, data, x, u=None):
        data.Fx[:] = np.eye(NQ)
        data.Lx[:] = 0.0
        data.Lxx[:] = 0.0


class CounterVelocityOCP:
    '''Solve the Iteration 2 bounded velocity quadratic with Box-FDDP'''

    def __init__(self, dt=0.02, max_iterations=100, regularization=1e-9):
        self.running_model = VelocityActionModel(dt)
        self.terminal_model = VelocityTerminalModel()
        self.problem = crocoddyl.ShootingProblem(
            np.zeros(NQ), [self.running_model], self.terminal_model,
        )
        self.solver = crocoddyl.SolverBoxFDDP(self.problem)
        self.solver.th_stop = 1e-9
        self.max_iterations = int(max_iterations)
        self.regularization = float(regularization)

    def solve(self, q, matrix, target, lower, upper):
        '''Solve one bounded velocity command'''
        started = time.perf_counter()
        q = np.asarray(q, dtype=np.float64)
        if q.shape != (NQ,) or not np.all(np.isfinite(q)):
            raise ValueError('velocity OCP state is invalid')
        self.problem.x0 = np.copy(q)
        self.running_model.update(matrix, target, lower, upper)
        seed = np.clip(np.zeros(NQ), lower, upper)
        controls = [seed]
        states = list(self.problem.rollout(controls))
        converged = bool(self.solver.solve(
            states,
            controls,
            self.max_iterations,
            True,
            self.regularization,
        ))
        velocity = np.asarray(self.solver.us[0], dtype=np.float64)
        accepted = bool(
            velocity.shape == (NQ,)
            and np.all(np.isfinite(velocity))
            and np.all(velocity >= np.asarray(lower) - 1e-8)
            and np.all(velocity <= np.asarray(upper) + 1e-8)
        )
        return VelocityOCPResult(
            accepted=accepted,
            converged=converged,
            velocity=velocity,
            cost=float(self.solver.cost),
            solve_time=time.perf_counter() - started,
            iterations=int(self.solver.iter),
            stopping_criterion=float(self.solver.stop),
        )
