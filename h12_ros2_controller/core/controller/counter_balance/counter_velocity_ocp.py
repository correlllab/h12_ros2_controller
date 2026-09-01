import time
from dataclasses import dataclass

import crocoddyl
import numpy as np


NQ = 4
KKT_TOLERANCE = 5e-4


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
    kkt_violation: float
    regularization: float
    boxqp_polished: bool


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
        self.solver = None
        self.max_iterations = int(max_iterations)
        self.regularization = float(regularization)

    def solve(self, q, matrix, target, lower, upper):
        '''Solve one bounded velocity command'''
        started = time.perf_counter()
        q = np.asarray(q, dtype=np.float64)
        if q.shape != (NQ,) or not np.all(np.isfinite(q)):
            raise ValueError('velocity OCP state is invalid')
        self.problem.x0 = np.copy(q)
        lower = np.asarray(lower, dtype=np.float64)
        upper = np.asarray(upper, dtype=np.float64)
        fixed = np.isclose(lower, upper, atol=1e-12)
        solve_lower = np.copy(lower)
        solve_upper = np.copy(upper)
        solve_lower[fixed] -= 1e-10
        solve_upper[fixed] += 1e-10
        self.running_model.update(
            matrix, target, solve_lower, solve_upper,
        )
        solved = False
        converged = False
        used_regularization = self.regularization
        last_error = None
        for regularization in (
                self.regularization, 1e-6, 1e-3):
            try:
                self.solver = crocoddyl.SolverBoxFDDP(self.problem)
                self.solver.th_stop = 1e-9
                seed = np.clip(np.zeros(NQ), solve_lower, solve_upper)
                controls = [seed]
                states = list(self.problem.rollout(controls))
                converged = bool(self.solver.solve(
                    states,
                    controls,
                    self.max_iterations,
                    True,
                    regularization,
                ))
                used_regularization = regularization
                solved = True
                break
            except Exception as error:
                last_error = error
        initial_velocity = (
            np.asarray(self.solver.us[0], dtype=np.float64)
            if solved else np.clip(np.zeros(NQ), solve_lower, solve_upper)
        )
        hessian = np.asarray(matrix).T @ np.asarray(matrix)
        gradient_at_zero = -np.asarray(matrix).T @ np.asarray(target)
        try:
            boxqp = crocoddyl.BoxQP(NQ, 100, 0.1, 1e-9, 1e-9)
            velocity = boxqp.solve(
                hessian,
                gradient_at_zero,
                solve_lower,
                solve_upper,
                initial_velocity,
            ).x
        except Exception as error:
            raise RuntimeError('Crocoddyl BoxQP velocity solve failed') from (
                error if last_error is None else last_error
            )
        velocity = np.clip(
            np.asarray(velocity, dtype=np.float64), lower, upper,
        )
        gradient = np.asarray(matrix).T @ (
            np.asarray(matrix) @ velocity - np.asarray(target)
        )
        kkt_violation = _projected_gradient_violation(
            velocity, gradient, lower, upper,
        )
        residual = np.asarray(matrix) @ velocity - np.asarray(target)
        cost = 0.5 * float(residual @ residual)
        accepted = bool(
            velocity.shape == (NQ,)
            and np.all(np.isfinite(velocity))
            and np.isfinite(cost)
            and np.isfinite(kkt_violation)
            and kkt_violation <= KKT_TOLERANCE
            and np.all(velocity >= np.asarray(lower) - 1e-8)
            and np.all(velocity <= np.asarray(upper) + 1e-8)
        )
        return VelocityOCPResult(
            accepted=accepted,
            converged=converged,
            velocity=velocity,
            cost=cost,
            solve_time=time.perf_counter() - started,
            iterations=int(self.solver.iter) if solved else 0,
            stopping_criterion=(
                float(self.solver.stop) if solved else np.nan
            ),
            kkt_violation=float(kkt_violation),
            regularization=float(used_regularization),
            boxqp_polished=True,
        )


def _projected_gradient_violation(velocity, gradient, lower, upper):
    '''Return maximum KKT violation for one box-constrained quadratic'''
    tolerance = 2e-4
    at_lower = velocity <= lower + tolerance
    at_upper = velocity >= upper - tolerance
    fixed = np.isclose(lower, upper, atol=tolerance)
    violation = np.abs(gradient)
    violation[at_lower] = np.maximum(-gradient[at_lower], 0.0)
    violation[at_upper] = np.maximum(gradient[at_upper], 0.0)
    violation[fixed] = 0.0
    return float(np.max(violation))
