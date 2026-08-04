import time
from dataclasses import dataclass

import crocoddyl
import numpy as np
import pinocchio as pin

from h12_ros2_controller.utility.joint_definition import (
    FREEFLYER_NQ,
    FREEFLYER_NV,
    LEFT_ARM_INDEX,
    RIGHT_ARM_INDEX,
)


COM_SCALE = 0.01


@dataclass
class CounterBalancePlan:
    '''One-step counter-balance solution and diagnostics'''

    solved: bool
    status: str
    xs: np.ndarray
    us: np.ndarray
    solver: object
    command: np.ndarray
    zero_com_error: float
    optimized_com_error: float
    zero_cost: float
    optimized_cost: float
    solve_time: float
    control_lower: np.ndarray
    control_upper: np.ndarray


class CounterBalanceActionModel(crocoddyl.ActionModelAbstract):
    '''One-step velocity action over four counter-arm joints'''

    def __init__(self, model, full_q_template, motor_ids, q_ref, com_target,
                 dt, position_lower, position_upper, velocity_limit,
                 w_com, w_control, w_posture, w_terminal_posture,
                 w_soft_limit, soft_limit_margin, is_terminal=False,
                 control_lower=None, control_upper=None):
        self.model = model
        self.full_q_template = np.asarray(
            full_q_template,
            dtype=np.float64,
        ).copy()
        self.motor_ids = np.asarray(motor_ids, dtype=int)
        self.q_indices = FREEFLYER_NQ + self.motor_ids
        self.v_indices = FREEFLYER_NV + self.motor_ids
        self.q_ref = np.asarray(q_ref, dtype=np.float64).copy()
        self.com_target = np.asarray(com_target, dtype=np.float64).copy()
        self.dt = float(dt)
        self.position_lower = np.asarray(
            position_lower,
            dtype=np.float64,
        ).copy()
        self.position_upper = np.asarray(
            position_upper,
            dtype=np.float64,
        ).copy()
        self.velocity_limit = np.asarray(
            velocity_limit,
            dtype=np.float64,
        ).copy()
        self.w_com = float(w_com)
        self.w_control = float(w_control)
        self.w_posture = float(w_posture)
        self.w_terminal_posture = float(w_terminal_posture)
        self.w_soft_limit = float(w_soft_limit)
        self.is_terminal = bool(is_terminal)
        span = self.position_upper - self.position_lower
        margin = float(soft_limit_margin) * span
        self.soft_lower = self.position_lower + margin
        self.soft_upper = self.position_upper - margin
        nu = 0 if self.is_terminal else len(self.motor_ids)
        super().__init__(crocoddyl.StateVector(len(self.motor_ids)), nu)
        if not self.is_terminal:
            self.u_lb = np.asarray(control_lower, dtype=np.float64).copy()
            self.u_ub = np.asarray(control_upper, dtype=np.float64).copy()

    def _full_q(self, q_counter):
        full_q = self.full_q_template.copy()
        full_q[self.q_indices] = q_counter
        return full_q

    def _soft_limit_terms(self, q):
        below = q < self.soft_lower
        above = q > self.soft_upper
        residual = np.zeros_like(q)
        residual[below] = self.soft_lower[below] - q[below]
        residual[above] = q[above] - self.soft_upper[above]
        derivative = above.astype(np.float64) - below.astype(np.float64)
        return residual, derivative

    def _valid(self, q, u=None):
        arrays = [q, self.full_q_template, self.q_ref, self.com_target]
        if u is not None:
            arrays.append(u)
        return all(np.all(np.isfinite(array)) for array in arrays)

    def calc(self, data, x, u=None):
        q = np.asarray(x, dtype=np.float64)
        velocity = None if self.is_terminal or u is None else np.asarray(
            u,
            dtype=np.float64,
        )
        if not self._valid(q, velocity):
            data.xnext = np.copy(q)
            data.cost = 1e12
            return

        if self.is_terminal:
            q_next = q
        else:
            q_next = q + self.dt * velocity
        data.xnext = np.copy(q_next)
        if (
            np.any(q_next < self.position_lower - 1e-10)
            or np.any(q_next > self.position_upper + 1e-10)
        ):
            data.cost = 1e12
            return

        posture_error = q_next - self.q_ref
        if self.is_terminal:
            full_q = self._full_q(q_next)
            com = pin.centerOfMass(self.model, data.pin_data, full_q)
            com_error = (np.asarray(com[:2]) - self.com_target) / COM_SCALE
            data.com = np.asarray(com, dtype=np.float64).copy()
            data.cost = 0.5 * self.w_com * com_error.dot(com_error)
            data.cost += (
                0.5 * self.w_terminal_posture
                * posture_error.dot(posture_error)
            )
            return

        soft_error, _ = self._soft_limit_terms(q_next)
        normalized_velocity = velocity / self.velocity_limit
        data.cost = (
            0.5 * self.w_control
            * normalized_velocity.dot(normalized_velocity)
        )
        data.cost += 0.5 * self.w_posture * posture_error.dot(posture_error)
        data.cost += 0.5 * self.w_soft_limit * soft_error.dot(soft_error)

    def calcDiff(self, data, x, u=None):
        q = np.asarray(x, dtype=np.float64)
        data.Fx[:] = np.eye(len(q))
        data.Lx[:] = 0.0
        data.Lxx[:] = 0.0
        data.Fu[:] = 0.0
        data.Lu[:] = 0.0
        data.Lxu[:] = 0.0
        data.Luu[:] = 0.0
        velocity = None if self.is_terminal or u is None else np.asarray(
            u,
            dtype=np.float64,
        )
        if not self._valid(q, velocity):
            return
        if self.is_terminal or u is None:
            full_q = self._full_q(q)
            com = pin.centerOfMass(self.model, data.pin_data, full_q)
            com_error = (np.asarray(com[:2]) - self.com_target) / COM_SCALE
            com_jacobian = pin.jacobianCenterOfMass(
                self.model,
                data.pin_data,
                full_q,
            )[:2, self.v_indices] / COM_SCALE
            posture_error = q - self.q_ref
            data.Lx[:] = (
                self.w_com * com_jacobian.T @ com_error
                + self.w_terminal_posture * posture_error
            )
            data.Lxx[:] = (
                self.w_com * com_jacobian.T @ com_jacobian
                + self.w_terminal_posture * np.eye(len(q))
            )
            return

        q_next = q + self.dt * velocity
        posture_error = q_next - self.q_ref
        soft_error, soft_derivative = self._soft_limit_terms(q_next)
        q_gradient = (
            self.w_posture * posture_error
            + self.w_soft_limit * soft_derivative * soft_error
        )
        q_hessian = (
            self.w_posture * np.eye(len(q))
            + self.w_soft_limit * np.diag(soft_derivative ** 2)
        )
        velocity_hessian = self.w_control * np.diag(
            1.0 / self.velocity_limit ** 2
        )
        data.Fu[:] = self.dt * np.eye(len(q))
        data.Lx[:] = q_gradient
        data.Lu[:] = (
            self.dt * q_gradient
            + velocity_hessian @ velocity
        )
        data.Lxx[:] = q_hessian
        data.Lxu[:] = self.dt * q_hessian
        data.Luu[:] = (
            self.dt ** 2 * q_hessian
            + velocity_hessian
        )

    def createData(self):
        data = crocoddyl.ActionDataAbstract(self)
        data.pin_data = self.model.createData()
        data.com = np.full(3, np.nan, dtype=np.float64)
        return data


class CounterBalanceDDP:
    '''Minimal one-step full-model CoM counter-balance solver'''

    def __init__(self, robot_model, dt=None, config=None, arm='right'):
        if arm not in ('left', 'right'):
            raise ValueError('arm must be "left" or "right"')
        self.robot_model = robot_model
        self.model = robot_model.model
        self.arm = arm
        arm_ids = LEFT_ARM_INDEX if arm == 'left' else RIGHT_ARM_INDEX
        self.motor_ids = list(arm_ids[:4])
        cfg = (config or {}).get('counter_balance', {})
        self.dt = float(dt if dt is not None else cfg.get('dt', 0.02))
        self.maxiter = int(cfg.get('maxiter', 5))
        self.w_com = float(cfg.get('w_com', 1.0))
        self.w_control = float(cfg.get('w_control', 0.01))
        self.w_posture = float(cfg.get('w_posture', 0.01))
        self.w_terminal_posture = float(
            cfg.get('w_terminal_posture', 0.01)
        )
        self.w_soft_limit = float(cfg.get('w_soft_limit', 1.0))
        self.soft_limit_margin = float(cfg.get('soft_limit_margin', 0.10))
        self.position_lower, self.position_upper = self._position_limits(
            cfg,
            config or {},
        )
        self.velocity_limit = self._velocity_limits(cfg, config or {})
        self._validate_settings()

    def solve(self, full_q_template, com_target, q_ref=None):
        '''Solve one bounded counter-arm velocity command'''
        started = time.perf_counter()
        full_q_template = np.asarray(full_q_template, dtype=np.float64)
        com_target = np.asarray(com_target, dtype=np.float64).reshape(-1)
        if (
            full_q_template.shape != (self.model.nq,)
            or com_target.shape != (2,)
        ):
            return self._failure_plan('invalid_input', started)
        q = full_q_template[FREEFLYER_NQ + np.asarray(self.motor_ids)]
        q_ref = q if q_ref is None else np.asarray(q_ref, dtype=np.float64)
        finite_values = (full_q_template, com_target, q_ref, q)
        if q_ref.shape != (4,) or not all(
            np.all(np.isfinite(value)) for value in finite_values
        ):
            return self._failure_plan('invalid_input', started)
        if np.any(q < self.position_lower) or np.any(q > self.position_upper):
            return self._failure_plan('state_out_of_bounds', started)

        control_lower = np.maximum(
            -self.velocity_limit,
            (self.position_lower - q) / self.dt,
        )
        control_upper = np.minimum(
            self.velocity_limit,
            (self.position_upper - q) / self.dt,
        )
        running = self._action_model(
            full_q_template,
            q_ref,
            com_target,
            control_lower,
            control_upper,
        )
        terminal = self._action_model(
            full_q_template,
            q_ref,
            com_target,
            control_lower,
            control_upper,
            is_terminal=True,
        )
        problem = crocoddyl.ShootingProblem(q, [running], terminal)
        solver = crocoddyl.SolverBoxDDP(problem)
        zero_command = np.zeros(4, dtype=np.float64)
        zero_xs = [np.copy(q), np.copy(q)]
        try:
            solved = solver.solve(zero_xs, [zero_command], self.maxiter)
            xs = np.asarray(solver.xs, dtype=np.float64)
            us = np.asarray(solver.us, dtype=np.float64)
        except Exception:
            return self._failure_plan(
                'solver_failure',
                started,
                solver=solver,
                control_lower=control_lower,
                control_upper=control_upper,
            )
        if us.shape != (1, 4) or not np.all(np.isfinite(us)):
            return self._failure_plan(
                'invalid_output',
                started,
                solver=solver,
                xs=xs,
                us=us,
                control_lower=control_lower,
                control_upper=control_upper,
            )
        command = np.copy(us[0])
        if (
            np.any(command < control_lower - 1e-8)
            or np.any(command > control_upper + 1e-8)
        ):
            return self._failure_plan(
                'output_out_of_bounds',
                started,
                solver=solver,
                xs=xs,
                us=us,
                control_lower=control_lower,
                control_upper=control_upper,
            )

        zero_error, zero_cost = self._evaluate(
            running,
            terminal,
            q,
            zero_command,
        )
        optimized_error, optimized_cost = self._evaluate(
            running,
            terminal,
            q,
            command,
        )
        if not np.all(np.isfinite([
            zero_error,
            optimized_error,
            zero_cost,
            optimized_cost,
        ])):
            return self._failure_plan(
                'invalid_diagnostics',
                started,
                solver=solver,
                xs=xs,
                us=us,
                control_lower=control_lower,
                control_upper=control_upper,
            )
        return CounterBalancePlan(
            solved=bool(solved),
            status='solved' if solved else 'best_effort',
            xs=xs,
            us=us,
            solver=solver,
            command=command,
            zero_com_error=zero_error,
            optimized_com_error=optimized_error,
            zero_cost=zero_cost,
            optimized_cost=optimized_cost,
            solve_time=time.perf_counter() - started,
            control_lower=control_lower,
            control_upper=control_upper,
        )

    def evaluate_com_error(self, full_q, com_target):
        '''Evaluate planar CoM error with private Pinocchio data'''
        full_q = np.asarray(full_q, dtype=np.float64)
        com_target = np.asarray(com_target, dtype=np.float64)
        if (
            full_q.shape != (self.model.nq,)
            or com_target.shape != (2,)
            or not np.all(np.isfinite(full_q))
            or not np.all(np.isfinite(com_target))
        ):
            return np.nan
        data = self.model.createData()
        com = pin.centerOfMass(self.model, data, full_q)
        return float(np.linalg.norm(np.asarray(com[:2]) - com_target))

    def _action_model(self, full_q_template, q_ref, com_target,
                      control_lower, control_upper, is_terminal=False):
        return CounterBalanceActionModel(
            model=self.model,
            full_q_template=full_q_template,
            motor_ids=self.motor_ids,
            q_ref=q_ref,
            com_target=com_target,
            dt=self.dt,
            position_lower=self.position_lower,
            position_upper=self.position_upper,
            velocity_limit=self.velocity_limit,
            w_com=self.w_com,
            w_control=self.w_control,
            w_posture=self.w_posture,
            w_terminal_posture=self.w_terminal_posture,
            w_soft_limit=self.w_soft_limit,
            soft_limit_margin=self.soft_limit_margin,
            is_terminal=is_terminal,
            control_lower=control_lower,
            control_upper=control_upper,
        )

    def _evaluate(self, running, terminal, q, command):
        running_data = running.createData()
        running.calc(running_data, q, command)
        terminal_data = terminal.createData()
        terminal.calc(terminal_data, running_data.xnext)
        error = np.linalg.norm(terminal_data.com[:2] - terminal.com_target)
        return float(error), float(running_data.cost + terminal_data.cost)

    def _position_limits(self, cfg, config):
        body = self.robot_model.model_body
        lower = np.asarray(body.lowerPositionLimit, dtype=np.float64)[
            self.motor_ids
        ].copy()
        upper = np.asarray(body.upperPositionLimit, dtype=np.float64)[
            self.motor_ids
        ].copy()
        publisher = config.get('limits', {}).get('q_clip_limits')
        if publisher is not None:
            publisher = np.asarray(publisher, dtype=np.float64)[self.motor_ids]
            lower = np.maximum(lower, publisher[:, 0])
            upper = np.minimum(upper, publisher[:, 1])
        configured = cfg.get('position_limits')
        if configured is not None:
            configured = np.asarray(configured, dtype=np.float64)
            if configured.shape != (4, 2):
                raise ValueError('counter_balance.position_limits must be 4x2')
            lower = np.maximum(lower, configured[:, 0])
            upper = np.minimum(upper, configured[:, 1])
        return lower, upper

    def _velocity_limits(self, cfg, config):
        body = self.robot_model.model_body
        model_limit = np.asarray(body.velocityLimit, dtype=np.float64)[
            self.motor_ids
        ].copy()
        model_limit[~np.isfinite(model_limit) | (model_limit <= 0.0)] = np.inf
        limit = model_limit
        controller_limit = float(
            config.get('controller', {}).get('dq_lim', np.inf)
        )
        if np.isfinite(controller_limit) and controller_limit > 0.0:
            limit = np.minimum(limit, controller_limit)
        publisher_limit = config.get('limits', {}).get('dq_clip_limits')
        if publisher_limit is not None:
            publisher_limit = np.asarray(publisher_limit, dtype=np.float64)[
                self.motor_ids
            ]
            limit = np.minimum(limit, publisher_limit)
        configured = cfg.get('velocity_limits', cfg.get('max_velocity'))
        if configured is not None:
            configured = np.asarray(configured, dtype=np.float64)
            if configured.ndim == 0:
                configured = np.full(4, float(configured))
            if configured.shape != (4,):
                raise ValueError(
                    'counter_balance.velocity_limits must be scalar or length 4'
                )
            limit = np.minimum(limit, configured)
        limit[~np.isfinite(limit)] = 1.0
        return limit

    def _validate_settings(self):
        if not np.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError('counter-balance dt must be positive')
        if self.maxiter < 0:
            raise ValueError('counter_balance.maxiter must be nonnegative')
        weights = np.asarray([
            self.w_com,
            self.w_control,
            self.w_posture,
            self.w_terminal_posture,
            self.w_soft_limit,
        ])
        if not np.all(np.isfinite(weights)) or np.any(weights < 0.0):
            raise ValueError(
                'counter-balance weights must be finite and nonnegative'
            )
        if not 0.0 <= self.soft_limit_margin < 0.5:
            raise ValueError(
                'counter_balance.soft_limit_margin must be in [0, 0.5)'
            )
        if (
            not np.all(np.isfinite(self.position_lower))
            or not np.all(np.isfinite(self.position_upper))
            or np.any(self.position_lower >= self.position_upper)
        ):
            raise ValueError('counter-balance position limits are invalid')
        if (
            not np.all(np.isfinite(self.velocity_limit))
            or np.any(self.velocity_limit <= 0.0)
        ):
            raise ValueError('counter-balance velocity limits are invalid')

    @staticmethod
    def _failure_plan(status, started, solver=None, xs=None, us=None,
                      control_lower=None, control_upper=None):
        empty4 = np.empty((0, 4), dtype=np.float64)
        return CounterBalancePlan(
            solved=False,
            status=status,
            xs=empty4 if xs is None else xs,
            us=empty4 if us is None else us,
            solver=solver,
            command=np.zeros(4, dtype=np.float64),
            zero_com_error=np.nan,
            optimized_com_error=np.nan,
            zero_cost=np.nan,
            optimized_cost=np.nan,
            solve_time=time.perf_counter() - started,
            control_lower=(
                np.full(4, np.nan) if control_lower is None else control_lower
            ),
            control_upper=(
                np.full(4, np.nan) if control_upper is None else control_upper
            ),
        )
