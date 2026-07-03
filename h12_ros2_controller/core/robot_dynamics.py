import time
from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from h12_ros2_controller.utility.joint_definition import (
    BODY_JOINTS, NUM_MOTOR, FREEFLYER_NV,
    LEFT_ARM_INDEX, RIGHT_ARM_INDEX
)


class RobotDynamics:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self._wrench_last_dq = None
        self._wrench_last_time = None
        self._wrench_ddq = np.zeros(NUM_MOTOR)
        self._wrench_filtered = {}

    def create_momentum_ddp(self, dt=None, config=None, arm='left'):
        '''Create a Crocoddyl momentum planner for one arm'''
        return MomentumDDP(self.robot_model, dt=dt, config=config, arm=arm)

    def get_com(self, q: np.ndarray=None):
        robot = self.robot_model
        q = robot.state['q'] if q is None else q
        q_full = robot.full_q(q)
        com = pin.centerOfMass(robot.model, robot.data, q_full)
        return com

    def get_com_velocity(self, q: np.ndarray=None, dq: np.ndarray=None):
        '''Get center of mass velocity for the full model'''
        robot = self.robot_model
        q = robot.state['q'] if q is None else q
        dq = robot.state['dq'] if dq is None else dq
        pin.centerOfMass(robot.model, robot.data, robot.full_q(q), robot.full_v(dq))
        return np.copy(robot.data.vcom[0])

    def get_angular_centroidal_momentum_matrix(self,
                                               q: np.ndarray=None,
                                               joint_ids=None):
        '''Get the motor-joint angular centroidal momentum matrix'''
        robot = self.robot_model
        q = robot.state['q'] if q is None else q
        pin.computeCentroidalMap(robot.model, robot.data, robot.full_q(q))
        angular_map = robot.data.Ag[3:, FREEFLYER_NV:]
        if joint_ids is None:
            return np.copy(angular_map)
        return np.copy(angular_map[:, joint_ids])

    def get_angular_centroidal_momentum(self,
                                        q: np.ndarray=None,
                                        dq: np.ndarray=None,
                                        joint_ids=None):
        '''Get motor-joint angular centroidal momentum'''
        robot = self.robot_model
        dq = robot.state['dq'] if dq is None else dq
        angular_map = self.get_angular_centroidal_momentum_matrix(q, joint_ids)
        if joint_ids is None:
            return angular_map @ dq
        return angular_map @ dq[joint_ids]

    def get_com_reduced(self, q_reduced: np.ndarray=None):
        '''Get center of mass for the reduced model (model_body_reduced)'''
        robot = self.robot_model
        q_reduced = robot.state_reduced['q'] if q_reduced is None else q_reduced
        com = pin.centerOfMass(robot.model_body_reduced, robot.data_body_reduced, q_reduced)
        return com

    def get_zmp(self, q: np.ndarray=None):
        robot = self.robot_model
        q = robot.state['q'] if q is None else q
        com = self.get_com(q)

        # find reference support plane
        left_foot_pos = robot.get_frame_position('left_ankle_roll_link', q)
        right_foot_pos = robot.get_frame_position('right_ankle_roll_link', q)
        ground_point = 0.5 * (left_foot_pos + right_foot_pos)
        ground_height = ground_point[2]

        # centroidal momentum time derivative
        q_full = robot.full_q(q)
        dq_full = robot.full_v(robot.state['dq'])
        ddq_full = robot.full_v(robot.state['ddq'])
        pin.computeCentroidalMomentumTimeVariation(
            robot.model, robot.data, q_full, dq_full, ddq_full
        )
        dhg = robot.data.dhg

        F_com = dhg.linear
        tau_com = dhg.angular

        # gravity force
        g = np.array([0, 0, -9.81])
        m = pin.computeTotalMass(robot.model)
        F_g = m * g

        # isolate contact resultant force
        F_contact = F_com - F_g
        # shift torque from CoM -> support_plane_point on the plane
        tau_contact = tau_com + np.cross(com - ground_point, F_contact)

        eps = 1e-6
        # TODO gate ZMP when contact is unreliable (low Fz / contact loss)
        if abs(F_contact[2]) < eps:
            return np.zeros(3)

        # ZMP on support plane
        zmp_x = ground_point[0] - tau_contact[1] / F_contact[2]
        zmp_y = ground_point[1] + tau_contact[0] / F_contact[2]
        zmp = np.array([zmp_x, zmp_y, ground_height])

        return zmp

    def get_gravity_compensation(self, q: np.ndarray=None, imu_quat=None):
        robot = self.robot_model
        q = robot.state['q'] if q is None else q
        q_full = robot.full_q(q, imu_quat)
        tau_full = pin.rnea(robot.model,
                            robot.data,
                            q_full,
                            np.zeros(robot.model.nv),
                            np.zeros(robot.model.nv))
        # return motor-only torques (skip free-flyer)
        return tau_full[FREEFLYER_NV:]

    def get_frame_jacobian(self, frame_name: str, q: np.ndarray=None, imu_quat=None):
        '''
        Get the frame jacobian in the world frame
        q is motor-only (27), returns motor-only jacobian (6 x 27)
        '''
        robot = self.robot_model
        if q is not None:
            q_full = robot.full_q(q, imu_quat)
            data = robot.model.createData()
        else:
            q_full = robot.full_q(robot.state['q'])
            data = robot.data
        # update kinematics
        pin.forwardKinematics(robot.model, data, q_full)
        pin.updateFramePlacements(robot.model, data)
        # compute jacobian
        frame_id = robot.model.getFrameId(frame_name)
        jacobian_full = pin.computeFrameJacobian(
            robot.model,
            data,
            q_full,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        # return motor-only columns (skip free-flyer columns)
        return jacobian_full[:, FREEFLYER_NV:]

    def get_joint_jacobian(self, joint_name: str, q: np.ndarray=None):
        '''
        Get the joint jacobian in the local frame of the joint
        q is motor-only (27), returns motor-only jacobian (6 x 27)
        '''
        robot = self.robot_model
        if q is not None:
            q_full = robot.full_q(q)
            data = robot.model.createData()
        else:
            q_full = robot.full_q(robot.state['q'])
            data = robot.data
        # update kinematics
        pin.forwardKinematics(robot.model, data, q_full)
        pin.updateFramePlacements(robot.model, data)
        # compute jacobian
        joint_id = robot.model.getJointId(joint_name)
        jacobian_full = pin.computeJointJacobian(
            robot.model,
            data,
            q_full,
            joint_id
        )
        # return motor-only columns (skip free-flyer columns)
        return jacobian_full[:, FREEFLYER_NV:]

    def get_frame_twist(self, frame_name: str):
        robot = self.robot_model
        frame_id = robot.model.getFrameId(frame_name)
        twist = pin.getFrameVelocity(
            robot.model,
            robot.data,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return np.concatenate([twist.linear, twist.angular])

    def get_frame_wrench(self, frame_name: str,
                         q: np.ndarray=None, tau: np.ndarray=None, imu_quat=None):
        '''
        Estimate frame wrench with arm-only dynamics compensation and SVD damping

        This keeps bad configurations quiet by damping weak Jacobian directions
        instead of amplifying torque noise into large force readings
        '''
        robot = self.robot_model
        q = robot.state['q'] if q is None else q
        tau = robot.state['tau'] if tau is None else tau
        dq = robot.state['dq']
        ddq = self._estimate_wrench_ddq(dq)

        joint_idx = self._get_frame_wrench_joint_indices(frame_name)
        tau_model = self.get_motion_compensation(q, dq, imu_quat=imu_quat, ddq=ddq)
        tau_residual = tau[joint_idx] - tau_model[joint_idx]
        jac = self.get_frame_jacobian(frame_name, q, imu_quat)[:, joint_idx]
        wrench = self._solve_damped_svd_wrench(jac, tau_residual)
        wrench *= self._motion_quality(dq[joint_idx], ddq[joint_idx])
        return self._filter_frame_wrench(frame_name, wrench)

    def get_frame_wrench_damped(self, frame_name: str,
                                q: np.ndarray=None, tau: np.ndarray=None, imu_quat=None):
        '''
        Estimate wrench from joint torques using a condition-aware least-squares solve

        Uses adaptive damped least-squares when the Jacobian is ill-conditioned to
        reduce sensitivity to torque noise near singular configurations
        '''
        robot = self.robot_model
        cond_threshold = 10
        base_damping = 1e-1
        max_damping = 10
        svd_tol = 1e-3

        q = robot.state['q'] if q is None else q
        tau = robot.state['tau'] if tau is None else tau
        tau_gravity = self.get_gravity_compensation(q, imu_quat)
        jac = self.get_frame_jacobian(frame_name, q, imu_quat)
        tau_residual = tau - tau_gravity

        singular_values = np.linalg.svd(jac.T, compute_uv=False)
        if singular_values.size == 0 or singular_values[0] < svd_tol:
            return np.zeros(6)

        cond_number = singular_values[0] / max(singular_values[-1], svd_tol)
        if cond_number <= cond_threshold:
            damping = 0.0
        else:
            # increase damping smoothly once condition number passes threshold
            damping = base_damping * (cond_number / cond_threshold)
            damping = np.clip(damping, base_damping, max_damping)

        jj_t = jac @ jac.T
        rhs = jac @ tau_residual
        damping_sq = damping * damping
        wrench = np.linalg.solve(jj_t + damping_sq * np.eye(6), rhs)
        return wrench

    def _get_frame_wrench_joint_indices(self, frame_name: str):
        '''Select the arm joints that should explain a wrist wrench'''
        if frame_name.startswith('left_'):
            return LEFT_ARM_INDEX
        if frame_name.startswith('right_'):
            return RIGHT_ARM_INDEX
        return np.arange(NUM_MOTOR)

    def get_motion_compensation(self, q: np.ndarray=None,
                                dq: np.ndarray=None, imu_quat=None,
                                ddq: np.ndarray=None):
        '''Estimate model torque for motion, excluding external contact'''
        robot = self.robot_model
        q = robot.state['q'] if q is None else q
        dq = robot.state['dq'] if dq is None else dq
        ddq = np.zeros(NUM_MOTOR) if ddq is None else ddq
        q_full = robot.full_q(q, imu_quat)
        dq_full = robot.full_v(dq)
        ddq_full = robot.full_v(ddq)
        tau_full = pin.rnea(
            robot.model,
            robot.data,
            q_full,
            dq_full,
            ddq_full,
        )
        return tau_full[FREEFLYER_NV:]

    def _estimate_wrench_ddq(self, dq: np.ndarray):
        '''Estimate filtered joint acceleration for wrench compensation'''
        now = time.time()
        dq = np.asarray(dq, dtype=float)
        if self._wrench_last_dq is None:
            self._wrench_last_dq = np.copy(dq)
            self._wrench_last_time = now
            return np.copy(self._wrench_ddq)

        dt = now - self._wrench_last_time
        if dt < 1e-3:
            return np.copy(self._wrench_ddq)
        if dt > 0.2:
            # avoid a large derivative after pauses or debugger stops
            self._wrench_last_dq = np.copy(dq)
            self._wrench_last_time = now
            self._wrench_ddq[:] = 0.0
            return np.copy(self._wrench_ddq)

        ddq_raw = (dq - self._wrench_last_dq) / dt
        ddq_raw = np.clip(ddq_raw, -40.0, 40.0)
        self._wrench_ddq = 0.9 * self._wrench_ddq + 0.1 * ddq_raw
        self._wrench_last_dq = np.copy(dq)
        self._wrench_last_time = now
        return np.copy(self._wrench_ddq)

    @staticmethod
    def _motion_quality(dq: np.ndarray, ddq: np.ndarray):
        '''Fade unreliable readings during faster arm motion'''
        velocity_norm = np.linalg.norm(dq)
        accel_norm = np.linalg.norm(ddq)
        velocity_quality = np.clip(1.0 - velocity_norm / 5.0, 0.0, 1.0)
        accel_quality = np.clip(1.0 - accel_norm / 20.0, 0.0, 1.0)
        return velocity_quality * accel_quality

    def _filter_frame_wrench(self, frame_name: str, wrench: np.ndarray):
        '''Low-pass filter wrench readings per frame'''
        beta = 0.75
        previous = self._wrench_filtered.get(frame_name)
        if previous is None:
            filtered = np.copy(wrench)
        else:
            filtered = beta * previous + (1.0 - beta) * wrench
        self._wrench_filtered[frame_name] = filtered
        return np.copy(filtered)

    @staticmethod
    def _solve_damped_svd_wrench(jac: np.ndarray, tau_residual: np.ndarray):
        '''Solve jac.T * wrench = tau while suppressing weak directions'''
        damping = 1e-1
        svd_floor = 5e-2
        a = jac.T
        u, singular_values, vt = np.linalg.svd(a, full_matrices=False)
        if singular_values.size == 0 or singular_values[0] < svd_floor:
            return np.zeros(6)

        gains = singular_values / (singular_values * singular_values + damping * damping)
        gains[singular_values < svd_floor] = 0.0
        wrench = vt.T @ (gains * (u.T @ tau_residual))

        # fade the whole reading near singular poses so bad estimates look ignorable
        quality = np.clip(singular_values[-1] / svd_floor, 0.0, 1.0)
        return quality * wrench

    def get_frame_wrench_raw(self, frame_name: str,
                             q: np.ndarray=None, tau: np.ndarray=None, imu_quat=None):
        '''
        Pseudo-inverse solution without damping for reference/debugging
        '''
        robot = self.robot_model
        q = robot.state['q'] if q is None else q
        tau = robot.state['tau'] if tau is None else tau
        tau_gravity = self.get_gravity_compensation(q, imu_quat)
        jac = self.get_frame_jacobian(frame_name, q, imu_quat)
        tau_residual = tau - tau_gravity
        wrench = np.linalg.pinv(jac.T) @ tau_residual
        return wrench

    def compute_frame_twist(self, frame_name: str, dq: np.ndarray):
        jac = self.get_frame_jacobian(frame_name)
        twist = jac @ dq
        return twist


@dataclass
class MomentumPlan:
    '''Crocoddyl momentum trajectory and solver state'''

    solved: bool
    xs: np.ndarray
    us: np.ndarray
    solver: object
    arm_ids: list[int]
    q_ref: np.ndarray
    target_momentum: np.ndarray
    momenta: np.ndarray
    peak_momentum: np.ndarray
    final_posture_error: float
    phase_lengths: tuple[int, int, int]

    @property
    def first_velocity(self):
        return self.velocity_at(0)

    def first_body_velocity(self, nv):
        return self.body_velocity_at(0, nv)

    def velocity_at(self, index):
        if not len(self.us):
            return np.zeros(len(self.arm_ids), dtype=np.float64)
        return np.copy(self.us[min(index, len(self.us) - 1)])

    def body_velocity_at(self, index, nv):
        velocity = np.zeros(nv, dtype=np.float64)
        velocity[self.arm_ids] = self.velocity_at(index)
        return velocity


class MomentumDDP:
    '''Crocoddyl DDP planner for selected-arm angular momentum'''

    def __init__(self, robot_model, dt=None, config=None, arm='left'):
        try:
            import crocoddyl
        except ImportError as err:
            raise ImportError(
                'MomentumDDP requires the optional crocoddyl dependency'
            ) from err

        self._crocoddyl = crocoddyl
        self.robot_model = robot_model
        ddp_cfg = (config or {}).get('momentum_ddp', {})
        self.enabled = bool(ddp_cfg.get('enabled', False))
        self.arm = ddp_cfg.get('arm', arm)
        self.dt = float(ddp_cfg.get('dt', dt if dt is not None else 0.04))
        self.hold_duration = float(ddp_cfg.get('hold_duration', 0.16))
        self.momentum_duration = float(ddp_cfg.get('momentum_duration', 0.36))
        self.return_duration = float(ddp_cfg.get('return_duration', 0.56))
        self.maxiter = int(ddp_cfg.get('maxiter', 2))
        self.w_momentum = float(ddp_cfg.get('w_momentum', 120.0))
        self.w_u = float(ddp_cfg.get('w_u', 2e-3))
        self.w_limit = float(ddp_cfg.get('w_limit', 100.0))
        self.max_velocity = float(ddp_cfg.get('max_velocity', 8.0))
        self.arm_ids = self._arm_ids(self.arm)
        self.arm_model = self._build_arm_model()

    def solve(self, target_momentum, q_ref=None):
        '''Solve a hold-swing-return arm momentum trajectory'''
        target_momentum = np.asarray(target_momentum, dtype=np.float64)
        q_ref = self.current_arm_q() if q_ref is None else np.asarray(q_ref, dtype=np.float64)
        running_models, terminal_model = self._build_phase_models(q_ref, target_momentum)
        problem = self._crocoddyl.ShootingProblem(q_ref, running_models, terminal_model)
        solver = self._crocoddyl.SolverDDP(problem)
        solved = solver.solve(*self._warm_start(q_ref, target_momentum), self.maxiter)
        xs = np.asarray(solver.xs, dtype=np.float64)
        us = np.asarray(solver.us, dtype=np.float64)
        momenta = self._trajectory_momenta(xs, us)
        return MomentumPlan(
            solved=bool(solved),
            xs=xs,
            us=us,
            solver=solver,
            arm_ids=self.arm_ids,
            q_ref=np.copy(q_ref),
            target_momentum=np.copy(target_momentum),
            momenta=momenta,
            peak_momentum=self._peak_useful_momentum(momenta, target_momentum),
            final_posture_error=float(np.linalg.norm(pin.difference(self.arm_model, q_ref, xs[-1]))),
            phase_lengths=self._phase_steps(),
        )

    def current_arm_q(self):
        '''Return the selected arm configuration'''
        return np.copy(self.robot_model.state['q'][self.arm_ids])

    def angular_momentum(self, q, velocity):
        '''Return the reduced arm model angular centroidal momentum'''
        data = self.arm_model.createData()
        pin.ccrba(self.arm_model, data, q, velocity)
        return np.asarray(data.hg.angular, dtype=np.float64)

    def _trajectory_momenta(self, xs, us):
        return np.array([
            self.angular_momentum(q, us[min(index, len(us) - 1)])
            for index, q in enumerate(xs[:-1])
        ]) if len(us) else np.empty((0, 3), dtype=np.float64)

    @staticmethod
    def _peak_useful_momentum(momenta, target_momentum):
        if not len(momenta):
            return np.zeros(3, dtype=np.float64)
        target_norm = np.linalg.norm(target_momentum)
        if target_norm > 1e-9:
            direction = target_momentum / target_norm
            return np.copy(momenta[np.argmax(momenta @ direction)])
        return np.copy(momenta[np.argmax(np.linalg.norm(momenta, axis=1))])

    def _build_phase_models(self, q_ref, target_momentum):
        action_model = self._make_action_model_class()
        hold_steps, momentum_steps, return_steps = self._phase_steps()
        phases = (
            [(np.zeros(3), 20.0, 60.0, self.w_u)] * hold_steps
            + [(target_momentum, self.w_momentum, 0.5, self.w_u)] * momentum_steps
            + [(np.zeros(3), 10.0, 100.0, self.w_u)] * return_steps
        )
        running_models = [
            self._crocoddyl.ActionModelNumDiff(action_model(
                self.arm_model, q_ref, momentum, self.dt, w_momentum,
                w_q, w_u, self.w_limit, self.max_velocity,
            ))
            for momentum, w_momentum, w_q, w_u in phases
        ]
        terminal = action_model(
            self.arm_model, q_ref, np.zeros(3), self.dt, 20.0, 250.0,
            0.0, self.w_limit, self.max_velocity, is_terminal=True,
        )
        return running_models, self._crocoddyl.ActionModelNumDiff(terminal)

    def _warm_start(self, q_ref, target_momentum):
        hold_steps, momentum_steps, return_steps = self._phase_steps()
        q = np.copy(q_ref)
        velocity_zero = np.zeros(self.arm_model.nv, dtype=np.float64)
        velocity_seed = self._target_joint_velocity(q_ref, target_momentum)
        xs = [np.copy(q_ref) for _ in range(hold_steps)]
        us = [np.copy(velocity_zero) for _ in range(hold_steps)]
        for _ in range(momentum_steps):
            xs.append(np.copy(q))
            us.append(np.copy(velocity_seed))
            q = self._integrate_bounded(q, velocity_seed)
        for index in range(return_steps):
            velocity_return = pin.difference(self.arm_model, q, q_ref)
            velocity_return /= max(1, return_steps - index) * self.dt
            velocity_return = np.clip(
                velocity_return, -self._velocity_limits(), self._velocity_limits()
            )
            xs.append(np.copy(q))
            us.append(velocity_return)
            q = self._integrate_bounded(q, velocity_return)
        xs.append(np.copy(q_ref))
        return xs, us

    def _integrate_bounded(self, q, velocity):
        return np.clip(
            pin.integrate(self.arm_model, q, self.dt * velocity),
            self.arm_model.lowerPositionLimit,
            self.arm_model.upperPositionLimit,
        )

    def _target_joint_velocity(self, q_ref, target_momentum):
        data = self.arm_model.createData()
        pin.ccrba(self.arm_model, data, q_ref, np.zeros(self.arm_model.nv))
        angular_map = np.asarray(data.Ag[3:6, :], dtype=np.float64)
        velocity = angular_map.T @ np.linalg.solve(
            angular_map @ angular_map.T + 1e-4 * np.eye(3), target_momentum
        )
        return np.clip(velocity, -self._velocity_limits(), self._velocity_limits())

    def _velocity_limits(self):
        limits = np.asarray(self.arm_model.velocityLimit, dtype=np.float64).copy()
        limits[~np.isfinite(limits) | (limits <= 0.0)] = self.max_velocity
        return np.minimum(limits, self.max_velocity)

    def _phase_steps(self):
        hold_steps = 0 if self.hold_duration <= 0.0 else max(
            1, int(np.round(self.hold_duration / self.dt))
        )
        return (
            hold_steps,
            max(1, int(np.round(self.momentum_duration / self.dt))),
            max(1, int(np.round(self.return_duration / self.dt))),
        )

    def _build_arm_model(self):
        arm_joint_names = self._arm_joint_names(self.arm)
        frozen_joints = [
            self.robot_model.model_body.getJointId(name)
            for name in BODY_JOINTS if name not in arm_joint_names
        ]
        model = pin.buildReducedModel(
            self.robot_model.model_body,
            frozen_joints,
            np.copy(self.robot_model.state['q']),
        )
        model.gravity.linear = np.array([0.0, 0.0, -9.81])
        return model

    @staticmethod
    def _arm_ids(arm):
        if arm == 'left':
            return LEFT_ARM_INDEX
        if arm == 'right':
            return RIGHT_ARM_INDEX
        raise ValueError('arm must be "left" or "right"')

    @staticmethod
    def _arm_joint_names(arm):
        return [BODY_JOINTS[index] for index in MomentumDDP._arm_ids(arm)]

    def _make_action_model_class(self):
        crocoddyl = self._crocoddyl

        class MomentumActionModel(crocoddyl.ActionModelAbstract):
            def __init__(self, model, q_ref, target_momentum, dt, w_momentum,
                         w_q, w_u, w_limit, max_velocity, is_terminal=False):
                self.model = model
                self.q_ref = np.asarray(q_ref, dtype=np.float64)
                self.target_momentum = np.asarray(target_momentum, dtype=np.float64)
                self.dt = float(dt)
                self.w_momentum = float(w_momentum)
                self.w_q = float(w_q)
                self.w_u = float(w_u)
                self.w_limit = float(w_limit)
                self.is_terminal = is_terminal
                self.velocity_limit = np.minimum(
                    np.where(
                        np.isfinite(model.velocityLimit) & (model.velocityLimit > 0.0),
                        model.velocityLimit,
                        max_velocity,
                    ),
                    max_velocity,
                )
                super().__init__(crocoddyl.StateVector(model.nq), 0 if is_terminal else model.nv)

            def calc(self, data, x, u=None):
                q = np.asarray(x, dtype=np.float64)
                raw_velocity = np.zeros(self.model.nv) if self.is_terminal or u is None else np.asarray(u)
                velocity = np.clip(raw_velocity, -self.velocity_limit, self.velocity_limit)
                if not np.isfinite(q).all():
                    data.xnext = np.copy(q)
                    data.cost = 1e12
                    return
                try:
                    pin.ccrba(self.model, data.pin_data, q, velocity)
                    momentum_error = data.pin_data.hg.angular - self.target_momentum
                    posture_error = pin.difference(self.model, self.q_ref, q)
                    lower = np.maximum(self.model.lowerPositionLimit - q, 0.0)
                    upper = np.maximum(q - self.model.upperPositionLimit, 0.0)
                    cost = 0.5 * self.w_momentum * momentum_error.dot(momentum_error)
                    cost += 0.5 * self.w_q * posture_error.dot(posture_error)
                    cost += 0.5 * self.w_limit * (lower + upper).dot(lower + upper)
                    data.xnext = np.copy(q) if self.is_terminal else pin.integrate(
                        self.model, q, self.dt * velocity
                    )
                    data.cost = cost if self.is_terminal else cost + 0.5 * self.w_u * raw_velocity.dot(raw_velocity)
                except Exception:
                    data.xnext = np.copy(q)
                    data.cost = 1e12

            def createData(self):
                data = crocoddyl.ActionDataAbstract(self)
                data.pin_data = self.model.createData()
                return data

        return MomentumActionModel
