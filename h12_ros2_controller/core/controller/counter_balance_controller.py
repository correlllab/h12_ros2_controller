import numpy as np
import pinocchio as pin
import pink

from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.counter_balance_ddp import CounterBalanceDDP
from h12_ros2_controller.core.support_region import (
    signed_support_margin,
    support_rectangle,
)
from h12_ros2_controller.utility.joint_definition import (
    LEFT_ARM_INDEX,
    RIGHT_ARM_INDEX,
)


class CounterBalanceController(FrameController):
    '''Frame controller with fixed opposite-arm CoM counter-balancing'''

    def __init__(self, urdf_path: str, urdf_sphere_path: str,
                 srdf_sphere_path: str, free_arm: str,
                 init: bool = True, handless: bool = False,
                 visualize: bool = False, config: dict = None):
        if free_arm not in ('left', 'right'):
            raise ValueError('free_arm must be "left" or "right"')
        self.free_arm = free_arm
        self.counter_arm = 'right' if free_arm == 'left' else 'left'
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            init=init,
            handless=handless,
            visualize=visualize,
            config=config,
        )
        self.free_ids = list(
            LEFT_ARM_INDEX if self.free_arm == 'left' else RIGHT_ARM_INDEX
        )
        self.counter_arm_ids = list(
            RIGHT_ARM_INDEX if self.free_arm == 'left' else LEFT_ARM_INDEX
        )
        self.counter_ids = self.counter_arm_ids[:4]
        self.counter_wrist_ids = self.counter_arm_ids[4:]
        self.arm_ids = list(LEFT_ARM_INDEX) + list(RIGHT_ARM_INDEX)
        self.counter_config = self.config.get('counter_balance', {})
        self.improvement_tolerance = float(
            self.counter_config.get('improvement_tolerance', 1e-6)
        )
        if (
            not np.isfinite(self.improvement_tolerance)
            or self.improvement_tolerance < 0.0
        ):
            raise ValueError(
                'counter_balance.improvement_tolerance must be nonnegative'
            )
        self.lock_tolerance = float(
            self.counter_config.get('lock_tolerance', 1e-6)
        )
        if not np.isfinite(self.lock_tolerance) or self.lock_tolerance < 0.0:
            raise ValueError('counter_balance.lock_tolerance must be nonnegative')
        self.backtrack_scales = self._backtrack_scales(
            self.counter_config.get(
                'backtrack_scales',
                [1.0, 0.5, 0.25, 0.125, 0.0],
            )
        )
        support_cfg = self.counter_config.get('support_geometry', {})
        self.support_geometry = {
            'front': float(support_cfg.get('front', 0.174)),
            'rear': float(support_cfg.get('rear', 0.086)),
            'half_width': float(support_cfg.get('half_width', 0.043)),
            'max_yaw_divergence': float(
                support_cfg.get('max_yaw_divergence', 0.349)
            ),
        }
        self.counter_balance_ddp = CounterBalanceDDP(
            self.robot_model,
            dt=self.dt,
            config=self.config,
            arm=self.counter_arm,
        )
        self._free_reduced_mask = self._motor_ids_to_reduced_mask(
            self.free_ids
        )
        self._counter_reduced_mask = self._motor_ids_to_reduced_mask(
            self.counter_arm_ids
        )
        self._counter_lock = self._make_counter_lock()
        self._reference_captured = False
        self.q_counter_ref = None
        self.counter_wrist_ref = None
        self.com_offset_ref = None
        self.latest_plan = None
        self.latest_support = None
        self.latest_status = 'idle'
        self.latest_free_command = np.zeros(7, dtype=np.float64)
        self.latest_counter_pre_limit = np.zeros(4, dtype=np.float64)
        self.latest_counter_command = np.zeros(4, dtype=np.float64)
        self.latest_applied_counter_command = np.zeros(4, dtype=np.float64)
        self.latest_com = None
        self.latest_com_target = None
        self.latest_applied_com_error = None
        self.latest_backtrack_scale = 0.0
        self.latest_clipped = False
        self.latest_collision_rejection = False
        self.latest_published = False
        self.solver_count = 0
        self.solver_failures = 0

    def add_frame_task(self, task_name: str, frame_name: str,
                       target: np.ndarray = None,
                       position_cost: float = 50.0,
                       orientation_cost: float = 30.0,
                       lm_damping: float = 3.0):
        '''Add a task only when its support belongs to the free arm'''
        self._validate_frame_ownership(frame_name)
        target_transform = self._frame_target_transform(target)
        self.ik_solver.add_frame_task(
            task_name,
            frame_name,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=lm_damping,
        )
        if target_transform is not None:
            self.ik_solver.frame_tasks[
                task_name
            ].transform_target_to_world = target_transform
        self._reference_captured = False

    @staticmethod
    def _frame_target_transform(target):
        if target is None:
            return None
        target = np.asarray(target, dtype=np.float64)
        if not np.all(np.isfinite(target)):
            raise ValueError('Target must contain only finite values')
        if target.shape == (6,):
            return pin.SE3(
                pin.rpy.rpyToMatrix(np.array(target[3:])),
                np.array(target[:3]),
            )
        if target.shape == (4, 4):
            return pin.SE3(target)
        raise ValueError(
            'Target must be either a 6D pose or a 4x4 transformation matrix'
        )

    def remove_frame_task(self, task_name: str):
        '''Remove a frame task and clear references when idle'''
        super().remove_frame_task(task_name)
        if not self.ik_solver.frame_tasks:
            self._clear_reference()

    def clear_frame_tasks(self):
        '''Clear frame tasks and the manipulation reference'''
        super().clear_frame_tasks()
        self._clear_reference()

    def capture_reference(self, support=None, com=None):
        '''Capture counter posture, wrist hold, and support-relative CoM'''
        state_q = np.asarray(self.robot_model.state['q'], dtype=np.float64)
        self.q_counter_ref = np.copy(state_q[self.counter_ids])
        self.counter_wrist_ref = np.copy(state_q[self.counter_wrist_ids])
        support = self._support_rectangle() if support is None else support
        self.latest_support = support
        if not support.valid:
            self._reference_captured = False
            return False
        if com is None:
            com = self.robot_model.get_com()
        com = np.asarray(com, dtype=np.float64)
        if com.shape[0] < 2 or not np.all(np.isfinite(com)):
            self._reference_captured = False
            return False
        self.latest_com = np.copy(com)
        self.com_offset_ref = np.copy(com[:2] - support.center)
        self._reference_captured = True
        return True

    def capture_balance_reference(self):
        '''Capture the counter-balance reference explicitly'''
        return self.capture_reference()

    def control_step(self, com=False):
        '''Run one constrained free-arm and counter-balance tick'''
        del com
        self._reset_tick_diagnostics()
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.latest_status = 'estopped'
            return self._zero_body_command()
        if not self.ik_solver.frame_tasks:
            self.latest_status = 'idle'
            self._publish_hold()
            self._clear_reference()
            return self._zero_body_command()

        self.update_robot_model()
        self.update_ik_solver()
        self._refresh_counter_lock()
        support = self._support_rectangle()
        self.latest_support = support
        try:
            current_com = np.asarray(
                self.robot_model.get_com(),
                dtype=np.float64,
            )
        except Exception:
            current_com = np.full(3, np.nan, dtype=np.float64)
        if np.all(np.isfinite(current_com)):
            self.latest_com = np.copy(current_com)
        if not self._reference_captured:
            self.capture_reference(support=support, com=current_com)

        try:
            free_command = self._free_arm_command()
        except Exception:
            self.latest_status = 'ik_failure'
            self._publish_hold()
            return self._zero_body_command()
        self.latest_free_command = np.copy(free_command[self.free_ids])

        free_candidate = self._integrate_candidate(free_command)
        if not self._candidate_valid(free_candidate):
            self.latest_status = 'free_candidate_invalid'
            self.latest_collision_rejection = True
            self._publish_hold()
            return self._zero_body_command()

        if not support.valid or not self._reference_captured:
            self.latest_status = 'invalid_support'
            self._publish_command(free_command, free_candidate)
            return free_command

        com_target = support.center + self.com_offset_ref
        self.latest_com_target = np.copy(com_target)
        full_q_template = self.robot_model.full_q(free_candidate)
        self.solver_count += 1
        try:
            plan = self.counter_balance_ddp.solve(
                full_q_template,
                com_target,
                q_ref=self.q_counter_ref,
            )
        except Exception:
            plan = None
        self.latest_plan = plan
        if plan is None or plan.status not in ('solved', 'best_effort'):
            self.solver_failures += 1
            self.latest_status = (
                'solver_failure' if plan is None else plan.status
            )
            self._publish_command(free_command, free_candidate)
            return free_command
        if (
            not np.isfinite(plan.zero_com_error)
            or not np.isfinite(plan.optimized_com_error)
            or plan.optimized_com_error
            > plan.zero_com_error + self.improvement_tolerance
        ):
            self.solver_failures += 1
            self.latest_status = 'no_improvement'
            self._publish_command(free_command, free_candidate)
            return free_command

        plan_command = np.asarray(plan.command, dtype=np.float64)
        if plan_command.shape != (4,) or not np.all(np.isfinite(plan_command)):
            self.solver_failures += 1
            self.latest_status = 'invalid_output'
            self._publish_command(free_command, free_candidate)
            return free_command
        control_lower = np.asarray(plan.control_lower, dtype=np.float64)
        control_upper = np.asarray(plan.control_upper, dtype=np.float64)
        if (
            control_lower.shape != (4,)
            or control_upper.shape != (4,)
            or not np.all(np.isfinite(control_lower))
            or not np.all(np.isfinite(control_upper))
            or np.any(plan_command < control_lower - 1e-8)
            or np.any(plan_command > control_upper + 1e-8)
        ):
            self.solver_failures += 1
            self.latest_status = 'output_out_of_bounds'
            self._publish_command(free_command, free_candidate)
            return free_command

        merged = np.copy(free_command)
        merged[self.counter_ids] = plan_command
        merged[self.counter_wrist_ids] = 0.0
        self.latest_counter_pre_limit = np.copy(plan_command)
        limited = self._apply_arm_limits(merged)
        limited[self.counter_wrist_ids] = 0.0
        limited[:self.arm_ids[0]] = 0.0
        self.latest_clipped = not np.allclose(limited, merged, atol=1e-10)
        free_limited = np.zeros_like(limited)
        free_limited[self.free_ids] = limited[self.free_ids]
        free_limited_candidate = self._integrate_candidate(free_limited)
        if not self._candidate_valid(free_limited_candidate):
            self.latest_status = 'free_candidate_invalid'
            self.latest_collision_rejection = True
            self._publish_hold()
            return self._zero_body_command()

        command, candidate, scale = self._backtrack_counter(
            free_limited,
            limited[self.counter_ids],
        )
        if command is None:
            self.latest_status = 'free_candidate_invalid'
            self.latest_collision_rejection = True
            self._publish_hold()
            return self._zero_body_command()
        self.latest_backtrack_scale = scale
        self.latest_counter_command = np.copy(command[self.counter_ids])
        applied_error = self._candidate_com_error(candidate, com_target)
        zero_error = self._candidate_com_error(
            free_limited_candidate,
            com_target,
        )
        if (
            not np.isfinite(applied_error)
            or not np.isfinite(zero_error)
            or applied_error > zero_error + self.improvement_tolerance
        ):
            command = free_limited
            candidate = free_limited_candidate
            scale = 0.0
            applied_error = zero_error
            self.latest_backtrack_scale = 0.0
            self.latest_counter_command[:] = 0.0
            self.latest_status = 'no_applied_improvement'
        self.latest_applied_com_error = applied_error
        if scale < 1.0:
            self.latest_collision_rejection = True
            if self.latest_status != 'no_applied_improvement':
                self.latest_status = 'collision_backtracked'
        else:
            self.latest_status = plan.status
        self._publish_command(command, candidate)
        return command

    def control_step_reduced(self, com=False):
        '''Run one counter-balance tick for reduced-model callers'''
        return self.control_step(com=com)

    def diagnostics(self):
        '''Return serializable counter-balance diagnostics'''
        support = self.latest_support
        plan = self.latest_plan
        com_margin = None
        if support is not None and self.latest_com is not None:
            margin = signed_support_margin(self.latest_com, support)
            com_margin = float(margin) if np.isfinite(margin) else None
        return {
            'status': self.latest_status,
            'free_arm': self.free_arm,
            'counter_arm': self.counter_arm,
            'support_valid': bool(support.valid) if support else False,
            'support_invalid_reason': (
                support.invalid_reason if support else 'not evaluated'
            ),
            'support_center': (
                support.center.tolist() if support and support.valid else None
            ),
            'support_half_extents': (
                support.half_extents.tolist()
                if support and support.valid else None
            ),
            'com': (
                self.latest_com.tolist()
                if self.latest_com is not None else None
            ),
            'com_target': (
                self.latest_com_target.tolist()
                if self.latest_com_target is not None else None
            ),
            'com_margin': com_margin,
            'free_command': self.latest_free_command.tolist(),
            'counter_pre_limit_command': (
                self.latest_counter_pre_limit.tolist()
            ),
            'counter_command': self.latest_counter_command.tolist(),
            'applied_counter_command': (
                self.latest_applied_counter_command.tolist()
            ),
            'counter_wrist_command': [0.0, 0.0, 0.0],
            'backtrack_scale': float(self.latest_backtrack_scale),
            'clipped': bool(self.latest_clipped),
            'collision_rejection': bool(self.latest_collision_rejection),
            'invalid_support': bool(support is not None and not support.valid),
            'estopped': bool(
                getattr(self.low_cmd_handler, '_estopped', False)
            ),
            'published': bool(self.latest_published),
            'solver_count': int(self.solver_count),
            'solver_failures': int(self.solver_failures),
            'solver_status': plan.status if plan else None,
            'solved': bool(plan.solved) if plan else False,
            'solve_time': float(plan.solve_time) if plan else None,
            'zero_com_error': (
                float(plan.zero_com_error)
                if plan and np.isfinite(plan.zero_com_error) else None
            ),
            'optimized_com_error': (
                float(plan.optimized_com_error)
                if plan and np.isfinite(plan.optimized_com_error) else None
            ),
            'applied_com_error': (
                float(self.latest_applied_com_error)
                if self.latest_applied_com_error is not None
                and np.isfinite(self.latest_applied_com_error) else None
            ),
        }

    def _validate_frame_ownership(self, frame_name):
        model = self.robot_model.model_body_reduced
        frame_id = model.getFrameId(frame_name)
        if frame_id >= len(model.frames):
            raise ValueError(f'Unknown frame: {frame_name}')
        support_mask = self._reduced_support_mask(frame_name)
        if not np.any(support_mask):
            raise ValueError('Frame task must be supported by the free arm')
        if not np.all(self._free_reduced_mask[support_mask]):
            raise ValueError('Frame task crosses fixed arm ownership')

    def _motor_ids_to_reduced_mask(self, motor_ids):
        active_motor_ids = np.flatnonzero(self.robot_model.reduced_mask)
        reduced_ids = {
            motor_id: reduced_id
            for reduced_id, motor_id in enumerate(active_motor_ids)
        }
        mask = np.zeros(
            self.robot_model.model_body_reduced.nv,
            dtype=bool,
        )
        for motor_id in motor_ids:
            if motor_id not in reduced_ids:
                raise ValueError('Arm joint is missing from the reduced model')
            mask[reduced_ids[motor_id]] = True
        return mask

    def _make_counter_lock(self):
        selection = np.zeros(
            (len(self.counter_arm_ids), self.robot_model.model_body_reduced.nv),
            dtype=np.float64,
        )
        selection[:, self._counter_reduced_mask] = np.eye(
            len(self.counter_arm_ids)
        )
        return pink.tasks.LinearHolonomicTask(
            selection,
            np.zeros(len(self.counter_arm_ids), dtype=np.float64),
            np.copy(self.robot_model.state_reduced['q']),
        )

    def _refresh_counter_lock(self):
        self._counter_lock.q_0 = np.copy(
            self.robot_model.state_reduced['q']
        )

    def _free_arm_command(self):
        command = self.ik_solver.ik_step_reduced(
            constraints=[self._counter_lock],
        )
        command = np.asarray(command, dtype=np.float64)
        if (
            command.shape != (self.robot_model.model_body.nv,)
            or not np.all(np.isfinite(command))
        ):
            raise ValueError('Pink returned an invalid velocity')
        if np.max(np.abs(command[self.counter_arm_ids])) > self.lock_tolerance:
            raise RuntimeError('Pink violated the reserved-arm hard lock')
        free_command = np.zeros_like(command)
        free_command[self.free_ids] = command[self.free_ids]
        return self._apply_arm_limits(free_command)

    def _apply_arm_limits(self, command):
        limited = self._limit_joint_vel(np.asarray(command, dtype=np.float64))
        state_q = np.asarray(self.robot_model.state['q'], dtype=np.float64)
        for ids in (self.free_ids, self.counter_arm_ids):
            velocity_limit = self._effective_velocity_limits(ids)
            lower, upper = self._position_limits(ids)
            values = np.clip(limited[ids], -velocity_limit, velocity_limit)
            values = np.minimum(values, (upper - state_q[ids]) / self.dt)
            values = np.maximum(values, (lower - state_q[ids]) / self.dt)
            limited[ids] = values
        return limited

    def _effective_velocity_limits(self, ids):
        model_limit = np.asarray(
            self.robot_model.model_body.velocityLimit,
            dtype=np.float64,
        )[ids]
        model_limit = np.where(
            np.isfinite(model_limit) & (model_limit > 0.0),
            model_limit,
            np.inf,
        )
        limits = np.minimum(model_limit, self.dq_lim)
        publisher_limit = self.config.get('limits', {}).get('dq_clip_limits')
        if publisher_limit is not None:
            limits = np.minimum(
                limits,
                np.asarray(publisher_limit, dtype=np.float64)[ids],
            )
        if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
            raise ValueError('Arm velocity limits must be finite and positive')
        return limits

    def _position_limits(self, ids):
        lower = np.asarray(
            self.robot_model.model_body.lowerPositionLimit,
            dtype=np.float64,
        )[ids].copy()
        upper = np.asarray(
            self.robot_model.model_body.upperPositionLimit,
            dtype=np.float64,
        )[ids].copy()
        publisher_limit = self.config.get('limits', {}).get('q_clip_limits')
        if publisher_limit is not None:
            publisher_limit = np.asarray(publisher_limit, dtype=np.float64)[ids]
            lower = np.maximum(lower, publisher_limit[:, 0])
            upper = np.minimum(upper, publisher_limit[:, 1])
        return lower, upper

    def _integrate_candidate(self, command):
        q = pin.integrate(
            self.robot_model.model_body,
            np.asarray(self.robot_model.state['q'], dtype=np.float64),
            self.dt * np.asarray(command, dtype=np.float64),
        )
        if self.counter_wrist_ref is not None:
            q[self.counter_wrist_ids] = self.counter_wrist_ref
        return q

    def _candidate_valid(self, q):
        q = np.asarray(q, dtype=np.float64)
        if q.shape != (self.robot_model.model_body.nq,) or not np.all(
            np.isfinite(q)
        ):
            return False
        publisher_limit = self.config.get('limits', {}).get('q_clip_limits')
        if publisher_limit is not None:
            publisher_limit = np.asarray(publisher_limit, dtype=np.float64)
            if np.any(q < publisher_limit[:, 0]) or np.any(
                q > publisher_limit[:, 1]
            ):
                return False
        return bool(self.robot_model.check_valid(q))

    def _backtrack_counter(self, free_command, counter_command):
        for scale in self.backtrack_scales:
            command = np.copy(free_command)
            command[self.counter_ids] = scale * counter_command
            command[self.counter_wrist_ids] = 0.0
            candidate = self._integrate_candidate(command)
            if self._candidate_valid(candidate):
                return command, candidate, scale
        return None, None, 0.0

    def _candidate_com_error(self, candidate, com_target):
        full_q = self.robot_model.full_q(candidate)
        return self.counter_balance_ddp.evaluate_com_error(
            full_q,
            com_target,
        )

    def _support_rectangle(self):
        try:
            left = self.robot_model.get_frame_transformation(
                'left_ankle_roll_link'
            )
            right = self.robot_model.get_frame_transformation(
                'right_ankle_roll_link'
            )
        except Exception:
            left = None
            right = None
        return support_rectangle(left, right, **self.support_geometry)

    def _publish_command(self, command, candidate):
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.latest_status = 'estopped'
            return False
        gravity = self.robot_model.dynamics.get_gravity_compensation(
            self.robot_model.state['q']
        )
        try:
            self.low_cmd_handler.set_joint_commands(
                q=np.asarray(candidate, dtype=np.float64)[self.arm_ids],
                dq=np.asarray(command, dtype=np.float64)[self.arm_ids],
                tau=np.asarray(gravity, dtype=np.float64)[self.arm_ids],
                joint_ids=self.arm_ids,
            )
        except Exception:
            if getattr(self.low_cmd_handler, '_estopped', False):
                self.latest_status = 'estopped'
                return False
            raise
        self.latest_published = True
        applied = np.asarray(
            self.low_cmd_handler.dq_cmd,
            dtype=np.float64,
        )
        self.latest_applied_counter_command = np.copy(
            applied[self.counter_ids]
        )
        if self.latest_com_target is not None:
            applied_q = np.copy(self.robot_model.state['q'])
            applied_q[self.arm_ids] = np.asarray(
                self.low_cmd_handler.q_cmd,
                dtype=np.float64,
            )[self.arm_ids]
            self.latest_applied_com_error = self._candidate_com_error(
                applied_q,
                self.latest_com_target,
            )
        return True

    def _publish_hold(self):
        command = self._zero_body_command()
        candidate = np.copy(self.robot_model.state['q'])
        if self.counter_wrist_ref is not None:
            candidate[self.counter_wrist_ids] = self.counter_wrist_ref
        self._publish_command(command, candidate)

    def _reset_tick_diagnostics(self):
        self.latest_plan = None
        self.latest_support = None
        self.latest_free_command[:] = 0.0
        self.latest_counter_pre_limit[:] = 0.0
        self.latest_counter_command[:] = 0.0
        self.latest_applied_counter_command[:] = 0.0
        self.latest_com = None
        self.latest_com_target = None
        self.latest_applied_com_error = None
        self.latest_backtrack_scale = 0.0
        self.latest_clipped = False
        self.latest_collision_rejection = False
        self.latest_published = False

    def _clear_reference(self):
        self._reference_captured = False
        self.q_counter_ref = None
        self.counter_wrist_ref = None
        self.com_offset_ref = None

    def _zero_body_command(self):
        return np.zeros(self.robot_model.model_body.nv, dtype=np.float64)

    @staticmethod
    def _backtrack_scales(values):
        scales = np.asarray(values, dtype=np.float64).reshape(-1)
        if (
            scales.size == 0
            or not np.all(np.isfinite(scales))
            or np.any(scales < 0.0)
            or np.any(scales > 1.0)
        ):
            raise ValueError(
                'counter_balance.backtrack_scales must be within [0, 1]'
            )
        scales = np.unique(np.concatenate([scales, [0.0, 1.0]]))[::-1]
        return scales.tolist()
