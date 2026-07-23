import numpy as np

from h12_ros2_controller.core.controller.upper_controller import (
    UpperController,
)
from h12_ros2_controller.core.controller.zmp_legacy.balance_observer import (
    BalanceObserver,
)


class DirectZmpController(UpperController):
    '''Direct measured-balance to arm-motion controller'''

    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 init: bool = True,
                 handless: bool = False,
                 visualize: bool = False,
                 config: dict = None):
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            init=init,
            handless=handless,
            visualize=visualize,
            config=config,
        )
        self.zmp_config = self.config.get('zmp', {})
        self.enabled = bool(self.zmp_config.get('enabled', False))
        self.output_enabled = False
        self.observer = BalanceObserver(self.robot_model, self.dt, self.config)
        self.balance_ddp = self.robot_model.dynamics.create_balance_ddp(
            dt=self.dt,
            config=self.config,
        )
        feedback = self.zmp_config.get('feedback', {})
        self.zmp_gain = self._vec2(feedback.get('zmp_gain', [1.0, 1.0]))
        self.control_zmp_gain = self._vec2(
            feedback.get('control_zmp_gain', self.zmp_gain)
        )
        self.angular_velocity_gain = self._vec2(
            feedback.get('angular_velocity_gain', [0.0, 0.0])
        )
        self.tilt_gain = self._vec2(
            feedback.get('tilt_gain', [0.0, 0.0])
        )
        self.stop_on_rate_decay = bool(
            feedback.get('stop_on_rate_decay', False)
        )
        self.rate_peak_threshold = float(
            feedback.get('rate_peak_threshold', 0.10)
        )
        self.stop_rate_threshold = float(
            feedback.get('stop_rate_threshold', 0.05)
        )
        self.enter_threshold = float(
            feedback.get('enter_threshold', feedback.get('deadband', 0.002))
        )
        self.exit_threshold = float(
            feedback.get('exit_threshold', self.enter_threshold)
        )
        self.max_error = float(feedback.get('max_error', 0.04))
        self.abort_threshold = float(
            feedback.get('abort_threshold', np.inf)
        )
        self.q_ref = self.balance_ddp.current_arm_q()
        self.latest_balance_state = None
        self.latest_balance_error = np.zeros(2, dtype=np.float64)
        self.latest_command = np.zeros(
            self.robot_model.model_body.nv,
            dtype=np.float64,
        )
        self.latest_plan = None
        self.latest_applied_momentum = np.zeros(3, dtype=np.float64)
        self.tilt_reference = self._imu_tilt()
        self.latest_tilt_error = np.zeros(2, dtype=np.float64)
        self.active = False
        self.response_pitch_direction = 0.0
        self.response_peak_seen = False
        self.recovery_latched = False
        self.latest_status = 'disabled' if not self.enabled else 'calibrating'

    def control_step(self, com=False):
        '''Run one direct balance feedback tick'''
        del com
        self.update_robot_model()
        self.update_ik_solver()
        freeze_reference = self._reference_should_freeze()
        balance_state = self.observer.observe(
            freeze_center_reference=freeze_reference,
        )
        self.latest_balance_state = balance_state
        self.latest_tilt_error = self._imu_tilt() - self.tilt_reference
        balance_error = self._control_error(balance_state)
        self.latest_balance_error = balance_error

        if not self.enabled or not self.output_enabled:
            self.latest_status = 'observing'
            return self._zero_command()
        if not balance_state.armed:
            self.latest_status = 'calibrating'
            return self._zero_command()
        raw_error = self.zmp_gain * balance_state.zmp_residual
        if np.linalg.norm(raw_error) > self.abort_threshold:
            self.active = False
            self.latest_status = 'out_of_range'
            return self._zero_command()
        trigger_error = self.zmp_gain * np.asarray(
            balance_state.zmp_residual,
            dtype=np.float64,
        )
        if self.recovery_latched:
            self.latest_status = 'recovery_hold'
            return self._hold_arm_position()
        was_active = self.active
        self._update_active(balance_error, trigger_error=trigger_error)
        if not self.active:
            self.latest_status = 'deadband'
            return self._zero_command()
        pitch_rate = float(balance_state.angular_velocity[1])
        if self._response_should_stop(trigger_error, pitch_rate, was_active):
            self.active = False
            self.recovery_latched = True
            self.latest_status = 'rate_reversal'
            return self._zero_command()

        plan = self.balance_ddp.solve(balance_error, q_ref=self.q_ref)
        self.latest_plan = plan
        if not len(plan.us) or not np.isfinite(plan.us).all():
            self.latest_status = 'solver_failure'
            return self._zero_command()
        command = self._limit_joint_vel(
            plan.body_velocity(self.robot_model.model_body.nv)
        )
        self.latest_applied_momentum = self.balance_ddp.angular_momentum(
            self.balance_ddp.current_arm_q(),
            command[self.balance_ddp.arm_ids],
        )
        self.latest_command = np.copy(command)
        self.latest_status = 'solved' if plan.solved else 'best_effort'
        if np.linalg.norm(command) <= 1e-12:
            return command
        return self._apply_balance_command(command)

    def control_step_reduced(self, com=False):
        '''Run one direct balance feedback tick for reduced callers'''
        return self.control_step(com=com)

    def set_output_enabled(self, enabled):
        '''Enable or suppress arm output while retaining observation'''
        self.output_enabled = bool(enabled)

    def reset_balance_reference(self):
        '''Reset quiet references and the arm posture reference'''
        self.observer.reset_calibration()
        self.q_ref = self.balance_ddp.current_arm_q()
        self.latest_plan = None
        self.latest_applied_momentum = np.zeros(3, dtype=np.float64)
        self.tilt_reference = self._imu_tilt()
        self.latest_tilt_error = np.zeros(2, dtype=np.float64)
        self.active = False
        self.response_pitch_direction = 0.0
        self.response_peak_seen = False
        self.recovery_latched = False
        self.latest_balance_error = np.zeros(2, dtype=np.float64)
        self.latest_command = np.zeros(
            self.robot_model.model_body.nv,
            dtype=np.float64,
        )

    def diagnostics(self):
        '''Return serializable controller diagnostics'''
        state = self.latest_balance_state
        plan = self.latest_plan
        active_solution = self.latest_status in ('solved', 'best_effort')
        return {
            'status': self.latest_status,
            'calibrated': bool(state.calibrated) if state else False,
            'armed': bool(state.armed) if state else False,
            'zmp_residual': (
                state.zmp_residual.tolist() if state else [0.0, 0.0]
            ),
            'zmp_residual_velocity': (
                state.zmp_residual_velocity.tolist()
                if state else [0.0, 0.0]
            ),
            'center_shift': (
                state.center_shift.tolist() if state else [0.0, 0.0]
            ),
            'com_velocity': (
                state.com_velocity.tolist() if state else [0.0, 0.0]
            ),
            'angular_velocity': (
                state.angular_velocity.tolist() if state else [0.0] * 3
            ),
            'angular_acceleration': (
                state.angular_acceleration.tolist() if state else [0.0] * 3
            ),
            'tilt_error': self.latest_tilt_error.tolist(),
            'support_margin': float(state.support_margin) if state else 0.0,
            'balance_error': self.latest_balance_error.tolist(),
            'command_norm': float(np.linalg.norm(self.latest_command)),
            'max_arm_velocity': float(np.max(np.abs(self.latest_command))),
            'arm_command': self.latest_command[
                self.balance_ddp.arm_ids
            ].tolist(),
            'arm_q': self.balance_ddp.current_arm_q().tolist(),
            'arm_dq': self.balance_ddp.current_arm_dq().tolist(),
            'solve_time': (
                float(plan.solve_time) if plan and active_solution else 0.0
            ),
            'solved': bool(plan.solved) if plan and active_solution else False,
            'predicted_error': (
                plan.predicted_error.tolist()
                if plan and active_solution else [0.0, 0.0]
            ),
            'measured_momentum': (
                plan.measured_momentum.tolist()
                if plan and active_solution else [0.0] * 3
            ),
            'seed_momentum': (
                plan.seed_momentum.tolist()
                if plan and active_solution else [0.0] * 3
            ),
            'solver_momentum': (
                plan.commanded_momentum.tolist()
                if plan and active_solution else [0.0] * 3
            ),
            'applied_momentum': (
                self.latest_applied_momentum.tolist()
                if active_solution else [0.0] * 3
            ),
        }

    def _feedback_error(self, zmp_residual):
        error = self.zmp_gain * np.asarray(zmp_residual, dtype=np.float64)
        return self._clip_error(error)

    def _control_error(self, balance_state):
        angular_velocity = balance_state.angular_velocity
        angular_error = self.angular_velocity_gain * np.array(
            [-angular_velocity[1], angular_velocity[0]],
            dtype=np.float64,
        )
        tilt_error = self.tilt_gain * np.array(
            [-self.latest_tilt_error[1], self.latest_tilt_error[0]],
            dtype=np.float64,
        )
        zmp_error = self.control_zmp_gain * np.asarray(
            balance_state.zmp_residual,
            dtype=np.float64,
        )
        error = zmp_error + angular_error + tilt_error
        return self._clip_error(error)

    def _clip_error(self, error):
        norm = np.linalg.norm(error)
        if norm > self.max_error > 0.0:
            error = error * self.max_error / norm
        return error

    def _reference_should_freeze(self):
        if self.latest_balance_state is None:
            return False
        return bool(
            self.latest_balance_state.armed
            and self.output_enabled
            and self.active
        )

    def _update_active(self, balance_error, trigger_error=None):
        norm = float(np.linalg.norm(balance_error))
        trigger_norm = float(np.linalg.norm(
            balance_error if trigger_error is None else trigger_error
        ))
        if self.active:
            self.active = norm > self.exit_threshold
        else:
            self.active = trigger_norm >= self.enter_threshold

    def _response_should_stop(self, trigger_error, pitch_rate, was_active):
        if not was_active and abs(trigger_error[0]) >= self.enter_threshold:
            self.response_pitch_direction = float(-np.sign(trigger_error[0]))
        directed_rate = self.response_pitch_direction * pitch_rate
        if directed_rate >= self.rate_peak_threshold:
            self.response_peak_seen = True
        return bool(
            self.stop_on_rate_decay
            and self.response_pitch_direction != 0.0
            and self.response_peak_seen
            and directed_rate <= self.stop_rate_threshold
        )

    def _zero_command(self):
        self.latest_command = np.zeros(
            self.robot_model.model_body.nv,
            dtype=np.float64,
        )
        self.latest_applied_momentum = np.zeros(3, dtype=np.float64)
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.latest_status = 'estopped'
            return np.copy(self.latest_command)
        arm_ids = self.balance_ddp.arm_ids
        self.low_cmd_handler.set_joint_commands(
            dq=np.zeros(len(arm_ids), dtype=np.float64),
            joint_ids=arm_ids,
        )
        return np.copy(self.latest_command)

    def _apply_balance_command(self, command):
        if getattr(self.low_cmd_handler, '_estopped', False):
            self.latest_status = 'estopped'
            return self._zero_command()
        self.ik_solver.integrate(command)
        arm_ids = self.balance_ddp.arm_ids
        gravity = self.robot_model.dynamics.get_gravity_compensation(
            self.robot_model.state['q']
        )
        self.low_cmd_handler.set_joint_commands(
            q=self.ik_solver.q[arm_ids],
            dq=command[arm_ids],
            tau=gravity[arm_ids],
            joint_ids=arm_ids,
        )
        self.update_robot_model()
        return command

    def _hold_arm_position(self):
        arm_ids = self.balance_ddp.arm_ids
        gravity = self.robot_model.dynamics.get_gravity_compensation(
            self.robot_model.state['q']
        )
        self.low_cmd_handler.set_joint_commands(
            q=self.robot_model.state['q'][arm_ids],
            dq=np.zeros(len(arm_ids), dtype=np.float64),
            tau=gravity[arm_ids],
            joint_ids=arm_ids,
        )
        return self._zero_command()

    def _imu_tilt(self):
        imu_state = self.robot_model.state.get('imu_state')
        quaternion = getattr(imu_state, 'quaternion', None)
        if quaternion is None:
            return np.zeros(2, dtype=np.float64)
        w, x, y, z = np.asarray(quaternion, dtype=np.float64)
        roll = np.arctan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )
        pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
        return np.array([roll, pitch], dtype=np.float64)

    @staticmethod
    def _vec2(value):
        array = np.asarray(value, dtype=np.float64).reshape(-1)
        if array.size != 2:
            raise ValueError('expected a two-element feedback gain')
        return array


__all__ = ['DirectZmpController']
