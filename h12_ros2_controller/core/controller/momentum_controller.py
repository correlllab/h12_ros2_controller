import numpy as np

from h12_ros2_controller.core.controller.upper_controller import UpperController


class MomentumController(UpperController):
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 handless: bool=False,
                 visualize: bool=False,
                 config: dict=None):
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            handless=handless,
            visualize=visualize,
            config=config,
        )
        self.momentum = self.robot_model.dynamics.create_momentum_ddp(
            dt=self.dt,
            config=self.config,
            arm='left',
        )
        controller_cfg = self.config.get('controller', {})
        self.return_threshold = float(
            controller_cfg.get('threshold_joint', 1e-3)
        )
        self.return_max_steps = max(
            1,
            int(float(controller_cfg.get('timeout', 5.0)) / self.dt),
        )
        self._latest_plan = None
        self._plan_index = 0
        self._return_q_reduced = None
        self._return_index = 0

    @property
    def latest_plan(self):
        if self._latest_plan is None:
            return None
        return self._latest_plan

    @property
    def plan_done(self):
        if self._latest_plan is None:
            return True
        if self._plan_index < len(self._latest_plan.us):
            return False
        return self._return_complete()

    def plan_momentum(self, target_momentum):
        '''Solve and store a DDP momentum trajectory'''
        # refresh model and ik state before taking the current arm posture
        self.update_robot_model()
        self.update_ik_solver()
        self._return_q_reduced = np.copy(self.robot_model.state_reduced['q'])
        self._latest_plan = self.momentum.solve(target_momentum)
        self._plan_index = 0
        self._return_index = 0
        return self._latest_plan

    def execute_plan_step(self, plan=None):
        '''Execute one stored momentum trajectory velocity command'''
        plan = self._latest_plan if plan is None else plan
        if plan is None:
            return np.zeros(self.robot_model.model_body.nv)
        if self._plan_index >= len(plan.us):
            return self._execute_return_step()

        # map selected-arm ddp velocity into the full body command vector
        vel_body = plan.body_velocity_at(
            self._plan_index,
            self.robot_model.model_body.nv,
        )
        vel_cmd = self._apply_velocity_command(vel_body)
        self._plan_index += 1
        return vel_cmd

    def _execute_return_step(self):
        '''Run one closed-loop return step toward the saved start posture'''
        if self._return_q_reduced is None or self._return_complete():
            return np.zeros(self.robot_model.model_body.nv)

        self._return_index += 1
        return self.goto_reduced_configuration(self._return_q_reduced)

    def _return_complete(self):
        if self._return_q_reduced is None:
            return True
        if self._return_index >= self.return_max_steps:
            return True
        error = np.max(
            np.abs(self.robot_model.state_reduced['q'] - self._return_q_reduced)
        )
        return error < self.return_threshold
