import os
import time
import pink
import qpsolvers
import meshcat_shapes
import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.robot_model import RobotModel

class IKSolver:
    def __init__(self,
                 robot_model: RobotModel,
                 dt: float = 0.02,
                 dmin: float = 0.05):
        '''
        Initialize the IK solver with robot model and collision parameters

        @param robot_model: the robot model instance
        @param urdf_sphere_path: path to the sphere collision URDF
        @param srdf_sphere_path: path to the sphere collision SRDF
        @param dt: time step for IK solving
        @param dmin: minimum distance for collision avoidance
        '''
        self.robot_model = robot_model
        self.dt = dt
        self.dmin = dmin

        # task management
        self.frame_tasks = {}  # maps task_name -> FrameTask
        # basic tasks
        self.posture_task = pink.tasks.PostureTask(cost=1e-3)
        self.config_task = pink.tasks.PostureTask(cost=30.0)

        assert(self.robot_model.init_collision), 'Collision model is not initialized.'
        self.collision_barrier_reduced = pink.barriers.SelfCollisionBarrier(
            n_collision_pairs=len(self.robot_model.collision_model_reduced.collisionPairs),
            gain=20.0,
            safe_displacement_gain=1.0,
            d_min=dmin,
        )

        # configurations
        self.configuration = pink.Configuration(
            robot_model.model,
            robot_model.data,
            robot_model.zero_q,
        )
        self.reduced_configuration = pink.Configuration(
            robot_model.model_reduced,
            robot_model.data_reduced,
            robot_model.zero_q_reduced,
            collision_model=self.robot_model.collision_model_reduced,
            collision_data=self.robot_model.collision_data_reduced
        )

        # limits
        self.limits = [
            pink.limits.ConfigurationLimit(robot_model.model),
            pink.limits.VelocityLimit(robot_model.model)
        ]
        self.limits_reduced = [
            pink.limits.ConfigurationLimit(robot_model.model_reduced),
            pink.limits.VelocityLimit(robot_model.model_reduced)
        ]

        # solver selection
        self.solver = qpsolvers.available_solvers[0]
        for preferred in ['proxqp', 'daqp', 'quadprog', 'osqp']:
            if preferred in qpsolvers.available_solvers:
                self.solver = preferred
                break

    def add_frame_task(self, task_name: str, frame_name: str,
                       position_cost: float = 50.0,
                       orientation_cost: float = 30.0,
                       lm_damping: float = 3.0):
        '''Add a frame task to the IK solver'''
        self.frame_tasks[task_name] = pink.tasks.FrameTask(
            frame_name,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=lm_damping
        )
        # add frame to visualizer
        if self.robot_model.viz is not None:
            viewer = self.robot_model.viz.viewer
            meshcat_shapes.frame(viewer[task_name], opacity=1.0)
            meshcat_shapes.frame(viewer[frame_name], opacity=1.0)

    def remove_frame_task(self, task_name: str):
        '''Remove a frame task from the IK solver'''
        frame_name = self.frame_tasks[task_name].frame
        self.frame_tasks.pop(task_name, None)
        # remove frame from visualizer
        if self.robot_model.viz is not None:
            viewer = self.robot_model.viz.viewer
            viewer[task_name].delete()
            viewer[frame_name].delete()

    def update_visualizer(self):
        if self.robot_model.viz is not None:
            viewer = self.robot_model.viz.viewer
            for task_name, task in self.frame_tasks.items():
                viewer[task_name].set_transform(task.transform_target_to_world.np)
                frame_name = self.frame_tasks[task_name].frame
                frame_id = self.robot_model.model.getFrameId(frame_name)
                viewer[frame_name].set_transform(self.robot_model.data.oMf[frame_id].np)

    def update_configurations(self):
        '''Update Pink configurations with current robot state'''
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

    def set_from_configuration(self):
        '''Set initial targets for all tasks from current configuration'''
        self.update_configurations()
        for task in self.frame_tasks.values():
            task.set_target_from_configuration(self.configuration)

    def set_from_reduced_configuration(self):
        '''Set initial targets for all tasks from reduced configuration'''
        self.update_configurations()
        for task in self.frame_tasks.values():
            task.set_target_from_configuration(self.reduced_configuration)

    def _ik(self, tasks, dt):
        '''Helper function solving IK for all tasks'''
        return pink.solve_ik(
            self.configuration,
            tasks,
            dt=dt,
            solver=self.solver,
            limits=self.limits,
            barriers=[],
            safety_break=False
        )

    def _ik_reduced(self, tasks, dt):
        '''Helper function solving IK on reduced model for all tasks'''
        return pink.solve_ik(
            self.reduced_configuration,
            tasks,
            dt=dt,
            solver=self.solver,
            limits=self.limits_reduced,
            barriers=[self.collision_barrier_reduced],
            safety_break=False
        )

    def ik_step(self):
        '''Solve one step of IK for all tasks and return joint velocity'''
        self.update_configurations()
        self.posture_task.set_target_from_configuration(self.configuration)
        tasks = list(self.frame_tasks.values()) + [self.posture_task]
        return self._ik(tasks, self.dt)

    def ik_step_reduced(self):
        '''Solve one step of IK on reduced model for all tasks and return joint velocity'''
        self.update_configurations()
        self.posture_task.set_target_from_configuration(self.reduced_configuration)
        tasks = list(self.frame_tasks.values()) + [self.posture_task]
        vel = self._ik_reduced(tasks, self.dt)
        # convert reduced vel to full vel
        vel_full = self.robot_model.zero_q
        vel_full[self.robot_model.reduced_mask] = vel
        return vel_full

    def solve_ik(self, alpha=0.1,
                 timeout=1.0, linear_threshold=5e-3, angular_threshold=2e-2):
        # reset configuration to zero position
        self.configuration.update(self.robot_model.zero_q)
        # optimization loop
        start_time = time.time()
        while time.time() - start_time < timeout:
            self.posture_task.set_target_from_configuration(self.configuration)
            tasks = list(self.frame_tasks.values()) + [self.posture_task]
            vel = self._ik(tasks, alpha)
            self.configuration.integrate_inplace(vel, alpha)
            # check convergence
            linear_err = 0
            angular_err = 0
            for task in self.frame_tasks.values():
                err = task.compute_error(self.configuration)
                linear_err += np.linalg.norm(err[:3])
                angular_err += np.linalg.norm(err[3:])
            if linear_err < linear_threshold and angular_err < angular_threshold:
                return {
                    'q': np.copy(self.configuration.q),
                    'success': True
                }

        return {
            'q': np.copy(self.configuration.q),
            'success': False
        }

    def solve_ik_reduced(self, alpha=0.1,
                         timeout=1.0, linear_threshold=5e-3, angular_threshold=2e-2):
        # reset configuration to zero position
        self.reduced_configuration.update(self.robot_model.zero_q_reduced)
        # optimization loop
        start_time = time.time()
        while time.time() - start_time < timeout:
            self.posture_task.set_target_from_configuration(self.reduced_configuration)
            tasks = list(self.frame_tasks.values()) + [self.posture_task]
            vel = self._ik_reduced(tasks, alpha)
            self.reduced_configuration.integrate_inplace(vel, alpha)
            # check convergence
            linear_err = 0
            angular_err = 0
            for task in self.frame_tasks.values():
                err = task.compute_error(self.reduced_configuration)
                linear_err += np.linalg.norm(err[:3])
                angular_err += np.linalg.norm(err[3:])
            if linear_err < linear_threshold and angular_err < angular_threshold:
                q_full = self.robot_model.zero_q
                q_full[self.robot_model.reduced_mask] = self.reduced_configuration.q
                return {
                    'q': q_full,
                    'success': True
                }

        q_full = self.robot_model.zero_q
        q_full[self.robot_model.reduced_mask] = self.reduced_configuration.q
        return {
            'q': q_full,
            'success': False
        }

    def goto_configuration(self, q: np.ndarray):
        '''Solve one step of IK to reach a target joint configuration.'''
        self.update_configurations()
        self.config_task.set_target(q)
        return self._ik([self.config_task], self.dt)

    def goto_reduced_configuration(self, q_reduced: np.ndarray):
        '''Solve one step of IK to reach a target reduced joint configuration.'''
        self.update_configurations()
        self.config_task.set_target(q_reduced)
        vel = self._ik_reduced([self.config_task], self.dt)
        # convert reduced vel to full vel
        vel_full = self.robot_model.zero_q
        vel_full[self.robot_model.reduced_mask] = vel
        return vel_full
