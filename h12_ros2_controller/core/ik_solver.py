import os
import numpy as np

import pink
import qpsolvers
import pinocchio as pin

from h12_ros2_controller.core.robot_model import RobotModel

class IKSolver:
    def __init__(self,
                 robot_model: RobotModel,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 dt: float = 0.02,
                 dmin: float = 0.05):
        '''
        Initialize the IK solver with robot model and collision parameters.

        Args:
            robot_model: The robot model instance
            urdf_sphere_path: Path to the sphere collision URDF
            srdf_sphere_path: Path to the sphere collision SRDF
            dt: Time step for IK solving
            dmin: Minimum distance for collision avoidance
        '''
        self.robot_model = robot_model
        self.dt = dt
        self.dmin = dmin

        # task management
        self.frame_tasks = {}  # maps task_name -> FrameTask
        # basic tasks
        self.posture_task = pink.PostureTask(cost=1e-3)
        self.joint_task = pink.PostureTask(cost=30.0)

        # collision models
        sphere_model, _, collision_model = pin.buildModelsFromUrdf(
            filename=urdf_sphere_path,
            package_dirs=os.path.dirname(urdf_sphere_path),
        )
        self.sphere_model_reduced, self.collision_model_reduced = pin.buildReducedModel(
            sphere_model,
            collision_model,
            robot_model.frozen_ids,
            robot_model.zero_q
        )
        self.collision_data_reduced = pink.utils.process_collision_pairs(
            self.sphere_model_reduced,
            self.collision_model_reduced,
            srdf_sphere_path,
        )
        self.collision_barrier_reduced = pink.barriers.SelfCollisionBarrier(
            n_collision_pairs=len(self.collision_model_reduced.collisionPairs),
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
            collision_model=self.collision_model_reduced,
            collision_data=self.collision_data_reduced
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
                       lm_damping: float = 1.0):
        '''Add a frame task to the IK solver.'''
        self.frame_tasks[task_name] = pink.FrameTask(
            frame_name,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=lm_damping
        )

    def remove_frame_task(self, task_name: str):
        '''Remove a frame task from the IK solver.'''
        self.frame_tasks.pop(task_name, None)

    def update_configurations(self):
        '''Update Pink configurations with current robot state.'''
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

    def set_initial_targets(self):
        '''Set initial targets for all tasks from current configuration.'''
        for task in self.frame_tasks.values():
            task.set_target_from_configuration(self.configuration)

    def solve_ik(self):
        '''Solve IK for all tasks and return joint velocities.'''
        self.update_configurations()
        self.posture_task.set_target_from_configuration(self.configuration)
        tasks = list(self.frame_tasks.values()) + [self.posture_task]
        return pink.solve_ik(
            self.configuration,
            tasks,
            dt=self.dt,
            solver=self.solver,
            limits=self.limits,
            barriers=[],
            safety_break=False
        )

    def solve_ik_reduced(self):
        '''Solve IK for reduced model with collision avoidance.'''
        self.update_configurations()
        self.posture_task.set_target_from_configuration(self.reduced_configuration)
        tasks = list(self.frame_tasks.values()) + [self.posture_task]
        vel = pink.solve_ik(
            self.reduced_configuration,
            tasks,
            dt=self.dt,
            solver=self.solver,
            limits=self.limits_reduced,
            barriers=[self.collision_barrier_reduced],
            safety_break=False
        )
        # convert reduced vel to full vel
        vel_full = np.zeros(self.robot_model.model.nv)
        vel_full[self.robot_model.reduced_mask] = vel
        return vel_full

    def goto_configuration(self, q: np.ndarray):
        '''Solve IK to reach a target joint configuration.'''
        self.update_configurations()
        self.joint_task.set_target(q)
        return pink.solve_ik(
            self.configuration,
            [self.joint_task],
            dt=self.dt,
            solver=self.solver,
            limits=self.limits,
            barriers=[],
            safety_break=False
        )

    def goto_reduced_configuration(self, q_reduced: np.ndarray):
        '''Solve IK to reach a target reduced joint configuration.'''
        self.update_configurations()
        self.joint_task.set_target(q_reduced)
        vel = pink.solve_ik(
            self.reduced_configuration,
            [self.joint_task],
            dt=self.dt,
            solver=self.solver,
            limits=self.limits_reduced,
            barriers=[self.collision_barrier_reduced],
            safety_break=False
        )
        # convert reduced vel to full vel
        vel_full = np.zeros(self.robot_model.model.nv)
        vel_full[self.robot_model.reduced_mask] = vel
        return vel_full
