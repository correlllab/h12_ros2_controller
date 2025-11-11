import numpy as np
import pinocchio as pin
from h12_ros2_controller.core.upper_controller import UpperController

class FrameController(UpperController):
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 dt=0.02, v_lim=1.0, w_lim=2.0, dq_lim=2.0, d_min=0.02,
                 visualize=False,
                 sport_mode=False):
        super().__init__(urdf_path, urdf_sphere_path, srdf_sphere_path,
                         dt, v_lim, w_lim, dq_lim, d_min,
                         visualize, sport_mode)

    def add_frame_task(self, task_name: str, frame_name: str, target: np.ndarray = None):
        '''Add a frame task with optional pose'''
        self.ik_solver.add_frame_task(task_name, frame_name)
        if target is not None:
            if target.shape == (6,):
                self.set_frame_task_pose(task_name, target)
            elif target.shape == (4, 4):
                self.set_frame_task_transformation(task_name, target)
            else:
                raise ValueError('Target must be either a 6D pose or a 4x4 transformation matrix')

    def set_frame_task_pose(self, task_name: str, pose: np.ndarray):
        '''Set pose for a frame task'''
        assert len(pose) == 6, 'Pose must be [x, y, z, roll, pitch, yaw]'
        task = self.ik_solver.frame_tasks.get(task_name)
        if task is None:
            raise ValueError(f'Task {task_name} not found')
        position = np.array(pose[:3])
        rpy = np.array(pose[3:])
        rotation = pin.rpy.rpyToMatrix(rpy)
        transform = pin.SE3(rotation, position)
        task.transform_target_to_world = transform

    def set_frame_task_transformation(self, task_name: str, transformation: np.ndarray):
        '''Set transformation for a frame task'''
        assert transformation.shape == (4, 4), 'Transformation must be a 4x4 matrix'
        task = self.ik_solver.frame_tasks.get(task_name)
        if task is None:
            raise ValueError(f'Task {task_name} not found')
        transform = pin.SE3(transformation)
        task.transform_target_to_world = transform

    def get_frame_task_error(self, task_name: str):
        '''Get error for a frame task'''
        task = self.ik_solver.frame_tasks.get(task_name)
        if task is None:
            raise ValueError(f'Task {task_name} not found')
        return task.compute_error(self.ik_solver.configuration_reduced)

    def remove_frame_task(self, task_name: str):
        '''Remove a frame task'''
        self.ik_solver.remove_frame_task(task_name)

    def clear_frame_tasks(self):
        '''Clear all frame tasks'''
        self.ik_solver.clear_frame_tasks()

    def control_step(self):
        '''Solve IK for all tasks'''
        # solve IK and apply the control
        vel = self.ik_solver.ik_step()
        vel = self.limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self.apply_joint_position(self.ik_solver.q)
        self.update_robot_model()

    def control_step_reduced(self):
        '''Solve IK for all tasks with the reduced model'''
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced()
        vel = self.limit_joint_vel(vel)
        # integrate IK solver and command the joint position
        self.ik_solver.integrate(vel)
        self.apply_joint_position(self.ik_solver.q)
        self.update_robot_model()

    def sim_step(self):
        # solve IK and apply the control
        vel = self.ik_solver.ik_step()
        vel = self.limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.ik_solver.q
        self.update_robot_model()

    def sim_step_reduced(self):
        # solve IK and apply the control
        vel = self.ik_solver.ik_step_reduced()
        vel = self.limit_joint_vel(vel)
        # integrate IK solver
        self.ik_solver.integrate(vel)
        # force robot model to use local variable tracking states
        self.robot_model.state_subscriber = None
        self.robot_model._q = self.ik_solver.q
        self.update_robot_model()

    def get_frame_pose(self, frame_name: str):
        '''Get current pose of a frame'''
        position = self.robot_model.get_frame_position(frame_name)
        rotation = self.robot_model.get_frame_rotation(frame_name)
        rpy = pin.rpy.matrixToRpy(rotation)
        return np.concatenate([position, rpy])

    def get_frame_transformation(self, frame_name: str):
        '''Get current transformation matrix of a frame'''
        return self.robot_model.get_frame_transformation(frame_name)
