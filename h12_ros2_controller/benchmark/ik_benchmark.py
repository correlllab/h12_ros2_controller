import time
import numpy as np
import pinocchio as pin
import meshcat.geometry as g

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS

def solve_ik_reduced(ik_solver, task_name, pos, rpy):
    task = ik_solver.frame_tasks[task_name]
    task.transform_target_to_world.translation = pos
    task.transform_target_to_world.rotation = pin.rpy.rpyToMatrix(rpy)

    start_time = time.perf_counter()
    q = ik_solver.solve_ik_reduced()
    end_time = time.perf_counter()
    t = end_time - start_time

    return q, t

def main():
    robot_model = RobotModel('./assets/h1_2/h1_2.urdf')
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model.init_visualizer()

    ik_solver = IKSolver(
        robot_model,
        './assets/h1_2/h1_2_sphere.urdf',
        './assets/h1_2/h1_2_sphere_collision.srdf'
    )
    task_name = 'left_ee'
    ik_solver.add_frame_task(task_name, 'left_wrist_yaw_link')
    ik_solver.set_from_configuration()

    # Add a sphere marker for the IK target
    vis = robot_model.viz.viewer
    vis['target'].set_object(g.Sphere(0.02), g.MeshLambertMaterial(color=0xff0000))

    while True:
        pos = np.random.uniform([0.2, 0, 0.2], [0.5, 0.3, 0.4])
        rpy = np.zeros(3)

        # Update target marker position
        target_transform = np.eye(4)
        target_transform[:3, 3] = pos
        vis['target'].set_transform(target_transform)

        q, t = solve_ik_reduced(ik_solver, task_name, pos, rpy)
        print(f'IK solved in {t}s')

        robot_model._q = q
        robot_model.update_kinematics()
        robot_model.update_visualizer()

if __name__ == '__main__':
    main()
