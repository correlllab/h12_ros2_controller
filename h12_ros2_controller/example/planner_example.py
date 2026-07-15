import argparse
import time
import numpy as np

import os
import sys
from pathlib import Path
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlannerConfig,
    ReducedJointPlanner,
)
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.path_definition import (
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
)


def build_robot_model(visualize=False):
    '''Build an offline robot model for reduced arm planning'''
    for path in [
        URDF_MAGPIE_PATH,
        URDF_MAGPIE_SPHERE_PATH,
        SRDF_MAGPIE_SPHERE_PATH,
    ]:
        if not Path(path).exists():
            raise FileNotFoundError(f'missing robot asset: {path}')

    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model.init_collision_model(
        URDF_MAGPIE_SPHERE_PATH,
        SRDF_MAGPIE_SPHERE_PATH,
    )
    if visualize:
        robot_model.init_visualizer()
    return robot_model


def inspect_path(robot_model, path, point_cloud_checker=None):
    '''Print validity for every waypoint'''
    all_valid = True
    for idx, q_reduced in enumerate(path):
        # print joint-limit and collision results separately for debugging
        start_time = time.perf_counter()
        within_limits = robot_model.check_within_limits_reduced(q_reduced)
        collision_free = robot_model.check_collision_free_reduced(q_reduced)
        cloud_free = True
        if point_cloud_checker is not None:
            cloud_free = point_cloud_checker.check_collision_free_reduced(q_reduced)
        valid = within_limits and collision_free and cloud_free
        check_time = time.perf_counter() - start_time
        all_valid = all_valid and valid

        print(
            f'Waypoint {idx:03d}: '
            f'limits={within_limits}, '
            f'collision_free={collision_free}, '
            f'cloud_free={cloud_free}, '
            f'valid={valid}, '
            f'check_time={check_time:.4f} s'
        )
    return all_valid


def parse_args():
    '''Parse command line arguments'''
    parser = argparse.ArgumentParser(
        description='Plan an offline OMPL path in reduced arm joint space.',
    )
    parser.add_argument(
        '--goal-config',
        default='arms_front_45',
        choices=sorted(NAMED_CONFIGS),
        help='Named reduced configuration to plan to.',
    )
    parser.add_argument(
        '--start-config',
        choices=sorted(NAMED_CONFIGS),
        help=(
            'Optional named start configuration; defaults to current model '
            'state.'
        ),
    )
    parser.add_argument('--planner', default='RRTConnect')
    parser.add_argument('--timeout', type=float, default=1.0)
    parser.add_argument('--moving-speed', type=float, default=0.25)
    parser.add_argument('--dt', type=float, default=1.0 / 30.0)
    parser.add_argument('--min-interpolation-steps', type=int, default=2)
    parser.add_argument('--max-interpolation-steps', type=int, default=300)
    parser.add_argument('--validity-resolution', type=float, default=0.0025)
    parser.add_argument('--constraint-check-steps', type=int, default=10)
    parser.add_argument('--frame-z-min-margin', type=float, default=1e-3)
    parser.add_argument('--frame-z-corridor-margin', type=float, default=0.05)
    parser.add_argument(
        '--config',
        default='debug.yaml',
        help=(
            'YAML under config/ that supplies the obstacle point cloud; use '
            'pcd_debug.yaml to plan against the example cloud.'
        ),
    )
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--visualize-delay', type=float, default=0.05)
    return parser.parse_args()


def main():
    args = parse_args()
    robot_model = build_robot_model(visualize=args.visualize)
    start = robot_model.state_reduced['q']
    if args.start_config is not None:
        start = NAMED_CONFIGS[args.start_config]
    goal = NAMED_CONFIGS[args.goal_config]

    # the fixed obstacle point cloud is sourced from the yaml, not the cli
    planner_cfg = load_controller_config(args.config)['planner']
    point_cloud_path = planner_cfg.get('point_cloud_path')
    config = PlannerConfig(
        planner=args.planner,
        timeout=args.timeout,
        moving_speed=args.moving_speed,
        dt=args.dt,
        min_interpolation_steps=args.min_interpolation_steps,
        max_interpolation_steps=args.max_interpolation_steps,
        validity_resolution=args.validity_resolution,
        constraint_check_steps=args.constraint_check_steps,
        frame_z_min_margin=args.frame_z_min_margin,
        frame_z_corridor_margin=args.frame_z_corridor_margin,
        point_cloud_path=point_cloud_path,
        point_cloud_margin=float(planner_cfg.get('point_cloud_margin', 0.02)),
    )
    # the planner preloads the cloud from config; draw the same cloud here
    planner = ReducedJointPlanner(robot_model, config=config)
    if point_cloud_path:
        points = np.load(point_cloud_path)
        print(f'Loaded {len(points)} obstacle points from {point_cloud_path}')
        if args.visualize:
            robot_model.visualizer.visualize_point_cloud(points)

    result = planner.plan(start, goal)

    print(f'Planner: {result.planner_name}')
    print(f'Success: {result.success}')
    print(f'Reason: {result.reason}')
    print(f'Planning time: {result.planning_time:.4f} s')
    print(f'Path shape: {result.path.shape}')
    if result.success:
        np.set_printoptions(precision=4, suppress=True)
        print(result.path)
        path_valid = inspect_path(
            robot_model,
            result.path,
            point_cloud_checker=planner.point_cloud_checker,
        )
        print(f'All waypoints valid: {path_valid}')
        if args.visualize:
            robot_model.visualizer.visualize_reduced_path(result.path)
            robot_model.visualizer.play_reduced_path(
                result.path,
                delay=args.visualize_delay,
            )


if __name__ == '__main__':
    main()
