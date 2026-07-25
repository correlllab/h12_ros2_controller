import argparse
import csv
import json
import os
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlannerConfig,
    ReducedJointPlanner,
)
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    FILTERED_GENERATED_GRASPS_PATH,
    SRDF_MAGPIE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
    SRDF_HANDLESS_COLLISION_PATH,
    SRDF_HANDLESS_SPHERE_COLLISION_PATH,
    URDF_HANDLESS_PATH,
    URDF_HANDLESS_SPHERE_PATH,
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)


_FRAME_NAMES = (
    # keep frame selection explicit so left and right runs are comparable
    'right_graspgenx_frame',
    'left_graspgenx_frame',
)
_PLAN_RESULT_FIELDS = (
    'source_index',
    'collision_representation',
    'repeat',
    'success',
    'reason',
    'planning_time',
    'validity_checks',
    'waypoint_count',
)
# use the same Magpie kinematics with different collision geometries
_COLLISION_REPRESENTATIONS = {
    'sphere': (URDF_HANDLESS_SPHERE_PATH, SRDF_HANDLESS_SPHERE_COLLISION_PATH),
    'mesh': (URDF_HANDLESS_PATH, SRDF_HANDLESS_COLLISION_PATH),
}


def validate_transform(transform, tolerance=1e-5):
    '''Validate one homogeneous rigid transform'''
    transform = np.asarray(transform, dtype=float)
    # reject malformed or non-finite pose data early
    if transform.shape != (4, 4):
        return False, 'transform must have shape (4, 4)'
    if not np.all(np.isfinite(transform)):
        return False, 'transform must contain only finite values'
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=tolerance):
        return False, 'transform must have homogeneous final row'

    rotation = transform[:3, :3]
    # reject matrices that are not proper rigid-body rotations
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        atol=tolerance,
    ):
        return False, 'transform rotation must be orthonormal'
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=tolerance):
        return False, 'transform rotation must have determinant one'
    return True, ''


def load_grasps(path):
    '''Load generated grasp transforms from an npy file'''
    # keep object arrays disabled because targets must be numeric matrices
    grasps = np.load(path, allow_pickle=False)
    if grasps.ndim != 3 or grasps.shape[1:] != (4, 4):
        raise ValueError('Generated grasps must have shape (N, 4, 4)')
    return np.asarray(grasps, dtype=float)


def select_indices(total, indices=None, count=None):
    '''Select source transform indices for one run'''
    # default to the complete input set unless the CLI narrows it
    if indices is None:
        selected = list(range(total))
    else:
        selected = list(indices)
    if len(set(selected)) != len(selected):
        raise ValueError('Target indices must not contain duplicates')
    if any(index < 0 or index >= total for index in selected):
        raise ValueError(f'Target indices must be between 0 and {total - 1}')
    if count is not None:
        if count < 0:
            raise ValueError('Target count must not be negative')
        selected = selected[:count]
    return selected


def build_ik_solver(start, d_min):
    '''Build the offline reduced model and sphere-collision IK solver'''
    # use sphere collision for consistent IK goal preparation
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model._q[robot_model.reduced_mask] = start
    robot_model.update_kinematics()
    robot_model.init_collision_model(
        URDF_MAGPIE_SPHERE_PATH,
        SRDF_MAGPIE_SPHERE_PATH,
    )
    ik_solver = IKSolver(robot_model, d_min=d_min)
    return robot_model, ik_solver


def _planner_config(config, frame_name):
    '''Build planner settings from the existing YAML planner section'''
    planner = config['planner']
    defaults = PlannerConfig()
    return PlannerConfig(
        planner=planner.get('planner', defaults.planner),
        timeout=planner.get('timeout', defaults.timeout),
        range=planner.get('range', defaults.range),
        try_direct=planner.get('try_direct', defaults.try_direct),
        simplify=planner.get('simplify', defaults.simplify),
        simplify_time=planner.get('simplify_time', defaults.simplify_time),
        shortcut=planner.get('shortcut', defaults.shortcut),
        moving_speed=planner.get('moving_speed', defaults.moving_speed),
        min_interpolation_steps=planner.get(
            'min_interpolation_steps', defaults.min_interpolation_steps
        ),
        max_interpolation_steps=planner.get(
            'max_interpolation_steps', defaults.max_interpolation_steps
        ),
        validity_resolution=planner.get(
            'validity_resolution', defaults.validity_resolution
        ),
        constraint_check_steps=planner.get(
            'constraint_check_steps', defaults.constraint_check_steps
        ),
        frame_names=(frame_name,),
        frame_z_min=planner.get('frame_z_min', defaults.frame_z_min),
        frame_z_min_margin=planner.get(
            'frame_z_min_margin', defaults.frame_z_min_margin
        ),
        frame_z_corridor_margin=planner.get(
            'frame_z_corridor_margin', defaults.frame_z_corridor_margin
        ),
        enforce_workspace_constraints=defaults.enforce_workspace_constraints,
        # keep every benchmark free of point-cloud obstacles
        point_cloud_path='',
        point_cloud_margin=planner.get(
            'point_cloud_margin', defaults.point_cloud_margin
        ),
    )


def _prepare_planning_goals(grasps, frame_name, start, d_min, ik_alpha,
                            ik_timeout, linear_threshold, angular_threshold):
    '''Solve filtered pose targets into reduced joint-space goals'''
    robot_model, ik_solver = build_ik_solver(start, d_min)
    task_name = 'grasp_target'
    ik_solver.add_frame_task(task_name, frame_name)
    goals = []
    skipped = Counter()
    # re-solve filtered targets to obtain reduced joint-space goals
    for index, transform in enumerate(grasps):
        valid, reason = validate_transform(transform)
        if not valid:
            skipped[reason] += 1
            continue

        ik_solver.frame_tasks[task_name].transform_target_to_world = pin.SE3(
            transform,
        )
        try:
            # prepare planning input outside the plan timer
            result = ik_solver.solve_ik_reduced(
                alpha=ik_alpha,
                timeout=ik_timeout,
                linear_threshold=linear_threshold,
                angular_threshold=angular_threshold,
                initial_positions=[('start', start)],
            )
        except Exception as exc:
            skipped[type(exc).__name__] += 1
            continue

        goal = result.q[robot_model.reduced_mask]
        # retain only goals that pass the same raw validity checks as planning
        if not result.success:
            skipped['ik residual threshold was not reached'] += 1
            continue
        if not robot_model.check_within_limits_reduced(goal):
            skipped['goal is outside reduced joint limits'] += 1
            continue
        if not robot_model.check_collision_free_reduced(goal):
            skipped['goal is in sphere self-collision'] += 1
            continue
        goals.append((index, goal))
    return goals, dict(skipped)


def _build_planner(start, representation, config, frame_name):
    '''Build one offline planner with the selected self-collision geometry'''
    # swap only the collision URDF/SRDF pair between benchmark cases
    urdf_path, srdf_path = _COLLISION_REPRESENTATIONS[representation]
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model._q[robot_model.reduced_mask] = start
    robot_model.update_kinematics()
    robot_model.init_collision_model(urdf_path, srdf_path)
    # use the same YAML planner settings as the full pipeline benchmark
    return ReducedJointPlanner(
        robot_model,
        config=_planner_config(config, frame_name),
    )


def _warm_planner(planner, start, active_mask):
    '''Initialize OMPL structures outside measured target runs'''
    # use a trivial raw query to initialize OMPL without path refinement
    planner.plan(start, start, active_mask=active_mask, raw_ompl_only=True)


def _joint_path_length(path):
    # sum Euclidean distance between consecutive reduced configurations
    if len(path) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def _plan_row(index, representation, repeat, result):
    # combine planner status with path metadata for one CSV row
    metadata = result.metadata
    return {
        'source_index': int(index),
        'collision_representation': representation,
        'repeat': int(repeat),
        'success': bool(result.success),
        'reason': result.reason,
        'planning_time': float(result.metadata.get('ompl_solve_time', 0.0)),
        'validity_checks': int(metadata.get('validity_checks', 0)),
        'waypoint_count': int(result.path.shape[0]),
    }


def _plan_summary(rows):
    '''Summarize planner results by collision representation'''
    representations = {}
    for representation in _COLLISION_REPRESENTATIONS:
        # compare each collision representation over the same attempts
        group = [
            row for row in rows
            if row['collision_representation'] == representation
        ]
        if not group:
            continue
        successful = [row for row in group if row['success']]
        planning_time = np.asarray(
            [row['planning_time'] for row in group],
            dtype=float,
        )
        representations[representation] = {
            'attempts': len(group),
            'success_count': len(successful),
            'success_rate': len(successful) / len(group),
            'mean_planning_time': float(np.mean(planning_time)),
            'median_planning_time': float(np.median(planning_time)),
            'p95_planning_time': float(np.percentile(planning_time, 95)),
            'failure_reasons': dict(Counter(
                row['reason'] for row in group if not row['success']
            )),
        }
    return representations


def _write_plan_results(rows, summary, output):
    '''Write planner comparison rows and summary'''
    output_path = Path(output)
    # keep machine-readable rows and aggregate statistics beside each other
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=_PLAN_RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    summary_path = output_path.with_suffix('.json')
    with summary_path.open('w', encoding='utf-8') as file:
        json.dump(summary, file, indent=2, sort_keys=True)
        file.write('\n')
    return summary_path


def run_planner_benchmark(config_name, frame_name, indices, count,
                          start_config, ik_alpha, ik_timeout, repeats, output):
    '''Compare self-collision representations using filtered grasp targets'''
    if frame_name not in _FRAME_NAMES:
        raise ValueError(f'Unsupported GraspGenX frame: {frame_name}')
    if start_config not in NAMED_CONFIGS:
        raise ValueError(f'Unknown start configuration: {start_config}')
    if repeats < 1:
        raise ValueError('Repeats must be at least one')

    config = load_controller_config(config_name)
    controller = config['controller']
    grasps = load_grasps(FILTERED_GENERATED_GRASPS_PATH)
    selected = select_indices(len(grasps), indices=indices, count=count)
    start = np.asarray(NAMED_CONFIGS[start_config], dtype=float)
    selected_grasps = grasps[selected]
    # solve all goals once before comparing planner representations
    goals, skipped = _prepare_planning_goals(
        selected_grasps,
        frame_name,
        start,
        float(controller['d_min']),
        ik_alpha,
        ik_timeout,
        float(controller['threshold_linear']),
        float(controller['threshold_angular']),
    )
    active_mask = np.zeros(len(start), dtype=bool)
    # the default benchmark moves only the right arm
    active_mask[7:] = True
    rows = []
    for representation in _COLLISION_REPRESENTATIONS:
        # construct a fresh planner so collision state does not leak between cases
        planner = _build_planner(
            start,
            representation,
            config,
            frame_name,
        )
        _warm_planner(planner, start, active_mask)
        for repeat in range(repeats):
            for filtered_index, goal in goals:
                # stop after OMPL returns a raw collision-valid solution
                result = planner.plan(
                    start,
                    goal,
                    active_mask=active_mask,
                    raw_ompl_only=True,
                )
                rows.append(_plan_row(
                    selected[filtered_index],
                    representation,
                    repeat,
                    result,
                ))

    summary = {
        # record skipped goal preparation separately from planner failures
        'config': str(config_name),
        'input_path': FILTERED_GENERATED_GRASPS_PATH,
        'frame_name': frame_name,
        'start_config': start_config,
        'selected_targets': len(selected),
        'prepared_goals': len(goals),
        'preparation_skips': skipped,
        'planner_timeout': config['planner'].get('timeout'),
        'measurement': 'raw OMPL solve time only',
        'warmup_queries_per_representation': 1,
        'repeats': int(repeats),
        'representations': _plan_summary(rows),
    }
    summary_path = _write_plan_results(rows, summary, output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f'Wrote results to {output}')
    print(f'Wrote summary to {summary_path}')
    return rows, summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run the filtered-target planner benchmark.',
    )
    parser.add_argument('--config', default='debug.yaml')
    parser.add_argument('--frame', choices=_FRAME_NAMES,
                        default='right_graspgenx_frame')
    parser.add_argument('--indices', type=int, nargs='+')
    parser.add_argument('--count', type=int)
    parser.add_argument('--start-config', choices=sorted(NAMED_CONFIGS),
                        default='home')
    parser.add_argument('--ik-alpha', type=float, default=0.1)
    parser.add_argument('--ik-timeout', type=float)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--output', default='data/planner/planner_benchmark.csv')
    args = parser.parse_args()
    run_planner_benchmark(
        config_name=args.config,
        frame_name=args.frame,
        indices=args.indices,
        count=args.count,
        start_config=args.start_config,
        ik_alpha=args.ik_alpha,
        ik_timeout=10.0 if args.ik_timeout is None else args.ik_timeout,
        repeats=args.repeats,
        output=args.output,
    )
