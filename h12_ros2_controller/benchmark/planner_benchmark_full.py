import argparse
import csv
import json
import os
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.benchmark.planner_benchmark import (
    _COLLISION_REPRESENTATIONS,
    _FRAME_NAMES,
    _joint_path_length,
    _planner_config,
    _warm_planner,
    load_grasps,
    select_indices,
    validate_transform,
)
from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.planner.reduced_joint_planner import (
    ReducedJointPlanner,
)
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    FILTERED_GENERATED_GRASPS_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)


_RESULT_FIELDS = (
    # include separate IK, plan, and total pipeline timings
    'source_index',
    'collision_representation',
    'ik_collision_representation',
    'ik_success',
    'plan_success',
    'reason',
    'ik_time',
    'raw_plan_time',
    'postprocess_time',
    'plan_time',
    'pipeline_time',
    'ik_seed',
    'ik_iterations',
    'validity_checks',
    'waypoint_count',
    'joint_path_length',
    'frame_path_length',
)


def _active_task_mask(start, frame_name):
    '''Return the reduced joint mask for one GraspGenX frame task'''
    mask = np.zeros(len(start), dtype=bool)
    # pin the inactive arm so each target represents single-arm motion
    if frame_name == 'right_graspgenx_frame':
        mask[7:] = True
    else:
        mask[:7] = True
    return mask


def _build_pipeline(start, representation, frame_name, config):
    '''Build one shared-IK and representation-specific planning pipeline'''
    # solve IK with the sphere model so both planner cases share one goal
    ik_robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    ik_robot_model.init_reduced_model(ENABLED_JOINTS)
    ik_robot_model._q[ik_robot_model.reduced_mask] = start
    ik_robot_model.update_kinematics()
    ik_robot_model.init_collision_model(
        URDF_MAGPIE_SPHERE_PATH,
        SRDF_MAGPIE_SPHERE_PATH,
    )
    ik_solver = IKSolver(
        ik_robot_model,
        d_min=float(config['controller']['d_min']),
    )

    # use the selected collision model only for planning validity checks
    collision_urdf, collision_srdf = _COLLISION_REPRESENTATIONS[representation]
    planner_robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    planner_robot_model.init_reduced_model(ENABLED_JOINTS)
    planner_robot_model._q[planner_robot_model.reduced_mask] = start
    planner_robot_model.update_kinematics()
    planner_robot_model.init_collision_model(collision_urdf, collision_srdf)
    planner_config = _planner_config(config, frame_name)
    # force the RRT branch so full timing includes RRT and path refinement
    planner_config.try_direct = False
    planner = ReducedJointPlanner(
        planner_robot_model,
        config=planner_config,
    )
    return ik_robot_model, ik_solver, planner


def _attempt_for_result(result):
    # find the iteration count for the winning IK seed
    for attempt in result.attempts:
        if attempt.seed_name == result.seed_name:
            return attempt
    return None


def _empty_row(index, representation):
    # initialize a complete row so failure paths serialize consistently
    return {
        'source_index': int(index),
        'collision_representation': representation,
        'ik_collision_representation': 'sphere',
        'ik_success': False,
        'plan_success': False,
        'reason': '',
        'ik_time': 0.0,
        'raw_plan_time': 0.0,
        'postprocess_time': 0.0,
        'plan_time': 0.0,
        'pipeline_time': 0.0,
        'ik_seed': '',
        'ik_iterations': 0,
        'validity_checks': 0,
        'waypoint_count': 0,
        'joint_path_length': 0.0,
        'frame_path_length': 0.0,
    }


def _evaluate_target(robot_model, ik_solver, planner, task_name, index,
                     representation, frame_name, transform, start, ik_alpha,
                     ik_timeout, linear_threshold, angular_threshold):
    '''Run one controller-equivalent IK and unbounded planning pipeline'''
    row = _empty_row(index, representation)
    # validate source data before starting the timed pipeline
    valid, reason = validate_transform(transform)
    if not valid:
        row['reason'] = reason
        return row

    ik_solver.frame_tasks[task_name].transform_target_to_world = pin.SE3(
        transform,
    )
    # separate IK timing from the total controller-like pipeline timing
    pipeline_start = time.perf_counter()
    ik_start = time.perf_counter()
    try:
        ik_result = ik_solver.solve_ik_reduced(
            alpha=ik_alpha,
            timeout=ik_timeout,
            linear_threshold=linear_threshold,
            angular_threshold=angular_threshold,
            initial_positions=[('start', start)],
        )
    except Exception as exc:
        # record solver failures without preventing the other representation
        row['reason'] = f'IK exception: {type(exc).__name__}: {exc}'
        row['ik_time'] = time.perf_counter() - ik_start
        row['pipeline_time'] = time.perf_counter() - pipeline_start
        return row

    row['ik_time'] = time.perf_counter() - ik_start
    row['ik_success'] = bool(ik_result.success)
    row['ik_seed'] = ik_result.seed_name
    attempt = _attempt_for_result(ik_result)
    if attempt is not None:
        row['ik_iterations'] = int(attempt.iterations)
    if not ik_result.success:
        row['reason'] = 'IK residual threshold was not reached'
        row['pipeline_time'] = time.perf_counter() - pipeline_start
        return row

    # use the sphere-derived goal for both collision representations
    goal = ik_result.q[robot_model.reduced_mask]
    active_mask = _active_task_mask(start, frame_name)
    plan_start = time.perf_counter()
    # plan and measure the representation-specific validity cost
    result = planner.plan(start, goal, active_mask=active_mask)
    row['plan_time'] = time.perf_counter() - plan_start
    row['pipeline_time'] = time.perf_counter() - pipeline_start
    row['plan_success'] = bool(result.success)
    row['reason'] = result.reason
    row['raw_plan_time'] = float(
        result.metadata.get('ompl_solve_time', 0.0)
    )
    row['postprocess_time'] = float(
        result.metadata.get('postprocess_time', 0.0)
    )
    row['validity_checks'] = int(result.metadata.get('validity_checks', 0))
    row['waypoint_count'] = int(result.path.shape[0])
    row['joint_path_length'] = _joint_path_length(result.path)
    row['frame_path_length'] = float(
        result.metadata.get('frame_path_lengths', {}).get(frame_name, 0.0)
    )
    return row


def _summary(rows):
    '''Summarize full pipeline measurements by collision representation'''
    summary = {}
    for representation in _COLLISION_REPRESENTATIONS:
        # compare both total pipeline outcomes and successful plan timings
        group = [
            row for row in rows
            if row['collision_representation'] == representation
        ]
        if not group:
            continue
        ik_successes = [row for row in group if row['ik_success']]
        plan_successes = [row for row in group if row['plan_success']]
        summary[representation] = {
            'attempts': len(group),
            'ik_success_count': len(ik_successes),
            'plan_success_count': len(plan_successes),
            'mean_ik_time': float(np.mean([row['ik_time'] for row in group])),
            'mean_raw_plan_time': float(
                np.mean([row['raw_plan_time'] for row in ik_successes])
            ) if ik_successes else 0.0,
            'mean_postprocess_time': float(
                np.mean([row['postprocess_time'] for row in ik_successes])
            ) if ik_successes else 0.0,
            'mean_plan_time': float(
                np.mean([row['plan_time'] for row in ik_successes])
            ) if ik_successes else 0.0,
            'mean_pipeline_time': float(
                np.mean([row['pipeline_time'] for row in group])
            ),
            'failure_reasons': dict(Counter(
                row['reason'] for row in group if not row['plan_success']
            )),
        }
    return summary


def _write_results(rows, summary, output):
    '''Write full pipeline results and summary files'''
    output_path = Path(output)
    # write one row per representation and target
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=_RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    summary_path = output_path.with_suffix('.json')
    with summary_path.open('w', encoding='utf-8') as file:
        json.dump(summary, file, indent=2, sort_keys=True)
        file.write('\n')
    return summary_path


def main(config_name, frame_name, indices, count, start_config, ik_alpha,
         ik_timeout, output):
    '''Run controller-equivalent full pipeline benchmarks'''
    if frame_name not in _FRAME_NAMES:
        raise ValueError(f'Unsupported GraspGenX frame: {frame_name}')
    if start_config not in NAMED_CONFIGS:
        raise ValueError(f'Unknown start configuration: {start_config}')

    config = load_controller_config(config_name)
    controller = config['controller']
    # use identical targets and start state for every representation
    grasps = load_grasps(FILTERED_GENERATED_GRASPS_PATH)
    selected = select_indices(len(grasps), indices=indices, count=count)
    start = np.asarray(NAMED_CONFIGS[start_config], dtype=float)
    rows = []
    for representation in _COLLISION_REPRESENTATIONS:
        # rebuild collision-dependent planner state for each comparison case
        robot_model, ik_solver, planner = _build_pipeline(
            start,
            representation,
            frame_name,
            config,
        )
        task_name = 'grasp_target'
        ik_solver.add_frame_task(task_name, frame_name)
        _warm_planner(
            planner,
            start,
            _active_task_mask(start, frame_name),
        )
        for index in selected:
            # each row executes IK followed by planning like the controller
            rows.append(_evaluate_target(
                robot_model,
                ik_solver,
                planner,
                task_name,
                index,
                representation,
                frame_name,
                grasps[index],
                start,
                ik_alpha,
                ik_timeout,
                float(controller['threshold_linear']),
                float(controller['threshold_angular']),
            ))

    summary = {
        'config': str(config_name),
        'input_path': FILTERED_GENERATED_GRASPS_PATH,
        'frame_name': frame_name,
        'start_config': start_config,
        'selected_targets': len(selected),
        'ik_timeout': float(ik_timeout),
        'planner_timeout': config['planner'].get('timeout'),
        'planner_mode': 'RRT with full path processing',
        'warmup_queries_per_representation': 1,
        'ik_collision_representation': 'sphere',
        'representations': _summary(rows),
    }
    summary_path = _write_results(rows, summary, output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f'Wrote results to {output}')
    print(f'Wrote summary to {summary_path}')
    return rows, summary


if __name__ == '__main__':
    # keep this script usable as a standalone offline benchmark
    parser = argparse.ArgumentParser(
        description='Run controller-equivalent planner benchmarks.',
    )
    parser.add_argument('--config', default='debug.yaml')
    parser.add_argument('--frame', choices=_FRAME_NAMES,
                        default='right_graspgenx_frame')
    parser.add_argument('--indices', type=int, nargs='+')
    parser.add_argument('--count', type=int)
    parser.add_argument('--start-config', choices=sorted(NAMED_CONFIGS),
                        default='home')
    parser.add_argument('--ik-alpha', type=float, default=0.1)
    parser.add_argument('--ik-timeout', type=float, default=10.0)
    parser.add_argument(
        '--output',
        default='data/planner/planner_benchmark_full.csv',
    )
    args = parser.parse_args()
    main(
        config_name=args.config,
        frame_name=args.frame,
        indices=args.indices,
        count=args.count,
        start_config=args.start_config,
        ik_alpha=args.ik_alpha,
        ik_timeout=args.ik_timeout,
        output=args.output,
    )
