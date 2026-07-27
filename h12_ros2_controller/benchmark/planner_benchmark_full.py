import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

import numpy as np
import pinocchio as pin
from ompl import util as ou

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.benchmark.collision_models import (
    add_model_flag,
    get_collision_models,
    model_output_path,
    resolve_model,
)
from h12_ros2_controller.benchmark.planner_benchmark import (
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
    URDF_MAGPIE_PATH,
)

_RESULT_FIELDS = (
    'source_index',
    'collision_representation',
    'seed',
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
    if frame_name == 'right_graspgenx_frame':
        mask[7:] = True
    else:
        mask[:7] = True
    return mask


def _build_pipeline(start, representation, frame_name, config, representations,
                    ik_sphere_urdf, ik_sphere_srdf):
    '''Build one shared-IK and representation-specific planning pipeline'''
    ik_robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    ik_robot_model.init_reduced_model(ENABLED_JOINTS)
    ik_robot_model._q[ik_robot_model.reduced_mask] = start
    ik_robot_model.update_kinematics()
    ik_robot_model.init_collision_model(ik_sphere_urdf, ik_sphere_srdf)
    ik_solver = IKSolver(
        ik_robot_model,
        d_min=float(config['controller']['d_min']),
    )

    collision_urdf, collision_srdf = representations[representation]
    planner_robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    planner_robot_model.init_reduced_model(ENABLED_JOINTS)
    planner_robot_model._q[planner_robot_model.reduced_mask] = start
    planner_robot_model.update_kinematics()
    planner_robot_model.init_collision_model(collision_urdf, collision_srdf)
    planner_config = _planner_config(config, frame_name)
    planner_config.try_direct = False
    planner = ReducedJointPlanner(
        planner_robot_model,
        config=planner_config,
    )
    return ik_robot_model, ik_solver, planner


def _attempt_for_result(result):
    for attempt in result.attempts:
        if attempt.seed_name == result.seed_name:
            return attempt
    return None


def _empty_row(index, representation, seed):
    return {
        'source_index': int(index),
        'collision_representation': representation,
        'seed': int(seed),
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
                     ik_timeout, linear_threshold, angular_threshold, seed):
    '''Run one controller-equivalent IK and unbounded planning pipeline'''
    row = _empty_row(index, representation, seed)
    valid, reason = validate_transform(transform)
    if not valid:
        row['reason'] = reason
        return row

    ik_solver.frame_tasks[task_name].transform_target_to_world = pin.SE3(
        transform,
    )
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
    except Exception as exc:  # noqa: BLE001
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

    goal = ik_result.q[robot_model.reduced_mask]
    active_mask = _active_task_mask(start, frame_name)
    plan_start = time.perf_counter()
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


def _summary(rows, representations):
    '''Summarize full pipeline measurements by collision representation'''
    summary = {}
    for representation in representations:
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
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(
            file,
            fieldnames=_RESULT_FIELDS,
            lineterminator='\n',
        )
        writer.writeheader()
        writer.writerows(rows)

    summary_path = output_path.with_suffix('.json')
    with summary_path.open('w', encoding='utf-8') as file:
        json.dump(summary, file, indent=2, sort_keys=True)
        file.write('\n')
    return summary_path


def _run_representation(config_name, frame_name, indices, count, start_config,
                        ik_alpha, ik_timeout, variant, representation, seed):
    '''Run one full representation pipeline with a fresh OMPL RNG state'''
    ou.RNG.setSeed(int(seed))
    if frame_name not in _FRAME_NAMES:
        raise ValueError(f'Unsupported GraspGenX frame: {frame_name}')
    if start_config not in NAMED_CONFIGS:
        raise ValueError(f'Unknown start configuration: {start_config}')

    config = load_controller_config(config_name)
    controller = config['controller']
    models = get_collision_models(variant)
    ik_sphere_urdf, ik_sphere_srdf = models['sphere']
    grasps = load_grasps(FILTERED_GENERATED_GRASPS_PATH)
    selected = select_indices(len(grasps), indices=indices, count=count)
    start = np.asarray(NAMED_CONFIGS[start_config], dtype=float)
    robot_model, ik_solver, planner = _build_pipeline(
        start,
        representation,
        frame_name,
        config,
        models,
        ik_sphere_urdf,
        ik_sphere_srdf,
    )
    task_name = 'grasp_target'
    ik_solver.add_frame_task(task_name, frame_name)
    _warm_planner(planner, start, _active_task_mask(start, frame_name))
    rows = []
    for index in selected:
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
            seed,
        ))
    return rows


def _run_representation_worker(config_name, frame_name, indices, count,
                               start_config, ik_alpha, ik_timeout,
                               variant, representation, seed):
    '''Run an isolated full-pipeline representation worker'''
    with tempfile.TemporaryDirectory(prefix='planner_benchmark_full_') as directory:
        output = os.path.join(directory, 'rows.json')
        command = [
            sys.executable,
            os.path.abspath(__file__),
            f'--{variant}',
            '--worker-representation', representation,
            '--worker-output', output,
            '--config', config_name,
            '--frame', frame_name,
            '--start-config', start_config,
            '--ik-alpha', str(ik_alpha),
            '--ik-timeout', str(ik_timeout),
            '--seed', str(seed),
        ]
        if count is not None:
            command.extend(['--count', str(count)])
        if indices is not None:
            command.extend(['--indices', *(str(index) for index in indices)])
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        with open(output, encoding='utf-8') as file:
            return json.load(file)


def main(config_name, variant, frame_name, indices, count, start_config,
         ik_alpha, ik_timeout, seed, output):
    '''Run controller-equivalent full pipeline benchmarks'''
    if frame_name not in _FRAME_NAMES:
        raise ValueError(f'Unsupported GraspGenX frame: {frame_name}')
    if start_config not in NAMED_CONFIGS:
        raise ValueError(f'Unknown start configuration: {start_config}')

    config = load_controller_config(config_name)
    models = get_collision_models(variant)
    grasps = load_grasps(FILTERED_GENERATED_GRASPS_PATH)
    selected = select_indices(len(grasps), indices=indices, count=count)
    rows = []
    for representation in models:
        rows.extend(_run_representation_worker(
            config_name,
            frame_name,
            selected,
            None,
            start_config,
            ik_alpha,
            ik_timeout,
            variant,
            representation,
            seed,
        ))

    summary = {
        'config': str(config_name),
        'model_variant': variant,
        'input_path': FILTERED_GENERATED_GRASPS_PATH,
        'frame_name': frame_name,
        'start_config': start_config,
        'selected_targets': len(selected),
        'ik_timeout': float(ik_timeout),
        'planner_timeout': config['planner'].get('timeout'),
        'planner_mode': 'RRT with full path processing',
        'warmup_queries_per_representation': 1,
        'base_seed': int(seed),
        'seed_strategy': 'fresh worker seed = base',
        'ik_collision_representation': f'{variant} sphere',
        'representations': _summary(rows, models),
    }
    summary_path = _write_results(rows, summary, output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f'Wrote results to {output}')
    print(f'Wrote summary to {summary_path}')
    return rows, summary


def _worker_main(args):
    rows = _run_representation(
        args.config,
        args.frame,
        args.indices,
        args.count,
        args.start_config,
        args.ik_alpha,
        args.ik_timeout,
        resolve_model(args),
        args.worker_representation,
        args.seed,
    )
    with open(args.worker_output, 'w', encoding='utf-8') as file:
        json.dump(rows, file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run controller-equivalent planner benchmarks.',
    )
    add_model_flag(parser)
    parser.add_argument('--config', default='debug.yaml')
    parser.add_argument('--frame', choices=_FRAME_NAMES,
                        default='right_graspgenx_frame')
    parser.add_argument('--indices', type=int, nargs='+')
    parser.add_argument('--count', type=int)
    parser.add_argument('--start-config', choices=sorted(NAMED_CONFIGS),
                        default='home')
    parser.add_argument('--ik-alpha', type=float, default=0.1)
    parser.add_argument('--ik-timeout', type=float, default=10.0)
    parser.add_argument('--seed', type=int, default=20260725)
    parser.add_argument(
        '--worker-representation',
        help=argparse.SUPPRESS,
    )
    parser.add_argument('--worker-output', help=argparse.SUPPRESS)
    parser.add_argument('--output')
    args = parser.parse_args()
    variant = resolve_model(args)
    output = args.output or model_output_path(
        'data/planner/planner_benchmark_full.csv',
        variant,
    )
    if args.worker_representation:
        if not args.worker_output:
            parser.error('--worker-output is required for a worker')
        _worker_main(args)
    else:
        main(
            config_name=args.config,
            variant=variant,
            frame_name=args.frame,
            indices=args.indices,
            count=args.count,
            start_config=args.start_config,
            ik_alpha=args.ik_alpha,
            ik_timeout=args.ik_timeout,
            seed=args.seed,
            output=output,
        )
