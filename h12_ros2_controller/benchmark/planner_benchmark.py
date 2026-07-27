import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path

import numpy as np
from ompl import util as ou

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.benchmark.collision_models import (
    add_model_flag,
    get_collision_models,
    model_output_path,
    resolve_model,
)
from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlannerConfig,
    ReducedJointPlanner,
)
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    FILTERED_GENERATED_GRASPS_Q_PATH,
    URDF_MAGPIE_PATH,
)

_FRAME_NAMES = (
    'right_graspgenx_frame',
    'left_graspgenx_frame',
)
_PLAN_RESULT_FIELDS = (
    'source_index',
    'collision_representation',
    'repeat',
    'seed',
    'success',
    'reason',
    'planning_time',
    'validity_checks',
    'waypoint_count',
)


def validate_transform(transform, tolerance=1e-5):
    '''Validate one homogeneous rigid transform'''
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        return False, 'transform must have shape (4, 4)'
    if not np.all(np.isfinite(transform)):
        return False, 'transform must contain only finite values'
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=tolerance):
        return False, 'transform must have homogeneous final row'
    rotation = transform[:3, :3]
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
    grasps = np.load(path, allow_pickle=False)
    if grasps.ndim != 3 or grasps.shape[1:] != (4, 4):
        raise ValueError('Generated grasps must have shape (N, 4, 4)')
    return np.asarray(grasps, dtype=float)


def select_indices(total, indices=None, count=None):
    '''Select source transform indices for one run'''
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


def load_filtered_goals(path, nq):
    '''Load one cached reduced joint goal for every filtered grasp target'''
    goals = np.load(path, allow_pickle=False)
    if goals.ndim != 2 or goals.shape[1] != nq:
        raise ValueError(f'Filtered goals must have shape (N, {nq})')
    if not np.all(np.isfinite(goals)):
        raise ValueError('Filtered goals must contain only finite values')
    return np.asarray(goals, dtype=float)


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
        point_cloud_path='',
        point_cloud_margin=planner.get(
            'point_cloud_margin', defaults.point_cloud_margin
        ),
    )


def _build_planner(start, urdf_path, srdf_path, config, frame_name):
    '''Build one offline planner with the selected self-collision geometry'''
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model._q[robot_model.reduced_mask] = start
    robot_model.update_kinematics()
    robot_model.init_collision_model(urdf_path, srdf_path)
    return ReducedJointPlanner(
        robot_model,
        config=_planner_config(config, frame_name),
    )


def _warm_planner(planner, start, active_mask):
    '''Initialize OMPL structures outside measured target runs'''
    planner.plan(start, start, active_mask=active_mask, raw_ompl_only=True)


def _joint_path_length(path):
    if len(path) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def _plan_row(index, representation, repeat, seed, result):
    metadata = result.metadata
    return {
        'source_index': int(index),
        'collision_representation': representation,
        'repeat': int(repeat),
        'seed': int(seed),
        'success': bool(result.success),
        'reason': result.reason,
        'planning_time': float(result.metadata.get('ompl_solve_time', 0.0)),
        'validity_checks': int(metadata.get('validity_checks', 0)),
        'waypoint_count': int(result.path.shape[0]),
    }


def _plan_summary(rows, representations):
    '''Summarize planner results by collision representation'''
    result_set = {}
    for representation in representations:
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
        result_set[representation] = {
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
    return result_set


def _write_plan_results(rows, summary, output):
    '''Write planner comparison rows and summary'''
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(
            file,
            fieldnames=_PLAN_RESULT_FIELDS,
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
                        representations, representation, repeat, seed):
    '''Run one representation in a process with a fresh OMPL RNG state'''
    ou.RNG.setSeed(int(seed))
    if frame_name not in _FRAME_NAMES:
        raise ValueError(f'Unsupported GraspGenX frame: {frame_name}')
    if start_config not in NAMED_CONFIGS:
        raise ValueError(f'Unknown start configuration: {start_config}')
    config = load_controller_config(config_name)
    urdf_path, srdf_path = representations[representation]
    goals = load_filtered_goals(
        FILTERED_GENERATED_GRASPS_Q_PATH,
        len(NAMED_CONFIGS[start_config]),
    )
    selected = select_indices(len(goals), indices=indices, count=count)
    start = np.asarray(NAMED_CONFIGS[start_config], dtype=float)
    active_mask = np.zeros(len(start), dtype=bool)
    active_mask[7:] = True
    planner = _build_planner(
        start,
        urdf_path,
        srdf_path,
        config,
        frame_name,
    )
    _warm_planner(planner, start, active_mask)
    rows = []
    for source_index in selected:
        result = planner.plan(
            start,
            goals[source_index],
            active_mask=active_mask,
            raw_ompl_only=True,
        )
        rows.append(_plan_row(
            source_index,
            representation,
            repeat,
            seed,
            result,
        ))
    return rows


def _run_representation_worker(config_name, frame_name, indices, count,
                               start_config, variant, representation,
                               repeat, seed):
    '''Run an isolated representation worker and return its result rows'''
    with tempfile.TemporaryDirectory(prefix='planner_benchmark_') as directory:
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
            '--repeat', str(repeat),
            '--seed', str(seed),
        ]
        if count is not None:
            command.extend(['--count', str(count)])
        if indices is not None:
            command.extend(['--indices', *(str(index) for index in indices)])
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        with open(output, encoding='utf-8') as file:
            return json.load(file)


def run_planner_benchmark(config_name, variant, frame_name, indices, count,
                          start_config, repeats, seed, output):
    '''Compare self-collision representations using filtered grasp targets'''
    if repeats < 1:
        raise ValueError('Repeats must be at least one')
    config = load_controller_config(config_name)
    models = get_collision_models(variant)
    goals = load_filtered_goals(
        FILTERED_GENERATED_GRASPS_Q_PATH,
        len(NAMED_CONFIGS[start_config]),
    )
    selected = select_indices(len(goals), indices=indices, count=count)
    rows = []
    for repeat in range(repeats):
        repeat_seed = seed + repeat
        for representation in models:
            rows.extend(_run_representation_worker(
                config_name,
                frame_name,
                selected,
                None,
                start_config,
                variant,
                representation,
                repeat,
                repeat_seed,
            ))

    summary = {
        'config': str(config_name),
        'model_variant': variant,
        'input_path': FILTERED_GENERATED_GRASPS_Q_PATH,
        'frame_name': frame_name,
        'start_config': start_config,
        'selected_targets': len(selected),
        'prepared_goals': len(selected),
        'planner_timeout': config['planner'].get('timeout'),
        'measurement': 'raw OMPL solve time only',
        'warmup_queries_per_representation': 1,
        'base_seed': int(seed),
        'seed_strategy': 'fresh worker seed = base + repeat',
        'repeats': int(repeats),
        'representations': _plan_summary(rows, models),
    }
    summary_path = _write_plan_results(rows, summary, output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f'Wrote results to {output}')
    print(f'Wrote summary to {summary_path}')
    return rows, summary


def _worker_main(args):
    representations = get_collision_models(resolve_model(args))
    rows = _run_representation(
        args.config,
        args.frame,
        args.indices,
        args.count,
        args.start_config,
        representations,
        args.worker_representation,
        args.repeat,
        args.seed,
    )
    with open(args.worker_output, 'w', encoding='utf-8') as file:
        json.dump(rows, file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run the filtered-target planner benchmark.',
    )
    add_model_flag(parser)
    parser.add_argument('--config', default='debug.yaml')
    parser.add_argument('--frame', choices=_FRAME_NAMES,
                        default='right_graspgenx_frame')
    parser.add_argument('--indices', type=int, nargs='+')
    parser.add_argument('--count', type=int)
    parser.add_argument('--start-config', choices=sorted(NAMED_CONFIGS),
                        default='home')
    parser.add_argument('--seed', type=int, default=20260725)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--repeat', type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument(
        '--worker-representation',
        help=argparse.SUPPRESS,
    )
    parser.add_argument('--worker-output', help=argparse.SUPPRESS)
    parser.add_argument('--output')
    args = parser.parse_args()
    variant = resolve_model(args)
    output = args.output or model_output_path(
        'data/planner/planner_benchmark.csv',
        variant,
    )
    if args.worker_representation:
        if not args.worker_output:
            parser.error('--worker-output is required for a worker')
        _worker_main(args)
    else:
        run_planner_benchmark(
            config_name=args.config,
            variant=variant,
            frame_name=args.frame,
            indices=args.indices,
            count=args.count,
            start_config=args.start_config,
            repeats=args.repeats,
            seed=args.seed,
            output=output,
        )
