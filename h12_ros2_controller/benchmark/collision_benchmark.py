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
from h12_ros2_controller.core.ik_solver import IKSolver
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    FILTERED_GENERATED_GRASPS_PATH,
    SRDF_MAGPIE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)


_REPRESENTATIONS = {
    'sphere': (URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH),
    'mesh': (URDF_MAGPIE_PATH, SRDF_MAGPIE_PATH),
}
_RESULT_FIELDS = (
    'state_name',
    'state_source',
    'source_index',
    'representation',
    'repeat',
    'collision_free',
    'elapsed_seconds',
)


def _load_grasps(path):
    grasps = np.load(path, allow_pickle=False)
    if grasps.ndim != 3 or grasps.shape[1:] != (4, 4):
        raise ValueError('Generated grasps must have shape (N, 4, 4)')
    return np.asarray(grasps, dtype=float)


def _select_indices(total, indices, count):
    selected = list(range(total)) if indices is None else list(indices)
    if len(set(selected)) != len(selected):
        raise ValueError('Target indices must not contain duplicates')
    if any(index < 0 or index >= total for index in selected):
        raise ValueError(f'Target indices must be between 0 and {total - 1}')
    if count is not None:
        if count < 0:
            raise ValueError('Target count must not be negative')
        selected = selected[:count]
    return selected


def _build_ik_solver(start, d_min):
    '''Build the sphere-collision solver used to prepare shared states'''
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model._q[robot_model.reduced_mask] = start
    robot_model.update_kinematics()
    robot_model.init_collision_model(
        URDF_MAGPIE_SPHERE_PATH,
        SRDF_MAGPIE_SPHERE_PATH,
    )
    return robot_model, IKSolver(robot_model, d_min=d_min)


def _prepare_states(input_path, frame_name, indices, count, start, d_min,
                    timeout, linear_threshold, angular_threshold):
    '''Prepare identical reduced joint states before collision timing'''
    grasps = _load_grasps(input_path)
    selected = _select_indices(len(grasps), indices, count)
    robot_model, ik_solver = _build_ik_solver(start, d_min)
    task_name = 'collision_benchmark_target'
    ik_solver.add_frame_task(task_name, frame_name)
    states = [('home', 'named_config', None, start.copy())]
    skipped = Counter()

    for index in selected:
        transform = grasps[index]
        if not np.all(np.isfinite(transform)):
            skipped['non-finite transform'] += 1
            continue
        ik_solver.frame_tasks[task_name].transform_target_to_world = pin.SE3(
            transform,
        )
        try:
            result = ik_solver.solve_ik_reduced(
                timeout=timeout,
                linear_threshold=linear_threshold,
                angular_threshold=angular_threshold,
                initial_positions=[('start', start)],
            )
        except Exception as exc:
            skipped[type(exc).__name__] += 1
            continue
        if not result.success:
            skipped['ik residual threshold was not reached'] += 1
            continue
        state = result.q[robot_model.reduced_mask].copy()
        states.append((
            f'grasp_{index}',
            'filtered_grasp',
            index,
            state,
        ))
    return states, skipped


def _build_collision_model(start, representation):
    '''Build one collision model and return setup metadata'''
    urdf_path, srdf_path = _REPRESENTATIONS[representation]
    started = time.perf_counter()
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model._q[robot_model.reduced_mask] = start
    robot_model.update_kinematics()
    robot_model.init_collision_model(urdf_path, srdf_path)
    return robot_model, {
        'setup_time': time.perf_counter() - started,
        'geometry_count': len(robot_model.collision_model_body_reduced.geometryObjects),
        'collision_pair_count': len(robot_model.collision_model_body_reduced.collisionPairs),
    }


def _measure_states(states, start, repeats, warmup):
    rows = []
    setup = {}
    for representation in _REPRESENTATIONS:
        robot_model, setup[representation] = _build_collision_model(
            start,
            representation,
        )
        for state_name, state_source, source_index, q in states:
            # warm up bindings and allocation outside measured samples
            for _ in range(warmup):
                robot_model.check_collision_free_reduced(q)
            for repeat in range(repeats):
                started = time.perf_counter()
                collision_free = robot_model.check_collision_free_reduced(q)
                elapsed = time.perf_counter() - started
                rows.append({
                    'state_name': state_name,
                    'state_source': state_source,
                    'source_index': '' if source_index is None else source_index,
                    'representation': representation,
                    'repeat': repeat,
                    'collision_free': bool(collision_free),
                    'elapsed_seconds': elapsed,
                })
    return rows, setup


def _summarize(rows, setup, skipped):
    summary = {
        'state_count': len({row['state_name'] for row in rows}),
        'skipped_targets': dict(skipped),
        'representations': {},
    }
    for representation in _REPRESENTATIONS:
        group = [
            row for row in rows
            if row['representation'] == representation
        ]
        timings = np.asarray(
            [row['elapsed_seconds'] for row in group],
            dtype=float,
        )
        summary['representations'][representation] = {
            **setup[representation],
            'checks': len(group),
            'collision_free_count': sum(
                row['collision_free'] for row in group
            ),
            'collision_count': sum(
                not row['collision_free'] for row in group
            ),
            'mean_check_time': float(np.mean(timings)),
            'median_check_time': float(np.median(timings)),
            'p95_check_time': float(np.percentile(timings, 95)),
            'maximum_check_time': float(np.max(timings)),
            'total_check_time': float(np.sum(timings)),
        }
    return summary


def _write_results(rows, summary, output):
    output_path = Path(output)
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


def main(config_name, frame_name, indices, count, start_config, ik_timeout,
         repeats, warmup, output):
    '''Generate a sphere-versus-mesh collision checking comparison'''
    if frame_name not in ('right_graspgenx_frame', 'left_graspgenx_frame'):
        raise ValueError(f'Unsupported GraspGenX frame: {frame_name}')
    if start_config not in NAMED_CONFIGS:
        raise ValueError(f'Unknown start configuration: {start_config}')
    if repeats < 1 or warmup < 0:
        raise ValueError('Repeats must be positive and warmup nonnegative')

    config = load_controller_config(config_name)
    controller = config['controller']
    start = np.asarray(NAMED_CONFIGS[start_config], dtype=float)
    states, skipped = _prepare_states(
        FILTERED_GENERATED_GRASPS_PATH,
        frame_name,
        indices,
        count,
        start,
        float(controller['d_min']),
        ik_timeout,
        float(controller['threshold_linear']),
        float(controller['threshold_angular']),
    )
    rows, setup = _measure_states(states, start, repeats, warmup)
    summary = _summarize(rows, setup, skipped)
    summary.update({
        'config': str(config_name),
        'input_path': FILTERED_GENERATED_GRASPS_PATH,
        'frame_name': frame_name,
        'start_config': start_config,
        'ik_timeout': float(ik_timeout),
        'repeats': repeats,
        'warmup': warmup,
    })
    summary_path = _write_results(rows, summary, output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f'Wrote results to {output}')
    print(f'Wrote summary to {summary_path}')
    return rows, summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Compare sphere and mesh collision checking times.',
    )
    parser.add_argument('--config', default='debug.yaml')
    parser.add_argument('--frame', default='right_graspgenx_frame')
    parser.add_argument('--indices', type=int, nargs='+')
    parser.add_argument('--count', type=int)
    parser.add_argument('--start-config', choices=sorted(NAMED_CONFIGS),
                        default='home')
    parser.add_argument('--ik-timeout', type=float, default=10.0)
    parser.add_argument('--repeats', type=int, default=10)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument(
        '--output',
        default='data/planner/collision_benchmark.csv',
    )
    args = parser.parse_args()
    main(
        config_name=args.config,
        frame_name=args.frame,
        indices=args.indices,
        count=args.count,
        start_config=args.start_config,
        ik_timeout=args.ik_timeout,
        repeats=args.repeats,
        warmup=args.warmup,
        output=args.output,
    )
