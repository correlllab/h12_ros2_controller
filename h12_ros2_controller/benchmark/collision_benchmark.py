import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.benchmark.collision_models import (
    add_model_flag,
    get_collision_models,
    model_output_path,
    resolve_model,
)
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    FILTERED_GENERATED_GRASPS_Q_PATH,
    URDF_MAGPIE_PATH,
)

_RESULT_FIELDS = (
    'state_name',
    'state_source',
    'source_index',
    'representation',
    'check_type',
    'repeat',
    'collision_free',
    'elapsed_seconds',
)


def _load_goals(path, nq):
    goals = np.load(path, allow_pickle=False)
    if goals.ndim != 2 or goals.shape[1] != nq:
        raise ValueError(f'Filtered goals must have shape (N, {nq})')
    if not np.all(np.isfinite(goals)):
        raise ValueError('Filtered goals must contain only finite values')
    return np.asarray(goals, dtype=float)


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


def _prepare_states(input_path, indices, count, start):
    '''Prepare identical reduced joint states before collision timing'''
    goals = _load_goals(input_path, len(start))
    selected = _select_indices(len(goals), indices, count)
    states = [('home', 'named_config', None, start.copy())]
    for index in selected:
        states.append((
            f'grasp_{index}',
            'filtered_goal',
            index,
            goals[index].copy(),
        ))
    return states


def _build_collision_model(start, urdf_path, srdf_path):
    '''Build one collision model and return setup metadata'''
    started = time.perf_counter()
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model._q[robot_model.reduced_mask] = start
    robot_model.update_kinematics()
    robot_model.init_collision_model(urdf_path, srdf_path)
    return robot_model, {
        'setup_time': time.perf_counter() - started,
        'geometry_count': len(
            robot_model.collision_model_body_reduced.geometryObjects
        ),
        'collision_pair_count': len(
            robot_model.collision_model_body_reduced.collisionPairs
        ),
    }


def _global_warmup(robot_model, start):
    '''Trigger lazy one-time initialisation outside measured samples'''
    robot_model.check_collision_free_reduced(start)


def _per_state_result(state_name, state_source, source_index, representation,
                      check_type, repeat, collision_free, elapsed):
    return {
        'state_name': state_name,
        'state_source': state_source,
        'source_index': '' if source_index is None else source_index,
        'representation': representation,
        'check_type': check_type,
        'repeat': repeat,
        'collision_free': bool(collision_free),
        'elapsed_seconds': elapsed,
    }


def _measure_states(states, start, representations, steady_repeats):
    rows = []
    setup = {}
    for representation, (urdf_path, srdf_path) in representations.items():
        robot_model, setup[representation] = _build_collision_model(
            start,
            urdf_path,
            srdf_path,
        )
        _global_warmup(robot_model, start)
        for state_name, state_source, source_index, q in states:
            # measure the very first collision check on this configuration
            started = time.perf_counter()
            collision_free = robot_model.check_collision_free_reduced(q)
            elapsed = time.perf_counter() - started
            rows.append(_per_state_result(
                state_name, state_source, source_index,
                representation, 'cold', 0,
                collision_free, elapsed,
            ))

            # measure steady-state repeats on the same configuration
            for repeat in range(steady_repeats):
                started = time.perf_counter()
                collision_free = robot_model.check_collision_free_reduced(q)
                elapsed = time.perf_counter() - started
                rows.append(_per_state_result(
                    state_name, state_source, source_index,
                    representation, 'steady', repeat,
                    collision_free, elapsed,
                ))
    return rows, setup


def _timing_summary(timings):
    '''Report mean, median, p95, and maximum for an array of timings'''
    if len(timings) == 0:
        return {
            'mean': 0.0,
            'median': 0.0,
            'p95': 0.0,
            'maximum': 0.0,
            'total': 0.0,
            'count': 0,
        }
    arr = np.asarray(timings, dtype=float)
    return {
        'mean': float(np.mean(arr)),
        'median': float(np.median(arr)),
        'p95': float(np.percentile(arr, 95)),
        'maximum': float(np.max(arr)),
        'total': float(np.sum(arr)),
        'count': len(arr),
    }


def _summarize(rows, setup, representations):
    summary = {
        'state_count': len({row['state_name'] for row in rows}),
        'representations': {},
    }
    for representation in representations:
        group = [
            row for row in rows
            if row['representation'] == representation
        ]
        cold = [
            row['elapsed_seconds']
            for row in group if row['check_type'] == 'cold'
        ]
        steady = [
            row['elapsed_seconds']
            for row in group if row['check_type'] == 'steady'
        ]
        summary['representations'][representation] = {
            **setup[representation],
            'cold_start': _timing_summary(cold),
            'steady_state': _timing_summary(steady),
            'collision_free_count': sum(
                row['collision_free'] for row in group
            ),
            'collision_count': sum(
                not row['collision_free'] for row in group
            ),
        }
    return summary


def _write_results(rows, summary, output):
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


def main(config_name, variant, frame_name, indices, count, start_config,
         steady_repeats, output):
    '''Generate a sphere-versus-mesh collision checking comparison'''
    if frame_name not in ('right_graspgenx_frame', 'left_graspgenx_frame'):
        raise ValueError(f'Unsupported GraspGenX frame: {frame_name}')
    if start_config not in NAMED_CONFIGS:
        raise ValueError(f'Unknown start configuration: {start_config}')
    if steady_repeats < 1:
        raise ValueError('Steady repeats must be at least one')

    load_controller_config(config_name)
    models = get_collision_models(variant)
    start = np.asarray(NAMED_CONFIGS[start_config], dtype=float)
    states = _prepare_states(
        FILTERED_GENERATED_GRASPS_Q_PATH,
        indices,
        count,
        start,
    )
    rows, setup = _measure_states(states, start, models, steady_repeats)
    summary = _summarize(rows, setup, models)
    summary.update({
        'config': str(config_name),
        'model_variant': variant,
        'input_path': FILTERED_GENERATED_GRASPS_Q_PATH,
        'frame_name': frame_name,
        'start_config': start_config,
        'steady_repeats': steady_repeats,
        'warmup': 'global only (no per-state warmup)',
        'measurement': 'cold-start matches planner per-state cost',
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
    add_model_flag(parser)
    parser.add_argument('--config', default='debug.yaml')
    parser.add_argument('--frame', default='right_graspgenx_frame')
    parser.add_argument('--indices', type=int, nargs='+')
    parser.add_argument('--count', type=int)
    parser.add_argument('--start-config', choices=sorted(NAMED_CONFIGS),
                        default='home')
    parser.add_argument('--steady-repeats', type=int, default=4)
    parser.add_argument('--output')
    args = parser.parse_args()
    variant = resolve_model(args)
    output = args.output or model_output_path(
        'data/planner/collision_benchmark.csv',
        variant,
    )
    main(
        config_name=args.config,
        variant=variant,
        frame_name=args.frame,
        indices=args.indices,
        count=args.count,
        start_config=args.start_config,
        steady_repeats=args.steady_repeats,
        output=output,
    )
