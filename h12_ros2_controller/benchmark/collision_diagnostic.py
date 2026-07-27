import os
import sys
import time

import numpy as np
import pinocchio as pin

sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.benchmark.collision_benchmark import (
    _build_collision_model,
    _prepare_states,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    FILTERED_GENERATED_GRASPS_Q_PATH,
)


def _grasp_states(count):
    start = np.asarray(NAMED_CONFIGS['home'], dtype=float)
    states = _prepare_states(
        FILTERED_GENERATED_GRASPS_Q_PATH,
        None,
        count,
        start,
    )
    return states, {}


def _per_state_measure(representation, states, steady_repeats=5):
    robot_model, _ = _build_collision_model(
        np.asarray(NAMED_CONFIGS['home'], dtype=float),
        representation,
    )
    # one global warmup
    robot_model.check_collision_free_reduced(states[0][3])

    all_queries = []
    for state_name, state_source, source_index, q in states:
        cold = time.perf_counter()
        robot_model.check_collision_free_reduced(q)
        cold = time.perf_counter() - cold

        steady = []
        for _ in range(steady_repeats):
            started = time.perf_counter()
            robot_model.check_collision_free_reduced(q)
            steady.append(time.perf_counter() - started)

        all_queries.append({
            'name': state_name,
            'source': state_source,
            'index': source_index,
            'representation': representation,
            'cold': cold,
            'steady': sorted(steady),
        })
    return robot_model, all_queries


def _correlation_analysis(all_queries):
    cold_times = np.array([q['cold'] for q in all_queries])
    steady_means = np.array([np.mean(q['steady']) for q in all_queries])
    steady_medians = np.array([np.median(q['steady']) for q in all_queries])

    print(f'\n{"="*60}')
    print('COLD VS STEADY CORRELATION (Pearson r)')
    print(f'{"="*60}')
    r_mean = np.corrcoef(cold_times, steady_means)[0, 1]
    r_med = np.corrcoef(cold_times, steady_medians)[0, 1]
    print(f'  cold vs steady_mean:  r = {r_mean:.4f}')
    print(f'  cold vs steady_median: r = {r_med:.4f}')
    print()
    if r_mean > 0.95:
        print('  INTERPRETATION: highly deterministic')
        print('  Collision cost is configuration-dependent, not runtime jitter.')
        print('  The same state produces the same cost every query.')
    elif r_mean > 0.7:
        print('  INTERPRETATION: mostly deterministic with some jitter')
    else:
        print('  INTERPRETATION: low correlation, likely nondeterministic factors')
        print('  (allocation, GC, scheduler, BVH rebuild, etc.)')

    return r_mean


def _find_slow_states(all_queries, top_n=5):
    by_cold = sorted(all_queries, key=lambda x: x['cold'], reverse=True)
    by_steady = sorted(all_queries, key=lambda x: np.mean(x['steady']), reverse=True)

    print(f'\n{"="*60}')
    print('SLOWEST STATES BY COLD QUERY')
    print(f'{"="*60}')
    for q in by_cold[:top_n]:
        print(f'  {q["name"]:20s} cold={q["cold"]:.4f}s  steady_mean={np.mean(q["steady"]):.4f}s  '
              f'steady_med={np.median(q["steady"]):.4f}s  steady_range={np.min(q["steady"]):.2e}-{np.max(q["steady"]):.2e}')

    print(f'\n{"="*60}')
    print('SLOWEST STATES BY STEADY MEAN')
    print(f'{"="*60}')
    for q in by_steady[:top_n]:
        print(f'  {q["name"]:20s} cold={q["cold"]:.4f}s  steady_mean={np.mean(q["steady"]):.4f}s  '
              f'steady_med={np.median(q["steady"]):.4f}s  steady_range={np.min(q["steady"]):.2e}-{np.max(q["steady"]):.2e}')

    return by_cold[:top_n], by_steady[:top_n]


def _profile_collision_pairs(robot_model, q, label, top_n=10):
    model = robot_model.model_body_reduced
    data = robot_model.data_body_reduced
    cm = robot_model.collision_model_body_reduced
    cd = robot_model.collision_data_body_reduced

    # ensure geometry placements are updated for this configuration
    pin.forwardKinematics(model, data, q)
    pin.updateGeometryPlacements(model, data, cm, cd)

    pairs = list(enumerate(cm.collisionPairs))
    pair_times = []
    for idx, pair in pairs:
        g1 = cm.geometryObjects[pair.first]
        g2 = cm.geometryObjects[pair.second]
        started = time.perf_counter()
        pin.computeCollision(cm, cd, idx)
        elapsed = time.perf_counter() - started
        result = cd.collisionResults[idx]
        pair_times.append({
            'idx': idx,
            'name1': g1.name,
            'name2': g2.name,
            'geom1_type': str(g1.geometry.__class__.__name__),
            'geom2_type': str(g2.geometry.__class__.__name__),
            'time': elapsed,
            'colliding': result.isCollision(),
            'contacts': result.numContacts(),
        })

    # sort by time descending
    sorted_pairs = sorted(pair_times, key=lambda x: x['time'], reverse=True)

    print(f'\n{"="*60}')
    print(f'TOP {top_n} COLLISION PAIRS FOR {label}')
    print(f'{"="*60}')
    total = sum(p['time'] for p in pair_times)
    for p in sorted_pairs[:top_n]:
        pct = p['time'] / total * 100
        print(f'  {p["idx"]:3d} {p["time"]:.6f}s ({pct:5.1f}%)  '
              f'{p["geom1_type"]:20s} {p["geom2_type"]:20s}  '
              f'{p["name1"]:40s} <-> {p["name2"]}')
    print(f'  {"...":>3s}')
    print(f'  Total time for all {len(pair_times)} pairs: {total:.6f}s')
    print(f'  Top-{top_n} share of total: {sum(p["time"] for p in sorted_pairs[:top_n]) / total * 100:.1f}%')
    print(f'  Colliding pairs: {sum(1 for p in pair_times if p["colliding"])}')

    # geometry type breakdown
    geom_types = {}
    for p in pair_times:
        key = f'{p["geom1_type"]}+{p["geom2_type"]}'
        if key not in geom_types:
            geom_types[key] = {'count': 0, 'time': 0.0}
        geom_types[key]['count'] += 1
        geom_types[key]['time'] += p['time']

    print(f'\n{"="*60}')
    print('GEOMETRY TYPE BREAKDOWN')
    print(f'{"="*60}')
    for key, info in sorted(geom_types.items(), key=lambda x: x[1]['time'], reverse=True):
        pct = info['time'] / total * 100
        print(f'  {info["count"]:3d} pairs  {info["time"]:.4f}s ({pct:5.1f}%)  {key}')

    return sorted_pairs


def _full_diagnostic(count=100, steady_repeats=5):
    print('Preparing states...')
    states, skipped = _grasp_states(count)
    print(f'  {len(states)} states prepared, {sum(skipped.values())} skipped')

    print('\nProfiling mesh representation...')
    robot_model, mesh_queries = _per_state_measure('mesh', states[:count], steady_repeats)

    r = _correlation_analysis(mesh_queries)

    slow_cold, _slow_steady = _find_slow_states(mesh_queries, top_n=5)

    # pick the slowest state by cold time for pair profiling
    slowest = slow_cold[0]
    slow_q = None
    for state_name, state_source, source_index, q in states[:count]:
        if f'grasp_{source_index}' == slowest['name']:
            slow_q = q
            break
    if slow_q is None:
        for state_name, state_source, source_index, q in states[:count]:
            if state_name == slowest['name']:
                slow_q = q
                break

    # also pick a fast state for comparison
    fastest_idx = np.argmin([q['cold'] for q in mesh_queries])
    fastest = mesh_queries[fastest_idx]
    fast_q = None
    for state_name, state_source, source_index, q in states[:count]:
        if state_name == fastest['name']:
            fast_q = q
            break

    # profile individual collision pairs for the slowest and fastest states
    if slow_q is not None and fast_q is not None:
        _profile_collision_pairs(robot_model, slow_q, f'SLOW STATE: {slowest["name"]}')
        _profile_collision_pairs(robot_model, fast_q, f'FAST STATE: {fastest["name"]}')

    return r, mesh_queries


if __name__ == '__main__':
    r, queries = _full_diagnostic(count=30, steady_repeats=10)
