'''Finite headless benchmark of the offline IK-to-planner route'''

import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
import time
from dataclasses import asdict, replace
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.append(str(Path(__file__).resolve().parents[2]))

from h12_ros2_controller.core.planner.kinematic_pipeline import (
    KinematicPipeline, KinematicPlan, candidate_metrics, planner_config_from_controller,
)
from h12_ros2_controller.core.planner.planner_process import PlannerClient
from h12_ros2_controller.core.planner.reduced_joint_planner import moving_grasp_frames
from h12_ros2_controller.utility.controller_config import (
    _resolve_config_path, load_controller_config,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    PROJECT_ROOT, SRDF_MAGPIE_SPHERE_PATH, URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)


CORPUS = PROJECT_ROOT / 'config' / 'ik_benchmark_cases.json'
COMPARISON_CASES = (
    'left_pitch_80_local', 'right_pitch_80_local',
    'left_pitch_80_return', 'right_pitch_80_return',
    'left_pitch_75_local',
)
COMPARISON_PLANNERS = ('RRTConnect', 'BITstar', 'InformedRRTstar')


def compare_planners(cases_path=CORPUS, case_ids=COMPARISON_CASES, seed=42,
                      repeats=1, config_name='sim_safety_split.yaml', output=None):
    '''Compare independently seeded offline planners on identical saved cases'''
    if not case_ids or repeats < 1 or seed < 0:
        raise ValueError('Comparison needs cases, positive repeats, and nonnegative seed')
    raw = json.loads(Path(cases_path).read_text())
    if isinstance(raw, dict):
        raw = [raw]
    found = {case['id'] for case in raw}
    if len(set(case_ids)) != len(case_ids) or not set(case_ids) <= found:
        raise ValueError('Comparison case IDs must be distinct and present in corpus')
    summary = {}
    with tempfile.TemporaryDirectory() as temporary:
        for name in COMPARISON_PLANNERS:
            result_path = Path(temporary) / f'{name}.json'
            command = [
                sys.executable, '-m', 'h12_ros2_controller.benchmark.ik_benchmark',
                '--cases', str(cases_path), '--case-ids', ','.join(case_ids),
                '--planner', name, '--seed', str(seed), '--repeats', str(repeats),
                '--config', config_name, '--output', str(result_path),
                '--mode', 'offline', '--reuse-policy', 'isolated',
            ]
            subprocess.run(command, check=True, capture_output=True, text=True)
            payload = json.loads(result_path.read_text())
            if not payload['seeded_ompl']:
                raise RuntimeError('OMPL seed API unavailable; comparison is not reproducible')
            records = payload['results']
            successes = [record for record in records if record['success']]
            times = [record['timings']['planner_calls'] for record in records]
            detours = [record['plan_metadata']['joint_path_ratio'] for record in successes]
            deviations = [max(record['plan_metadata'].get(
                'hand_segment_deviation', {}).values(), default=0.0) for record in successes]
            ompl_attempts = [
                attempt for record in records
                for attempt in record['candidate_planning_attempts']
                if not attempt['direct_only']
            ]
            ompl_times = [
                attempt['metadata'].get('stage_timings', {}).get('ompl')
                for attempt in ompl_attempts
            ]
            ompl_times = [duration for duration in ompl_times if duration is not None]
            summary[name] = {
                'successes': len(successes), 'attempts': len(records),
                'success_rate': len(successes) / len(records),
                'planner_calls_p50_s': float(np.percentile(times, 50)),
                'planner_calls_p95_s': float(np.percentile(times, 95)),
                'joint_detour_median': float(np.median(detours)) if detours else None,
                'hand_deviation_max_m': max(deviations, default=None),
                'frame_cases': sum(record['ik_success'] is not None for record in records),
                'named_cases': sum(record['ik_success'] is None for record in records),
                'ompl_cases': sum(any(not attempt['direct_only'] for attempt in
                                      record['candidate_planning_attempts']) for record in records),
                'ompl_attempts': len(ompl_attempts),
                'ompl_search_p50_s': (float(np.percentile(ompl_times, 50))
                                      if ompl_times else None),
                'ompl_search_p95_s': (float(np.percentile(ompl_times, 95))
                                      if ompl_times else None),
                'results': records,
            }
            row = summary[name]
            print(f'{name}: {row["successes"]}/{row["attempts"]} '
                  f'({row["success_rate"]:.0%}), planner calls '
                  f'p50={row["planner_calls_p50_s"]:.3f}s '
                  f'p95={row["planner_calls_p95_s"]:.3f}s, '
                  f'joint detour median={row["joint_detour_median"]}, '
                  f'hand deviation max={row["hand_deviation_max_m"]}, '
                  f'OMPL cases={row["ompl_cases"]}, '
                  f'OMPL search p95={row["ompl_search_p95_s"]}')
    if output is not None:
        Path(output).write_text(json.dumps(_jsonable({
            'schema_version': 1, 'seed': seed, 'repeats': repeats,
            'case_ids': list(case_ids), 'corpus_sha256': hashlib.sha256(
                Path(cases_path).read_bytes()).hexdigest(),
            'config': config_name, 'mode': 'offline', 'reuse_policy': 'isolated',
            'trial_independence': 'cache-disabled shared RNG stream',
            'planners': summary,
        }), indent=2, allow_nan=False) + '\n')
    return summary


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _asset_ids(config_name):
    config_path = _resolve_config_path(config_name)
    paths = (config_path, Path(URDF_MAGPIE_PATH), Path(URDF_MAGPIE_SPHERE_PATH),
             Path(SRDF_MAGPIE_SPHERE_PATH))
    return {
        str(path.relative_to(PROJECT_ROOT)) if path.is_relative_to(PROJECT_ROOT)
        else str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in paths
    }


def _dependency_versions():
    '''Record installed solver versions alongside benchmark results'''
    versions = {}
    for name in ('ompl', 'pin', 'pin-pink', 'qpsolvers'):
        try:
            versions[name] = version(name)
        except PackageNotFoundError:
            versions[name] = None
    return versions


def _resolve_case(case, pipeline):
    case = dict(case)
    if _resolve_config_path(case.get('config', pipeline.config_name)) != _resolve_config_path(
        pipeline.config_name
    ):
        raise ValueError('Saved case config differs from the selected YAML profile')
    model = pipeline.robot_model
    if 'start_name' in case:
        q = np.zeros_like(model._q)
        q[model.reduced_mask] = NAMED_CONFIGS[case.pop('start_name')]
        case['start_q'] = q.tolist()
    if 'goal_name' in case:
        goal = NAMED_CONFIGS[case.pop('goal_name')]
        reduced = model.model_body_reduced
        data = reduced.createData()
        pin.forwardKinematics(reduced, data, goal)
        pin.updateFramePlacements(reduced, data)
        targets = case['tasks'] if 'tasks' in case else [case]
        for target in targets:
            frame_id = reduced.getFrameId(target['frame'])
            if frame_id >= len(reduced.frames):
                raise ValueError(f'Unknown reduced-model frame: {target["frame"]}')
            transform = data.oMf[frame_id]
            target['pose'] = np.r_[transform.translation,
                                   pin.rpy.matrixToRpy(transform.rotation)].tolist()
    case.setdefault('config', pipeline.config_name)
    return case


class _MeasuredPlanner:
    def __init__(self, pipeline, mode, seed=None):
        self.pipeline = pipeline
        self.offline = pipeline.planner
        self.frame_names = self.offline.frame_names
        self.attempts = []
        self.client = None
        self.setup_time = 0.0
        self.reuse_enabled = False
        if mode == 'worker':
            self.client = PlannerClient(
                URDF_MAGPIE_PATH, URDF_MAGPIE_SPHERE_PATH,
                SRDF_MAGPIE_SPHERE_PATH, self.offline.config,
                ompl_seed=seed,
                request_timeout=self.offline.config.worker_request_timeout,
                startup_timeout=self.offline.config.worker_startup_timeout,
            )
            started = time.perf_counter()
            self.client.warmup()
            self.setup_time = time.perf_counter() - started

    def plan(self, start, goal, active_mask=None, direct_only=False,
             local_frames=(), timeout=None, local_joint_padding=None,
             deadline=None):
        started = time.perf_counter()
        if self.client is None:
            result = self.offline.plan(
                start, goal, active_mask=active_mask, direct_only=direct_only,
                local_frames=local_frames, timeout=timeout,
                allow_reuse=self.reuse_enabled,
                local_joint_padding=local_joint_padding,
                deadline=deadline,
            )
        else:
            result = self.client.plan_configuration(
                self.pipeline.robot_model.state['q'], start, goal,
                active_mask=active_mask, direct_only=direct_only,
                local_frames=local_frames, timeout=timeout,
                allow_reuse=self.reuse_enabled,
                local_joint_padding=local_joint_padding,
                planning_budget=(None if deadline is None else
                                 max(0.01, deadline - time.perf_counter())),
            )
        self.attempts.append({
            'direct_only': direct_only, 'success': result.success,
            'reason': result.reason, 'wall_time': time.perf_counter() - started,
            'planning_time': result.planning_time, 'metadata': result.metadata,
            'goal_q': np.asarray(goal, dtype=float).tolist(),
        })
        return result

    def rebind_reduced_model(self):
        self.offline.rebind_reduced_model()

    def shutdown(self):
        if self.client is not None:
            self.client.shutdown()


def _record(index, repeat, case, result, start, pipeline, planner, total):
    ik = result.ik
    plan = result.plan
    selected = result.selected_candidate
    quality = None
    if selected is not None:
        quality = candidate_metrics(
            selected, start, pipeline.robot_model.model_body_reduced,
            pipeline.robot_model.reduced_mask,
        )
    attempts = [] if ik is None else [{
        'seed_name': attempt.seed_name, 'success': attempt.success,
        'reason': attempt.reason, 'elapsed_time': attempt.elapsed_time,
        'iterations': attempt.iterations, 'linear_error': attempt.linear_error,
        'angular_error': attempt.angular_error, 'score': attempt.score,
        'frame_errors': attempt.frame_errors,
    } for attempt in ik.attempts]
    planning_attempts = []
    for attempt in planner.attempts:
        attempt = dict(attempt)
        attempt['seed_name'] = None
        if ik is not None:
            attempt['seed_name'] = next((
                candidate.seed_name for candidate in ik.candidates
                if np.allclose(candidate.q[pipeline.robot_model.reduced_mask],
                               attempt['goal_q'], atol=1e-8, rtol=0)
            ), None)
        planning_attempts.append(attempt)
    return _jsonable({
        'index': index, 'repeat': repeat, 'id': case.get('id', str(index)),
        'category': case.get('category'), 'frame': case.get('frame'),
        'start_q': start, 'pose': case.get('pose'),
        'ik_success': ik.success if ik is not None else None,
        'success': bool(plan is not None and plan.success),
        'reason': plan.reason if plan is not None else
        result.fallback_reason or
        '; '.join(attempt['reason'] for attempt in attempts if attempt['reason']) or 'IK failed',
        'bimanual_fallback_reason': result.fallback_reason,
        'selected_seed': selected.seed_name if selected is not None else None,
        'ik_attempts': attempts, 'candidate_planning_attempts': planning_attempts,
        'candidate_route_attempts': getattr(result, 'candidate_attempts', []),
        'phase_timings': getattr(result, 'phase_timings', {}),
        'quality': quality, 'plan_metadata': plan.metadata if plan is not None else None,
        'timings': {
            'total': total, 'pipeline': result.elapsed_time,
            'setup_start': total - result.elapsed_time,
            'ik_attempts': sum(attempt['elapsed_time'] for attempt in attempts),
            'planner_calls': sum(attempt['wall_time'] for attempt in planning_attempts),
            'selected_planner': plan.planning_time if plan is not None else None,
        },
    })


def main(samples=3, ik_timeout=None, seed=0, visualize=False,
          config_name='sim_safety_split.yaml', start_name='arms_front_45', cases_path=None,
         repeats=1, output=None, mode='offline', reuse_policy='isolated',
         planner_name=None, case_ids=None):
    if repeats < 1 or seed < 0 or mode not in ('offline', 'worker'):
        raise ValueError('repeats must be positive, seed nonnegative, and mode valid')
    if reuse_policy not in ('isolated', 'sequential'):
        raise ValueError('reuse_policy must be isolated or sequential')
    seed_api = False
    if mode == 'offline':
        from ompl import util as ou
        if hasattr(ou.RNG, 'setSeed'):
            # OMPL's global seed is only effective before its first RNG is used
            ou.RNG.setSeed(max(1, seed))
            seed_api = ou.RNG.getSeed() == max(1, seed)
    setup_started = time.perf_counter()
    planner_config = None
    if planner_name is not None:
        planner_config = replace(
            planner_config_from_controller(load_controller_config(config_name)),
            planner=planner_name,
        )
    pipeline = KinematicPipeline(visualize=visualize, config_name=config_name,
                                 planner_config=planner_config)
    if ik_timeout is None:
        ik_timeout = pipeline.planner_config.ik_total_timeout
    pipeline_setup_time = time.perf_counter() - setup_started
    raw_cases = None
    if cases_path is not None:
        raw_cases = json.loads(Path(cases_path).read_text())
        if isinstance(raw_cases, dict):
            raw_cases = [raw_cases]
        if not isinstance(raw_cases, list) or not raw_cases:
            raise ValueError('cases must contain a saved case or nonempty list')
        if case_ids is not None:
            raw_cases = [case for case in raw_cases if case.get('id') in case_ids]
            if len(raw_cases) != len(case_ids):
                raise ValueError('Every selected case ID must occur exactly once')
        raw_cases = [_resolve_case(case, pipeline) for case in raw_cases]
    else:
        staging = pipeline.plan_named(start_name)
        if not staging.plan.success:
            raise RuntimeError(f'Cannot reach benchmark starting pose: {staging.plan.reason}')
        pipeline.replay(staging.plan.path)
    planner = _MeasuredPlanner(pipeline, mode, seed=max(1, seed))
    planner.reuse_enabled = reuse_policy == 'sequential'
    pipeline.planner = planner
    rng = np.random.default_rng(seed)
    records = []
    try:
        count = len(raw_cases) if raw_cases is not None else samples
        for repeat in range(repeats):
            for index in range(count):
                if raw_cases is not None:
                    case = raw_cases[index]
                    start = np.asarray(case['start_q'], dtype=float)
                    planner.attempts = []
                    started = time.perf_counter()
                    if 'plan_goal_name' in case:
                        pipeline.set_start(start)
                        goal = NAMED_CONFIGS[case['plan_goal_name']]
                        named_start = pipeline.robot_model.state_reduced['q']
                        local_frames = moving_grasp_frames(
                            named_start, goal, planner.frame_names,
                        )
                        plan = planner.plan(
                            named_start, goal,
                            direct_only=case.get('direct_only', False),
                            local_frames=local_frames, local_joint_padding=0.0,
                            deadline=started + pipeline.planner_config.named_planning_budget,
                        )
                        result = KinematicPlan(plan=plan, elapsed_time=time.perf_counter() - started)
                    else:
                        result = pipeline.plan_case(case)
                else:
                    transform = pipeline.robot_model.get_frame_transformation_reduced(
                        'left_grasp_frame', pipeline.robot_model.state_reduced['q'],
                    )
                    pose = np.r_[transform[:3, 3] + rng.uniform(-0.025, 0.025, 3),
                                 pin.rpy.matrixToRpy(transform[:3, :3])]
                    case = {'frame': 'left_grasp_frame', 'pose': pose.tolist()}
                    start = pipeline.robot_model.state['q'].copy()
                    planner.attempts = []
                    started = time.perf_counter()
                    result = pipeline.plan_frame(
                        case['frame'], case['pose'], ik_timeout=ik_timeout,
                    )
                record = _record(index, repeat, case, result, start, pipeline,
                                 planner, time.perf_counter() - started)
                records.append(record)
                print(f'{repeat + 1}.{index + 1} {record["id"]}: '
                      f'IK={record["ik_success"]}, plan={record["success"]}, '
                      f'elapsed={record["timings"]["total"]:.4f}s, '
                      f'reason={record["reason"]}')
                if record['success']:
                    pipeline.show_path(result.plan.path)
                    pipeline.replay(result.plan.path)
        successes = sum(record['success'] for record in records)
        print(f'Completed {len(records)} attempts, {successes} planned')
        if records:
            times = [record['timings']['total'] for record in records]
            print(f'End-to-end p50={np.percentile(times, 50):.3f}s, '
                  f'p95={np.percentile(times, 95):.3f}s, max={max(times):.3f}s '
                  '(excludes process setup)')
        if output is not None:
            revision = subprocess.run(
                ['git', 'rev-parse', 'HEAD'], cwd=PROJECT_ROOT,
                capture_output=True, text=True, check=False,
            )
            status = subprocess.run(
                ['git', 'status', '--porcelain'], cwd=PROJECT_ROOT,
                capture_output=True, text=True, check=False,
            )
            payload = {
                'schema_version': 1, 'mode': mode, 'seed': seed,
                'planner': pipeline.planner.offline.config.planner,
                'case_ids': list(case_ids) if case_ids is not None else None,
                'reuse_policy': reuse_policy,
                'trial_independence': ('cache-disabled shared RNG stream'
                                       if reuse_policy == 'isolated' else
                                       'sequential with path reuse'),
                'ompl_seed': (max(1, seed) if seed_api else
                              planner.client.ompl_seed_effective if mode == 'worker' else None),
                'seeded_ompl': (seed_api if mode == 'offline' else
                                planner.client.ompl_seed_effective == max(1, seed)),
                'repeats': repeats,
                'revision': revision.stdout.strip() if revision.returncode == 0 else None,
                'worktree_dirty': status.returncode != 0 or bool(status.stdout.strip()),
                'dependency_versions': _dependency_versions(),
                'effective_planner_config': _jsonable(asdict(pipeline.planner_config)),
                'config': config_name, 'assets': _asset_ids(config_name),
                'corpus_sha256': hashlib.sha256(Path(cases_path).read_bytes()).hexdigest()
                if cases_path is not None else None,
                'pipeline_setup_time': pipeline_setup_time,
                'worker_setup_time': planner.setup_time if mode == 'worker' else None,
                'results': records,
            }
            Path(output).write_text(json.dumps(payload, indent=2, allow_nan=False) + '\n')
        return successes
    finally:
        planner.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--config', default='sim_safety_split.yaml', help='Controller YAML profile')
    parser.add_argument('--start', default='arms_front_45',
                        help='Named starting configuration')
    parser.add_argument('--cases', help='Saved frame case JSON (single case or list)')
    parser.add_argument('--corpus', action='store_const', const=CORPUS, dest='cases',
                        help='Use the checked-in representative case corpus')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--output', help='Write per-case results as JSON')
    parser.add_argument('--mode', choices=('offline', 'worker'), default='offline')
    parser.add_argument('--reuse-policy', choices=('isolated', 'sequential'),
                        default='isolated',
                        help='isolated disables cache; trials still share a progressing RNG stream')
    parser.add_argument('--planner', choices=COMPARISON_PLANNERS,
                        help='Use this planner instead of the configured default')
    parser.add_argument('--planners', action='store_true',
                        help='Compare three planners in separately seeded offline runs')
    parser.add_argument('--case-ids', help='Comma-separated IDs from --cases')
    args = parser.parse_args()
    if args.planners:
        if args.planner or args.mode != 'offline' or args.reuse_policy != 'isolated' or args.visualize:
            parser.error('--planners requires offline, isolated, headless mode without --planner')
        compare_planners(args.cases or CORPUS,
                         tuple(args.case_ids.split(',')) if args.case_ids else COMPARISON_CASES,
                         args.seed, args.repeats, args.config, args.output)
    else:
        main(samples=args.samples, seed=args.seed, visualize=args.visualize,
             config_name=args.config, start_name=args.start, cases_path=args.cases,
             repeats=args.repeats, output=args.output, mode=args.mode,
             reuse_policy=args.reuse_policy, planner_name=args.planner,
             case_ids=tuple(args.case_ids.split(',')) if args.case_ids else None)
