import json
from dataclasses import replace

import numpy as np

from ompl import geometric as og

from h12_ros2_controller.benchmark.ik_benchmark import (
    COMPARISON_CASES, _asset_ids, _resolve_case, benchmark_corpus_path,
    compare_planners, main,
)
from h12_ros2_controller.core.planner.kinematic_pipeline import KinematicPipeline
from h12_ros2_controller.core.planner.reduced_joint_planner import ReducedJointPlanner
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS


def test_corpus_has_fixed_paired_and_failure_cases():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    cases = json.loads(benchmark_corpus_path().read_text())
    resolved = {case['id']: _resolve_case(case, pipeline) for case in cases}
    assert BODY_JOINTS[12] == 'torso_joint'
    assert resolved['torso_offset_left_pitch']['start_q'][12] == 0.03
    assert resolved['torso_offset_left_pitch']['start_q'][0] == 0.0
    assert _asset_ids('sim_safety_split.yaml') == _asset_ids('config/sim_safety_split.yaml')
    assert all(case['config'] == 'sim_safety_split.yaml' for case in cases)
    assert len(resolved) == len(cases)
    assert len(cases) >= 20
    assert {'direct', 'reverse', 'near_wrist_limit', 'blocked_direct', 'infeasible'} <= {
        case['category'] for case in cases
    }
    assert {'reverse_local', 'orientation_edge', 'frozen_joint',
            'dual_arm_named', 'nearby_goal'} <= {
        case['category'] for case in cases
    }
    assert {'dual_frame', 'dual_frame_stress'} <= {
        case['category'] for case in cases
    }
    forward = resolved['home_left_pitch']
    reverse = resolved['reverse_left_pitch']
    assert np.any(np.asarray(forward['start_q']) != np.asarray(reverse['start_q']))
    assert np.allclose(forward['start_q'], resolved['unreachable_left']['start_q'])
    assert forward['pose'] != reverse['pose']
    assert all(len(case['start_q']) == pipeline.robot_model._q.size for case in resolved.values())
    assert all(len(case['pose']) == 6 for case in resolved.values() if 'frame' in case)
    assert all(len(task['pose']) == 6 for case in resolved.values()
               for task in case.get('tasks', []))


def test_benchmark_outputs_case_attempts_and_phase_timings(tmp_path):
    corpus = json.loads(benchmark_corpus_path().read_text())
    selected = [next(case for case in corpus if case['id'] == name)
                for name in ('home_left_pitch', 'cross_staging_blocked',
                             'unreachable_left')]
    cases = tmp_path / 'cases.json'
    output = tmp_path / 'results.json'
    cases.write_text(json.dumps(selected))

    assert main(cases_path=cases, repeats=2, seed=42, output=output) == 2
    payload = json.loads(output.read_text())
    assert payload['schema_version'] == 1
    assert payload['mode'] == 'offline'
    assert payload['reuse_policy'] == 'isolated'
    assert payload['trial_independence'] == 'cache-disabled shared RNG stream'
    assert payload['effective_planner_config']['local_joint_padding'] == 0.6
    assert 'ompl' in payload['dependency_versions']
    assert not any(
        (record['plan_metadata'] or {}).get('cache_hit', False)
        for record in payload['results']
    )
    assert isinstance(payload['seeded_ompl'], bool)
    assert payload['seed'] == 42
    assert len(payload['results']) == 6
    assert payload['assets']['config/sim_safety_split.yaml']
    direct, blocked, unreachable = payload['results'][:3]
    assert direct['success'] and direct['ik_success']
    assert direct['quality']['wrist_margin'] is not None
    assert direct['candidate_planning_attempts'][0]['metadata']['stage_timings']['total'] >= 0
    assert direct['candidate_route_attempts'][0]['seed']
    assert direct['phase_timings']['ik'] >= 0
    assert blocked['category'] == 'blocked_direct' and not blocked['success']
    assert 'self-collision' in blocked['reason']
    assert blocked['ik_success'] is None
    assert blocked['candidate_planning_attempts'][0]['direct_only']
    assert not unreachable['ik_success'] and not unreachable['success']
    assert unreachable['ik_attempts']
    for record in payload['results']:
        assert record['timings']['total'] >= record['timings']['pipeline'] >= 0
        assert record['timings']['ik_attempts'] >= 0
        assert record['timings']['planner_calls'] >= 0


def test_worker_corpus_seeding_and_named_return_locality(tmp_path):
    cases = json.loads(benchmark_corpus_path().read_text())
    selected = [case for case in cases if case['id'] == 'left_pitch_80_return']
    path = tmp_path / 'reverse.json'
    output = tmp_path / 'worker.json'
    path.write_text(json.dumps(selected))

    successes = main(cases_path=path, mode='worker', seed=42, output=output)

    payload = json.loads(output.read_text())
    result = payload['results'][0]
    assert successes == int(result['success'])
    assert payload['seeded_ompl'] and payload['ompl_seed'] == 42
    if result['success']:
        assert result['plan_metadata']['joint_path_ratio'] <= 2.0
        assert result['plan_metadata']['local_x_bounds']['left_grasp_frame'] > 0.0
    else:
        assert 'exceeds max_interpolation_steps' in result['reason']
    assert result['candidate_planning_attempts'][0]['metadata']['client_timings']['total'] > 0


def test_sequential_benchmark_prefers_direct_return_over_cached_route(tmp_path):
    cases = [
        {'id': 'home_front', 'start_name': 'home',
         'plan_goal_name': 'arms_front_45', 'config': 'sim_safety_split.yaml'},
        {'id': 'front_home', 'start_name': 'arms_front_45',
         'plan_goal_name': 'home', 'config': 'sim_safety_split.yaml'},
    ]
    path = tmp_path / 'sequential.json'
    output = tmp_path / 'sequential_results.json'
    path.write_text(json.dumps(cases))

    assert main(cases_path=path, output=output, reuse_policy='sequential') == 2

    payload = json.loads(output.read_text())
    assert payload['reuse_policy'] == 'sequential'
    assert payload['results'][1]['reason'] == 'direct path'
    assert not payload['results'][1]['plan_metadata'].get('cache_hit', False)


def test_comparison_planners_and_path_length_objective():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    for name, planner_type in (('RRTConnect', og.RRTConnect),
                               ('BITstar', og.BITstar),
                               ('InformedRRTstar', og.InformedRRTstar)):
        planner = ReducedJointPlanner(
            pipeline.robot_model, replace(pipeline.planner.config, planner=name),
        )
        assert isinstance(planner.ss.getPlanner(), planner_type)
        objective = planner.ss.getProblemDefinition().getOptimizationObjective()
        if name == 'RRTConnect':
            assert objective is None
        else:
            assert objective.getDescription() == 'Path Length'


def test_comparison_uses_five_fixed_cases_and_reports_metrics(tmp_path):
    output = tmp_path / 'comparison.json'
    summary = compare_planners(output=output)
    payload = json.loads(output.read_text())
    assert tuple(payload['case_ids']) == COMPARISON_CASES
    assert payload['seed'] == 42 and payload['reuse_policy'] == 'isolated'
    assert set(summary) == {'RRTConnect', 'BITstar', 'InformedRRTstar'}
    for row in summary.values():
        assert row['attempts'] == len(COMPARISON_CASES)
        assert row['frame_cases'] + row['named_cases'] == row['attempts']
        assert row['ompl_attempts'] >= row['ompl_cases']
        if row['ompl_search_p95_s'] is not None:
            assert row['ompl_search_p95_s'] >= row['ompl_search_p50_s'] >= 0
        assert 0 <= row['success_rate'] <= 1
        assert row['planner_calls_p95_s'] >= row['planner_calls_p50_s'] >= 0
        assert all(record['plan_metadata'] is None or
                   not record['plan_metadata'].get('cache_hit') for record in row['results'])
