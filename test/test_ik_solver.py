from types import SimpleNamespace
import json

import numpy as np
import pinocchio as pin
import pytest

from h12_ros2_controller.core.ik_solver import IKAttemptResult, IKSolver
from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.core.planner.kinematic_pipeline import (
    KinematicPipeline, plan_ik_candidates, planner_config_from_controller,
)
from h12_ros2_controller.core.planner.reduced_joint_planner import PlanResult
from h12_ros2_controller.benchmark.ik_benchmark import main as run_benchmark
from h12_ros2_controller.example import kinematic_plan_goto
from h12_ros2_controller.utility import named_config
from h12_ros2_controller.utility.controller_config import load_controller_config


class Configuration:
    def __init__(self, q):
        self.q = np.array(q, dtype=float)

    def update(self, q):
        self.q = np.array(q, dtype=float)

    def integrate_inplace(self, vel, dt):
        self.q += vel * dt


@pytest.fixture
def solver(monkeypatch):
    monkeypatch.setitem(named_config.NAMED_CONFIGS, 'home', np.zeros(4))
    monkeypatch.setitem(named_config.NAMED_CONFIGS, 't_pose', np.array([1., 0., 0., 0.]))
    monkeypatch.setitem(named_config.NAMED_CONFIGS, 'arms_front_45',
                        np.array([0., 1., 0., 0.]))
    model = SimpleNamespace(nq=4, nv=4, names=['universe', 'shoulder', 'elbow',
                                         'wrist_roll', 'wrist_yaw'],
                            idx_qs=[0, 0, 1, 2, 3],
                            lowerPositionLimit=np.full(4, -2.),
                            upperPositionLimit=np.full(4, 2.))
    robot = SimpleNamespace(model_body=model, model_body_reduced=model,
                            reduced_mask=np.ones(4, dtype=bool),
                            zero_q_body=np.zeros(4), state={'q': np.zeros(4)})
    instance = IKSolver.__new__(IKSolver)
    instance.robot_model = robot
    instance.configuration = Configuration(np.zeros(4))
    instance.configuration_reduced = Configuration(np.zeros(4))
    instance.posture_task = SimpleNamespace(set_target_from_configuration=lambda config: None)
    instance.frame_tasks = {'target': object()}
    instance._frame_task_errors = lambda config: (0., 0.)
    return instance


def test_reduced_ranks_wrist_motion_and_preserves_diagnostics(solver):
    outcomes = {'current': [0., 0., 1., 0.],
                't_pose': [1., 0., 0., 0.],
                'arms_front_45': [0., 1., 0., 0.]}

    def attempt(name, seed, alpha, deadline, linear, angular):
        q = np.array(outcomes[name])
        return IKAttemptResult(q, True, 0., 0., name, 1, 0.01)

    solver._solve_ik_reduced_attempt = attempt
    result = solver.solve_ik_reduced(initial_positions=[('bad', [float('nan')] * 4)])
    assert [item.seed_name for item in result.candidates] == [
        't_pose', 'arms_front_45', 'current']
    assert result.seed_name == 't_pose'
    assert result.attempts[-1].reason == 'bad seed must contain only finite values'
    assert len(result.attempts) == 4  # home duplicates current
    assert np.array_equal(solver.q_reduced, result.q)


def test_full_keeps_first_success_and_deduplicates(solver):
    calls = []

    def attempt(name, seed, alpha, deadline, linear, angular):
        calls.append(name)
        return IKAttemptResult(seed.copy(), True, 0., 0., name, 0, 0.)

    solver._solve_ik_attempt = attempt
    result = solver.solve_ik(initial_positions=[np.zeros(4)])
    assert result.seed_name == 'current'
    assert calls == ['current']
    assert len(result.candidates) == 1


def test_total_deadline_and_seed_cap(solver, monkeypatch):
    times = iter([0., 0., 0., 2.])
    monkeypatch.setattr('h12_ros2_controller.core.ik_solver.time.monotonic',
                        lambda: next(times))
    calls = []

    def attempt(name, seed, alpha, deadline, linear, angular):
        calls.append((name, deadline))
        return IKAttemptResult(seed.copy(), False, 1., 1., name, 0, 0., 'timeout')

    solver._solve_ik_reduced_attempt = attempt
    result = solver.solve_ik_reduced(timeout=10., total_timeout=1., max_seeds=2)
    assert calls == [('current', 1.)]
    assert len(result.attempts) == 1
    assert not result.success


def test_nonfinite_velocity_isolated_and_posture_fixed(solver):
    targets = []
    solver.posture_task.set_target_from_configuration = lambda config: targets.append(config.q.copy())
    solver._frame_task_errors = lambda config: (1., 0.)
    solver._ik_reduced = lambda tasks, dt: np.full(4, float('nan'))
    result = solver.solve_ik_reduced(timeout=0.1, total_timeout=0.5, max_seeds=2)
    assert not result.success
    assert all(item.reason == 'non-finite velocity' for item in result.attempts)
    assert len(targets) == len(result.attempts)
    assert np.all(np.isfinite(result.q))


def test_reference_positions_selects_nearest_candidate(solver):
    def attempt(name, seed, alpha, deadline, linear, angular):
        return IKAttemptResult(seed.copy(), True, 0., 0., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    result = solver.solve_ik_reduced(reference_positions=[1., 0., 0., 0.])
    assert result.seed_name == 't_pose'
    assert result.candidates[0].score <= result.candidates[1].score


def test_attempt_exception_does_not_prevent_other_seeds(solver):
    def attempt(name, seed, alpha, deadline, linear, angular):
        if name == 'current':
            raise ValueError('QP failed')
        return IKAttemptResult(seed.copy(), True, 0., 0., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    result = solver.solve_ik_reduced()
    assert result.success
    assert result.attempts[0].reason == 'ValueError: QP failed'
    assert result.seed_name == 't_pose'


def test_start_and_active_mask_keep_untasked_joints_fixed(solver):
    start = np.array([0., 0.3, 0.4, -0.2])
    mask = np.array([True, False, False, False])
    solver._frame_task_errors = lambda config: (abs(0.5 - config.q[0]), 0.)
    solver._ik_reduced = lambda tasks, dt: np.ones(4)
    result = solver.solve_ik_reduced(alpha=0.5, timeout=0.1,
                                     start_q=start, active_mask=mask)
    assert result.success
    assert np.allclose(result.q, [0.5, 0.3, 0.4, -0.2])
    assert np.array_equal(result.candidates[0].q[~mask], start[~mask])
    with pytest.raises(ValueError, match='active_mask'):
        solver.solve_ik_reduced(active_mask=[True])


def test_reduced_result_preserves_frozen_start_coordinates(solver):
    solver.robot_model.model_body = SimpleNamespace(nq=5)
    solver.robot_model.reduced_mask = np.array([True, True, False, True, True])
    solver.robot_model.state['q'] = np.array([0., 0., 0.7, 0., 0.])
    start = np.array([0., 0., -0.4, 0., 0.])
    solver._solve_ik_reduced_attempt = lambda name, seed, *args: IKAttemptResult(
        solver._reduced_q_to_full(seed), True, 0., 0., name, 0, 0.)
    result = solver.solve_ik_reduced(start_q=start, max_seeds=1)
    assert result.q[2] == -0.4
    assert result.candidates[0].q.shape == (5,)


def test_limit_margin_and_reference_affect_score(solver):
    neutral = np.zeros(4)
    near_limit = np.array([1.99, 0., 0., 0.])
    assert solver._naturalness_score(near_limit, near_limit, True) > 0
    assert solver._naturalness_score(neutral, neutral, True) == 0
    assert solver._naturalness_score(np.array([0., 0., 1., 0.]), neutral, True) > \
        solver._naturalness_score(np.array([1., 0., 0., 0.]), neutral, True)
    with pytest.raises(ValueError, match='alpha'):
        solver.solve_ik_reduced(alpha=0)


def test_preferred_wrist_posture_moves_inward_from_limit(solver):
    solver._batch_reduced_reference = np.array([0., 0., 1.99, 0.])
    preferred, costs = solver._batch_posture_preference(np.zeros(4))

    assert preferred[2] < 1.99
    assert np.array_equal(preferred[[0, 1]], [0., 0.])
    assert costs[2] > costs[0]


def test_batch_wrist_cost_restored_after_solve(solver):
    targets = []
    solver.posture_task.cost = 1e-3
    solver.posture_task.set_target = lambda q: targets.append(q.copy())
    start = np.array([0., 0., 1.99, 0.])

    result = solver.solve_ik_reduced(start_q=start, max_seeds=1)

    assert result.success
    assert targets[0][2] < start[2]
    assert solver.posture_task.cost == 1e-3


def test_reduced_candidates_deduplicate_and_reject_infeasible(solver):
    def attempt(name, seed, alpha, deadline, linear, angular):
        q = np.array([0., 0., 0., 0.])
        if name == 't_pose':
            q[0] = 2.1
        elif name == 'arms_front_45':
            q[0] = 0.5
        return IKAttemptResult(q, True, 0., 0., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    solver.robot_model.check_collision_free_reduced = lambda q: q[0] != 0.5
    result = solver.solve_ik_reduced()
    assert [candidate.seed_name for candidate in result.candidates] == ['current']
    assert result.attempts[1].reason == 'joint limits'
    assert result.attempts[2].reason == 'self-collision'


def test_empty_frame_tasks_do_not_report_success(solver):
    solver.frame_tasks.clear()
    result = solver.solve_ik_reduced()
    assert not result.success
    assert result.attempts[0].reason == 'no frame tasks'


def test_offline_and_controller_resolve_same_planner_profile():
    config = load_controller_config('debug.yaml')
    controller = object.__new__(UpperController)
    controller.config = config
    controller.dt = 1.0 / config['frequency']['ctrl_hz']

    actual = planner_config_from_controller(config)
    expected = controller._load_planner_config()

    assert actual == expected
    assert actual.min_interpolation_steps == 30
    assert actual.frame_z_min == -0.05


def test_saved_case_uses_same_hardware_free_benchmark_route(tmp_path):
    pipeline = KinematicPipeline()
    start = pipeline.robot_model.state['q'].copy()
    start[pipeline.robot_model.reduced_mask] = named_config.NAMED_CONFIGS[
        'arms_front_45'
    ]
    pipeline.set_start(start)
    transform = pipeline.robot_model.get_frame_transformation_reduced(
        'left_grasp_frame', pipeline.robot_model.state_reduced['q'],
    )
    pose = np.r_[transform[:3, 3], pin.rpy.matrixToRpy(transform[:3, :3])]
    case = {
        'config': 'debug.yaml', 'start_q': start.tolist(),
        'frame': 'left_grasp_frame', 'pose': pose.tolist(),
        'ik_timeout': 0.3,
    }
    path = tmp_path / 'case.json'
    path.write_text(json.dumps(case))

    result = pipeline.plan_case(case)

    assert result.ik.success
    assert result.plan.success
    assert run_benchmark(cases_path=str(path)) == 1


def test_offline_frame_goal_reaches_named_seeds_with_one_second_budget():
    pipeline = KinematicPipeline()
    pose = np.array([0.4, 0.2, 0.2, 0.0, np.deg2rad(80), 0.0])

    result = pipeline.plan_frame('left_grasp_frame', pose, ik_timeout=1.0)

    assert result.ik.success
    assert result.plan.success
    assert result.ik.seed_name == 't_pose'
    assert [attempt.seed_name for attempt in result.ik.attempts][:2] == [
        'current', 't_pose',
    ]


def test_candidate_selector_probes_all_direct_paths_before_ompl():
    candidates = [SimpleNamespace(q=np.array([float(i)]), score=0.0,
                                  seed_name=f'seed_{i}') for i in (1, 2)]
    calls = []

    def plan_goal(goal, direct_only, timeout):
        calls.append((float(goal[0]), direct_only))
        return PlanResult(
            success=bool(goal[0] == 2),
            path=np.array([[0.0], goal]),
            reason='direct blocked' if goal[0] == 1 else 'direct path',
        )

    plan, candidate = plan_ik_candidates(
        SimpleNamespace(candidates=candidates), plan_goal, np.array([True]),
    )

    assert plan.success
    assert candidate.seed_name == 'seed_2'
    assert calls == [(1.0, True), (2.0, True)]


def test_candidate_selector_picks_shorter_direct_path():
    candidates = [SimpleNamespace(q=np.array([float(i)]), score=0.0,
                                  seed_name=f'seed_{i}') for i in (1, 2)]

    def plan_goal(goal, direct_only, timeout):
        assert direct_only
        length = 2.0 if goal[0] == 1 else 1.0
        return PlanResult(
            success=True, path=np.array([[0.0], goal]),
            metadata={'joint_path_length': length},
        )

    plan, candidate = plan_ik_candidates(
        SimpleNamespace(candidates=candidates), plan_goal, np.array([True]),
    )

    assert plan.metadata['joint_path_length'] == 1.0
    assert candidate.seed_name == 'seed_2'


def test_candidate_selector_retries_after_rejected_ompl_detour():
    candidates = [SimpleNamespace(q=np.array([float(i)]), score=0.0,
                                  seed_name=f'seed_{i}') for i in (1, 2)]
    calls = []

    def plan_goal(goal, direct_only, timeout):
        calls.append((float(goal[0]), direct_only, timeout))
        if direct_only or goal[0] == 1:
            return PlanResult(
                success=False, path=np.empty((0, 1)),
                reason='path detour ratio exceeds local limit',
            )
        return PlanResult(
            success=True, path=np.array([[0.0], goal]), reason='success',
            metadata={'joint_path_ratio': 1.1},
        )

    plan, candidate = plan_ik_candidates(
        SimpleNamespace(candidates=candidates), plan_goal, np.array([True]),
        planning_budget=1.0,
    )

    assert plan.success
    assert candidate.seed_name == 'seed_2'
    assert [call[:2] for call in calls] == [
        (1.0, True), (2.0, True), (1.0, False), (2.0, False),
    ]
    assert all(0 < timeout <= 0.75 for _, direct, timeout in calls if not direct)


def test_problematic_pitch_goal_has_local_path_from_home():
    pipeline = KinematicPipeline()
    pose = np.array([0.42, 0.16, 0.25, np.deg2rad(15),
                     np.deg2rad(80), np.deg2rad(-15)])

    result = pipeline.plan_frame('left_grasp_frame', pose, ik_timeout=1.0)

    assert result.plan.success, result.plan.reason
    assert result.plan.metadata['joint_path_ratio'] <= 2.0
    assert (result.plan.metadata['min_frame_x']['left_grasp_frame'] >=
            result.plan.metadata['local_x_bounds']['left_grasp_frame'] - 0.01)


def test_interactive_goto_reads_command_before_dispatch(monkeypatch, capsys):
    monkeypatch.setattr(
        kinematic_plan_goto, 'KinematicPipeline',
        lambda **kwargs: SimpleNamespace(),
    )
    commands = iter(['save case.json', 'quit'])
    monkeypatch.setattr('builtins.input', lambda prompt: next(commands))

    kinematic_plan_goto.main()

    assert 'Plan a frame goal, then enter save <file>' in capsys.readouterr().out
