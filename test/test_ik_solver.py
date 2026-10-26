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
from h12_ros2_controller.core.planner import kinematic_pipeline
from h12_ros2_controller.core.planner.reduced_joint_planner import PlanResult, PlannerConfig
from h12_ros2_controller.benchmark.ik_benchmark import main as run_benchmark
from h12_ros2_controller.example import kinematic_plan_goto
from h12_ros2_controller.utility import named_config
from h12_ros2_controller.utility.controller_config import load_controller_config


class Configuration:
    def __init__(self, q):
        self.q = np.array(q, dtype=float)
        self.model = SimpleNamespace(nq=len(q), nv=len(q))

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
    instance.d_min = 0.02
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


def test_two_frame_convergence_uses_each_target_tolerance(solver):
    error = np.array([0.0006, 0., 0., 0.0012, 0., 0.])
    solver.frame_tasks = {
        'left': SimpleNamespace(frame='left_grasp_frame',
                                compute_error=lambda configuration: error),
        'right': SimpleNamespace(frame='right_grasp_frame',
                                 compute_error=lambda configuration: error),
    }

    linear, angular = IKSolver._frame_task_errors(solver, solver.configuration_reduced)
    details = solver._frame_error_details(solver.configuration_reduced)

    assert linear == pytest.approx(0.0006)
    assert angular == pytest.approx(0.0012)
    assert linear < 0.001 and angular < 0.002
    assert set(details) == {'left_grasp_frame', 'right_grasp_frame'}
    error[0] = float('nan')
    assert IKSolver._frame_task_errors(solver, solver.configuration_reduced) == (
        float('inf'), float('inf'),
    )


def test_frame_acceptance_uses_cartesian_position_not_se3_log_component(solver):
    task = SimpleNamespace(
        frame='left_grasp_frame',
        transform_target_to_world=SimpleNamespace(
            translation=np.zeros(3), rotation=np.eye(3),
        ),
        compute_error=lambda configuration: np.array([0.0005, 0., 0., 0., 0., 0.]),
    )
    configuration = SimpleNamespace(get_transform_frame_to_world=lambda frame:
                                    SimpleNamespace(
                                        translation=np.array([0.0012, 0., 0.]),
                                        rotation=np.eye(3),
                                    ))
    solver.frame_tasks = {'left': task}

    linear, angular = IKSolver._frame_task_errors(solver, configuration)

    assert linear == pytest.approx(0.0012)
    assert angular == 0.0
    assert linear > 0.001


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


def test_preferred_seed_order_mask_and_initial_positions(solver):
    calls = []
    start = np.array([0., 0.4, 0., 0.])
    mask = np.array([True, False, True, True])

    def attempt(name, seed, *args, **kwargs):
        calls.append((name, seed.copy()))
        return IKAttemptResult(seed.copy(), False, 1., 1., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    result = solver.solve_ik_reduced(
        start_q=start, active_mask=mask, preferred_positions=[0.5, 1., 0., 0.],
        initial_positions=[('extra', [0.8, 0., 0., 0.])], max_seeds=3,
    )
    assert [name for name, _ in calls] == ['current', 'preferred', 't_pose']
    assert np.array_equal(calls[1][1], [0.5, 0.4, 0., 0.])
    assert [item.seed_name for item in result.attempts] == [name for name, _ in calls]
    calls.clear()
    solver.solve_ik_reduced(
        start_q=start, active_mask=mask, preferred_positions=[0.5, 0., 0., 0.],
        initial_positions=[('extra', [0.8, 0., 0., 0.])], max_seeds=4,
    )
    assert [name for name, _ in calls] == [
        'current', 'preferred', 't_pose', 'extra',
    ]


@pytest.mark.parametrize('preferred', [
    [float('nan'), 0., 0., 0.],
    [0., float('inf'), 0., 0.],
    [0., 0., 0.],
    [[0., 0., 0., 0.]],
    ['invalid', 0., 0., 0.],
])
def test_preferred_seed_rejects_invalid_vectors(solver, preferred):
    with pytest.raises(ValueError, match='preferred'):
        solver.solve_ik_reduced(preferred_positions=preferred)


def test_preferred_full_seed_preserves_frozen_start(solver):
    solver.robot_model.model_body = SimpleNamespace(nq=5)
    solver.robot_model.reduced_mask = np.array([True, True, False, True, True])
    solver.robot_model.state['q'] = np.zeros(5)
    start = np.array([0., 0., 0.7, 0., 0.])
    preferred = np.array([0.5, 0., 0.7, 0., 0.])
    calls = []

    def attempt(name, seed, *args, **kwargs):
        calls.append((name, seed.copy()))
        return IKAttemptResult(solver._reduced_q_to_full(seed), False,
                               1., 1., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    solver.solve_ik_reduced(start_q=start, preferred_positions=preferred, max_seeds=2)
    assert [name for name, _ in calls] == ['current', 'preferred']
    assert np.array_equal(calls[1][1], [0.5, 0., 0., 0.])
    preferred[2] = -0.7
    with pytest.raises(ValueError, match='frozen joints'):
        solver.solve_ik_reduced(start_q=start, preferred_positions=preferred)
    preferred[2] = float('nan')
    with pytest.raises(ValueError, match='finite'):
        solver.solve_ik_reduced(start_q=start, preferred_positions=preferred)


def test_preferred_seed_dedup_and_state_isolation(solver):
    preferred = np.array([0.5, 0., 0., 0.])
    calls = []

    def attempt(name, seed, *args, **kwargs):
        calls.append(name)
        seed[0] = 1.5
        return IKAttemptResult(np.zeros(4), False, 1., 1., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    solver.solve_ik_reduced(preferred_positions=preferred,
                            initial_positions=[preferred.copy()], max_seeds=3)
    assert calls == ['current', 'preferred', 't_pose']
    assert np.array_equal(preferred, [0.5, 0., 0., 0.])
    calls.clear()
    solver.solve_ik_reduced(max_seeds=2)
    assert calls == ['current', 't_pose']
    calls.clear()
    solver.solve_ik_reduced(preferred_positions=np.zeros(4), max_seeds=2)
    assert calls == ['current', 't_pose']


def test_preferred_success_still_checks_feasibility(solver):
    def attempt(name, seed, *args, **kwargs):
        return IKAttemptResult(seed.copy(), name == 'preferred', 0., 0., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    result = solver.solve_ik_reduced(preferred_positions=[2.1, 0., 0., 0.],
                                     max_seeds=2)
    assert not result.success
    assert result.attempts[1].reason == 'preferred seed exceeds joint limits'


def test_arm_fallback_seeds_are_bounded_deterministic_and_preserve_mask(solver):
    solver.robot_model.model_body.names = [
        'universe', 'left_shoulder_pitch_joint', 'left_elbow_joint',
        'left_wrist_roll_joint', 'right_elbow_joint',
    ]
    solver.frame_tasks = {'target': SimpleNamespace(frame='left_grasp_frame')}
    start = np.array([1.9, 1.8, 0.4, -0.7])
    mask = np.array([True, True, True, False])
    calls = []

    def attempt(name, seed, alpha, deadline, linear, angular, active_mask=None):
        calls.append((name, seed.copy(), deadline))
        success = name == 'arm_fallback_1'
        return IKAttemptResult(seed.copy(), success, 0., 0., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    first = solver.solve_ik_reduced(start_q=start, active_mask=mask,
                                    total_timeout=1., max_seeds=6)
    assert [name for name, _, _ in calls] == [
        'current', 'home', 't_pose', 'arms_front_45',
        'arm_fallback_0', 'arm_fallback_1',
    ]
    assert first.success
    assert first.seed_name == 'arm_fallback_1'
    assert all(q[3] == start[3] for _, q, _ in calls)
    assert all(q[2] == start[2] for name, q, _ in calls
               if name.startswith('arm_fallback_'))
    assert np.array_equal(calls[-1][1][:2], [1.45, 2.])
    assert all(deadline <= calls[0][2] for _, _, deadline in calls)
    original = [(name, q.copy()) for name, q, _ in calls]
    calls.clear()
    solver.solve_ik_reduced(start_q=start, active_mask=mask,
                            total_timeout=1., max_seeds=6)
    assert all(name == previous_name and np.array_equal(q, previous_q)
               for (name, q, _), (previous_name, previous_q) in zip(calls, original))


def test_fallback_skipped_after_distinct_named_successes(solver):
    calls = []

    def attempt(name, seed, alpha, deadline, linear, angular):
        calls.append(name)
        return IKAttemptResult(seed.copy(), True, 0., 0., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    result = solver.solve_ik_reduced(max_seeds=8)
    assert calls == ['current', 't_pose', 'arms_front_45']
    assert len(result.candidates) == 3


def test_fallback_rejects_invalid_bounds_and_other_arm(solver):
    model = solver.robot_model.model_body_reduced
    model.names = ['universe', 'left_shoulder_joint', 'left_elbow_joint',
                   'left_wrist_joint', 'right_elbow_joint']
    model.lowerPositionLimit[0] = np.nan
    solver.frame_tasks = {'target': SimpleNamespace(frame='left_grasp_frame')}
    solver._batch_reduced_reference = np.zeros(4)
    seeds = [('current', np.zeros(4), '')]
    generated = solver._arm_fallback_seeds(seeds, np.ones(4, dtype=bool))
    assert generated
    assert all(q[0] == 0 and q[2] == 0 and q[3] == 0
               for _, q, _ in generated)
    assert all(-2 <= q[1] <= 2 for _, q, _ in generated)


def test_arm_fallback_keeps_full_model_frozen_joint(solver):
    solver.robot_model.model_body.nq = 5
    solver.robot_model.model_body_reduced = SimpleNamespace(
        nq=4, nv=4,
        names=['universe', 'left_shoulder_joint', 'left_elbow_joint',
               'left_wrist_joint', 'right_elbow_joint'],
        idx_qs=[0, 0, 1, 2, 3],
        lowerPositionLimit=np.full(4, -2.),
        upperPositionLimit=np.full(4, 2.),
    )
    solver.robot_model.reduced_mask = np.array([True, True, False, True, True])
    solver.robot_model.state['q'] = np.zeros(5)
    solver.frame_tasks = {'target': SimpleNamespace(frame='left_grasp_frame')}
    start = np.array([0., 0., -0.8, 0.2, 0.3])
    mask = np.array([True, True, False, False])

    def attempt(name, seed, *args, **kwargs):
        return IKAttemptResult(solver._reduced_q_to_full(seed),
                               name.startswith('arm_fallback_'), 0., 0., name, 0, 0.)

    solver._solve_ik_reduced_attempt = attempt
    result = solver.solve_ik_reduced(start_q=start, active_mask=mask, max_seeds=6)
    assert result.success
    assert all(item.q[2] == -0.8 for item in result.attempts)
    assert all(np.array_equal(item.q[[2, 3, 4]], start[[2, 3, 4]])
               for item in result.candidates)


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
    constraints_seen = []

    def constrained_step(tasks, dt, constraints=None):
        assert len(constraints) == 1
        constraint = constraints[0]
        constraints_seen.append(constraint.A.copy())
        np.testing.assert_array_equal(constraint.A, np.eye(4)[1:])
        np.testing.assert_allclose(constraint.q_0, solver.configuration_reduced.q)
        return np.array([1., 0., 0., 0.])

    solver._ik_reduced = constrained_step
    result = solver.solve_ik_reduced(alpha=0.5, timeout=0.1,
                                     start_q=start, active_mask=mask)
    assert result.success
    assert np.allclose(result.q, [0.5, 0.3, 0.4, -0.2])
    assert np.array_equal(result.candidates[0].q[~mask], start[~mask])
    assert constraints_seen
    with pytest.raises(ValueError, match='active_mask'):
        solver.solve_ik_reduced(active_mask=[True])


def test_real_reduced_qp_keeps_untasked_arm_velocity_zero():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    goal = np.asarray(named_config.NAMED_CONFIGS['arms_front_45'], dtype=float)
    transform = pipeline.robot_model.get_frame_transformation_reduced(
        'left_grasp_frame', goal,
    )
    pose = np.r_[transform[:3, 3], pin.rpy.matrixToRpy(transform[:3, :3])]
    pipeline.set_start(np.zeros(27))
    original = pipeline.ik_solver._ik_reduced
    velocities = []

    def constrained_step(tasks, dt, constraints=None):
        velocity = original(tasks, dt, constraints=constraints)
        assert constraints and len(constraints) == 1
        velocities.append(velocity.copy())
        assert np.max(np.abs(velocity[7:])) < 1e-6
        return velocity

    pipeline.ik_solver._ik_reduced = constrained_step
    result = pipeline.plan_frame('left_grasp_frame', pose)

    assert result.ik.success and result.plan.success
    assert velocities
    assert np.allclose(result.plan.path[:, 7:], 0.0)


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


def test_out_of_bound_seed_isolated_before_qp_solve(solver):
    result = solver.solve_ik_reduced(
        start_q=np.array([2.1, 0., 0., 0.]), max_seeds=2,
    )

    assert result.success
    assert any(attempt.seed_name == 'current' and
               attempt.reason == 'current seed exceeds joint limits'
               for attempt in result.attempts)
    assert all(attempt.seed_name != 'current' for attempt in result.candidates)


def test_preferred_wrist_posture_moves_inward_from_limit(solver):
    solver.wrist_posture_cost = 0.02
    solver._batch_reduced_reference = np.array([0., 0., 1.99, 0.])
    preferred, costs = solver._batch_posture_preference(np.zeros(4))

    assert preferred[2] < 1.99
    assert np.array_equal(preferred[[0, 1]], [0., 0.])
    assert costs[2] > costs[0]
    assert costs[2] == pytest.approx(0.02)


def test_fallback_seed_offsets_avoid_wrist_perturbation(solver):
    solver.shoulder_seed_offset = 0.2
    solver.elbow_seed_offset = 0.3
    solver._batch_reduced_reference = np.zeros(4)

    seeds = solver._arm_fallback_seeds([], np.array([True, True, True, True]))

    assert seeds
    assert all(abs(seed[0]) <= 0.2 for _, seed, _ in seeds)
    assert all(abs(seed[1]) <= 0.3 for _, seed, _ in seeds)
    assert all(np.array_equal(seed[2:], [0., 0.]) for _, seed, _ in seeds)


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
    solver.robot_model.check_collision_free_reduced = (
        lambda q, min_distance=0.0: q[0] != 0.5
    )
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
    config = load_controller_config('sim_safety_split.yaml')
    controller = object.__new__(UpperController)
    controller.config = config
    controller.dt = 1.0 / config['frequency']['ctrl_hz']

    actual = planner_config_from_controller(config)
    expected = controller._load_planner_config()

    assert actual == expected
    assert actual.min_interpolation_steps == 30
    assert actual.frame_z_min == -0.05
    assert min(actual.joint_velocity_limit) == pytest.approx(1.0)
    assert max(actual.joint_velocity_limit) == pytest.approx(1.0)
    assert actual.ik_seed_timeout == config['planner']['ik_seed_timeout']
    assert actual.ik_wrist_motion_weight == config['planner']['ik_wrist_motion_weight']
    assert actual.ik_wrist_posture_cost == config['planner']['ik_wrist_posture_cost']
    assert actual.candidate_planning_budget == config['planner']['candidate_planning_budget']
    assert actual.max_hand_deviation == config['planner']['max_hand_deviation']
    assert actual.local_joint_padding == 0.6
    assert actual.compact_radial_slack == config['planner']['compact_radial_slack']
    assert actual.self_collision_clearance == config['controller']['d_min']
    assert actual.worker_startup_timeout == config['planner']['worker_startup_timeout']
    config['planner']['joint_velocity_limit'] = 100.0
    capped = planner_config_from_controller(config)
    assert capped.joint_velocity_limit == actual.joint_velocity_limit
    config['planner']['self_collision_clearance'] = 0.0
    with pytest.raises(ValueError, match='must match controller d_min'):
        planner_config_from_controller(config)


@pytest.mark.parametrize('profile', [
    'safety_split.yaml', 'safety_full.yaml',
    'sim_safety_split.yaml', 'counter_safety_split.yaml',
    'real_benchmark_upper.yaml',
])
def test_planning_profiles_resolve_explicit_motion_policy(profile):
    config = load_controller_config(profile)
    resolved = planner_config_from_controller(config)
    for key in ('continuous_timing', 'joint_acceleration_limit',
                'local_x_backtrack', 'elbow_x_backtrack',
                'max_hand_deviation', 'compact_radial_slack',
                'preferred_clearance', 'local_joint_padding',
                'candidate_compact_weight', 'candidate_planning_budget',
                'worker_request_timeout', 'worker_startup_timeout'):
        assert getattr(resolved, key) == config['planner'][key]
    for key in ('execution_start_tolerance',
                'command_seed_max_gap', 'plan_max_age'):
        assert config['controller'][key] > 0.0
    assert config['planner']['frame_y_min'] < config['planner']['frame_y_max']


def test_offline_planning_requires_explicit_profile():
    with pytest.raises(ValueError, match='explicit planner YAML block'):
        KinematicPipeline(config_name='balance_safety_split.yaml')


def test_injected_planner_config_cannot_weaken_ik_self_clearance():
    with pytest.raises(ValueError, match='must match controller d_min'):
        KinematicPipeline(config_name='sim_safety_split.yaml', planner_config=PlannerConfig(
            dt=1.0 / 50.0, self_collision_clearance=0.0,
        ))


def test_controller_rejects_mismatched_planner_period_before_hardware(monkeypatch):
    config = load_controller_config('safety_split.yaml')
    config['planner']['dt'] = 0.1
    monkeypatch.setattr(
        'h12_ros2_controller.core.controller.upper_controller.RobotModel',
        lambda *args, **kwargs: pytest.fail('hardware initialization must not run'),
    )

    with pytest.raises(ValueError, match='Planner dt must match'):
        UpperController('', '', '', init=False, config=config)


def test_saved_case_uses_same_hardware_free_benchmark_route(tmp_path):
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
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
        'config': 'sim_safety_split.yaml', 'start_q': start.tolist(),
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
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    pose = np.array([0.4, 0.2, 0.2, 0.0, np.deg2rad(80), 0.0])

    result = pipeline.plan_frame('left_grasp_frame', pose, ik_timeout=1.0)

    assert result.ik.success
    assert result.plan.success
    assert result.phase_timings['ik'] > 0.0
    assert result.phase_timings['candidate_search'] > 0.0
    assert result.phase_timings['total'] == result.elapsed_time
    assert len(result.candidate_attempts) >= 1
    assert sum(attempt['wall_time'] for attempt in result.candidate_attempts) <= (
        result.phase_timings['candidate_search'] + 0.01
    )
    assert result.ik.seed_name == 't_pose'
    assert [attempt.seed_name for attempt in result.ik.attempts][:2] == [
        'current', 't_pose',
    ]


def test_two_frame_goals_share_one_ik_solve_and_reach_both_targets():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    model = pipeline.robot_model
    goal = np.asarray(named_config.NAMED_CONFIGS['arms_front_45'], dtype=float)
    tasks = []
    for name in ('left_grasp_frame', 'right_grasp_frame'):
        transform = model.get_frame_transformation_reduced(name, goal)
        tasks.append((name, np.r_[
            transform[:3, 3], pin.rpy.matrixToRpy(transform[:3, :3]),
        ]))

    result = pipeline.plan_frames(tasks)

    assert len(pipeline.ik_solver.frame_tasks) == 2
    assert result.ik.success
    assert result.plan.success, result.plan.reason
    assert result.plan.path.shape[1] == 14
    for name, pose in tasks:
        reached = model.get_frame_transformation_reduced(name, result.plan.path[-1])
        assert np.linalg.norm(reached[:3, 3] - pose[:3]) < pipeline.ik_linear_threshold
        target_rotation = pin.rpy.rpyToMatrix(pose[3:])
        assert (np.linalg.norm(pin.log3(target_rotation.T @ reached[:3, :3])) <
                pipeline.ik_angular_threshold)

    case = {
        'config': 'sim_safety_split.yaml', 'start_q': np.zeros(27).tolist(),
        'tasks': [{'frame': name, 'pose': pose.tolist()} for name, pose in tasks],
    }
    replay = pipeline.plan_case(case)
    assert replay.ik.success and replay.plan.success
    with pytest.raises(ValueError, match='distinct frame names'):
        pipeline.plan_frames((tasks[0], tasks[0]))
    assert len(pipeline.ik_solver.frame_tasks) == 2


def test_bimanual_overhang_never_accepts_failed_clearance_or_frame_tasks():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    tasks = [
        ('left_grasp_frame', np.array([
            0.47, 0.14, 0.34, np.deg2rad(15), np.deg2rad(60), np.deg2rad(-15),
        ])),
        ('right_grasp_frame', np.array([
            0.47, -0.14, 0.34, np.deg2rad(-15), np.deg2rad(60), np.deg2rad(15),
        ])),
    ]

    result = pipeline.plan_frames(tasks)

    if result.plan is None or not result.plan.success:
        assert result.selected_candidate is None
        assert result.fallback_reason or (result.plan is not None and result.plan.reason)
        return
    assert result.plan.metadata['joint_path_ratio'] <= 2.0
    for name, pose in tasks:
        transform = pipeline.robot_model.get_frame_transformation_reduced(
            name, result.plan.path[-1],
        )
        assert np.linalg.norm(transform[:3, 3] - pose[:3]) < pipeline.ik_linear_threshold
        assert np.linalg.norm(pin.log3(
            pin.rpy.rpyToMatrix(pose[3:]).T @ transform[:3, :3]
        )) < pipeline.ik_angular_threshold


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


def test_candidate_selector_does_not_stop_on_ratio_when_better_score_is_possible():
    candidates = [
        SimpleNamespace(q=np.array([1.0]), score=0.0, seed_name='first'),
        SimpleNamespace(q=np.array([0.5]), score=0.0, seed_name='compact'),
    ]
    calls = []

    def plan_goal(goal, direct_only, timeout):
        calls.append((float(goal[0]), direct_only))
        if direct_only:
            return PlanResult(success=False, path=np.empty((0, 1)), reason='blocked')
        length = 1.2 if goal[0] == 1.0 else 0.7
        return PlanResult(
            success=True, path=np.array([[0.0], goal]), reason='success',
            metadata={'joint_path_length': length, 'joint_path_ratio': 1.2},
        )

    plan, selected = plan_ik_candidates(
        SimpleNamespace(candidates=candidates), plan_goal, np.array([True]),
    )

    assert plan.success and selected.seed_name == 'compact'
    assert calls == [(1.0, True), (0.5, True),
                     (1.0, False), (0.5, False)]


def test_candidate_deadline_stops_probes_and_keeps_validated_path(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(kinematic_pipeline.time, 'perf_counter', lambda: clock[0])
    candidates = [SimpleNamespace(q=np.array([float(i)]), score=0.0,
                                  seed_name=f'seed_{i}') for i in (1, 2, 3)]
    calls = []

    def plan_goal(goal, direct_only, timeout):
        calls.append((goal[0], direct_only, timeout))
        clock[0] += 0.6
        return PlanResult(
            success=True, path=np.array([[0.0], goal]),
            metadata={'joint_path_length': goal[0]},
        )

    plan, candidate = plan_ik_candidates(
        SimpleNamespace(candidates=candidates), plan_goal, np.array([True]),
        deadline=1.0,
    )

    assert plan.success and candidate.seed_name == 'seed_1'
    assert [call[0] for call in calls] == [1.0, 2.0]
    assert calls[0][2] == pytest.approx(1.0)
    assert calls[1][2] == pytest.approx(0.4)


def test_candidate_deadline_returns_validated_path_after_later_timeout(monkeypatch):
    monkeypatch.setattr(kinematic_pipeline.time, 'perf_counter', lambda: 0.0)
    candidates = [SimpleNamespace(q=np.array([float(i)]), score=0.0,
                                  seed_name=f'seed_{i}') for i in (1, 2)]

    def plan_goal(goal, direct_only, timeout):
        if goal[0] == 2:
            raise TimeoutError('worker deadline')
        return PlanResult(success=True, path=np.array([[0.0], goal]))

    plan, candidate = plan_ik_candidates(
        SimpleNamespace(candidates=candidates), plan_goal, np.array([True]),
        deadline=1.0,
    )
    assert plan.success and candidate.seed_name == 'seed_1'


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
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    pose = np.array([0.42, 0.16, 0.25, np.deg2rad(15),
                     np.deg2rad(80), np.deg2rad(-15)])

    result = pipeline.plan_frame('left_grasp_frame', pose, ik_timeout=1.0)

    assert result.plan.success, result.plan.reason
    assert result.plan.metadata['joint_path_ratio'] <= 2.0
    assert (result.plan.metadata['min_frame_x']['left_grasp_frame'] >=
            result.plan.metadata['local_x_bounds']['left_grasp_frame'] - 0.01)

    pipeline.replay(result.plan.path)
    home = pipeline.plan_named('home')

    if home.plan.success:
        assert home.plan.metadata['joint_path_ratio'] <= 2.0
        assert (home.plan.metadata['min_frame_x']['left_grasp_frame'] >=
                home.plan.metadata['local_x_bounds']['left_grasp_frame'] - 0.01)
    else:
        assert 'local' in home.plan.reason or 'detour ratio' in home.plan.reason


def test_combined_overhang_arms_return_home_without_rearward_sweep():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    goals = (
        ('left_grasp_frame', [0.51, 0.14, 0.32, 15, 60, -15]),
        ('right_grasp_frame', [0.49, -0.18, 0.28, -15, 70, 15]),
    )
    for frame, values in goals:
        pose = np.asarray(values, dtype=float)
        pose[3:] = np.deg2rad(pose[3:])
        result = pipeline.plan_frame(frame, pose)
        assert result.plan.success, result.plan.reason
        pipeline.replay(result.plan.path)

    home = pipeline.plan_named('home')

    assert home.plan.success, home.plan.reason
    assert np.allclose(home.plan.path[-1], 0.0)
    assert (home.plan.metadata['joint_path_ratio'] <=
            pipeline.planner.config.max_local_path_ratio)
    for frame, minimum in home.plan.metadata['min_frame_x'].items():
        assert minimum >= home.plan.metadata['local_x_bounds'][frame] - 0.01


def test_nearby_offline_goal_reuses_prior_ik_seed_but_explicit_reset_clears_it():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    pose = np.array([0.42, 0.16, 0.25, np.deg2rad(15),
                     np.deg2rad(80), np.deg2rad(-15)])
    first = pipeline.plan_frame('left_grasp_frame', pose)
    assert first.plan.success, first.plan.reason
    original = pipeline.ik_solver.solve_ik_reduced
    inputs = []

    def record_ik(**kwargs):
        inputs.append(kwargs.get('preferred_positions'))
        return original(**kwargs)

    pipeline.ik_solver.solve_ik_reduced = record_ik
    nearby = pose.copy()
    nearby[0] += 0.01
    pipeline.plan_frame('left_grasp_frame', nearby)
    assert np.array_equal(inputs[-1], first.selected_candidate.q)

    pipeline.set_start(np.zeros(27))
    pipeline.plan_frame('left_grasp_frame', nearby)
    assert inputs[-1] is None


def test_interactive_goto_reads_command_before_dispatch(monkeypatch, capsys):
    monkeypatch.setattr(
        kinematic_plan_goto, 'KinematicPipeline',
        lambda **kwargs: SimpleNamespace(),
    )
    commands = iter(['save case.json', 'quit'])
    monkeypatch.setattr('builtins.input', lambda prompt: next(commands))

    kinematic_plan_goto.main()

    assert 'Plan a frame goal, then enter save <file>' in capsys.readouterr().out
