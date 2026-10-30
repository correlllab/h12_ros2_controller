from types import SimpleNamespace
from time import perf_counter

import numpy as np
import pytest

from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlanResult,
    PlannerConfig,
    ReducedJointPlanner,
    moving_grasp_frames,
)
from h12_ros2_controller.core.planner.kinematic_pipeline import KinematicPipeline
from h12_ros2_controller.core.planner import reduced_joint_planner


class FakeRobotModel:
    def __init__(self, lower=None, upper=None, collision_free=True):
        lower = np.asarray(lower if lower is not None else [-1.0, -2.0])
        upper = np.asarray(upper if upper is not None else [1.0, 2.0])
        self.init_reduced = True
        self.init_collision = True
        self.collision_free = collision_free
        self.model_body_reduced = SimpleNamespace(
            nq=lower.shape[0],
            lowerPositionLimit=lower,
            upperPositionLimit=upper,
            velocityLimit=np.ones(lower.shape[0]),
        )
        self.reduced_mask = np.ones(lower.shape[0], dtype=bool)
        self.state = {'q': np.zeros(lower.shape[0])}

    def check_within_limits_reduced(self, q_reduced):
        lower = self.model_body_reduced.lowerPositionLimit
        upper = self.model_body_reduced.upperPositionLimit
        return bool(np.all(q_reduced >= lower) and np.all(q_reduced <= upper))

    def reduced_limit_violations(self, q_reduced):
        lower = self.model_body_reduced.lowerPositionLimit
        upper = self.model_body_reduced.upperPositionLimit
        violations = []
        for i, value in enumerate(q_reduced):
            if value < lower[i] or value > upper[i]:
                bound = lower[i] if value < lower[i] else upper[i]
                violations.append({
                    'name': f'q[{i}]', 'value': value, 'bound': bound,
                    'overshoot': abs(value - bound),
                })
        return violations

    def check_collision_free_reduced(self, q_reduced, min_distance=0.0):
        return self.collision_free

    def get_frame_position(self, frame_name, q):
        return np.array([q[1], q[1], q[0]])


def test_named_motion_selects_only_moving_arm_grasp_frames():
    start = np.zeros(14)
    goal = start.copy()
    assert moving_grasp_frames(start, goal, ('left_grasp_frame',)) == ()

    goal[0] = 0.2
    frames = ('left_grasp_frame', 'right_grasp_frame')
    assert moving_grasp_frames(start, goal, frames) == ('left_grasp_frame',)

    goal[7] = 0.2
    assert moving_grasp_frames(start, goal, frames) == frames
    mask = np.r_[np.ones(7, dtype=bool), np.zeros(7, dtype=bool)]
    assert moving_grasp_frames(start, goal, frames, mask) == ('left_grasp_frame',)
    with pytest.raises(ValueError, match='requires left_grasp_frame'):
        moving_grasp_frames(start, goal, ('right_grasp_frame',))
    with pytest.raises(ValueError, match='14 reduced joint positions'):
        moving_grasp_frames(start[:7], goal[:7], frames)


@pytest.mark.parametrize('partial_mask', (False, True))
def test_two_arm_fallback_stitches_valid_timed_single_arm_paths(partial_mask):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ), config=PlannerConfig(prefer_coordinated_motion=False))
    start = np.zeros(14)
    goal = start.copy()
    goal[[0, 7]] = 0.1
    original = planner._plan_once

    def block_combined(first, second, **kwargs):
        if np.any(np.asarray(first)[:7] != np.asarray(second)[:7]) and np.any(
            np.asarray(first)[7:] != np.asarray(second)[7:]
        ):
            return PlanResult(
                success=False, path=np.empty((0, 14)),
                reason='OMPL returned only an approximate solution',
            )
        return original(first, second, **kwargs)

    planner._plan_once = block_combined
    active = np.zeros(14, dtype=bool)
    active[[0, 7]] = True
    result = planner.plan(
        start, goal, active_mask=active if partial_mask else None,
        local_frames=('left_grasp_frame', 'right_grasp_frame'),
    )

    assert result.success, result.reason
    assert result.metadata['path_kind'] == 'staged two-arm path'
    assert result.metadata['stage_leg_reasons'] == ['direct path', 'direct path']
    assert np.array_equal(result.path[0], start)
    assert np.array_equal(result.path[-1], goal)
    assert result.metadata['joint_path_ratio'] <= 2.0
    if partial_mask:
        assert np.array_equal(result.path[:, ~active], np.zeros((len(result.path), 12)))


@pytest.mark.parametrize('partial_mask', (False, True))
def test_two_arm_fallback_prefers_valid_synchronized_progress(partial_mask):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ))
    start = np.zeros(14)
    goal = start.copy()
    goal[[0, 7]] = 0.1
    original = planner._plan_once
    calls = []

    def block_combined(first, second, **kwargs):
        calls.append((np.asarray(first).copy(), np.asarray(second).copy()))
        if np.any(np.asarray(first)[:7] != np.asarray(second)[:7]) and np.any(
            np.asarray(first)[7:] != np.asarray(second)[7:]
        ):
            return PlanResult(success=False, path=np.empty((0, 14)), reason='no direct route')
        return original(first, second, **kwargs)

    planner._plan_once = block_combined
    mask = np.zeros(14, dtype=bool)
    mask[[0, 7]] = True
    result = planner.plan(
        start, goal, active_mask=mask if partial_mask else None,
        local_frames=('left_grasp_frame', 'right_grasp_frame'),
    )

    assert result.success, result.reason
    assert result.metadata['coordination'] == 'synchronous'
    assert len(calls) == 3
    assert np.allclose(result.path[-1], goal)
    velocities = np.abs(np.diff(result.path, axis=0))
    assert np.any((velocities[:, 0] > 1e-6) & (velocities[:, 7] > 1e-6))
    if partial_mask:
        assert np.allclose(result.path[:, ~mask], 0.0)


def test_staging_reuses_same_start_route_but_replans_changed_second_arm_context():
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ))
    start = np.zeros(14)
    goal = start.copy()
    goal[[0, 7]] = 0.1
    original = planner._plan_once
    calls = []

    def single_arm_only(first, second, **kwargs):
        calls.append((np.asarray(first).copy(), np.asarray(second).copy()))
        if len(calls) == 1:
            return PlanResult(success=False, path=np.empty((0, 14)), reason='no combined route')
        return original(first, second, **kwargs)

    planner._plan_once = single_arm_only
    planner._coordinate_arm_paths = lambda *args, **kwargs: (None, 'combined path blocked')
    result = planner.plan(
        start, goal, local_frames=('left_grasp_frame', 'right_grasp_frame'),
    )

    assert result.success, result.reason
    assert result.metadata['path_kind'] == 'staged two-arm path'
    assert len(calls) == 4
    assert calls[-1][0][7] == goal[7]
    assert calls[-1][0][0] == start[0]
    assert np.array_equal(calls[-1][1], goal)


def test_two_arm_fallback_retains_final_waypoint_cap():
    config = PlannerConfig(max_interpolation_steps=35,
                           max_staged_interpolation_steps=40,
                           prefer_coordinated_motion=False)
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ), config=config)
    start = np.zeros(14)
    goal = start.copy()
    goal[[0, 7]] = 0.1
    retiming_calls = []
    original_timing = planner._select_timed_path

    def timed_candidate(*args, **kwargs):
        retiming_calls.append(True)
        return original_timing(*args, **kwargs)

    def staged_legs(first, second, **kwargs):
        first = np.asarray(first)
        second = np.asarray(second)
        if np.any(first[:7] != second[:7]) and np.any(first[7:] != second[7:]):
            return PlanResult(
                success=False, path=np.empty((0, 14)), reason='no direct bimanual path',
            )
        path = np.linspace(first, second, 25)
        lengths = {'left_grasp_frame': float(abs(second[0] - first[0])),
                   'right_grasp_frame': 0.0}
        return PlanResult(success=True, path=path, reason='direct path', metadata={
            'frame_path_lengths': lengths,
            'frame_translation_lengths': lengths.copy(),
            'frame_rotation_lengths': {name: 0.0 for name in lengths},
            'actual_frame_speeds': {name: 0.2 for name in lengths},
            'length_estimate_samples': len(path),
        })

    planner._select_timed_path = timed_candidate
    planner._plan_once = staged_legs
    result = planner.plan(
        start, goal, local_frames=('left_grasp_frame', 'right_grasp_frame'),
    )

    assert retiming_calls
    assert result.success, result.reason
    assert len(result.path) <= config.max_interpolation_steps
    assert result.metadata['staged_retimed']


def test_rejects_staged_cap_below_single_plan_cap():
    with pytest.raises(ValueError, match='max_staged_interpolation_steps'):
        ReducedJointPlanner(FakeRobotModel(), config=PlannerConfig(
            max_interpolation_steps=30,
            max_staged_interpolation_steps=29,
        ))


@pytest.mark.parametrize('coordinated', [False, True])
def test_two_arm_fallback_cannot_restart_expired_request_deadline(monkeypatch, coordinated):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ), config=PlannerConfig(prefer_coordinated_motion=coordinated))
    clock = [0.0]
    calls = []
    deadlines = []
    monkeypatch.setattr(
        'h12_ros2_controller.core.planner.reduced_joint_planner.perf_counter',
        lambda: clock[0],
    )

    def unavailable(*args, **kwargs):
        calls.append(kwargs.get('timeout'))
        deadlines.append(kwargs.get('deadline'))
        clock[0] += 0.2
        return PlanResult(success=False, path=np.empty((0, 14)),
                          reason='OMPL returned an approximate solution')

    planner._plan_once = unavailable
    goal = np.zeros(14)
    goal[[0, 7]] = 0.1

    result = planner.plan(
        np.zeros(14), goal, local_frames=('left_grasp_frame', 'right_grasp_frame'),
        deadline=0.3,
    )

    assert not result.success
    assert len(calls) == 2
    assert deadlines == [0.3, 0.25 if coordinated else 0.3]
    assert 0.0 < calls[1] <= 0.1
    assert clock[0] == pytest.approx(0.4)
    assert 'budget exhausted' in result.reason


def test_bimanual_coordination_staggers_arms_around_cross_collision():
    robot = FakeRobotModel(lower=-np.ones(14), upper=np.ones(14))
    robot.check_collision_free_reduced = lambda q, min_distance=0.0: not (
        0.03 < q[0] < 0.07 and 0.03 < q[7] < 0.07
    )
    planner = ReducedJointPlanner(robot)
    start = np.zeros(14)
    goal = start.copy()
    goal[[0, 7]] = 0.1

    result, reason = planner._coordinate_same_start_arms(
        start, goal, None, ('left_grasp_frame', 'right_grasp_frame'),
        allow_reuse=False, started=perf_counter(), local_joint_padding=None,
    )

    assert result is not None, reason
    assert result.success
    assert result.metadata['coordination'] == 'staggered'
    assert np.array_equal(result.path[0], start)
    assert np.array_equal(result.path[-1], goal)
    assert planner._path_validity_failure(result.path) == ''


def test_requires_initialized_reduced_model():
    robot_model = FakeRobotModel()
    robot_model.init_reduced = False

    with pytest.raises(ValueError, match='reduced model'):
        ReducedJointPlanner(robot_model)


def test_requires_initialized_collision_model():
    robot_model = FakeRobotModel()
    robot_model.init_collision = False

    with pytest.raises(ValueError, match='collision model'):
        ReducedJointPlanner(robot_model)


def test_wrong_vector_shape_raises_when_ompl_available():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(FakeRobotModel())

    with pytest.raises(ValueError, match='start must have shape'):
        planner.plan([0.0, 0.0, 0.0], [0.0, 0.0])


def test_invalid_start_fails_with_clear_reason():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(FakeRobotModel())

    result = planner.plan([2.0, 0.0], [0.0, 0.0])

    assert not result.success
    assert result.path.shape == (0, 2)
    assert result.reason.startswith('start is outside reduced joint limits:')


def test_invalid_goal_fails_with_clear_reason():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(FakeRobotModel())

    result = planner.plan([0.0, 0.0], [0.0, 3.0])

    assert not result.success
    assert result.path.shape == (0, 2)
    assert result.reason.startswith('goal is outside reduced joint limits:')


def test_collision_state_fails_with_clear_reason():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(FakeRobotModel(collision_free=False))

    result = planner.plan([0.0, 0.0], [0.5, 0.5])

    assert not result.success
    assert result.path.shape == (0, 2)
    assert result.reason == 'start violates self-collision clearance'


def test_bounds_are_copied_to_ompl_space():
    pytest.importorskip('ompl')
    lower = np.array([-1.0, -0.5, -0.25])
    upper = np.array([1.0, 0.5, 0.25])
    planner = ReducedJointPlanner(FakeRobotModel(lower=lower, upper=upper))
    bounds = planner.space.getBounds()

    assert planner.nq == 3
    for idx in range(planner.nq):
        assert bounds.low[idx] == pytest.approx(lower[idx])
        assert bounds.high[idx] == pytest.approx(upper[idx])


def test_local_joint_bounds_restore_global_space_after_search():
    planner = ReducedJointPlanner(FakeRobotModel(), config=PlannerConfig(
        try_direct=False, timeout=0.2, local_joint_padding=0.2,
    ))
    start = np.zeros(2)
    goal = np.array([0.2, -0.2])
    planner._set_active_mask(None, start)
    planner._apply_active_bounds(start, goal, padding=0.2)
    local = planner.space.getBounds()
    assert local.low[0] == pytest.approx(-0.2)
    assert local.high[0] == pytest.approx(0.4)

    result = planner.plan(start, goal, local_frames=('left_grasp_frame',))

    assert result.success, result.reason
    assert result.metadata['search_region'] in ('local', 'global')
    global_bounds = planner.space.getBounds()
    assert global_bounds.low[0] == pytest.approx(planner.model_lower[0])
    assert global_bounds.high[0] == pytest.approx(planner.model_upper[0])

    named = planner.plan(start, goal, local_frames=('left_grasp_frame',),
                         local_joint_padding=0.0, allow_reuse=False)
    assert named.success
    assert named.metadata['search_region'] == 'global'


@pytest.mark.parametrize('fraction', (0.0, 1.0, float('nan')))
def test_local_search_rejects_invalid_budget_fraction(fraction):
    with pytest.raises(ValueError, match='local_search_fraction'):
        ReducedJointPlanner(FakeRobotModel(), config=PlannerConfig(
            local_search_fraction=fraction,
        ))


def test_plan_between_fake_valid_states():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(timeout=0.2),
    )

    result = planner.plan([0.0, 0.0], [0.5, 0.5])

    assert result.success
    assert result.path.shape[1] == 2
    assert result.path.shape[0] <= planner.config.max_interpolation_steps
    assert np.allclose(result.path[0], [0.0, 0.0])
    assert np.allclose(result.path[-1], [0.5, 0.5])
    assert result.metadata['path_kind'] == 'direct path'


def test_direct_only_reports_invalid_segment_without_using_ompl():
    robot_model = FakeRobotModel()
    robot_model.check_collision_free_reduced = (
        lambda q, min_distance=0.0: not 0.25 < q[0] < 0.35
    )
    planner = ReducedJointPlanner(robot_model)

    result = planner.plan([0.0, 0.0], [0.5, 0.5], direct_only=True)

    assert not result.success
    assert result.reason.startswith('direct path failed: path waypoint')


def test_local_corridor_rejects_rearward_grasp_sweep():
    planner = ReducedJointPlanner(FakeRobotModel())
    start = np.array([0.0, 0.0])
    goal = np.array([0.5, 0.0])
    planner._set_workspace_corridor(
        start, goal, local_frames=('left_grasp_frame',)
    )

    reason = planner._path_validity_failure(
        np.array([start, [0.25, -0.3], goal])
    )

    assert 'behind local x=' in reason


def test_local_path_quality_rejects_long_joint_detour():
    planner = ReducedJointPlanner(FakeRobotModel())
    start = np.array([0.0, 0.0])
    goal = np.array([0.5, 0.0])
    planner._set_workspace_corridor(
        start, goal, local_frames=('left_grasp_frame',)
    )

    metrics, reason = planner._path_quality_failure(
        np.array([start, [0.25, 1.0], goal]), start, goal,
        ('left_grasp_frame',),
    )

    assert metrics['joint_path_ratio'] > 2.0
    assert 'path detour ratio' in reason


def test_local_path_quality_rejects_hand_sweep_even_with_relaxed_joint_ratio():
    config = PlannerConfig(max_local_path_ratio=10.0, max_hand_deviation=0.3)
    planner = ReducedJointPlanner(FakeRobotModel(), config=config)
    start = np.array([0.0, 0.0])
    goal = np.array([0.5, 0.0])
    planner._set_workspace_corridor(
        start, goal, local_frames=('left_grasp_frame',)
    )

    metrics, reason = planner._path_quality_failure(
        np.array([start, [0.25, 0.5], goal]), start, goal,
        ('left_grasp_frame',),
    )

    assert metrics['hand_segment_deviation']['left_grasp_frame'] > 0.3
    assert 'hand deviates' in reason


def test_local_grasp_goal_reports_elbow_clearance_and_hand_sweep():
    pipeline = KinematicPipeline(
        config_name='sim_safety_split.yaml',
        planner_config=PlannerConfig(dt=1.0 / 50.0, max_interpolation_steps=600)
    )
    pose = np.array([0.42, 0.16, 0.25, np.deg2rad(15),
                     np.deg2rad(80), np.deg2rad(-15)])

    result = pipeline.plan_frame('left_grasp_frame', pose)

    assert result.plan.success, result.plan.reason
    metadata = result.plan.metadata
    assert (metadata['min_elbow_x']['left_grasp_frame'] >=
            metadata['elbow_local_x_bounds']['left_grasp_frame'] - 0.01)
    assert (metadata['hand_segment_deviation']['left_grasp_frame'] <=
            pipeline.planner.config.max_hand_deviation)
    assert 'left_elbow_link' in metadata['compact_radial_excess']
    assert 'left_wrist_yaw_link' in metadata['compact_radial_excess']
    assert metadata['minimum_self_clearance'] > pipeline.planner.config.self_collision_clearance
    assert metadata['excess_wrist_travel'] >= 0.0


def test_sphere_clearance_rejects_binary_free_near_contact():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    robot = pipeline.robot_model
    near_contact = np.array([
        -0.6097, -0.18, 0.1106, -0.2877, 0.2635, -0.015, -0.0555,
        -0.9543, 0.18, -0.0464, -0.1285, -0.1056, 0.1597, 0.1096,
    ])
    clearance = pipeline.planner.config.self_collision_clearance

    assert robot.check_collision_free_reduced(near_contact)
    assert not robot.check_collision_free_reduced(
        near_contact, min_distance=clearance,
    )
    assert robot.check_collision_free_reduced(
        np.zeros(14), min_distance=clearance,
    )
    result = pipeline.planner.plan(np.zeros(14), near_contact, direct_only=True)
    assert not result.success
    assert 'self-collision clearance' in result.reason


def test_named_joint_plans_do_not_inherit_locality_constraint():
    planner = ReducedJointPlanner(FakeRobotModel())
    start = np.array([0.0, 0.0])
    goal = np.array([0.5, 0.0])
    _, reason = planner._path_quality_failure(
        np.array([start, [0.25, 1.0], goal]), start, goal, (),
    )

    assert not reason


def test_reverse_path_reuse_revalidates_and_invalidates_on_context_change():
    isolated = ReducedJointPlanner(FakeRobotModel())
    isolated.plan([0.0, 0.0], [0.5, 0.5], allow_reuse=False)
    no_reuse = isolated.plan([0.5, 0.5], [0.0, 0.0], allow_reuse=False)
    assert no_reuse.success
    assert not no_reuse.metadata.get('cache_hit', False)

    planner = ReducedJointPlanner(FakeRobotModel())
    start = np.array([0.0, 0.0])
    goal = np.array([0.5, 0.5])
    forward = planner.plan(start, goal)

    reverse = planner.plan(goal, start)

    assert forward.success and reverse.success
    assert reverse.reason == 'direct path'
    assert not reverse.metadata.get('cache_hit', False)
    assert np.allclose(reverse.path, forward.path[::-1])

    planner.robot_model.model_body_reduced.velocityLimit[:] = 0.8
    changed_velocity = planner.plan(start, goal)
    assert changed_velocity.success
    assert not changed_velocity.metadata.get('cache_hit', False)

    planner.config.moving_speed *= 0.5
    changed_speed = planner.plan(start, goal)
    assert changed_speed.success
    assert not changed_speed.metadata.get('cache_hit', False)

    planner.rebind_reduced_model()
    changed_model = planner.plan(goal, start)
    assert changed_model.success
    assert not changed_model.metadata.get('cache_hit', False)


def test_reverse_cache_handles_a_blocked_direct_return():
    robot = FakeRobotModel()
    robot.check_collision_free_reduced = lambda q, min_distance=0.0: not (
        0.2 < q[0] < 0.3 and abs(q[1] - q[0]) < 0.04
    )
    planner = ReducedJointPlanner(robot, config=PlannerConfig(timeout=0.8))
    start = np.zeros(2)
    goal = np.array([0.5, 0.5])

    forward = planner.plan(start, goal)
    reverse = planner.plan(goal, start)

    assert forward.success, forward.reason
    assert forward.metadata['path_kind'] == 'ompl path'
    assert reverse.success, reverse.reason
    assert reverse.metadata['cache_hit']
    assert reverse.metadata['path_kind'] == 'validated reverse reuse'


def test_recent_path_cache_preserves_earlier_arm_route_across_candidate_search():
    planner = ReducedJointPlanner(FakeRobotModel())
    start = np.array([0.0, 0.0])
    goal = np.array([0.25, 0.1])
    assert planner.plan(start, goal).success
    for index in range(3):
        candidate = np.array([0.05 * (index + 1), 0.0])
        assert planner.plan(start, candidate).success
    assert any(
        np.array_equal(cached['start'], start) and
        np.array_equal(cached['goal'], goal)
        for cached in planner._recent_paths
    )

    reverse = planner.plan(goal, start)

    assert reverse.success and reverse.reason == 'direct path'
    for index in range(8):
        assert planner.plan(start, [0.02 * (index + 1), 0.0]).success
    assert len(planner._recent_paths) == planner.config.recent_path_cache_size


def test_nearby_reuse_retimes_and_revalidates_joint_connectors():
    robot = FakeRobotModel()
    planner = ReducedJointPlanner(robot, config=PlannerConfig(
        try_direct=False, timeout=0.2,
    ))
    start = np.zeros(2)
    prior_goal = np.array([0.4, 0.3])
    nearby_goal = np.array([0.42, 0.31])
    assert planner.plan(start, prior_goal).success

    reused = planner.plan(start, nearby_goal)

    assert reused.success, reused.reason
    assert reused.metadata['cache_type'] == 'nearby'
    assert reused.metadata['cache_hit']
    assert np.array_equal(reused.path[0], start)
    assert np.array_equal(reused.path[-1], nearby_goal)
    assert planner._path_validity_failure(reused.path) == ''

    robot.check_collision_free_reduced = lambda q, min_distance=0.0: not (
        0.405 < q[0] < 0.415
    )
    blocked = planner.plan(start, np.array([0.43, 0.31]))
    assert not blocked.metadata.get('cache_hit', False)


def test_plan_pins_idle_right_arm_when_goal_preserves_it():
    pytest.importorskip('ompl')
    lower = -np.ones(14)
    upper = np.ones(14)
    planner = ReducedJointPlanner(
        FakeRobotModel(lower=lower, upper=upper),
        config=PlannerConfig(timeout=0.2),
    )
    start = np.zeros(14)
    goal = np.zeros(14)
    start[7:] = 0.01
    goal[:7] = 0.5
    goal[7:] = start[7:]

    result = planner.plan(start, goal)

    assert result.success, result.reason
    assert np.allclose(result.path[:, 7:], start[7:])


def test_small_named_goal_is_not_silently_projected_away():
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ))
    start = np.zeros(14)
    goal = start.copy()
    goal[7] = 0.01

    result = planner.plan(start, goal)

    assert result.success, result.reason
    assert result.path[-1, 7] == pytest.approx(goal[7])


def test_named_single_arm_return_retries_local_search_after_timing_rejection():
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ), config=PlannerConfig(local_joint_padding=0.6))
    original = planner._plan_once
    paddings = []

    def reject_long_initial_route(*args, **kwargs):
        paddings.append(kwargs.get('local_joint_padding'))
        if kwargs.get('local_joint_padding') == 0.0:
            return PlanResult(success=False, path=np.empty((0, 14)),
                              reason='timing requires 907 points, exceeds max_interpolation_steps=600')
        return original(*args, **kwargs)

    planner._plan_once = reject_long_initial_route
    start = np.zeros(14)
    start[0] = 0.2
    result = planner.plan(
        start, np.zeros(14), local_frames=('left_grasp_frame',),
        local_joint_padding=0.0,
    )

    assert result.success, result.reason
    assert paddings == [0.0, 0.6]
    assert '907' in result.metadata['initial_failure']
    assert np.allclose(result.path[-1], 0.0)


def test_conflicting_explicit_active_mask_rejects_goal():
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ))
    goal = np.zeros(14)
    goal[7] = 0.01
    active = np.r_[np.ones(7, dtype=bool), np.zeros(7, dtype=bool)]

    result = planner.plan(np.zeros(14), goal, active_mask=active)

    assert not result.success
    assert 'inactive joints' in result.reason
    assert result.path.shape == (0, 14)


@pytest.mark.parametrize('setting', [
    {'frame_x_max': float('nan')},
    {'frame_z_min': {'left_grasp_frame': float('nan')}},
    {'frame_limit_margin': float('inf')},
    {'inactive_goal_tolerance': float('nan')},
])
def test_rejects_nonfinite_workspace_and_goal_policy(setting):
    with pytest.raises(ValueError, match='finite'):
        ReducedJointPlanner(FakeRobotModel(), config=PlannerConfig(**setting))


@pytest.mark.parametrize(('setting', 'reason'), [
    ({'frame_z_min': 0.6, 'frame_z_max': 0.5}, 'must not exceed'),
    ({'frame_z_min': {'misspelled_frame': 0.0}}, 'unknown frame names'),
])
def test_rejects_invalid_workspace_floor_configuration(setting, reason):
    with pytest.raises(ValueError, match=reason):
        ReducedJointPlanner(FakeRobotModel(), config=PlannerConfig(**setting))


def test_timing_cap_fails_instead_of_returning_fast_waypoints():
    planner = ReducedJointPlanner(
        FakeRobotModel(), config=PlannerConfig(max_interpolation_steps=10)
    )
    result = planner.plan([0.0, 0.0], [0.5, 0.5], direct_only=True)

    assert not result.success
    assert result.path.shape == (0, 2)
    assert result.metadata['required_points'] > 10
    assert 'max_interpolation_steps=10' in result.reason


def test_slow_speed_limits_actual_frame_motion():
    speed = 0.025
    planner = ReducedJointPlanner(
        FakeRobotModel(), config=PlannerConfig(
            moving_speed=speed, max_interpolation_steps=500,
        )
    )
    result = planner.plan([0.0, 0.0], [0.1, 0.0], direct_only=True)

    assert result.success, result.reason
    assert result.metadata['actual_max_frame_speed'] <= speed + 1e-9
    assert result.metadata['actual_max_joint_speed'] <= 1.0 + 1e-9
    assert result.metadata['actual_max_joint_acceleration'] <= 25.0 + 1e-9
    assert result.metadata['duration'] == pytest.approx(
        (len(result.path) - 1) * planner.config.dt
    )


def test_rotation_sweep_contributes_to_frame_speed():
    robot = FakeRobotModel()
    robot.get_frame_position = lambda name, q: np.zeros(3)
    robot.get_frame_rotation = lambda name, q: np.array([
        [np.cos(q[0]), -np.sin(q[0]), 0.0],
        [np.sin(q[0]), np.cos(q[0]), 0.0],
        [0.0, 0.0, 1.0],
    ])
    planner = ReducedJointPlanner(
        robot, config=PlannerConfig(moving_speed=0.02)
    )
    result = planner.plan([0.0, 0.0], [0.2, 0.0], direct_only=True)

    assert result.success, result.reason
    assert 0.0 < result.metadata['actual_max_frame_speed'] <= 0.02 + 1e-9
    assert result.metadata['frame_rotation_lengths']['left_grasp_frame'] > 0.0


def test_zero_motion_fits_two_point_cap():
    planner = ReducedJointPlanner(
        FakeRobotModel(), config=PlannerConfig(max_interpolation_steps=2)
    )
    result = planner.plan([0.0, 0.0], [0.0, 0.0], direct_only=True)

    assert result.success, result.reason
    assert result.path.shape == (2, 2)


def test_sharp_corner_timing_preserves_sparse_vertices_and_acceleration():
    planner = ReducedJointPlanner(
        FakeRobotModel(), config=PlannerConfig(
            moving_speed=2.0, joint_acceleration_limit=2.0,
            max_interpolation_steps=500,
        )
    )
    sparse = np.array([[0.0, 0.0], [0.5, 0.0], [0.5, 0.5]])
    path, metadata, failure = planner._timed_path(sparse)

    assert not failure
    assert np.array_equal(path[0], sparse[0])
    assert np.array_equal(path[-1], sparse[-1])
    assert any(np.array_equal(q, sparse[1]) for q in path)
    assert metadata['actual_max_joint_acceleration'] <= 2.0 + 1e-9
    assert metadata['actual_max_frame_speed'] <= 2.0 + 1e-9


def test_pchip_proposal_is_continuous_and_fully_validated():
    planner = ReducedJointPlanner(
        FakeRobotModel(), config=PlannerConfig(moving_speed=1.0)
    )
    sparse = np.array([[0.0, 0.0], [0.25, 0.15], [0.5, 0.35]])
    path, metadata, failure = planner._select_timed_path(
        sparse, sparse[0], sparse[-1], (),
    )

    assert not failure
    assert metadata['time_profile'] == 'pchip'
    assert np.array_equal(path[0], sparse[0])
    assert np.array_equal(path[-1], sparse[-1])
    vertex = next(idx for idx, q in enumerate(path) if np.array_equal(q, sparse[1]))
    assert np.linalg.norm(path[vertex] - path[vertex - 1]) > 1e-5
    assert np.linalg.norm(path[vertex + 1] - path[vertex]) > 1e-5
    assert planner._path_validity_failure(path) == ''
    assert metadata['actual_max_joint_acceleration'] <= 25.0 + 1e-9
    assert metadata['actual_max_frame_speed'] <= 1.0 + 1e-9


def test_pchip_collision_rejection_falls_back_to_validated_segments():
    robot = FakeRobotModel()
    robot.check_collision_free_reduced = lambda q, min_distance=0.0: not (
        0.1 < q[0] < 0.4 and q[1] > q[0] + 0.02
    )
    planner = ReducedJointPlanner(robot, config=PlannerConfig(
        moving_speed=1.0, frame_z_max=None, frame_x_max=None,
    ))
    sparse = np.array([[0.0, 0.0], [0.5, 0.5], [0.8, 0.5]])
    path, metadata, failure = planner._select_timed_path(
        sparse, sparse[0], sparse[-1], (),
    )

    assert not failure
    assert metadata['time_profile'] == 'segment'
    assert 'collision' in metadata['pchip_rejection']
    assert planner._path_validity_failure(path) == ''


def test_pchip_cap_rejection_uses_safe_segment_fallback():
    planner = ReducedJointPlanner(FakeRobotModel(), config=PlannerConfig(
        joint_acceleration_limit=2.0, max_interpolation_steps=2000,
        max_staged_interpolation_steps=4000,
    ))
    sparse = np.array([[0.0, 0.0], [0.25, 0.15], [0.5, 0.35]])
    fallback, _, failure = planner._timed_path(sparse)
    assert not failure
    slower, slower_metadata, failure = planner._timed_path(sparse, profile='pchip')
    assert not failure
    assert len(slower) > len(fallback)
    selected, selected_metadata, failure = planner._select_timed_path(
        sparse, sparse[0], sparse[-1], (),
    )
    assert not failure
    assert selected_metadata['pchip_rejection'] == 'segment timing is shorter'
    assert np.array_equal(selected, fallback)
    planner.config.max_interpolation_steps = len(fallback)
    proposal, _, rejection = planner._timed_path(sparse, profile='pchip')
    assert proposal is None
    assert 'max_interpolation_steps' in rejection

    path, metadata, failure = planner._select_timed_path(
        sparse, sparse[0], sparse[-1], (),
    )

    assert not failure
    assert metadata['time_profile'] == 'segment'
    assert np.array_equal(path, fallback)


def test_real_robot_timing_respects_reduced_velocity_limits():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    planner = pipeline.planner
    pipeline.robot_model.model_body_reduced.velocityLimit[0] = 0.2
    start = np.asarray(pipeline.robot_model.state['q'])[
        pipeline.robot_model.reduced_mask
    ].copy()
    goal = start.copy()
    goal[0] += 0.1
    path, metadata, failure = planner._timed_path(np.vstack([start, goal]))

    assert not failure
    assert np.array_equal(path[0], start)
    assert np.array_equal(path[-1], goal)
    assert metadata['actual_max_joint_speed'] <= min(
        np.broadcast_to(planner.config.joint_velocity_limit, (planner.nq,))[0],
        pipeline.robot_model.model_body_reduced.velocityLimit[0],
    ) + 1e-9
    assert metadata['actual_max_frame_speed'] <= planner.config.moving_speed + 1e-9


def test_path_validation_rejects_z_corridor_dip():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(
            constraint_check_steps=2,
            frame_z_max=None,
        ),
    )
    start = np.array([1.0, 0.0])
    goal = np.array([1.0, 0.0])
    path = np.array([
        start,
        [0.9, 0.0],
        goal,
    ])

    planner._set_workspace_corridor(start, goal)
    reason = planner._path_validity_failure(path)

    assert 'below z=' in reason


def test_z_corridor_uses_its_own_margin():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(
            frame_limit_margin=0.01,
            frame_z_corridor_margin=0.05,
        ),
    )
    start = np.array([1.0, 0.0])

    planner._set_workspace_corridor(start, start)

    assert planner.corridor_min_z['left_grasp_frame'] == pytest.approx(0.95)


def test_state_validation_rejects_y_limit_violation():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(
            # FakeRobotModel maps both x and y to q[1], so the x limit has to
            # be off for this to isolate the y constraint
            frame_x_max=None,
            frame_y_min=-0.5,
            frame_y_max=0.5,
            frame_limit_margin=0.01,
        ),
    )

    result = planner.plan([0.0, 0.52], [0.0, 0.0])

    assert not result.success
    assert result.reason.startswith('start moves left_grasp_frame above y=0.5000')


def test_state_validation_allows_y_limit_margin():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(
            # see above: x must be unconstrained to isolate the y margin
            frame_x_max=None,
            frame_y_min=-0.5,
            frame_y_max=0.5,
            frame_limit_margin=0.01,
        ),
    )

    reason = planner._validity_failure(np.array([0.0, 0.51]), 'state')

    assert reason == ''


def test_state_validation_rejects_x_limit_violation():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(
            frame_x_max=0.7,
            frame_limit_margin=0.01,
        ),
    )

    result = planner.plan([0.0, 0.72], [0.0, 0.0])

    assert not result.success
    assert result.reason.startswith('start moves left_grasp_frame beyond x=0.7000')


def test_state_validation_rejects_z_limit_violation():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(
            frame_z_max=0.5,
            frame_limit_margin=0.01,
        ),
    )

    result = planner.plan([0.52, 0.0], [0.0, 0.0])

    assert not result.success
    assert result.reason.startswith('start moves left_grasp_frame above z=0.5000')
    assert planner._validity_failure(np.array([0.51, 0.0]), 'state') == ''


def test_returned_segment_validation_checks_collision_between_waypoints():
    pytest.importorskip('ompl')
    robot = FakeRobotModel()
    robot.check_collision_free_reduced = (
        lambda q, min_distance=0.0: not (0.4 < q[0] < 0.6)
    )
    planner = ReducedJointPlanner(
        robot,
        config=PlannerConfig(constraint_check_steps=1, frame_z_max=None),
    )
    start = np.array([0.0, 0.0])
    goal = np.array([1.0, 0.0])
    planner._set_workspace_corridor(start, goal)

    assert 'path segment 0 violates self-collision clearance' == planner._path_validity_failure(
        np.array([start, goal])
    )


def test_direct_result_reports_real_stage_timings():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(FakeRobotModel())

    result = planner.plan([0.0, 0.0], [0.2, 0.0])

    assert result.success
    assert result.planning_time > 0.0
    assert result.metadata['stage_timings']['total'] == result.planning_time
    assert result.metadata['stage_timings']['direct_validation'] >= 0.0


def test_endpoint_failure_reports_real_stage_timings():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(FakeRobotModel())

    result = planner.plan([2.0, 0.0], [0.0, 0.0])

    assert not result.success
    assert result.planning_time > 0.0
    assert result.metadata['stage_timings']['setup'] > 0.0
    assert result.metadata['stage_timings']['total'] == result.planning_time


def test_ompl_timeout_reports_real_stage_timings():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(), config=PlannerConfig(try_direct=False)
    )
    setup = planner.ss
    try:
        planner.ss = SimpleNamespace(
            clear=lambda: None,
            setPlanner=lambda planner: None,
            setStartAndGoalStates=lambda start, goal: None,
            solve=lambda timeout: False,
        )
        result = planner.plan([0.0, 0.0], [0.2, 0.0])
    finally:
        planner.ss = setup

    assert not result.success
    assert result.planning_time > 0.0
    assert result.metadata['stage_timings']['ompl'] >= 0.0
    assert result.metadata['stage_timings']['total'] == result.planning_time


def test_rebind_reduced_model_refreshes_bounds():
    pytest.importorskip('ompl')
    robot = FakeRobotModel()
    planner = ReducedJointPlanner(robot)
    robot.model_body_reduced.upperPositionLimit = np.array([0.5, 1.0])

    planner.rebind_reduced_model()

    assert planner.space.getBounds().high[0] == pytest.approx(0.5)
    assert planner.space.getBounds().high[1] == pytest.approx(1.0)


@pytest.fixture
def deadline_clock(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(reduced_joint_planner, 'perf_counter', lambda: clock[0])
    return clock


def test_expired_initial_deadline_does_no_planning_work(deadline_clock, monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel())
    monkeypatch.setattr(planner, '_plan_once', lambda *args, **kwargs: pytest.fail(
        'expired request must not start its initial attempt',
    ))
    result = planner.plan([0.0, 0.0], [0.2, 0.0], deadline=0.0)
    assert not result.success
    assert 'deadline exceeded' in result.reason
    assert result.path.shape == (0, 2)
    assert planner._recent_paths == []
    assert planner._deadline is None


def test_initial_direct_retiming_stops_on_deadline_and_restores_scope(
        deadline_clock, monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel())
    poses = planner._frame_poses
    calls = []

    def expensive_poses(*args, **kwargs):
        calls.append(True)
        deadline_clock[0] += 0.01
        return poses(*args, **kwargs)

    monkeypatch.setattr(planner, '_frame_poses', expensive_poses)
    result = planner.plan([0.0, 0.0], [0.2, 0.0], deadline=0.05)
    assert not result.success
    assert 'deadline exceeded' in result.reason
    assert len(calls) == 5
    assert result.path.shape == (0, 2)
    assert not planner._recent_paths
    assert planner._deadline is None
    monkeypatch.setattr(planner, '_frame_poses', poses)
    assert planner.plan([0.0, 0.0], [0.2, 0.0]).success


@pytest.mark.parametrize('phase', ['validity', 'clearance'])
def test_initial_direct_validation_expiry_never_accepts_or_caches_partial_path(
        deadline_clock, monkeypatch, phase):
    robot = FakeRobotModel()
    planner = ReducedJointPlanner(robot)
    samples = np.linspace([0.0, 0.0], [0.2, 0.0], 20)
    monkeypatch.setattr(planner, '_timed_path', lambda *args, **kwargs: (
        samples.copy(), {}, '',
    ))
    calls = []

    def expensive_check(q, min_distance=0.0):
        calls.append(q.copy())
        deadline_clock[0] += 0.1
        return True if phase == 'validity' else 0.1

    if phase == 'validity':
        robot.check_collision_free_reduced = expensive_check
    else:
        robot.minimum_self_clearance_reduced = expensive_check
    result = planner.plan(samples[0], samples[-1], deadline=0.35)
    assert not result.success
    assert 'deadline exceeded' in result.reason
    assert result.path.shape == (0, 2)
    assert len(calls) == 4
    assert not planner._recent_paths


@pytest.mark.parametrize('phase', ['solve', 'simplify'])
def test_native_work_is_capped_by_remaining_initial_deadline(
        deadline_clock, monkeypatch, phase):
    planner = ReducedJointPlanner(FakeRobotModel(), PlannerConfig(
        try_direct=False, timeout=2.0, simplify_time=1.0,
    ))
    budgets = []
    path = SimpleNamespace(getStates=lambda: [[0.0, 0.0], [0.2, 0.0]])

    def solve(budget):
        budgets.append(('solve', budget))
        deadline_clock[0] += budget if phase == 'solve' else 0.1
        return phase == 'simplify'

    def simplify(budget):
        budgets.append(('simplify', budget))
        deadline_clock[0] += budget

    monkeypatch.setattr(planner, 'ss', SimpleNamespace(
        clear=lambda: None, setPlanner=lambda planner: None,
        setStartAndGoalStates=lambda *args: None,
        solve=solve, haveExactSolutionPath=lambda: True,
        getSolutionPath=lambda: path, simplifySolution=simplify,
    ))
    result = planner.plan([0.0, 0.0], [0.2, 0.0], deadline=0.25)
    assert not result.success
    assert result.path.shape == (0, 2)
    assert budgets[0] == ('solve', 0.25)
    if phase == 'simplify':
        assert budgets[1][0] == 'simplify'
        assert budgets[1][1] == pytest.approx(0.15)
    assert not planner._recent_paths
    assert planner.space.getBounds().low[0] == planner.model_lower[0]


@pytest.mark.parametrize('reuse', ['reverse', 'nearby'])
def test_reuse_validation_expires_without_accepting_cached_path(
        deadline_clock, monkeypatch, reuse):
    planner = ReducedJointPlanner(FakeRobotModel())
    start, goal = np.zeros(2), np.array([0.2, 0.0])
    assert planner.plan(start, goal).success
    planner.config.try_direct = False
    planner._recent_paths[0]['config'] = planner._path_context(())[0]
    original_cache = planner._recent_paths.copy()
    calls = []

    def expensive_collision(q, min_distance=0.0):
        calls.append(True)
        deadline_clock[0] += 0.1
        return True

    planner.robot_model.check_collision_free_reduced = expensive_collision
    monkeypatch.setattr(planner, 'ss', SimpleNamespace(
        clear=lambda: None,
        solve=lambda *args: pytest.fail('expired reuse must not start OMPL'),
    ))
    first, last = ((goal, start) if reuse == 'reverse' else
                   (start, goal + np.array([0.01, 0.0])))
    result = planner.plan(first, last, deadline=0.35)
    assert not result.success
    assert len(calls) == 4
    assert not result.metadata.get('cache_hit')
    assert planner._recent_paths == original_cache


def test_validated_pchip_survives_optional_segment_refinement_expiry(
        deadline_clock, monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel(), PlannerConfig(try_direct=False))
    sparse = np.array([[0.0, 0.0], [0.1, 0.0], [0.2, 0.0]])
    calls = []

    def collision(q, min_distance=0.0):
        deadline_clock[0] += 0.001
        return True

    planner.robot_model.check_collision_free_reduced = collision

    def timed(path, profile='segment'):
        calls.append(profile)
        if profile == 'segment':
            deadline_clock[0] = 1.0
            planner._check_deadline()
        return path.copy(), {'time_profile': profile, 'duration': 1.0}, ''

    monkeypatch.setattr(planner, '_timed_path', timed)
    monkeypatch.setattr(planner, 'ss', SimpleNamespace(
        clear=lambda: None, setPlanner=lambda planner: None,
        setStartAndGoalStates=lambda *args: None,
        solve=lambda budget: True, haveExactSolutionPath=lambda: True,
        getSolutionPath=lambda: SimpleNamespace(getStates=lambda: sparse),
    ))
    monkeypatch.setattr(planner, '_shorten_path', lambda path: None)
    result = planner.plan(sparse[0], sparse[-1], deadline=0.5)
    assert result.success, result.reason
    assert np.array_equal(result.path, sparse)
    assert result.metadata['time_profile'] == 'pchip'
    assert 'joint_path_length' in result.metadata
    timings = result.metadata['stage_timings']
    assert 0.0 < timings['validation'] < timings['total']
    assert timings['postprocess'] >= 0.0
    assert calls == ['pchip', 'segment']
    assert np.array_equal(planner._recent_paths[0]['path'], sparse)
    assert planner._deadline is None


def test_coordination_validation_checks_its_own_deadline(deadline_clock, monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ))
    start = np.zeros(14)
    goal = start.copy()
    goal[[0, 7]] = 0.1
    legs = []
    for arm in (0, 1):
        endpoint = start.copy()
        endpoint[arm * 7] = 0.1
        legs.append(PlanResult(True, np.linspace(start, endpoint, 10)))

    def expensive_collision(q, min_distance=0.0):
        deadline_clock[0] += 0.1
        return True

    planner.robot_model.check_collision_free_reduced = expensive_collision
    result, reason = planner._coordinate_arm_paths(
        start, goal, None, planner.frame_names, legs, 0.0, 0.25,
    )
    assert result is None
    assert reason == 'coordination budget exhausted'
    assert deadline_clock[0] == pytest.approx(0.3)
    assert planner._deadline is None


def test_validated_staged_path_survives_optional_coordination_expiry(
        deadline_clock, monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ), PlannerConfig(
        prefer_coordinated_motion=False, bimanual_refinement_duration=0.001,
    ))
    original = planner._plan_once

    def single_arm_only(start, goal, **kwargs):
        if np.any(start[:7] != goal[:7]) and np.any(start[7:] != goal[7:]):
            return PlanResult(False, np.empty((0, 14)), reason='combined route blocked')
        return original(start, goal, **kwargs)

    def expensive_refinement(*args):
        deadline_clock[0] = 1.0
        planner._check_deadline()

    monkeypatch.setattr(planner, '_plan_once', single_arm_only)
    monkeypatch.setattr(planner, '_coordinate_arm_paths_impl', expensive_refinement)
    goal = np.zeros(14)
    goal[[0, 7]] = 0.1
    result = planner.plan(
        np.zeros(14), goal, local_frames=planner.frame_names, deadline=0.5,
    )
    assert result.success, result.reason
    assert result.metadata['path_kind'] == 'staged two-arm path'
    assert np.array_equal(result.path[-1], goal)
    assert result.planning_time == 1.0
    assert result.metadata['stage_timings']['total'] == 1.0
    assert planner._deadline is None


def test_initial_edge_validation_expiry_is_not_a_valid_path(deadline_clock, monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel())
    samples = np.array([[0.0, 0.0], [0.2, 0.0]])
    monkeypatch.setattr(planner, '_timed_path', lambda *args, **kwargs: (
        samples.copy(), {}, '',
    ))
    calls = []

    def collision(q, min_distance=0.0):
        calls.append(q.copy())
        deadline_clock[0] += 0.1
        return True

    planner.robot_model.check_collision_free_reduced = collision
    result = planner.plan(samples[0], samples[-1], deadline=0.55)
    assert not result.success
    assert result.path.shape == (0, 2)
    assert 0.0 < calls[-1][0] < 0.2
    assert len(calls) == 6
    assert not planner._recent_paths


@pytest.mark.parametrize('named, deadline', [
    (False, None), (True, None), (False, 2.0), (False, 20.0),
])
def test_initial_attempt_honors_configured_and_explicit_deadline(
        deadline_clock, monkeypatch, named, deadline):
    planner = ReducedJointPlanner(FakeRobotModel(), PlannerConfig(
        candidate_planning_budget=4.0, named_planning_budget=6.0,
    ))
    samples = np.array([[0.0, 0.0], [0.2, 0.0]])
    monkeypatch.setattr(planner, '_timed_path', lambda *args, **kwargs: (
        samples.copy(), {}, '',
    ))

    def expensive_collision(q, min_distance=0.0):
        deadline_clock[0] += 1.0
        return True

    planner.robot_model.check_collision_free_reduced = expensive_collision
    result = planner.plan(
        samples[0], samples[-1], local_joint_padding=0.0 if named else None,
        deadline=deadline,
    )
    budget = (planner.config.named_planning_budget if named else
              planner.config.candidate_planning_budget)
    expected = budget if deadline is None else min(budget, deadline)
    assert not result.success
    assert 'deadline exceeded' in result.reason
    assert result.path.shape == (0, 2)
    assert deadline_clock[0] == expected
    assert not planner._recent_paths
    assert planner._deadline is None


@pytest.mark.parametrize('reuse', ['nearby', 'reverse'])
def test_cache_rejects_changed_inactive_pins_below_nearby_tolerance(reuse):
    planner = ReducedJointPlanner(FakeRobotModel())
    mask = np.array([True, False])
    start, goal = np.zeros(2), np.array([0.2, 0.0])
    assert planner.plan(start, goal, active_mask=mask).success
    assert len(planner._recent_paths) == 1
    old_path = planner._recent_paths[0]['path'].copy()
    start[1] = goal[1] = 0.05
    assert 0.05 < planner.config.nearby_reuse_joint_tolerance
    if reuse == 'reverse':
        start, goal = goal, start
    planner._set_active_mask(mask, start)
    planner._set_workspace_corridor(start, goal)
    method = (planner._nearby_previous_path if reuse == 'nearby' else
              planner._reversed_previous_path)

    assert method(start, goal, ()) is None
    result = planner.plan(start, goal, active_mask=mask)

    assert result.success, result.reason
    assert not result.metadata.get('cache_hit', False)
    assert np.array_equal(result.path[:, 1], np.full(len(result.path), 0.05))
    assert np.array_equal(planner._recent_paths[0]['path'], old_path)


@pytest.mark.parametrize('route', [
    'direct', 'nearby', 'reverse', 'ompl', 'coordinated', 'staged',
])
def test_final_path_validation_rejects_inactive_waypoint_motion(monkeypatch, route):
    planner = ReducedJointPlanner(FakeRobotModel())
    start, goal = np.zeros(2), np.array([0.2, 0.0])
    bad_path = np.array([start, [0.1, 0.05], goal])
    mask = np.array([True, False])
    if route == 'direct':
        monkeypatch.setattr(planner, '_timed_path', lambda *args, **kwargs: (
            bad_path.copy(), {}, '',
        ))
    elif route in ('nearby', 'reverse'):
        planner.config.try_direct = False
        reused = PlanResult(True, bad_path.copy())
        method = ('_nearby_previous_path' if route == 'nearby' else
                  '_reversed_previous_path')
        monkeypatch.setattr(planner, method, lambda *args: reused)
    elif route in ('coordinated', 'staged'):
        def combined(*args):
            planner._set_active_mask(mask, start)
            return PlanResult(True, bad_path.copy(), metadata={'path_kind': route})

        monkeypatch.setattr(planner, '_plan', combined)
    else:
        planner.config.try_direct = False
        monkeypatch.setattr(planner, 'ss', SimpleNamespace(
            clear=lambda: None, setPlanner=lambda planner: None,
            setStartAndGoalStates=lambda *args: None,
            solve=lambda budget: True, haveExactSolutionPath=lambda: True,
            getSolutionPath=lambda: SimpleNamespace(getStates=lambda: [start, goal]),
        ))
        monkeypatch.setattr(planner, '_shorten_path', lambda path: None)
        monkeypatch.setattr(planner, '_select_timed_path', lambda *args: (
            bad_path.copy(), {}, '',
        ))

    result = planner.plan(start, goal, active_mask=mask, direct_only=route == 'direct')

    assert not result.success
    assert 'inactive joints' in result.reason
    assert result.path.shape == (0, 2)
    assert not planner._recent_paths


def test_edge_validation_spacing_does_not_inherit_prior_bounds_or_mask():
    planner = ReducedJointPlanner(FakeRobotModel(), PlannerConfig(
        constraint_check_steps=1, frame_z_max=None,
    ))
    path = np.array([[0.0, 0.0], [0.2, 0.0]])
    samples = []
    planner.robot_model.check_collision_free_reduced = lambda q, **kwargs: (
        samples.append(q.copy()) or True
    )
    for prior_mask, padding in ((None, 0.0), ([False, True], 0.01)):
        planner._set_active_mask(prior_mask, path[0])
        planner._apply_active_bounds(path[0], path[-1], padding)
        planner._set_active_mask([True, False], path[0])
        samples.clear()
        assert planner._path_validity_failure(path) == ''
        if prior_mask is None:
            expected = np.array(samples)
        else:
            assert np.array_equal(samples, expected)
    assert len(expected) == 41
    assert np.max(np.diff(np.sort(expected[:, 0]))) <= 0.005 + 1e-12


@pytest.mark.parametrize('rejections', [
    ['path detour ratio exceeds local limit', ''],
    ['timing requires too many points', ''],
    ['', 'must not search global'],
    ['local quality rejected', 'global timing rejected'],
])
def test_region_search_requires_full_acceptance(monkeypatch, deadline_clock, rejections):
    planner = ReducedJointPlanner(FakeRobotModel(), PlannerConfig(
        try_direct=False, simplify=False, timeout=1.0, local_joint_padding=0.1,
    ))
    sparse = np.array([[0.0, 0.0], [0.2, 0.1]])
    budgets = []
    validations = []

    def solve(budget):
        budgets.append(budget)
        deadline_clock[0] += 0.1
        return True

    def validate(*args):
        failure = rejections[len(validations)]
        validations.append(True)
        deadline_clock[0] += 0.1
        return sparse.copy(), {'validation_time': 0.1}, failure

    monkeypatch.setattr(planner, 'ss', SimpleNamespace(
        clear=lambda: None, setPlanner=lambda planner: None,
        setStartAndGoalStates=lambda *args: None, solve=solve,
        haveExactSolutionPath=lambda: True,
        getSolutionPath=lambda: SimpleNamespace(getStates=lambda: sparse),
    ))
    monkeypatch.setattr(planner, '_select_timed_path', validate)
    result = planner._plan_once(
        *sparse, local_frames=planner.frame_names, deadline=0.45,
    )
    expected = 2 if rejections[0] else 1
    assert len(budgets) == len(validations) == expected
    assert result.success == (not rejections[expected - 1])
    assert budgets[0] <= 0.45
    if expected == 2:
        assert budgets[1] <= 0.25 + 1e-12
        assert result.metadata['region_failures']['local'] == rejections[0]
        assert result.metadata['stage_timings']['validation'] == pytest.approx(0.2)
    if result.success:
        assert result.metadata['search_region'] == ('local' if expected == 1 else 'global')
        assert np.array_equal(planner._recent_paths[-1]['path'], result.path)
    else:
        assert all(reason in result.reason for reason in rejections)
        assert not planner._recent_paths


def test_rejected_local_preserves_reason_when_global_deadline_expires(
        monkeypatch, deadline_clock):
    planner = ReducedJointPlanner(FakeRobotModel(), PlannerConfig(
        try_direct=False, simplify=False, timeout=1.0, local_joint_padding=0.1,
    ))
    sparse = np.array([[0.0, 0.0], [0.2, 0.1]])
    budgets = []

    def solve(budget):
        budgets.append(budget)
        deadline_clock[0] += 0.1 if len(budgets) == 1 else budget
        return True

    def reject(*args):
        deadline_clock[0] += 0.1
        return None, {}, 'local quality rejected'

    monkeypatch.setattr(planner, 'ss', SimpleNamespace(
        clear=lambda: None, setPlanner=lambda planner: None,
        setStartAndGoalStates=lambda *args: None, solve=solve,
        haveExactSolutionPath=lambda: True,
        getSolutionPath=lambda: SimpleNamespace(getStates=lambda: sparse),
    ))
    monkeypatch.setattr(planner, '_select_timed_path', reject)
    result = planner._plan_once(*sparse, local_frames=planner.frame_names, deadline=0.3)
    assert not result.success
    assert 'local quality rejected' in result.reason
    assert 'deadline exceeded' in result.reason
    assert budgets[1] == pytest.approx(0.1)
    assert deadline_clock[0] == pytest.approx(0.3)
    assert result.metadata['stage_timings']['ompl_global'] == pytest.approx(0.1)
    assert planner._deadline is None
    assert not planner._recent_paths


@pytest.mark.parametrize('outcome', ['better', 'invalid', 'worse', 'slower', 'expiry'])
def test_compaction_retains_valid_fallback_unless_quality_improves(
        monkeypatch, deadline_clock, outcome):
    planner = ReducedJointPlanner(FakeRobotModel())
    sparse = np.array([[0.0, 0.0], [0.3, 0.4], [0.5, 0.0]])
    planner._set_active_mask([True, False], sparse[0])
    sparse[:, 1] = 0.0
    metadata = {'joint_path_length': 2.0, 'duration': 2.0,
                'compact_radial_excess': {'elbow': 0.2}}
    candidate_metadata = {'joint_path_length': 1.0, 'duration': 2.0,
                          'compact_radial_excess': {'elbow': 0.1}}
    candidate = sparse.copy()
    candidate[1, 0] = 0.2
    calls = []

    def validate(proposal, *args):
        calls.append(proposal.copy())
        assert np.array_equal(proposal[[0, -1]], sparse[[0, -1]])
        assert np.array_equal(proposal[:, 1], sparse[:, 1])
        if outcome == 'expiry':
            deadline_clock[0] = 0.2
            planner._check_deadline()
        if outcome == 'worse':
            candidate_metadata['compact_radial_excess']['elbow'] = 0.3
        if outcome == 'slower':
            candidate_metadata['duration'] = 2.1
        return candidate, candidate_metadata, 'rejected' if outcome == 'invalid' else ''

    # make the middle knot non-collinear in the active coordinates
    sparse[1, 0] = 0.6
    monkeypatch.setattr(planner, '_path_validity_failure', lambda path: '')
    monkeypatch.setattr(planner, '_select_timed_path', validate)
    with planner._deadline_scope(0.2):
        path, quality = planner._compact_path(
            sparse, sparse, metadata, sparse[0], sparse[-1], (),
        )
    assert len(calls) == 1
    assert quality['compaction_attempts'] == 1
    assert quality['compaction_accepted'] == (outcome == 'better')
    assert path is (candidate if outcome == 'better' else sparse)
    assert planner._deadline is None


def test_compaction_skips_already_compact_path(monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel())
    sparse = np.array([[0., 0.], [0.3, 0.2], [0.5, 0.]])
    metadata = {'compact_radial_excess': {'elbow': 0.0}}
    monkeypatch.setattr(planner, '_select_timed_path',
                        lambda *args: pytest.fail('compact fast path must not refine'))
    path, quality = planner._compact_path(
        sparse, sparse, metadata, sparse[0], sparse[-1], (),
    )
    assert path is sparse and quality is metadata


def test_compaction_work_cap_and_expired_budget_keep_original(monkeypatch, deadline_clock):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(20), upper=np.ones(20),
    ))
    sparse = np.array([np.zeros(20), np.full(20, 0.6), np.full(20, 0.5)])
    metadata = {'joint_path_length': 5.0, 'duration': 2.0,
                'compact_radial_excess': {'elbow': 0.2}}
    checks = []
    monkeypatch.setattr(planner, '_path_validity_failure',
                        lambda path: checks.append(path.copy()) or '')
    monkeypatch.setattr(planner, '_select_timed_path', lambda *args: (None, {}, 'invalid'))
    path, quality = planner._compact_path(
        sparse, sparse, metadata, sparse[0], sparse[-1], (),
    )
    assert path is sparse
    assert len(checks) == quality['compaction_attempts'] == 14
    checks.clear()

    def slow_validation(path):
        checks.append(path.copy())
        deadline_clock[0] += 0.1
        planner._check_deadline()
        return ''

    monkeypatch.setattr(planner, '_path_validity_failure', slow_validation)
    path, quality = planner._compact_path(
        sparse, sparse, metadata, sparse[0], sparse[-1], (),
    )
    assert path is sparse and not quality['compaction_accepted']
    assert len(checks) == 5
    assert deadline_clock[0] == pytest.approx(0.5)
    assert planner._deadline is None


def test_local_global_searches_get_fresh_planners_and_current_resolution(monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel(), PlannerConfig(
        try_direct=False, simplify=False, timeout=0.2, local_joint_padding=0.1,
    ))
    instances = []
    searches = []
    current = [None]

    def fresh_planner():
        instance = SimpleNamespace(range=0.0, start_tree=True)
        instances.append(instance)
        return instance

    def solve(budget):
        instance = current[0]
        assert instance.range == 0.0 and instance.start_tree
        extent = planner.space.getMaximumExtent()
        assert planner.space.getLongestValidSegmentLength() == pytest.approx(
            extent * planner.config.validity_resolution,
        )
        searches.append((instance, extent, budget))
        instance.range = extent * 0.2
        instance.start_tree = False
        return False

    monkeypatch.setattr(planner, '_make_planner', fresh_planner)
    monkeypatch.setattr(planner, 'ss', SimpleNamespace(
        clear=lambda: None, setPlanner=lambda instance: current.__setitem__(0, instance),
        setStartAndGoalStates=lambda *args: None, solve=solve,
    ))

    result = planner.plan([0.0, 0.0], [0.2, 0.0], active_mask=[True, False],
                          local_frames=('left_grasp_frame',), allow_reuse=False)

    assert not result.success
    assert len(instances) == 2
    assert searches[0][1:] == pytest.approx((0.4, 0.07))
    assert searches[1][1:] == pytest.approx((2.0, 0.13))
    assert planner.space.getLongestValidSegmentLength() == pytest.approx(
        np.linalg.norm(planner.model_upper - planner.model_lower) *
        planner.config.validity_resolution,
    )


@pytest.mark.parametrize('failure', ['timeout', 'deadline', 'exception', 'interrupt'])
def test_failed_bimanual_search_cannot_contaminate_single_arm_or_home(
        monkeypatch, deadline_clock, failure):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ), PlannerConfig(try_direct=False, simplify=False, local_joint_padding=0.1))
    pending = []
    clears = []

    def clear():
        pending.clear()
        clears.append(True)

    def solve(budget):
        pending.append('retained tree')
        if failure == 'deadline':
            deadline_clock[0] = 1.0
        elif failure == 'exception':
            raise RuntimeError('native search failed')
        elif failure == 'interrupt':
            raise KeyboardInterrupt()
        return False

    monkeypatch.setattr(planner, 'ss', SimpleNamespace(
        clear=clear, setPlanner=lambda instance: None,
        setStartAndGoalStates=lambda *args: None, solve=solve,
    ))
    goal = np.zeros(14)
    goal[[0, 7]] = 0.1
    kwargs = dict(local_frames=planner.frame_names, deadline=0.5)
    if failure in ('exception', 'interrupt'):
        with pytest.raises(RuntimeError if failure == 'exception' else KeyboardInterrupt):
            planner.plan(np.zeros(14), goal, **kwargs)
    else:
        assert not planner.plan(np.zeros(14), goal, **kwargs).success

    assert clears and not pending
    assert planner._deadline is None
    assert np.all(planner.active_mask)
    assert np.array_equal(planner.pinned_q, np.zeros(14))
    assert not planner.corridor_min_z and not planner.local_min_x
    assert not planner.local_elbow_min_x
    assert not planner._recent_paths
    bounds = planner.space.getBounds()
    assert np.array_equal(bounds.low, planner.model_lower)
    assert np.array_equal(bounds.high, planner.model_upper)

    planner.config.try_direct = True
    start = np.zeros(14)
    start[7:] = 0.05
    goal = start.copy()
    goal[0] = 0.1
    single = planner.plan(start, goal, deadline=deadline_clock[0] + 0.5)
    home = planner.plan(goal, np.zeros(14), deadline=deadline_clock[0] + 0.5)
    assert single.success and home.success
    assert single.reason == home.reason == 'direct path'
    assert np.array_equal(single.path[:, 7:], np.full((len(single.path), 7), 0.05))
    assert np.array_equal(home.path[-1], np.zeros(14))
    assert set(single.metadata['stage_timings']) == {
        'setup', 'direct', 'direct_validation', 'total',
    }
    assert planner._deadline is None


def test_active_mask_is_copied_from_caller():
    planner = ReducedJointPlanner(FakeRobotModel())
    mask = np.array([True, False])
    planner._set_active_mask(mask, np.zeros(2))
    mask[:] = True
    assert np.array_equal(planner.active_mask, [True, False])


def test_expired_noop_cannot_leave_zero_extent_for_next_direct_plan(
        monkeypatch, deadline_clock):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ))
    timed = planner._timed_path

    def expire(*args, **kwargs):
        deadline_clock[0] = 1.0
        planner._check_deadline()

    monkeypatch.setattr(planner, '_timed_path', expire)
    start = np.zeros(14)
    result = planner.plan(start, start, deadline=0.5)
    assert not result.success
    assert planner.space.getMaximumExtent() > 0.0
    assert np.all(planner.active_mask)
    monkeypatch.setattr(planner, '_timed_path', timed)
    goal = start.copy()
    goal[0] = 0.1

    result = planner.plan(start, goal)

    assert result.success, result.reason
    assert result.reason == 'direct path'
    assert np.array_equal(result.path[:, 7:], np.zeros((len(result.path), 7)))


def test_longer_playback_horizon_preserves_velocity_and_acceleration_limits():
    planner = ReducedJointPlanner(FakeRobotModel(), PlannerConfig(
        joint_velocity_limit=0.1,
    ))
    path, _, reason = planner._timed_path(np.array([[0.0, 0.0], [1.5, 0.0]]))

    assert not reason
    assert 600 < len(path) <= planner.config.max_interpolation_steps
    assert np.max(np.abs(np.diff(path, axis=0) / planner.config.dt)) <= 0.1 + 1e-6
    assert np.max(np.abs(np.diff(path, n=2, axis=0) / planner.config.dt ** 2)) <= (
        planner.config.joint_acceleration_limit + 1e-6
    )
