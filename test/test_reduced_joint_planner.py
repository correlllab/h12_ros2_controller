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
    ))
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


def test_two_arm_fallback_retains_final_waypoint_cap():
    config = PlannerConfig(max_interpolation_steps=35,
                           max_staged_interpolation_steps=40)
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


def test_two_arm_fallback_cannot_restart_expired_request_deadline(monkeypatch):
    planner = ReducedJointPlanner(FakeRobotModel(
        lower=-np.ones(14), upper=np.ones(14),
    ))
    clock = [0.0]
    calls = []
    monkeypatch.setattr(
        'h12_ros2_controller.core.planner.reduced_joint_planner.perf_counter',
        lambda: clock[0],
    )

    def unavailable(*args, **kwargs):
        calls.append(kwargs.get('timeout'))
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
    assert calls[1] == pytest.approx(0.1)
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
