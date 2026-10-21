from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlannerConfig,
    ReducedJointPlanner,
    moving_grasp_frames,
)


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

    def check_collision_free_reduced(self, q_reduced):
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
    assert result.reason == 'start is in self-collision'


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


def test_plan_between_fake_valid_states():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(timeout=0.2, max_interpolation_steps=10),
    )

    result = planner.plan([0.0, 0.0], [0.5, 0.5])

    assert result.success
    assert result.path.shape[1] == 2
    assert result.path.shape[0] <= 10
    assert np.allclose(result.path[0], [0.0, 0.0])
    assert np.allclose(result.path[-1], [0.5, 0.5])
    assert result.metadata['path_kind'] == 'direct path'


def test_direct_only_reports_invalid_segment_without_using_ompl():
    robot_model = FakeRobotModel()
    robot_model.check_collision_free_reduced = lambda q: not 0.25 < q[0] < 0.35
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


def test_named_joint_plans_do_not_inherit_locality_constraint():
    planner = ReducedJointPlanner(FakeRobotModel())
    start = np.array([0.0, 0.0])
    goal = np.array([0.5, 0.0])
    _, reason = planner._path_quality_failure(
        np.array([start, [0.25, 1.0], goal]), start, goal, (),
    )

    assert not reason


def test_plan_pins_idle_right_arm_with_home_tolerance():
    pytest.importorskip('ompl')
    lower = -np.ones(14)
    upper = np.ones(14)
    planner = ReducedJointPlanner(
        FakeRobotModel(lower=lower, upper=upper),
        config=PlannerConfig(timeout=0.2, max_interpolation_steps=10),
    )
    start = np.zeros(14)
    goal = np.zeros(14)
    start[7:] = 0.01
    goal[:7] = 0.5

    result = planner.plan(start, goal)

    assert result.success, result.reason
    assert np.allclose(result.path[:, 7:], start[7:])


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
    robot.check_collision_free_reduced = lambda q: not (0.4 < q[0] < 0.6)
    planner = ReducedJointPlanner(
        robot,
        config=PlannerConfig(constraint_check_steps=1, frame_z_max=None),
    )
    start = np.array([0.0, 0.0])
    goal = np.array([1.0, 0.0])
    planner._set_workspace_corridor(start, goal)

    assert 'path segment 0 is in self-collision' == planner._path_validity_failure(
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
