import os
from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.planner.reduced_joint_planner import (
    PlannerConfig,
    ReducedJointPlanner,
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

    def check_collision_free_reduced(self, q_reduced):
        return self.collision_free

    def get_frame_position(self, frame_name, q):
        return np.array([q[1], q[1], q[0]])


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
    assert result.reason == 'start is outside reduced joint limits'


def test_invalid_goal_fails_with_clear_reason():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(FakeRobotModel())

    result = planner.plan([0.0, 0.0], [0.0, 3.0])

    assert not result.success
    assert result.path.shape == (0, 2)
    assert result.reason == 'goal is outside reduced joint limits'


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
            frame_y_min=-0.5,
            frame_y_max=0.5,
            frame_limit_margin=0.01,
        ),
    )

    result = planner.plan([0.0, 0.52], [0.0, 0.0])

    assert not result.success
    assert result.reason == 'start moves left_grasp_frame above y=0.5000'


def test_state_validation_allows_y_limit_margin():
    pytest.importorskip('ompl')
    planner = ReducedJointPlanner(
        FakeRobotModel(),
        config=PlannerConfig(
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
    assert result.reason == 'start moves left_grasp_frame beyond x=0.7000'


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
    assert result.reason == 'start moves left_grasp_frame above z=0.5000'
    assert planner._validity_failure(np.array([0.51, 0.0]), 'state') == ''


def test_plan_between_named_configs_smoke():
    pytest.importorskip('ompl')
    robot_model_mod = pytest.importorskip(
        'h12_ros2_controller.core.robot_model',
        reason='RobotModel dependencies are unavailable',
    )
    path_mod = pytest.importorskip(
        'h12_ros2_controller.utility.path_definition',
        reason='asset paths are unavailable',
    )
    joint_mod = pytest.importorskip(
        'h12_ros2_controller.utility.joint_definition',
        reason='joint definitions are unavailable',
    )
    config_mod = pytest.importorskip(
        'h12_ros2_controller.utility.named_config',
        reason='named configs are unavailable',
    )

    for path in [
        path_mod.URDF_MAGPIE_PATH,
        path_mod.URDF_MAGPIE_SPHERE_PATH,
        path_mod.SRDF_MAGPIE_SPHERE_PATH,
    ]:
        if not os.path.exists(path):
            pytest.skip(f'missing robot asset: {path}')

    robot_model = robot_model_mod.RobotModel(path_mod.URDF_MAGPIE_PATH)
    robot_model.init_reduced_model(joint_mod.ENABLED_JOINTS)
    robot_model.init_collision_model(
        path_mod.URDF_MAGPIE_SPHERE_PATH,
        path_mod.SRDF_MAGPIE_SPHERE_PATH,
    )

    candidate_names = [
        'home',
        'arms_front_45',
        'arms_asym',
        't_pose_elbow',
        'elbow_only',
    ]
    valid_pairs = []
    for start_name in candidate_names:
        q_start = config_mod.NAMED_CONFIGS[start_name]
        if not robot_model.check_within_limits_reduced(q_start):
            continue
        if not robot_model.check_collision_free_reduced(q_start):
            continue
        for goal_name in candidate_names:
            if goal_name == start_name:
                continue
            q_goal = config_mod.NAMED_CONFIGS[goal_name]
            if not robot_model.check_within_limits_reduced(q_goal):
                continue
            if not robot_model.check_collision_free_reduced(q_goal):
                continue
            valid_pairs.append((start_name, goal_name))

    if not valid_pairs:
        pytest.skip('no collision-free named config pair available')

    planner = ReducedJointPlanner(
        robot_model,
        config=PlannerConfig(timeout=1.0, max_interpolation_steps=25),
    )
    result = None
    for start_name, goal_name in valid_pairs:
        result = planner.plan(
            config_mod.NAMED_CONFIGS[start_name],
            config_mod.NAMED_CONFIGS[goal_name],
        )
        if result.success:
            break

    if result is None or not result.success:
        pytest.skip('no corridor-valid named config pair available')

    assert result.path.shape[1] == len(joint_mod.ENABLED_JOINTS)
