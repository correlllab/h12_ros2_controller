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

    def check_within_limits_reduced(self, q_reduced):
        lower = self.model_body_reduced.lowerPositionLimit
        upper = self.model_body_reduced.upperPositionLimit
        return bool(np.all(q_reduced >= lower) and np.all(q_reduced <= upper))

    def check_collision_free_reduced(self, q_reduced):
        return self.collision_free


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
        config=PlannerConfig(timeout=0.2, interpolation_steps=10),
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
        config=PlannerConfig(timeout=0.2, interpolation_steps=10),
    )
    start = np.zeros(14)
    goal = np.zeros(14)
    start[7:] = 0.01
    goal[:7] = 0.5

    result = planner.plan(start, goal)

    assert result.success, result.reason
    assert np.allclose(result.path[:, 7:], start[7:])


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

    start_name, goal_name = valid_pairs[0]
    planner = ReducedJointPlanner(
        robot_model,
        config=PlannerConfig(timeout=1.0, interpolation_steps=25),
    )
    result = planner.plan(
        config_mod.NAMED_CONFIGS[start_name],
        config_mod.NAMED_CONFIGS[goal_name],
    )

    assert result.success, result.reason
    assert result.path.shape[1] == len(joint_mod.ENABLED_JOINTS)
