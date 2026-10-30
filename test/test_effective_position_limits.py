from dataclasses import asdict, replace

import numpy as np
import pytest

from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, ENABLED_JOINTS
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.core.planner.planner_process import _PlannerWorker
from h12_ros2_controller.utility.path_definition import (
    URDF_MAGPIE_PATH, URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH,
)
from h12_ros2_controller.core.planner.kinematic_pipeline import (
    KinematicPipeline, planner_config_from_controller,
)
from h12_ros2_controller.core.planner.reduced_joint_planner import PlannerConfig


@pytest.fixture(scope='module')
def pipeline():
    return KinematicPipeline(config_name='sim_safety_split.yaml')


@pytest.fixture
def robot():
    return RobotModel(URDF_MAGPIE_PATH)


def test_pink_and_ompl_use_command_position_limit_intersection(pipeline):
    robot = pipeline.robot_model
    config = load_controller_config('sim_safety_split.yaml')
    command_limits = np.asarray(config['limits']['q_clip_limits'])
    raw = RobotModel(URDF_MAGPIE_PATH)
    assert raw.model_body.upperPositionLimit[18] == pytest.approx(0.4625)
    assert raw.model_body.lowerPositionLimit[18] == pytest.approx(-0.4625)
    lower = np.maximum(raw.model_body.lowerPositionLimit, command_limits[:, 0])
    upper = np.minimum(raw.model_body.upperPositionLimit, command_limits[:, 1])
    np.testing.assert_array_equal(robot.model_body.lowerPositionLimit, lower)
    np.testing.assert_array_equal(robot.model_body.upperPositionLimit, upper)
    np.testing.assert_array_equal(robot.model.lowerPositionLimit[7:], lower)
    np.testing.assert_array_equal(robot.model.upperPositionLimit[7:], upper)

    for configuration, limit, expected_lower, expected_upper in (
        (pipeline.ik_solver.configuration, pipeline.ik_solver.limits[0], lower, upper),
        (pipeline.ik_solver.configuration_reduced,
         pipeline.ik_solver.limits_reduced[0],
         lower[robot.reduced_mask], upper[robot.reduced_mask]),
    ):
        np.testing.assert_array_equal(limit.model.lowerPositionLimit, expected_lower)
        np.testing.assert_array_equal(limit.model.upperPositionLimit, expected_upper)
        _, bounds = limit.compute_qp_inequalities(configuration, 0.1)
        gain = limit.config_limit_gain
        np.testing.assert_allclose(bounds, np.r_[
            gain * (expected_upper - configuration.q),
            gain * (configuration.q - expected_lower),
        ])

    planner = pipeline.planner
    np.testing.assert_array_equal(planner.model_lower, lower[robot.reduced_mask])
    np.testing.assert_array_equal(planner.model_upper, upper[robot.reduced_mask])
    np.testing.assert_array_equal(planner.space.getBounds().low, planner.model_lower)
    np.testing.assert_array_equal(planner.space.getBounds().high, planner.model_upper)
    assert robot.state_subscriber is None


@pytest.mark.parametrize('side', ('left', 'right'))
@pytest.mark.parametrize('sign', (-1, 1))
def test_raw_wrist_boundary_is_not_a_feasible_planning_goal(pipeline, side, sign):
    robot = pipeline.robot_model
    motor_index = BODY_JOINTS.index(f'{side}_wrist_pitch_joint')
    reduced_index = list(np.flatnonzero(robot.reduced_mask)).index(motor_index)
    assert robot.model_body.upperPositionLimit[motor_index] == pytest.approx(0.4615)
    assert robot.model_body.lowerPositionLimit[motor_index] == pytest.approx(-0.4615)
    goal = np.zeros(pipeline.planner.nq)
    goal[reduced_index] = sign * 0.4625
    original = goal.copy()
    assert not robot.check_within_limits_reduced(goal)
    full_goal = robot.zero_q_body
    full_goal[robot.reduced_mask] = goal
    assert not robot.check_within_limits(full_goal)
    result = pipeline.planner.plan(np.zeros_like(goal), goal, local_frames=())
    assert not result.success
    assert 'limit' in result.reason.lower()
    np.testing.assert_array_equal(goal, original)


def test_limits_intersect_both_sides_and_leave_freeflyer_unchanged(robot):
    raw_lower = robot.model_body.lowerPositionLimit.copy()
    raw_upper = robot.model_body.upperPositionLimit.copy()
    base_lower = robot.model.lowerPositionLimit[:7].copy()
    base_upper = robot.model.upperPositionLimit[:7].copy()
    limits = np.column_stack((raw_lower - 1.0, raw_upper + 1.0))
    limits[13, 0] = raw_lower[13] + 0.2
    limits[14, 1] = raw_upper[14] - 0.3
    robot.apply_position_limits(limits)
    expected_lower = np.maximum(raw_lower, limits[:, 0])
    expected_upper = np.minimum(raw_upper, limits[:, 1])
    np.testing.assert_array_equal(robot.model_body.lowerPositionLimit, expected_lower)
    np.testing.assert_array_equal(robot.model_body.upperPositionLimit, expected_upper)
    np.testing.assert_array_equal(robot.model.lowerPositionLimit[:7], base_lower)
    np.testing.assert_array_equal(robot.model.upperPositionLimit[:7], base_upper)


def test_repeat_updates_use_raw_urdf_and_rebuilds_preserve_limits(robot):
    raw = np.column_stack((
        robot.model_body.lowerPositionLimit, robot.model_body.upperPositionLimit,
    ))
    first = raw.copy()
    first[:, 0] += 0.1
    first[:, 1] -= 0.1
    robot.apply_position_limits(first)
    second = raw.copy()
    second[:, 0] += 0.01
    second[:, 1] -= 0.01
    robot.apply_position_limits(second)
    np.testing.assert_array_equal(robot.model_body.lowerPositionLimit, second[:, 0])
    np.testing.assert_array_equal(robot.model_body.upperPositionLimit, second[:, 1])
    robot.init_reduced_model(ENABLED_JOINTS)
    reference = robot.zero_q_body
    reference[12] = 0.15
    robot.rebuild_reduced_model(reference)
    np.testing.assert_array_equal(
        robot.model_body_reduced.lowerPositionLimit, second[robot.reduced_mask, 0],
    )
    np.testing.assert_array_equal(
        robot.model_body_reduced.upperPositionLimit, second[robot.reduced_mask, 1],
    )
    with pytest.raises(RuntimeError, match='before reduced'):
        robot.apply_position_limits(raw)


@pytest.mark.parametrize('invalid', (
    np.zeros((14, 2)),
    np.zeros((27,)),
    np.tile([-np.inf, 1.0], (27, 1)),
    np.tile([-1.0, np.nan], (27, 1)),
    np.tile([1.0, -1.0], (27, 1)),
    np.zeros((27, 2)),
))
def test_malformed_motor_position_limits_fail_without_mutation(robot, invalid):
    lower = robot.model_body.lowerPositionLimit.copy()
    upper = robot.model_body.upperPositionLimit.copy()
    with pytest.raises(ValueError):
        robot.apply_position_limits(invalid)
    with pytest.raises(ValueError, match='motor_position_limits'):
        PlannerConfig(motor_position_limits=invalid)
    np.testing.assert_array_equal(robot.model_body.lowerPositionLimit, lower)
    np.testing.assert_array_equal(robot.model_body.upperPositionLimit, upper)


@pytest.mark.parametrize('gap', (0.0, 0.1))
def test_empty_urdf_intersection_fails_without_mutation(robot, gap):
    lower = robot.model_body.lowerPositionLimit.copy()
    upper = robot.model_body.upperPositionLimit.copy()
    limits = np.column_stack((lower, upper))
    limits[18] = [upper[18] + gap, upper[18] + 1.0]
    with pytest.raises(ValueError, match='intersection'):
        robot.apply_position_limits(limits)
    np.testing.assert_array_equal(robot.model_body.lowerPositionLimit, lower)
    np.testing.assert_array_equal(robot.model_body.upperPositionLimit, upper)
    np.testing.assert_array_equal(robot.model.lowerPositionLimit[7:], lower)
    np.testing.assert_array_equal(robot.model.upperPositionLimit[7:], upper)


def test_injected_narrower_config_reaches_pink_and_ompl():
    config = planner_config_from_controller(load_controller_config('sim_safety_split.yaml'))
    requested = np.asarray(config.motor_position_limits).copy()
    requested[18] = [-0.2, 0.25]
    injected = replace(config, motor_position_limits=tuple(map(tuple, requested)))
    pipeline = KinematicPipeline(planner_config=injected)
    robot = pipeline.robot_model
    reduced_index = list(np.flatnonzero(robot.reduced_mask)).index(18)
    for model in (robot.model_body, pipeline.ik_solver.limits[0].model):
        assert model.lowerPositionLimit[18] == pytest.approx(-0.2)
        assert model.upperPositionLimit[18] == pytest.approx(0.25)
    assert pipeline.ik_solver.limits_reduced[0].model.lowerPositionLimit[
        reduced_index
    ] == pytest.approx(-0.2)
    assert pipeline.planner.model_lower[reduced_index] == pytest.approx(-0.2)
    assert pipeline.planner.model_upper[reduced_index] == pytest.approx(0.25)


def test_worker_bounds_match_pipeline_and_survive_frozen_rebuild(pipeline):
    worker = _PlannerWorker({
        'urdf_path': URDF_MAGPIE_PATH,
        'urdf_sphere_path': URDF_MAGPIE_SPHERE_PATH,
        'srdf_sphere_path': SRDF_MAGPIE_SPHERE_PATH,
        'handless': False,
        'planner_config': asdict(pipeline.planner_config),
    })
    reference = worker.robot_model.zero_q_body
    reference[12] = 0.15
    worker._set_current_q(reference)
    np.testing.assert_array_equal(worker.planner.model_lower, pipeline.planner.model_lower)
    np.testing.assert_array_equal(worker.planner.model_upper, pipeline.planner.model_upper)
    np.testing.assert_array_equal(
        worker.robot_model.model_body.lowerPositionLimit,
        pipeline.robot_model.model_body.lowerPositionLimit,
    )
    assert worker.robot_model.reduced_reference_q[12] == pytest.approx(0.15)


def test_reported_bimanual_path_is_executable_without_position_clipping():
    pipeline = KinematicPipeline(config_name='safety_split.yaml')
    result = pipeline.plan_frames([
        ('left_grasp_frame', np.array([
            0.47, 0.14, 0.34, np.deg2rad(15), np.deg2rad(60), np.deg2rad(-15),
        ])),
        ('right_grasp_frame', np.array([
            0.47, -0.14, 0.34, np.deg2rad(-15), np.deg2rad(60), np.deg2rad(15),
        ])),
    ])

    assert result.plan is not None and result.plan.success, result.fallback_reason
    command = np.asarray(load_controller_config('safety_split.yaml')['limits']['q_clip_limits'])
    command = command[pipeline.robot_model.reduced_mask]
    assert np.all(result.plan.path >= command[:, 0] - 1e-9)
    assert np.all(result.plan.path <= command[:, 1] + 1e-9)
