import numpy as np
import pytest

from h12_ros2_controller.benchmark import (
    collision_models,
    planner_benchmark,
    planner_benchmark_full,
)
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.path_definition import (
    SRDF_MAGPIE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)


def _collision_group(name):
    name = name.removesuffix('_0')
    if '_sphere' in name:
        name = name.split('_sphere')[0]
    if name.startswith('lg_'):
        return 'left_wrist_yaw_link'
    if name.startswith('rg_'):
        return 'right_wrist_yaw_link'
    return name


def _active_group_pairs(urdf_path, srdf_path):
    robot_model = RobotModel(URDF_MAGPIE_PATH, handless=False)
    robot_model.init_reduced_model(ENABLED_JOINTS)
    robot_model.init_collision_model(urdf_path, srdf_path)
    collision_model = robot_model.collision_model_body_reduced
    return {
        tuple(sorted((
            _collision_group(collision_model.geometryObjects[pair.first].name),
            _collision_group(collision_model.geometryObjects[pair.second].name),
        )))
        for pair in collision_model.collisionPairs
    }


def test_validate_transform_accepts_identity():
    valid, reason = planner_benchmark.validate_transform(np.eye(4))

    assert valid
    assert reason == ''


def test_validate_transform_rejects_non_orthonormal_rotation():
    transform = np.eye(4)
    transform[0, 0] = 2.0

    valid, reason = planner_benchmark.validate_transform(transform)

    assert not valid
    assert reason == 'transform rotation must be orthonormal'


def test_select_indices_applies_count_and_rejects_duplicates():
    assert planner_benchmark.select_indices(5, count=2) == [0, 1]

    with pytest.raises(ValueError, match='must not contain duplicates'):
        planner_benchmark.select_indices(5, indices=[1, 1])


def test_default_frame_is_right_graspgenx_frame():
    assert planner_benchmark._FRAME_NAMES[0] == 'right_graspgenx_frame'


def test_plan_summary_groups_collision_representations():
    rows = [
        {
            'collision_representation': 'sphere',
            'success': True,
            'planning_time': 0.1,
            'reason': 'success',
        },
        {
            'collision_representation': 'sphere',
            'success': False,
            'planning_time': 0.2,
            'reason': 'timeout',
        },
        {
            'collision_representation': 'mesh',
            'success': True,
            'planning_time': 0.3,
            'reason': 'success',
        },
    ]

    summary = planner_benchmark._plan_summary(rows, {'sphere': None, 'mesh': None})

    assert summary['sphere']['success_rate'] == pytest.approx(0.5)
    assert summary['mesh']['mean_planning_time'] == pytest.approx(0.3)


def test_full_benchmark_active_mask_selects_requested_arm():
    start = np.zeros(14)

    right_mask = planner_benchmark_full._active_task_mask(
        start,
        'right_graspgenx_frame',
    )
    left_mask = planner_benchmark_full._active_task_mask(
        start,
        'left_graspgenx_frame',
    )

    assert not np.any(right_mask[:7])
    assert np.all(right_mask[7:])
    assert np.all(left_mask[:7])
    assert not np.any(left_mask[7:])


def test_magpie_mesh_and_sphere_use_matching_group_pairs():
    mesh_pairs = _active_group_pairs(URDF_MAGPIE_PATH, SRDF_MAGPIE_PATH)
    sphere_pairs = _active_group_pairs(
        URDF_MAGPIE_SPHERE_PATH,
        SRDF_MAGPIE_SPHERE_PATH,
    )

    assert mesh_pairs == sphere_pairs
    assert all(link1 != link2 for link1, link2 in mesh_pairs)


def test_get_collision_models_resolves_magpie():
    models = collision_models.get_collision_models('magpie')
    assert 'sphere' in models
    assert 'mesh' in models
    assert models['sphere'][0].endswith('magpie_sphere.urdf')


def test_get_collision_models_resolves_handless():
    models = collision_models.get_collision_models('handless')
    assert 'sphere' in models
    assert 'mesh' in models
    assert models['mesh'][0].endswith('handless.urdf')


def test_get_collision_models_rejects_unknown():
    with pytest.raises(ValueError, match='Unknown model variant'):
        collision_models.get_collision_models('unknown')


def test_model_output_path_adds_suffix():
    assert collision_models.model_output_path(
        'data/planner/collision_benchmark.csv',
        'handless',
    ) == 'data/planner/collision_benchmark_handless.csv'


def test_model_output_path_strips_prior_extension():
    result = collision_models.model_output_path(
        'results.csv',
        'magpie',
    )
    assert result.endswith('_magpie.csv')
    assert result == 'results_magpie.csv'
