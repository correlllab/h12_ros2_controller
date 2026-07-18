import numpy as np
import pytest

from h12_ros2_controller.benchmark import planner_benchmark
from h12_ros2_controller.benchmark import planner_benchmark_full


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

    summary = planner_benchmark._plan_summary(rows)

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
