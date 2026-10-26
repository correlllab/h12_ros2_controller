from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.planner.kinematic_pipeline import plan_ik_candidates
from h12_ros2_controller.core.planner.path_quality import (
    candidate_path_score, radial_excursions,
)
from h12_ros2_controller.core.planner.reduced_joint_planner import PlanResult, PlannerConfig


def test_radial_excursion_only_penalizes_reach_beyond_endpoints():
    torso = np.zeros((3, 3))
    landmarks = {
        'left_elbow_link': [[0.3, 0., 0.], [0.6, 0., 0.], [0.35, 0., 0.]],
        'left_wrist_yaw_link': [[0.4, 0., 0.], [0.5, 0., 0.], [0.45, 0., 0.]],
    }

    excess = radial_excursions(landmarks, torso, slack=0.08)

    assert excess['left_elbow_link'] == pytest.approx(0.17)
    assert excess['left_wrist_yaw_link'] == 0.0

    tilt = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    tilted = {name: np.asarray(points) @ tilt.T
              for name, points in landmarks.items()}
    torso_rotations = np.broadcast_to(tilt, (3, 3, 3))
    assert radial_excursions(tilted, torso, 0.08, torso_rotations) == excess


def test_candidate_quality_prefers_compact_path_without_weakening_clearance():
    config = PlannerConfig()
    shared = {
        'joint_path_length': 5.0,
        'hand_segment_deviation': {'left_grasp_frame': 0.1},
        'minimum_self_clearance': 0.03,
        'excess_wrist_travel': 0.4,
        'duration': 8.0,
    }
    natural = {**shared, 'compact_radial_excess': {'left_elbow_link': 0.0}}
    sweeping = {**shared, 'compact_radial_excess': {'left_elbow_link': 0.2}}

    assert candidate_path_score(natural, 1.0, config) == pytest.approx(5.88)
    assert candidate_path_score(sweeping, 1.0, config) == pytest.approx(6.48)

    candidates = [SimpleNamespace(q=np.array([1.]), seed_name='sweep', score=1.0),
                  SimpleNamespace(q=np.array([2.]), seed_name='compact', score=1.0)]

    def direct(goal, direct_only, timeout):
        assert direct_only
        return PlanResult(
            success=True, path=np.array([[0.], goal]),
            metadata=sweeping if goal[0] == 1. else natural,
        )

    plan, selected = plan_ik_candidates(
        SimpleNamespace(candidates=candidates), direct, np.array([True]),
        config=config,
    )

    assert plan.success
    assert selected.seed_name == 'compact'
