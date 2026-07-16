'''Tests for the selectable planning backend.

Selection/fallback logic and the cuRobo backend's pure-Python contract are
exercised here without a GPU (and without ompl, via monkeypatched factories),
which is the whole point of the seam: the Mac/Colima CPU sim must keep working.
'''

from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.planner import backend as B
from h12_ros2_controller.core.planner.backend import (
    PlannerBackendError,
    create_joint_planner,
)
from h12_ros2_controller.core.planner.planner_types import PlannerConfig
from h12_ros2_controller.core.planner.curobo_planner import CuroboJointPlanner


class FakeRobotModel:
    def __init__(self, nq=2, collision_free=True):
        self.init_reduced = True
        self.init_collision = True
        self.collision_free = collision_free
        self.model_body_reduced = SimpleNamespace(
            nq=nq,
            lowerPositionLimit=-np.ones(nq),
            upperPositionLimit=np.ones(nq),
            names=['universe'] + [f'j{i}' for i in range(nq)],
        )
        self.reduced_mask = np.ones(nq, dtype=bool)
        self.state = {'q': np.zeros(nq)}

    def check_within_limits_reduced(self, q):
        lower = self.model_body_reduced.lowerPositionLimit
        upper = self.model_body_reduced.upperPositionLimit
        return bool(np.all(q >= lower) and np.all(q <= upper))

    def check_collision_free_reduced(self, q):
        return self.collision_free

    def get_frame_position(self, frame_name, q):
        return np.array([0.0, 0.0, 0.0])


# -- factory selection / fallback -----------------------------------------

@pytest.fixture
def stub_factories(monkeypatch):
    monkeypatch.setattr(B, '_make_ompl', lambda rm, cfg: 'OMPL')
    monkeypatch.setattr(B, '_make_curobo', lambda rm, cfg: 'CUROBO')


def test_backend_ompl_selects_ompl(stub_factories):
    cfg = PlannerConfig(backend='ompl')
    assert create_joint_planner(object(), cfg) == 'OMPL'


def test_backend_auto_uses_ompl_without_gpu(stub_factories, monkeypatch):
    monkeypatch.setattr(B, 'curobo_available', lambda: False)
    cfg = PlannerConfig(backend='auto')
    assert create_joint_planner(object(), cfg) == 'OMPL'


def test_backend_auto_uses_curobo_with_gpu(stub_factories, monkeypatch):
    monkeypatch.setattr(B, 'curobo_available', lambda: True)
    cfg = PlannerConfig(backend='auto')
    planner = create_joint_planner(object(), cfg)
    assert planner.curobo == 'CUROBO'
    assert planner.ompl == 'OMPL'


def test_backend_auto_falls_back_when_curobo_construction_fails(monkeypatch):
    monkeypatch.setattr(B, 'curobo_available', lambda: True)
    monkeypatch.setattr(B, '_make_ompl', lambda rm, cfg: 'OMPL')

    def boom(rm, cfg):
        raise RuntimeError('bad robot_cfg')

    monkeypatch.setattr(B, '_make_curobo', boom)
    cfg = PlannerConfig(backend='auto')
    # a broken cuRobo backend must degrade to OMPL, never take down planning
    assert create_joint_planner(object(), cfg) == 'OMPL'


def test_backend_auto_falls_back_when_curobo_planning_fails(
        stub_factories, monkeypatch):
    class FailedCurobo:
        def plan(self, start, goal, active_mask=None):
            return SimpleNamespace(success=False, reason='bad robot config')

    class Ompl:
        def plan(self, start, goal, active_mask=None):
            return 'OMPL RESULT'

    monkeypatch.setattr(B, 'curobo_available', lambda: True)
    monkeypatch.setattr(B, '_make_curobo', lambda rm, cfg: FailedCurobo())
    monkeypatch.setattr(B, '_make_ompl', lambda rm, cfg: Ompl())
    planner = create_joint_planner(object(), PlannerConfig(backend='auto'))
    assert planner.plan([0.0], [1.0]) == 'OMPL RESULT'


def test_backend_curobo_explicit_raises_without_gpu(monkeypatch):
    monkeypatch.setattr(B, 'curobo_available', lambda: False)
    cfg = PlannerConfig(backend='curobo')
    with pytest.raises(PlannerBackendError, match='curobo'):
        create_joint_planner(object(), cfg)


def test_backend_curobo_explicit_uses_curobo_with_gpu(stub_factories,
                                                      monkeypatch):
    monkeypatch.setattr(B, 'curobo_available', lambda: True)
    cfg = PlannerConfig(backend='curobo')
    assert create_joint_planner(object(), cfg) == 'CUROBO'


def test_unsupported_backend_raises(stub_factories):
    cfg = PlannerConfig(backend='nonsense')
    with pytest.raises(PlannerBackendError, match='Unsupported'):
        create_joint_planner(object(), cfg)


def test_backend_auto_builds_real_ompl_when_available(monkeypatch):
    pytest.importorskip('ompl')
    from h12_ros2_controller.core.planner.reduced_joint_planner import (
        ReducedJointPlanner,
    )
    monkeypatch.setattr(B, 'curobo_available', lambda: False)
    planner = create_joint_planner(FakeRobotModel(), PlannerConfig())
    assert isinstance(planner, ReducedJointPlanner)


# -- cuRobo backend contract (no GPU required) ----------------------------

def test_curobo_requires_reduced_model():
    robot_model = FakeRobotModel()
    robot_model.init_reduced = False
    with pytest.raises(ValueError, match='reduced model'):
        CuroboJointPlanner(robot_model)


def test_curobo_requires_collision_model():
    robot_model = FakeRobotModel()
    robot_model.init_collision = False
    with pytest.raises(ValueError, match='collision model'):
        CuroboJointPlanner(robot_model)


def test_curobo_rejects_invalid_endpoint_before_touching_gpu():
    # out-of-limits start must fail via robot_model checks, never import cuRobo
    planner = CuroboJointPlanner(FakeRobotModel(), PlannerConfig())
    result = planner.plan(start=[5.0, 0.0], goal=[0.0, 0.0])
    assert not result.success
    assert 'joint limits' in result.reason


def test_curobo_uses_canonical_robot_assets():
    planner = CuroboJointPlanner(FakeRobotModel(), PlannerConfig())
    robot_cfg, urdf = planner._curobo_asset_paths()
    assert robot_cfg.endswith('h1_2_magpie_curobo.yml')
    assert urdf.endswith('h1_2_magpie_sphere.urdf')


def test_curobo_projects_inactive_joints():
    planner = CuroboJointPlanner(FakeRobotModel(), PlannerConfig())
    mask = np.array([True, False])
    projected = planner._project_inactive(
        np.array([9.0, 9.0]), mask, start=np.array([1.0, 2.0])
    )
    assert projected.tolist() == [9.0, 2.0]


def test_curobo_reorders_trajectory_columns():
    planner = CuroboJointPlanner(FakeRobotModel(), PlannerConfig())
    # cuRobo returns columns in [j1, j0]; expect reorder back to [j0, j1]
    positions = np.array([[10.0, 20.0]])
    reordered = planner._reorder_columns(positions, ['j1', 'j0'])
    assert reordered.tolist() == [[20.0, 10.0]]
