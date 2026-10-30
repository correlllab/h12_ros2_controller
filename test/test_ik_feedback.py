from types import SimpleNamespace

import numpy as np
import pinocchio as pin
import pytest

import h12_ros2_controller.core.ik_solver as ik_module
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.controller.upper_controller import UpperController
from h12_ros2_controller.core.planner import PlanResult, PlannerConfig
from h12_ros2_controller.core.planner.kinematic_pipeline import KinematicPipeline
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS


@pytest.fixture(scope='module')
def pipeline():
    return KinematicPipeline(config_name='safety_split.yaml')


def feedback_tasks(pipeline):
    solver = pipeline.ik_solver
    solver.clear_frame_tasks()
    q = pipeline.robot_model.state['q']
    for side in ('left', 'right'):
        frame = f'{side}_grasp_frame'
        current = pin.SE3(pipeline.robot_model.get_frame_transformation_body(frame, q))
        solver.add_frame_task(side, frame)
        solver.frame_tasks[side].set_target(
            current * pin.SE3(pin.rpy.rpyToMatrix(0.01, -0.02, 0.03),
                             np.array([0.005, 0.002, -0.003])),
        )
    return solver.frame_tasks


def placements(data):
    return np.asarray([transform.np.copy() for transform in data.oMf])


def test_full_and_reduced_task_errors_agree_at_matching_frozen_reference(pipeline):
    tasks = feedback_tasks(pipeline)
    solver = pipeline.ik_solver
    q = pipeline.robot_model.state['q']

    errors = solver.compute_errors(tasks, q)

    for name, task in tasks.items():
        np.testing.assert_allclose(errors[name], solver.compute_error(task, q))
        np.testing.assert_allclose(errors[name], solver.compute_error_reduced(
            task, q[pipeline.robot_model.reduced_mask],
        ), atol=1e-12)


def test_feedback_does_not_change_solver_configurations_or_working_fk(pipeline):
    tasks = feedback_tasks(pipeline)
    solver = pipeline.ik_solver
    robot = pipeline.robot_model
    buffers = (robot.data_body, robot.data_body_reduced,
               solver.configuration.data, solver.configuration_reduced.data)
    before = [placements(data) for data in buffers]
    full_q = solver.q
    reduced_q = solver.q_reduced
    q = robot.state['q'].copy()
    q[BODY_JOINTS.index('torso_joint')] = 0.17
    q[BODY_JOINTS.index('left_shoulder_pitch_joint')] = -0.2
    original_input = q.copy()

    first = solver.compute_errors(tasks, q)
    solver.compute_error_reduced(tasks['left'], q[robot.reduced_mask])
    solver.compute_error(tasks['right'], q)

    assert all(np.all(np.isfinite(error)) for error in first.values())
    np.testing.assert_array_equal(q, original_input)
    np.testing.assert_array_equal(solver.q, full_q)
    np.testing.assert_array_equal(solver.q_reduced, reduced_q)
    for original, data in zip(before, buffers):
        np.testing.assert_array_equal(placements(data), original)


def test_bimanual_feedback_reads_one_snapshot_and_builds_one_configuration(pipeline, monkeypatch):
    tasks = feedback_tasks(pipeline)
    robot = pipeline.robot_model
    q = robot.state['q'].copy()

    class AdvancingState:
        reads = 0

        @property
        def state(self):
            self.reads += 1
            snapshot = q.copy()
            snapshot[12] = self.reads * 0.1
            return {'q': snapshot}

    state_source = AdvancingState()
    controller = object.__new__(FrameController)
    controller.robot_model = state_source
    controller.ik_solver = pipeline.ik_solver
    original = ik_module.pink.Configuration
    created = []

    def configuration(model, data, snapshot, **kwargs):
        created.append((data, snapshot.copy(), kwargs))
        return original(model, data, snapshot, **kwargs)

    monkeypatch.setattr(ik_module.pink, 'Configuration', configuration)

    errors = controller.get_frame_task_errors()

    assert state_source.reads == 1
    assert len(created) == 1
    data, snapshot, options = created[0]
    assert data is not robot.data_body and data is not controller.ik_solver.configuration.data
    assert options['copy_data'] is False
    assert snapshot[12] == 0.1
    expected = controller.ik_solver.compute_errors(tasks, snapshot)
    for name in tasks:
        np.testing.assert_allclose(errors[name], expected[name])


@pytest.mark.parametrize('q', [
    np.zeros(14), np.zeros((27, 1)), np.full(27, np.nan), np.full(27, np.inf),
])
def test_full_feedback_rejects_invalid_motor_snapshots(pipeline, q):
    tasks = feedback_tasks(pipeline)
    with pytest.raises(ValueError, match='27 finite joint positions'):
        pipeline.ik_solver.compute_error(tasks['left'], q)


def test_empty_batch_does_not_allocate_fk(pipeline, monkeypatch):
    monkeypatch.setattr(ik_module.pink, 'Configuration',
                        lambda *args, **kwargs: pytest.fail('unnecessary FK allocation'))
    assert pipeline.ik_solver.compute_errors({}, np.zeros(27)) == {}


def test_controller_single_error_delegates_without_reading_an_explicit_snapshot():
    task = object()
    q = np.zeros(27)
    error = np.arange(6.0)
    calls = []
    controller = object.__new__(FrameController)
    controller.ik_solver = SimpleNamespace(
        frame_tasks={'left': task},
        compute_error=lambda target, snapshot: calls.append((target, snapshot)) or error,
    )

    assert controller.get_frame_task_error('left', q) is error
    assert calls[0][0] is task and calls[0][1] is q
    with pytest.raises(ValueError, match='Task missing not found'):
        controller.get_frame_task_error('missing')


@pytest.mark.parametrize('entrypoint', ('ik', 'plan'))
@pytest.mark.parametrize('failure_stage', ('collision', 'ik'))
def test_parent_reduced_rebuild_retries_same_frozen_reference_after_failure(
        entrypoint, failure_stage):
    controller = object.__new__(UpperController)
    measured = np.zeros(27)
    measured[12] = 0.02
    mask = np.r_[np.zeros(13, dtype=bool), np.ones(14, dtype=bool)]
    calls = []
    failed = [False]
    controller.robot_model = SimpleNamespace(
        state={'q': measured}, reduced_mask=mask, reduced_reference_q=np.zeros(27),
    )

    def maybe_fail(stage):
        if failure_stage == stage and not failed[0]:
            failed[0] = True
            raise RuntimeError(f'{stage} rebuild failed')

    def rebuild_model(reference_q):
        calls.append('model')
        # reproduce a committed kinematic reference before collision rebuilding
        controller.robot_model.reduced_reference_q = reference_q.copy()
        maybe_fail('collision')

    def rebuild_ik():
        calls.append('ik')
        maybe_fail('ik')

    controller.robot_model.rebuild_reduced_model = rebuild_model
    controller.ik_solver = SimpleNamespace(
        rebuild_reduced=rebuild_ik,
        update_configurations=lambda q: calls.append('seed'),
    )
    controller.check_state_fresh = lambda: None
    controller._command_seed = lambda q: (q.copy(), True)
    controller._tau_bias = np.zeros(27)
    controller.dt = 0.02
    controller.config = {'planner': {'planner': 'RRTConnect'}}
    controller._planner_speed_overrides = lambda: (None, None)

    def plan(**kwargs):
        calls.append('plan')
        return PlanResult(success=True, path=np.zeros((2, 14)))

    controller.planner = SimpleNamespace(
        planner_config=PlannerConfig(), plan_configuration=plan,
    )
    invoke = (controller.update_ik_solver if entrypoint == 'ik' else
              lambda: controller.plan_to_configuration(np.zeros(14)))

    with pytest.raises(RuntimeError, match='rebuild failed'):
        invoke()
    assert controller._reduced_rebuild_pending
    assert 'seed' not in calls and 'plan' not in calls

    invoke()

    assert not controller._reduced_rebuild_pending
    assert calls.count('model') == 2
    assert calls.count('ik') == (1 if failure_stage == 'collision' else 2)
    assert calls[-1] == ('seed' if entrypoint == 'ik' else 'plan')
    invoke()
    assert calls.count('model') == 2
