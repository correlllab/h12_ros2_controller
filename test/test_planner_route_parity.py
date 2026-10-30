from types import SimpleNamespace

import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.planner.kinematic_pipeline import (
    KinematicPipeline, frame_target_snapshot, frame_targets_close,
)
from h12_ros2_controller.core.planner.planner_process import PlannerClient, _PlannerWorker
from h12_ros2_controller.core.planner.reduced_joint_planner import PlannerConfig
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import (
    URDF_MAGPIE_PATH, URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH,
)


def test_worker_candidate_timeout_matches_in_process_argument_without_changing_config():
    worker = object.__new__(_PlannerWorker)
    worker._set_current_q = lambda q: None
    config = PlannerConfig(timeout=2.0)
    calls = []

    def plan(*args, **kwargs):
        calls.append((kwargs['timeout'], config.timeout))
        return 'planned'

    worker.planner = SimpleNamespace(config=config, plan=plan)
    result = worker.handle({
        'command': 'plan_configuration', 'current_q': np.zeros(27),
        'start': np.zeros(14), 'goal': np.zeros(14), 'timeout': 0.25,
    })

    assert result == 'planned'
    assert calls == [(0.25, 2.0)]
    assert config.timeout == 2.0


def test_frame_seed_reuse_treats_euler_wrapping_as_the_same_orientation():
    def task(yaw):
        return SimpleNamespace(
            frame='left_grasp_frame',
            transform_target_to_world=pin.SE3(
                pin.rpy.rpyToMatrix(0., 0., yaw), np.array([0.4, 0.2, 0.3]),
            ),
        )

    first = frame_target_snapshot({'left': task(np.pi - 0.01)})
    second = frame_target_snapshot({'left': task(-np.pi + 0.01)})

    assert frame_targets_close(first, second, PlannerConfig())
    assert not frame_targets_close(first, second, PlannerConfig(ik_reuse_rotation=0.001))


def test_wrist_frame_bimanual_targets_enable_grasp_locality_and_coordination():
    pipeline = KinematicPipeline()
    model = pipeline.robot_model
    goal = np.asarray(NAMED_CONFIGS['arms_front_45'], dtype=float)
    tasks = []
    for frame in ('left_wrist_yaw_link', 'right_wrist_yaw_link'):
        transform = model.get_frame_transformation_reduced(frame, goal)
        tasks.append((frame, np.r_[transform[:3, 3],
                                   pin.rpy.matrixToRpy(transform[:3, :3])]))
    original = pipeline.planner.plan
    frames_seen = []

    def inspect_frames(*args, **kwargs):
        frames_seen.append(kwargs['local_frames'])
        return original(*args, **kwargs)

    pipeline.planner.plan = inspect_frames
    result = pipeline.plan_frames(tasks)

    assert result.ik.success and result.plan.success
    assert frames_seen
    assert all(set(frames) == {'left_grasp_frame', 'right_grasp_frame'}
               for frames in frames_seen)


def test_real_controller_worker_and_offline_use_identical_direct_goal_pipeline():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    pose = np.array([0.4, 0.4, 0.3, 0., np.deg2rad(-30), 0.])
    offline = pipeline.plan_frame('left_grasp_frame', pose)
    assert offline.plan.success, offline.plan.reason
    pipeline.set_start(np.zeros(27))
    controller = object.__new__(FrameController)
    controller.robot_model = pipeline.robot_model
    controller.ik_solver = pipeline.ik_solver
    controller.config = load_controller_config('sim_safety_split.yaml')
    controller.dt = pipeline.planner_config.dt
    controller._tau_bias = np.zeros(27)
    controller.enabled_ids = list(range(13, 27))
    controller.low_cmd_handler = SimpleNamespace(q_cmd=np.zeros(27))
    controller.speed_scale = 1.0
    controller.max_state_lag_ms = 0.0
    controller._prior_ik = None
    controller.planner = PlannerClient(
        URDF_MAGPIE_PATH, URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH,
        pipeline.planner_config, ompl_seed=42,
    )
    controller.clear_frame_tasks()
    controller.add_frame_task('left_goal', 'left_grasp_frame', pose)

    try:
        controller.planner.warmup()
        path = controller.plan_to_ik_target()

        assert controller.last_plan_result.success
        assert controller.last_plan_result.reason == offline.plan.reason == 'direct path'
        np.testing.assert_allclose(path, offline.plan.path, atol=1e-8)
        assert controller.planner.planner_config == pipeline.planner_config
        assert 'execution_context' in controller.last_plan_result.metadata
    finally:
        controller.planner.shutdown()
