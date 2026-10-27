from types import SimpleNamespace
import os
import time

import numpy as np
import pytest

from h12_ros2_controller.example import frame_controller_counter_balance_goto as demo
from h12_ros2_controller.core.controller.upper_controller import (
    PathExecutionCancelled, UpperController,
)
from h12_ros2_controller.core.planner.kinematic_pipeline import KinematicPipeline
from h12_ros2_controller.core.planner import PlanResult, PlannerConfig
from h12_ros2_controller.core.planner.planner_process import _PlannerWorker
from h12_ros2_controller.utility.controller_config import load_controller_config
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS, UPPER_BODY_INDEX


def test_worker_rebinds_reduced_collision_on_frozen_joint_change():
    calls = []
    worker = object.__new__(_PlannerWorker)
    worker.robot_model = SimpleNamespace(
        _q=np.zeros(4),
        _dq=np.zeros(4),
        reduced_mask=np.array([True, True, False, False]),
        rebuild_reduced_model=lambda reference_q: calls.append(reference_q.copy()),
        update_kinematics=lambda: None,
    )
    worker.planner = SimpleNamespace(rebind_reduced_model=lambda: calls.append('rebind'))
    worker._frozen_reference = np.zeros(2)

    worker._set_current_q(np.array([0.1, 0.2, 0.0, 0.0]))
    assert calls == []

    worker._set_current_q(np.array([0.1, 0.2, 0.03, 0.0]))
    assert len(calls) == 2
    assert np.array_equal(calls[0], worker.robot_model._q)
    assert calls[1] == 'rebind'

    worker._set_current_q(np.array([0.2, 0.1, 0.03, 0.0]))
    assert len(calls) == 2

    with pytest.raises(ValueError, match='finite motor configuration'):
        worker._set_current_q(np.array([0.1, 0.2, np.nan, 0.0]))


def test_worker_invalid_speed_overrides_leave_configuration_unchanged():
    worker = object.__new__(_PlannerWorker)
    config = SimpleNamespace(
        moving_speed=0.3,
        min_interpolation_steps=30,
        max_interpolation_steps=300,
        max_staged_interpolation_steps=600,
        timeout=1.0,
    )
    worker.planner = SimpleNamespace(config=config)

    with pytest.raises(ValueError, match='max_interpolation_steps'):
        with worker._config_override(0.1, 1):
            pass
    assert (config.moving_speed, config.max_interpolation_steps) == (0.3, 300)

    with pytest.raises(ValueError, match='finite and positive'):
        with worker._config_override(np.nan, None):
            pass
    assert config.moving_speed == 0.3

    with worker._config_override(None, None, timeout=0.25):
        assert config.timeout == 0.25
    assert config.timeout == 1.0

    with worker._config_override(None, 600):
        assert config.max_staged_interpolation_steps == 1200
    assert config.max_staged_interpolation_steps == 600


def test_frame_planning_retries_ranked_ik_goals_from_one_start():
    controller = object.__new__(UpperController)
    controller.config = {'controller': {
        'threshold_linear': 0.001, 'threshold_angular': 0.002,
    }}
    q = np.zeros(27)
    mask = np.zeros(27, dtype=bool)
    mask[13:] = True
    controller.robot_model = SimpleNamespace(
        state={'q': q}, reduced_mask=mask,
    )
    controller.enabled_ids = list(range(13, 27))
    controller.low_cmd_handler = SimpleNamespace(q_cmd=q.copy())
    controller._tau_bias = np.zeros(27)
    controller.planner = SimpleNamespace(
        planner_config=PlannerConfig()
    )
    controller.update_ik_solver = lambda **kwargs: None
    controller._active_task_mask = lambda: np.ones(14, dtype=bool)
    goals = [q.copy(), q.copy()]
    goals[0][mask] = 1.0
    goals[1][mask] = 2.0
    updates = []
    calls = []

    def solve_ik_reduced(**kwargs):
        calls.append(kwargs)
        return SimpleNamespace(
            success=True,
            candidates=[SimpleNamespace(q=goal, score=float(idx),
                                        seed_name=f'candidate_{idx}',
                                        linear_error=0.0, angular_error=0.0)
                        for idx, goal in enumerate(goals)],
        )

    controller.ik_solver = SimpleNamespace(
        solve_ik_reduced=solve_ik_reduced,
        frame_tasks={},
        configuration_reduced=SimpleNamespace(update=updates.append),
    )

    def plan_goal(goal, **kwargs):
        calls.append(kwargs)
        return PlanResult(
            success=bool(np.all(goal == 2.0)),
            path=np.vstack([np.zeros(14), goal]),
            reason='blocked' if np.all(goal == 1.0) else 'success',
        )

    controller._plan_configuration_result = plan_goal
    path = controller.plan_to_ik_target(ik_timeout=0.8)

    assert len(calls) == 3
    assert np.array_equal(calls[0]['start_q'], q)
    assert calls[0]['total_timeout'] == 0.8
    assert calls[0]['angular_threshold'] == 0.002
    assert np.array_equal(calls[1]['start'], calls[2]['start'])
    assert np.array_equal(calls[1]['current_q'], calls[2]['current_q'])
    assert np.array_equal(path[-1], np.full(14, 2.0))
    assert np.array_equal(updates[0], path[-1])

    q[12] = 0.002
    controller.plan_to_ik_target(ik_timeout=0.8)
    assert np.array_equal(calls[3]['preferred_positions'], goals[1][mask])
    assert calls[3]['start_q'][12] == 0.002

    controller.plan_to_ik_target()
    assert calls[6]['total_timeout'] == 3.0
    assert calls[6]['timeout'] == 0.5
    assert calls[6]['alpha'] == 0.1

    controller.planner.planner_config.request_planning_budget = 1e-9
    with pytest.raises(TimeoutError, match='planning deadline exceeded'):
        controller.plan_to_ik_target(ik_timeout=0.8)
    assert len(calls) == 9


@pytest.mark.parametrize('options, planning_budget, response_timeout', [
    ({}, 6.0, 15.0),
    ({'planning_budget': 2.5, 'response_timeout': 3.5}, 2.5, 3.5),
    ({'local_frames': ()}, None, None),
    ({'local_frames': ('left_grasp_frame',),
      'planning_budget': 11.0, 'response_timeout': 9.0}, 11.0, 9.0),
])
def test_named_controller_goal_enables_moving_arm_locality(
        options, planning_budget, response_timeout):
    controller = object.__new__(UpperController)
    controller.dt = 1.0 / 30.0
    controller.config = {'planner': {'planner': 'RRTConnect'}}
    q = np.zeros(27)
    reduced_mask = np.r_[np.zeros(13, dtype=bool), np.ones(14, dtype=bool)]
    controller.robot_model = SimpleNamespace(
        state={'q': q}, reduced_mask=reduced_mask,
        reduced_reference_q=q.copy(),
    )
    controller.check_state_fresh = lambda: None
    controller._command_seed = lambda measured: (measured.copy(), True)
    controller._planner_speed_overrides = lambda: (None, None)
    controller.planner = SimpleNamespace(
        planner_config=SimpleNamespace(
            frame_names=('left_grasp_frame', 'right_grasp_frame'),
            named_planning_budget=6.0, worker_request_timeout=15.0,
        ),
    )
    calls = []
    def plan_configuration(**kwargs):
        calls.append(kwargs)
        return PlanResult(
            success=True,
            path=np.vstack([kwargs['start'], kwargs['goal']]),
        )

    controller.planner.plan_configuration = plan_configuration
    goal = np.zeros(14)
    goal[0] = 0.3

    controller._plan_configuration_result(goal, **options)

    assert calls[0]['local_frames'] == options.get('local_frames', ('left_grasp_frame',))
    assert calls[0]['local_joint_padding'] == (None if 'local_frames' in options else 0.0)
    assert calls[0]['planning_budget'] == planning_budget
    assert calls[0]['response_timeout'] == response_timeout
    assert np.array_equal(calls[0]['start'], np.zeros(14))


def test_offline_named_goal_uses_named_planning_deadline(monkeypatch):
    pipeline = object.__new__(KinematicPipeline)
    pipeline.planner_config = SimpleNamespace(named_planning_budget=6.0)
    pipeline.robot_model = SimpleNamespace(state_reduced={'q': np.zeros(14)})
    calls = []

    def plan(start, goal, **kwargs):
        calls.append(kwargs)
        return PlanResult(success=True, path=np.vstack([start, goal]))

    pipeline.planner = SimpleNamespace(
        frame_names=('left_grasp_frame', 'right_grasp_frame'), plan=plan,
    )
    monkeypatch.setattr(
        'h12_ros2_controller.core.planner.kinematic_pipeline.time.perf_counter',
        lambda: 100.0,
    )

    result = pipeline.plan_named('home')

    assert result.plan.success
    assert calls[0]['deadline'] == 106.0
    assert calls[0]['local_joint_padding'] == 0.0


@pytest.mark.parametrize('profile', [
    'safety_split.yaml', 'safety_full.yaml', 'sim_safety_split.yaml',
    'counter_safety_split.yaml', 'real_benchmark_upper.yaml',
])
def test_yaml_planning_budgets(profile):
    config = load_controller_config(profile)['planner']
    expected = {
        'named_planning_budget': 6.0, 'candidate_planning_budget': 4.0,
        'timeout': 2.0, 'candidate_ompl_timeout': 1.5,
        'request_planning_budget': 12.0, 'worker_request_timeout': 15.0,
        'prefer_coordinated_motion': True, 'coordination_search_fraction': 0.5,
        'ik_total_timeout': 3.0, 'ik_seed_timeout': 0.5,
    }
    for key, value in expected.items():
        assert config[key] == value, key


def test_execution_preflight_rejects_invalid_timing_without_commands():
    controller = object.__new__(UpperController)
    controller.dt = 1.0 / 30.0
    controller.robot_model = SimpleNamespace(model_body_reduced=SimpleNamespace(
        nq=14, velocityLimit=np.full(14, 9.0),
    ))
    config = PlannerConfig(dt=controller.dt, joint_velocity_limit=0.9,
                           joint_acceleration_limit=25.0)
    controller.planner = SimpleNamespace(planner_config=config)
    path = np.zeros((3, 14))
    path[1, 0], path[2, 0] = 0.01, 0.02
    controller._validate_path_timing(path)

    with pytest.raises(ValueError, match='velocity limits'):
        controller._validate_path_timing(np.array([path[0], np.full(14, 0.1)]))
    path[1, 0], path[2, 0] = 0.03, 0.03
    with pytest.raises(ValueError, match='acceleration limits'):
        controller._validate_path_timing(path)
    config.dt = 0.1
    with pytest.raises(ValueError, match='playback period'):
        controller._validate_path_timing(path)
    with pytest.raises(ValueError, match='empty path'):
        controller.execute_path(np.empty((0, 14)))


@pytest.mark.parametrize('entrypoint', (UpperController.execute_path, demo.execute_path))
def test_execution_requires_fresh_matching_unclipped_plan(monkeypatch, entrypoint):
    controller = object.__new__(UpperController)
    controller.dt = 1.0 / 30.0
    controller.max_state_lag_ms = 0.0
    mask = np.r_[np.zeros(13, dtype=bool), np.ones(14, dtype=bool)]
    q = np.zeros(27)
    controller.robot_model = SimpleNamespace(
        state={'q': q}, reduced_mask=mask,
        model_body_reduced=SimpleNamespace(nq=14, velocityLimit=np.full(14, 9.0)),
        check_within_limits_reduced=lambda values: True,
        check_collision_free=lambda values, min_distance=0.0: True,
    )
    controller.enabled_ids = list(range(13, 27))
    controller.low_cmd_handler = SimpleNamespace(
        q_cmd=q.copy(), _q_clip_limits=np.tile([-1.0, 1.0], (27, 1)),
    )
    controller.planner = SimpleNamespace(planner_config=PlannerConfig(dt=controller.dt))
    path = np.zeros((2, 14))
    commands = []
    controller._apply_joint_position = lambda values: commands.append(values.copy())
    controller.update_robot_model = lambda: None
    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.time.sleep',
                        lambda seconds: None)
    context = {
        'planned_at': time.monotonic(), 'start': path[0].copy(),
        'frozen': q[~mask].copy(), 'active_mask': None, 'dt': controller.dt,
    }
    controller.last_plan_result = PlanResult(
        success=True, path=path, metadata={'execution_context': context},
    )
    execute = lambda values: entrypoint(controller, values)

    with pytest.raises(ValueError, match='most recent validated plan'):
        execute(np.full((2, 14), 0.001))
    assert not commands

    controller.low_cmd_handler.q_cmd[13:] = 0.08
    with pytest.raises(ValueError, match='drifted from planned start'):
        execute(path)
    controller.low_cmd_handler.q_cmd[13:] = 0.02
    with pytest.raises(ValueError, match='drifted from planned start'):
        execute(path)
    controller.low_cmd_handler.q_cmd[:] = 0.0
    q[0] = 0.02
    execute(path)
    assert len(commands) == 2
    assert commands[-1][0] == 0.02
    commands.clear()
    q[0] = 0.0
    context['planned_at'] -= 31.0
    with pytest.raises(ValueError, match='too old'):
        execute(path)
    context['planned_at'] = time.monotonic()
    controller.low_cmd_handler._q_clip_limits[13:, 0] = 0.01
    with pytest.raises(ValueError, match='clipped'):
        execute(path)
    controller.low_cmd_handler._q_clip_limits[13:, 0] = -1.0

    clearances = []
    controller.robot_model.check_collision_free = (
        lambda values, min_distance=0.0:
        clearances.append((values.copy(), min_distance)) or True
    )
    execute(path)
    assert len(commands) == 2
    assert [minimum for _, minimum in clearances] == (
        [controller.planner.planner_config.self_collision_clearance] * 2
    )

    commands.clear()
    def command_then_drift(values):
        commands.append(values.copy())
        q[0] = 0.02

    controller._apply_joint_position = command_then_drift
    execute(path)
    assert len(commands) == 2
    assert clearances[-1][0][0] == 0.02

    commands.clear()
    q[0] = 0.0
    controller.robot_model.check_collision_free = (
        lambda values, min_distance=0.0: values[0] < 0.01
    )
    with pytest.raises(ValueError, match='current self-collision clearance'):
        execute(path)
    assert len(commands) == 1


def test_execution_cancellation_stops_before_next_command(monkeypatch):
    controller = object.__new__(UpperController)
    controller.dt = 1.0 / 30.0
    controller.max_state_lag_ms = 0.0
    controller.robot_model = SimpleNamespace(
        model_body_reduced=SimpleNamespace(nq=14), state={'q': np.zeros(27)},
        reduced_mask=np.r_[np.zeros(13, dtype=bool), np.ones(14, dtype=bool)],
    )
    commands = []
    controller._apply_joint_position = lambda values: commands.append(values.copy())
    controller.update_robot_model = lambda: None
    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.time.sleep',
                        lambda seconds: None)
    path = np.zeros((3, 14))

    with pytest.raises(PathExecutionCancelled):
        controller.execute_path(path, validate=False, cancel_check=lambda: True)
    assert not commands

    with pytest.raises(PathExecutionCancelled):
        controller.execute_path(
            path, validate=False, cancel_check=lambda: len(commands) == 1,
        )
    assert len(commands) == 1


def test_full_body_clearance_accepts_small_measured_torso_changes():
    pipeline = KinematicPipeline(config_name='sim_safety_split.yaml')
    robot = pipeline.robot_model
    q = robot.state['q'].copy()

    for torso in (0.0, 0.02, 0.05):
        q[BODY_JOINTS.index('torso_joint')] = torso
        assert robot.check_collision_free(q, min_distance=pipeline.planner.config.self_collision_clearance)


@pytest.mark.parametrize('startup_fails', (False, True))
def test_controller_warmup_overlaps_startup_and_cleans_up_failed_init(monkeypatch, startup_fails):
    events = []

    class FakeModel:
        def __init__(self, *args, **kwargs):
            q = np.zeros(27)
            self.model_body = SimpleNamespace(nv=27)
            self.state = {'q': q}
            self.state_reduced = {'q': q[13:]}
            self.visualizer = None

        def init_subscriber(self, **kwargs):
            pass

        def update_kinematics(self):
            pass

        def init_reduced_model(self, joints):
            pass

        def init_collision_model(self, *args):
            pass

        def shutdown(self):
            events.append('model.shutdown')

    class FakeCommands:
        def __init__(self, *args, **kwargs):
            pass

        def enable_motors(self, *args):
            pass

        def start(self):
            events.append('publisher.start')

        def stop_recording(self):
            pass

        def shutdown(self):
            events.append('publisher.shutdown')

    class FakePlanner:
        def __init__(self, *args, **kwargs):
            pass

        def warmup_background(self):
            events.append('planner.background')

        def shutdown(self):
            events.append('planner.shutdown')

    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.RobotModel',
                        FakeModel)
    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.LowCmdHandler',
                        FakeCommands)
    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.IKSolver',
                        lambda *args, **kwargs: SimpleNamespace())
    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.PlannerClient',
                        FakePlanner)
    monkeypatch.setattr('h12_ros2_controller.core.controller.upper_controller.time.sleep',
                        lambda seconds: None)
    config = load_controller_config('sim_safety_split.yaml')

    def startup(self, *args):
        events.append('controller.startup')
        if startup_fails:
            raise RuntimeError('startup failed')

    monkeypatch.setattr(UpperController, '_startup_routine', startup)

    if startup_fails:
        with pytest.raises(RuntimeError, match='startup failed'):
            UpperController('', '', '', init=True, config=config)
        assert events == [
            'publisher.start', 'planner.background', 'controller.startup', 'planner.shutdown',
            'model.shutdown', 'publisher.shutdown',
        ]
    else:
        controller = UpperController('', '', '', init=True, config=config)
        assert events == ['publisher.start', 'planner.background', 'controller.startup']
        controller.shutdown()


class CommandHandler:
    def __init__(self):
        self._estopped = False
        self.q_cmd = np.zeros(27)
        self.dq_cmd = np.zeros(27)
        self.tau_cmd = np.zeros(27)
        self.calls = []

    def set_joint_commands(self, q, dq, tau, joint_ids=None):
        if joint_ids is None:
            joint_ids = list(range(27))
        self.calls.append((np.copy(q), np.copy(dq), list(joint_ids)))
        self.q_cmd[joint_ids] = q
        self.dq_cmd[joint_ids] = dq
        self.tau_cmd[joint_ids] = tau


def controller_fixture():
    q = np.zeros(27)
    state = {'q': q, 'received': True, 'tick_valid': True,
             'arrival_monotonic': time.monotonic()}
    mask = np.zeros(27, dtype=bool)
    mask[13:] = True
    model = SimpleNamespace(
        state=state, state_reduced={'q': q[mask].copy()}, reduced_mask=mask,
        check_within_limits_reduced=lambda value: True,
        check_collision_free_reduced=lambda value, min_distance=0.0: True,
        check_collision_free=lambda value, min_distance=0.0: True,
        dynamics=SimpleNamespace(get_gravity_compensation=lambda value: np.ones(27)),
    )
    controller = SimpleNamespace(
        robot_model=model, low_cmd_handler=CommandHandler(),
        arm_ids=list(range(13, 27)), moving_arm='left', dt=0.1, dq_lim=1.,
        torso_target=0.0, d_min=0.02,
        config={'controller': {'timeout': 0.02, 'threshold_ik': 0.0001,
                               'threshold_joint': 0.01,
                               'threshold_linear': 0.01, 'threshold_angular': 0.01}},
        balance_calls=[],
    )
    controller.control_configuration_step = lambda q, dq, balance_scale: (
        controller.balance_calls.append((q.copy(), dq.copy(), balance_scale))
    )
    controller._move_torso_to_target = lambda start, target: None
    controller.init_steady_state = lambda: None
    controller.steady_state_step = lambda threshold: False
    controller.reduced_configuration_error = np.zeros(14)
    return controller


def test_demo_does_not_gate_motion_on_lowstate_age_flags():
    controller = controller_fixture()
    controller.robot_model.state.update(
        received=False, tick_valid=False, arrival_monotonic=0.0,
    )

    assert np.array_equal(demo.fresh_state(controller), np.zeros(27))
    controller.robot_model.state['q'][13] = np.nan
    with pytest.raises(ValueError, match='finite'):
        demo.fresh_state(controller)


def test_off_mode_writes_upper_joints_without_balance():
    controller = controller_fixture()
    q = np.zeros(27)
    q[13] = 0.2
    dq = np.zeros(27)
    dq[13] = 0.5

    demo.send(controller, q, dq, balanced=False)

    sent_q, sent_dq, ids = controller.low_cmd_handler.calls[-1]
    assert ids == list(UPPER_BODY_INDEX)
    assert sent_q[1] == 0.2
    assert np.array_equal(sent_dq, np.zeros(len(ids)))
    assert not controller.balance_calls
    assert np.array_equal(controller.low_cmd_handler.q_cmd[:12], np.zeros(12))


def test_on_mode_passes_moving_velocity_to_counter_controller():
    controller = controller_fixture()
    q = np.zeros(27)
    dq = np.zeros(27)
    q[13] = 0.2
    dq[13] = 0.4

    demo.send(controller, q, dq, balanced=True)

    sent_q, sent_dq, scale = controller.balance_calls[-1]
    assert sent_q[0] == 0.2
    assert sent_dq[0] == 0.4
    assert scale == 1.0
    assert not controller.low_cmd_handler.calls


def test_unbalanced_path_delegates_unchanged_without_custom_playback(monkeypatch):
    controller = controller_fixture()
    path = np.zeros((2, 14))
    calls = []
    controller.execute_path = lambda values: calls.append(values) or 'executed'
    monkeypatch.setattr(demo, 'check_path', lambda *args: pytest.fail('custom preflight'))

    assert demo.execute_path(controller, path) == 'executed'
    assert calls[0] is path
    assert not controller.low_cmd_handler.calls
    assert not controller.balance_calls


def test_live_command_skips_planner_joint_and_collision_checks():
    controller = controller_fixture()
    controller.robot_model.check_collision_free_reduced = (
        lambda value, min_distance=0.0: False
    )
    controller.robot_model.check_within_limits_reduced = lambda value: False

    demo.send(controller, np.zeros(27), np.zeros(27), balanced=False)

    assert controller.low_cmd_handler.calls
    with pytest.raises(ValueError, match='joint position limits'):
        demo.send(controller, np.zeros(27), np.zeros(27), balanced=False,
                  check_collision=True)
    controller.robot_model.check_within_limits_reduced = lambda value: True
    with pytest.raises(ValueError, match='self-collision'):
        demo.send(controller, np.zeros(27), np.zeros(27), balanced=False,
                  check_collision=True)


@pytest.mark.parametrize('counter_ids', [list(range(13, 20)), list(range(20, 27))])
@pytest.mark.parametrize('start', [0.6, 2.0])
def test_counter_arm_fades_smoothly_at_bounded_speed(monkeypatch, capsys, counter_ids, start):
    controller = controller_fixture()
    controller.dt = 0.1
    controller.counter_arm_ids = counter_ids
    controller.low_cmd_handler.q_cmd[counter_ids] = start
    controller.low_cmd_handler.q_cmd[12] = 0.4
    clock = [0.0]
    original = controller.low_cmd_handler.set_joint_commands

    def command(*, q, dq, tau, joint_ids):
        original(q=q, dq=dq, tau=tau, joint_ids=joint_ids)
        if joint_ids == counter_ids:
            controller.robot_model.state['q'][counter_ids] = q

    controller.low_cmd_handler.set_joint_commands = command
    monkeypatch.setattr(demo, 'fresh_state',
                        lambda c: c.robot_model.state['q'].copy())
    monkeypatch.setattr(demo.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(demo.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))

    assert demo.recover_counter_arm(controller)

    fade = [q for q, dq, ids in controller.low_cmd_handler.calls if ids == counter_ids]
    assert len(fade) >= 80
    assert np.array_equal(fade[0], np.full(7, start))
    assert np.array_equal(fade[-1], np.zeros(7))
    steps = np.abs(np.diff(fade, axis=0))
    assert np.max(steps) <= 0.25 * controller.dt + 1e-6
    assert np.max(steps[0]) < np.max(steps[len(steps) // 2])
    assert np.max(steps[-1]) < np.max(steps[len(steps) // 2])
    assert controller.low_cmd_handler.q_cmd[12] == 0.4
    assert controller.low_cmd_handler.calls[-1][2] == list(UPPER_BODY_INDEX)
    assert 'zero-posture error: 0.0000 rad' in capsys.readouterr().out


def test_counter_arm_fade_stops_on_invalid_gravity():
    controller = controller_fixture()
    controller.counter_arm_ids = list(range(20, 27))
    controller.robot_model.dynamics.get_gravity_compensation = lambda measured: np.full(27, np.nan)

    with pytest.raises(ValueError, match='Invalid gravity compensation'):
        demo.recover_counter_arm(controller)

    assert controller.low_cmd_handler.calls[-1][2] == list(UPPER_BODY_INDEX)


def test_counter_arm_fade_skips_unsafe_return_before_motion():
    controller = controller_fixture()
    controller.counter_arm_ids = list(range(20, 27))
    controller.low_cmd_handler.q_cmd[20:] = 0.5
    controller.robot_model.check_collision_free = lambda q, min_distance=0.0: q[20] > 0.2

    assert demo.recover_counter_arm(controller) is False

    assert np.allclose(controller.low_cmd_handler.q_cmd[20:], 0.5)
    assert len(controller.low_cmd_handler.calls) == 1


def test_planned_settling_runs_i_control_and_resets_mode_on_error():
    controller = controller_fixture()
    calls = []
    controller.init_steady_state = lambda: calls.append('init')

    def step(threshold):
        assert controller.plain_goto and controller.balanced_live
        calls.append('I step')
        raise RuntimeError('I-stage failed')

    controller.steady_state_step = step
    with pytest.raises(RuntimeError, match='I-stage failed'):
        demo.wait_for_target(controller, np.zeros(14), balanced=True)
    assert calls == ['init', 'I step']
    assert not controller.plain_goto and not controller.balanced_live


@pytest.mark.parametrize(('side', 'frame'), [
    ('left', 'left_grasp_frame'), ('right', 'right_grasp_frame'),
])
def test_pose_goal_chooses_fixed_grasp_frame(side, frame):
    controller = controller_fixture()
    calls = []
    controller.clear_frame_tasks = lambda: calls.append(('clear',))
    controller.add_frame_task = lambda *args: calls.append(args)
    controller.set_frame_target = lambda *args, **kwargs: calls.append((args, kwargs))
    pose = np.zeros(6)

    assert demo.prepare_goal(controller, None, side, pose, False) == (None, f'{side}_grasp_task')
    assert calls[-1][1] == frame
    demo.prepare_goal(controller, None, side, pose, True)
    assert calls[-1][0][0] == frame


def test_pose_prompt_matches_existing_frame_examples(monkeypatch, capsys):
    prompts = []
    responses = iter(('1 2', '1 2 3 inf 5 6', '1 2 3 90 0 0'))

    def answer(prompt):
        prompts.append(prompt)
        return next(responses)

    monkeypatch.setattr('builtins.input', answer)

    pose = demo.ask_pose()

    assert prompts == ['Enter x y z roll pitch yaw (degrees): '] * 3
    assert pose[3] == pytest.approx(np.pi / 2)
    assert 'Invalid input. Please enter exactly 6 values.' in capsys.readouterr().out


def test_startup_warms_planner_before_home_and_never_executes_warmup(monkeypatch):
    controller = controller_fixture()
    controller.robot_model.state_reduced['q'][0] = 0.2
    controller.robot_model.state['q'][BODY_JOINTS.index('torso_joint')] = 0.18
    events = []

    def plan(goal, start=None):
        events.append(('plan', np.copy(goal), None if start is None else np.copy(start)))
        return np.stack((controller.robot_model.state_reduced['q'], goal))

    controller.plan_to_configuration = plan
    monkeypatch.setattr(demo, 'execute_path', lambda c, path: events.append(('execute', path)))
    monkeypatch.setattr(demo, 'wait_for_target', lambda *args: None)
    controller._move_torso_to_target = lambda start, target: (
        events.append(('torso', start, target))
    )

    demo.startup_home(controller)

    assert [event[0] for event in events] == ['plan', 'plan', 'execute', 'torso']
    assert np.array_equal(events[0][1], events[0][2])
    assert np.array_equal(events[1][1], np.zeros(14))
    assert events[-1] == ('torso', 0.18, 0.0)


def test_failed_home_plan_does_not_command_motion(monkeypatch):
    controller = controller_fixture()
    controller.plan_to_configuration = lambda *args, **kwargs: (_ for _ in ()).throw(
        RuntimeError('invalid startup position')
    )
    monkeypatch.setattr(demo, 'execute_path', lambda *args: pytest.fail('motion attempted'))
    controller._move_torso_to_target = lambda *args: pytest.fail('torso moved before HOME')

    with pytest.raises(RuntimeError, match='invalid startup position'):
        demo.startup_home(controller)


def test_unsettled_home_is_advisory_not_startup_failure(monkeypatch, capsys):
    controller = controller_fixture()
    torso_moves = []
    controller.plan_to_configuration = lambda goal, start=None: np.zeros((2, 14))
    monkeypatch.setattr(demo, 'execute_path', lambda *args: None)
    controller._move_torso_to_target = lambda start, target: torso_moves.append((start, target))

    demo.startup_home(controller)

    assert torso_moves == [(0.0, 0.0)]
    assert 'HOME and torso commands complete' in capsys.readouterr().out


def test_unsettled_goal_returns_false_without_timeout_error(capsys):
    controller = controller_fixture()
    controller.dt = 0.001

    assert demo.wait_for_target(controller, np.ones(14)) is False

    assert 'Position not fully settled' in capsys.readouterr().out


def test_balanced_live_goto_timeout_holds_without_raising(monkeypatch, capsys):
    controller = controller_fixture()
    controller.dt = 0.1
    controller.update_ik_solver = lambda: None
    controller.update_robot_model = lambda: None
    controller.goto_reduced_configuration = lambda target: np.ones(27)
    controller._limit_joint_vel = lambda velocity: velocity
    controller.robot_model.check_collision_free_reduced = (
        lambda value, min_distance=0.0: False
    )
    controller.ik_solver = SimpleNamespace(
        goto_reduced_configuration=lambda target: np.ones(27),
        integrate=lambda velocity: None,
        q=np.zeros(27),
    )
    clock = [0.0]
    monkeypatch.setattr(demo, 'fresh_state',
                        lambda c: c.robot_model.state['q'].copy())
    monkeypatch.setattr(demo.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(demo.time, 'sleep',
                        lambda seconds: clock.__setitem__(0, clock[0] + seconds))

    assert demo.live_goto(controller, np.ones(14), balanced=True) is False

    output = capsys.readouterr().out
    assert 'Timed out before IK convergence' in output
    assert 'Balanced IK timing:' in output
    assert 'Counter statuses during motion:' in output
    assert controller.balanced_live is False


@pytest.mark.parametrize('frame_task', (None, 'left_grasp_task'))
@pytest.mark.parametrize('balanced', (False, True))
def test_plain_live_goto_uses_inherited_frame_steps_and_i_hold(frame_task, balanced, capsys):
    controller = controller_fixture()
    controller.dt = 0.001
    controller.config['controller']['timeout'] = 0.05
    calls = []
    controller.plain_goto = False
    controller.update_ik_solver = lambda: calls.append('sync')
    controller.goto_reduced_configuration = lambda target: (
        calls.append('configuration') or np.zeros(27)
    )
    controller.control_step_reduced = lambda com: (
        calls.append(('frame', com)) or np.zeros(27)
    )
    controller.get_frame_task_error = lambda task: np.zeros(6)
    controller.init_steady_state = lambda: calls.append('init_i')
    controller.steady_state_step = lambda threshold: calls.append('i_tick') or True

    assert demo.live_goto(controller, np.zeros(14), balanced=balanced,
                          frame_task=frame_task)

    assert calls == ['sync', ('frame', False) if frame_task else 'configuration',
                     'init_i', 'i_tick']
    assert controller.plain_goto is False
    assert not controller.balance_calls
    assert 'IK timing:' in capsys.readouterr().out
    assert controller.balanced_live is False


@pytest.mark.parametrize('moving_arm', ('left', 'right'))
def test_balanced_and_plain_integrate_identical_moving_commands(moving_arm):
    histories = []
    predictions = []
    for balanced in (False, True):
        controller = object.__new__(demo.DemoController)
        fake = controller_fixture()
        controller.robot_model = fake.robot_model
        controller.robot_model.model_body = SimpleNamespace(nv=27)
        controller.low_cmd_handler = fake.low_cmd_handler
        controller._tau_bias = np.zeros(27)
        controller._tau_bias_decay = 1.0
        controller.arm_ids = list(range(13, 27))
        local = list(range(7)) if moving_arm == 'left' else list(range(7, 14))
        controller.moving_local = np.asarray(local)
        controller.moving_ids = [13 + index for index in local]
        controller.plain_goto = True
        controller.balanced_live = balanced
        controller.planned_balance = False
        controller.update_robot_model = lambda: None
        controller.ik_solver = SimpleNamespace(q=np.zeros(27))
        controller.ik_solver.integrate = lambda velocity: (
            controller.ik_solver.q.__iadd__(velocity / 30.0)
        )

        def balance(q, dq, balance_scale):
            predictions.append(dq.copy())
            controller._write_arm_command(q, dq, np.ones(14))
            controller._record_moving_command_error(q, dq)
            assert controller.latest_moving_velocity_command_error == 0.0

        controller.control_configuration_step = balance
        velocity = np.zeros(27)
        velocity[controller.moving_ids] = 0.3
        positions = []
        for _ in range(10):
            controller._apply_velocity_command(velocity)
            positions.append(controller.low_cmd_handler.q_cmd[controller.moving_ids].copy())
            assert np.array_equal(controller.low_cmd_handler.dq_cmd[controller.moving_ids],
                                  np.zeros(7))
        histories.append(positions)

    assert np.allclose(histories[0], histories[1])
    assert np.allclose(histories[1][-1], 0.1)
    assert all(np.allclose(dq[local], 0.3) for dq in predictions)


def test_balanced_settling_keeps_counter_pose_and_moving_i_torque():
    controller = object.__new__(demo.DemoController)
    controller.plain_goto = True
    controller.balanced_live = True
    controller.moving_ids = list(range(13, 20))
    controller.low_cmd_handler = CommandHandler()
    controller.low_cmd_handler.q_cmd[20:] = 0.4
    controller.low_cmd_handler.tau_cmd[20:] = 2.0
    controller._publish_position_command(np.full(27, 0.2), np.zeros(27), np.full(27, 3.0))

    assert np.allclose(controller.low_cmd_handler.q_cmd[20:], 0.4)
    assert np.allclose(controller.low_cmd_handler.tau_cmd[20:], 2.0)
    assert np.allclose(controller.low_cmd_handler.q_cmd[13:20], 0.2)
    assert np.allclose(controller.low_cmd_handler.tau_cmd[13:20], 3.0)


def test_planned_balanced_collision_rejected_before_publication():
    controller = object.__new__(demo.DemoController)
    fake = controller_fixture()
    controller.planned_balance = True
    controller.arm_ids = list(range(13, 27))
    controller.d_min = fake.d_min
    controller.robot_model = fake.robot_model
    controller.robot_model.state['q'][2] = 0.5
    candidates = []

    def collision(q, min_distance=0.0):
        candidates.append((q.copy(), min_distance))
        return 0.01 >= min_distance

    controller.robot_model.check_collision_free = collision
    controller.low_cmd_handler = fake.low_cmd_handler

    with pytest.raises(ValueError, match='Combined planned'):
        controller._write_arm_command(np.zeros(14), np.zeros(14), np.zeros(14))
    assert not controller.low_cmd_handler.calls
    assert candidates[0][0][2] == 0.5
    assert candidates[0][1] == controller.d_min


def test_balanced_clearance_uses_each_measured_full_body_snapshot():
    controller = controller_fixture()
    candidates = []

    def collision(q, min_distance=0.0):
        candidates.append((q.copy(), min_distance))
        return q[2] < 0.5

    controller.robot_model.check_collision_free = collision
    arm_q = np.full(14, 0.1)
    assert demo.valid_arm_command(controller, arm_q)
    controller.robot_model.state['q'][2] = 0.5
    assert not demo.valid_arm_command(controller, arm_q)
    assert [q[2] for q, _ in candidates] == [0.0, 0.5]
    assert all(np.array_equal(q[13:], arm_q) for q, _ in candidates)
    assert all(margin == controller.d_min for _, margin in candidates)
    assert np.array_equal(controller.robot_model.state['q'][13:], np.zeros(14))


@pytest.mark.parametrize('delta, acceleration_limit, error', [
    (0.05, 25.0, None), (0.2, 25.0, 'velocity limits'),
    (0.05, 0.1, 'acceleration limits'),
])
def test_validated_plan_is_not_retimed_and_uses_controller_timing(
        delta, acceleration_limit, error):
    controller = controller_fixture()
    controller.dq_lim = 0.01
    controller.planner = SimpleNamespace(planner_config=PlannerConfig(
        dt=controller.dt, joint_acceleration_limit=acceleration_limit,
    ))
    controller.robot_model.model_body_reduced = SimpleNamespace(
        velocityLimit=np.full(14, 9.0),
    )
    controller._validate_path_timing = lambda path: UpperController._validate_path_timing(
        controller, path,
    )
    path = np.zeros((2, 14))
    path[1, 0] = delta
    original = path.copy()
    controller.last_plan_result = PlanResult(
        success=True, path=path.copy(),
        metadata={'execution_context': {'start': path[0].copy()}},
    )

    if error:
        with pytest.raises(ValueError, match=error):
            demo.check_path(controller, path)
    else:
        checked, velocities = demo.check_path(controller, path)
        assert checked is path
        assert np.array_equal(velocities, np.diff(original, axis=0) / controller.dt)
    assert np.array_equal(path, original)
    assert np.array_equal(controller.last_plan_result.path, original)
    assert not controller.low_cmd_handler.calls


def test_retiming_is_idempotent():
    controller = controller_fixture()
    path = np.zeros((2, 14))
    path[1, 0] = 0.9
    first, _ = demo.check_path(controller, path)
    second, _ = demo.check_path(controller, first)
    assert np.array_equal(first, second)


def test_plain_live_goto_resets_mode_on_error():
    controller = controller_fixture()
    controller.plain_goto = False
    controller.update_ik_solver = lambda: None
    controller.goto_reduced_configuration = lambda target: (_ for _ in ()).throw(
        RuntimeError('controller fault')
    )

    with pytest.raises(RuntimeError, match='controller fault'):
        demo.live_goto(controller, np.ones(14), balanced=False)

    assert controller.plain_goto is False


def test_plain_publication_bypasses_counter_overlay():
    controller = object.__new__(demo.DemoController)
    calls = []
    controller.plain_goto = True
    controller.balanced_live = False
    controller.upper_ids = list(UPPER_BODY_INDEX)
    controller.low_cmd_handler = SimpleNamespace(
        set_joint_commands=lambda *args: calls.append(args),
    )
    q = np.zeros(27)
    dq = np.zeros(27)
    tau = np.zeros(27)

    controller._publish_position_command(q, dq, tau)

    assert calls == [(q, dq, tau)]
    assert controller._steady_state_control_ids() == list(UPPER_BODY_INDEX)


def test_demo_does_not_override_original_velocity_limiters():
    assert '_limit_joint_vel' not in demo.DemoController.__dict__
    assert '_finalize_counter_velocity' not in demo.DemoController.__dict__


def test_path_rejects_stale_start_and_retimes_excessive_velocity():
    controller = controller_fixture()
    path = np.zeros((2, 14))
    path[0, 0] = 0.1
    with pytest.raises(RuntimeError, match='Robot moved'):
        demo.check_path(controller, path)
    path[0, 0] = 0
    path[1, 0] = 0.2
    retimed, velocities = demo.check_path(controller, path)

    assert np.array_equal(retimed[0], path[0])
    assert np.array_equal(retimed[-1], path[-1])
    assert len(retimed) > len(path)
    assert np.max(np.abs(velocities)) <= controller.dq_lim + 1e-6


def test_command_seeded_plan_is_not_rejected_for_safe_measured_lag():
    controller = controller_fixture()
    controller._validate_path_timing = lambda path: None
    controller.low_cmd_handler.q_cmd[13] = 0.08
    path = np.zeros((2, 14))
    path[:, 0] = (0.08, 0.09)
    controller.last_plan_result = PlanResult(
        success=True, path=path.copy(),
        metadata={'execution_context': {'start': path[0].copy()}},
    )

    checked, _ = demo.check_path(controller, path)

    assert np.array_equal(checked[0], path[0])
    controller.low_cmd_handler.q_cmd[13] = 0.12
    with pytest.raises(RuntimeError, match='Robot moved'):
        demo.check_path(controller, path)
    assert len(controller.low_cmd_handler.calls) == 0


def test_retimed_path_checks_intermediate_waypoints_before_motion():
    controller = controller_fixture()
    path = np.zeros((2, 14))
    path[1, 0] = 0.3
    controller.robot_model.check_collision_free_reduced = (
        lambda waypoint, min_distance=0.0: not 0.05 < waypoint[0] < 0.25
    )

    with pytest.raises(ValueError, match='invalid waypoint'):
        demo.execute_path(controller, path, balanced=True)

    assert not controller.low_cmd_handler.calls


def test_retimed_path_preserves_turning_waypoint_and_moving_velocity(monkeypatch):
    controller = controller_fixture()
    path = np.zeros((3, 14))
    path[1, 0] = 0.2
    path[2, 1] = 0.2
    commands = []
    monkeypatch.setattr(demo, 'send', lambda c, q, dq, balanced, check_collision:
                        commands.append((q[13:27].copy(), dq[13:27].copy(), balanced)))
    monkeypatch.setattr(demo.time, 'sleep', lambda seconds: None)

    demo.execute_path(controller, path, balanced=True)

    assert any(np.allclose(command[0], path[1]) for command in commands)
    assert np.allclose(commands[-1][0], path[-1])
    assert all(command[2] for command in commands)
    assert all(np.max(np.abs(command[1])) <= controller.dq_lim + 1e-6
               for command in commands)


def test_named_two_arm_goal_cannot_enable_counter_balance():
    controller = controller_fixture()
    controller.clear_frame_tasks = lambda: None
    controller.set_named_target = lambda name: (_ for _ in ()).throw(
        ValueError('named target must change exactly one moving arm')
    )

    with pytest.raises(ValueError, match='exactly one moving arm'):
        demo.prepare_goal(controller, 't_pose', None, None, True)


def test_unqualified_scan_prepositions_are_not_selectable():
    controller = controller_fixture()
    controller.clear_frame_tasks = lambda: None
    for name in ('scan_left_start', 'scan_right_start'):
        with pytest.raises(ValueError, match='Unknown configuration'):
            demo.prepare_goal(controller, name, None, None, False)


@pytest.mark.parametrize('arm', ('left', 'right'))
def test_overhang_named_waypoints_keep_one_arm_active(arm):
    controller = controller_fixture()
    controller.clear_frame_tasks = lambda: None
    controller._set_arm_ownership = lambda side: setattr(controller, 'moving_arm', side)
    active = slice(0, 7) if arm == 'left' else slice(7, 14)
    inactive = slice(7, 14) if arm == 'left' else slice(0, 7)

    for index in range(1, 5):
        name = f'overhang_{arm}_{index}'
        plain, task = demo.prepare_goal(controller, name, None, None, False)
        balanced, balanced_task = demo.prepare_goal(controller, name, None, None, True)
        assert task is None and balanced_task is None
        assert plain.shape == (14,)
        assert np.array_equal(plain, balanced)
        assert np.count_nonzero(plain[active]) == 7
        assert np.array_equal(plain[inactive], np.zeros(7))
        assert controller.moving_arm == arm


def test_overhang_joint_waypoints_are_mirrored():
    for index in range(1, 5):
        left = demo.OVERHANG_CONFIGS[f'overhang_left_{index}']
        right = demo.OVERHANG_CONFIGS[f'overhang_right_{index}']
        assert np.array_equal(left[7:], np.zeros(7))
        assert np.array_equal(right[:7], np.zeros(7))
        assert np.array_equal(right[7:], left[:7] * demo.OVERHANG_MIRROR)


def test_overhang_waypoints_move_forward_outward_and_up():
    from h12_ros2_controller.core.robot_model import RobotModel
    from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS

    if not os.path.exists(demo.URDF_MAGPIE_PATH):
        pytest.skip('robot assets unavailable')
    robot = RobotModel(demo.URDF_MAGPIE_PATH, handless=False)
    robot.init_reduced_model(ENABLED_JOINTS)
    for arm in ('left', 'right'):
        positions = [
            robot.get_frame_transformation_reduced(
                f'{arm}_grasp_frame', demo.OVERHANG_CONFIGS[f'overhang_{arm}_{index}']
            )[:3, 3]
            for index in range(1, 5)
        ]
        assert positions[1][0] - positions[0][0] > 0.18
        outward = 1 if arm == 'left' else -1
        assert outward * (positions[2][1] - positions[1][1]) > 0.19
        assert positions[3][2] - positions[2][2] > 0.14


def test_main_uses_one_controller_without_staged_startup_and_shuts_down(monkeypatch):
    controller = controller_fixture()
    events = []
    answers = iter(('y', 'y', 'q'))
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    monkeypatch.setattr(demo, 'load_controller_config', lambda name: {
        'mode': 'debug', 'topics': {'low_cmd': 'rt/safety/lowcmd_upper_in'},
        'reactive_counter_balance': {'weights': {'com': 1}},
    })
    monkeypatch.setattr(demo, 'init_channel_factory_guard', lambda config: events.append('domain'))
    monkeypatch.setattr(demo, 'DemoController',
                        lambda *args, **kwargs: (
                            events.append(('construct', kwargs['init'], kwargs['moving_arm']))
                            or controller
                        ))
    monkeypatch.setattr(demo, 'maybe_start_controller_logging', lambda c: None)
    monkeypatch.setattr(demo, 'startup_home', lambda c: events.append('home'))
    monkeypatch.setattr(demo, 'select_planner', lambda *args: None)
    controller.planner = SimpleNamespace(planner_config=PlannerConfig(frame_z_min=0.0))
    controller.shutdown = lambda: events.append('shutdown')

    demo.main()

    assert events == ['domain', ('construct', False, None), 'home', 'shutdown']
    assert controller.low_cmd_handler.calls[-1][2] == list(UPPER_BODY_INDEX)


def test_goal_and_repeat_prompts_match_goto_examples(monkeypatch, capsys):
    controller = controller_fixture()
    controller.planner = SimpleNamespace(planner_config=PlannerConfig(frame_z_min=0.0))
    controller.shutdown = lambda: None
    prompts = []
    answers = iter(('y', 'y', 'home', 'n', 'n', 'n'))

    def answer(prompt):
        prompts.append(prompt)
        return next(answers)

    monkeypatch.setattr('builtins.input', answer)
    monkeypatch.setattr(demo, 'load_controller_config', lambda name: {
        'mode': 'debug', 'topics': {'low_cmd': 'rt/safety/lowcmd_upper_in'},
        'reactive_counter_balance': {'weights': {'com': 1}},
    })
    monkeypatch.setattr(demo, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(demo, 'DemoController', lambda *args, **kwargs: controller)
    monkeypatch.setattr(demo, 'maybe_start_controller_logging', lambda c: None)
    monkeypatch.setattr(demo, 'startup_home', lambda c: None)
    monkeypatch.setattr(demo, 'select_planner', lambda *args: None)
    monkeypatch.setattr(demo, 'prepare_goal', lambda *args: (np.zeros(14), None))
    monkeypatch.setattr(demo, 'live_goto', lambda *args, **kwargs: False)

    demo.main()

    assert 'goto> ' in prompts
    assert 'Do you want to send another goal? (y/n): ' == prompts[-1]
    output = capsys.readouterr().out
    assert 'Available named configs:' in output
    assert 'frame <frame_name> x y z roll pitch yaw' in output


@pytest.mark.parametrize('phase', ('plan', 'execute', 'live'))
@pytest.mark.parametrize('error', (RuntimeError('failed'), AttributeError('bad API'), KeyboardInterrupt()))
def test_goal_failure_returns_to_prompt_and_next_goal_runs(monkeypatch, capsys, phase, error):
    controller = controller_fixture()
    controller.planner = SimpleNamespace()
    calls = []
    controller.shutdown = lambda: calls.append('shutdown')
    answers = ['y', 'y', 'home', 'n' if phase == 'live' else 'y', 'n']
    if phase != 'live':
        answers.append('y')
        if phase == 'execute':
            answers.append('y')
    answers.extend(('home', 'n', 'n', 'n'))
    responses = iter(answers)
    monkeypatch.setattr('builtins.input', lambda prompt: next(responses))
    monkeypatch.setattr(demo, 'load_controller_config', lambda name: {
        'mode': 'debug', 'topics': {'low_cmd': 'rt/safety/lowcmd_upper_in'},
        'reactive_counter_balance': {'weights': {'com': 1}},
    })
    monkeypatch.setattr(demo, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(demo, 'DemoController', lambda *args, **kwargs: controller)
    monkeypatch.setattr(demo, 'maybe_start_controller_logging', lambda c: None)
    monkeypatch.setattr(demo, 'startup_home', lambda c: None)
    monkeypatch.setattr(demo, 'select_planner', lambda *args: None)
    monkeypatch.setattr(demo, 'prepare_goal', lambda *args: (np.zeros(14), None))
    monkeypatch.setattr(demo, 'plan_goal', lambda *args: np.zeros((2, 14)))
    monkeypatch.setattr(demo, 'check_path', lambda c, path: (path, np.zeros((1, 14))))
    monkeypatch.setattr(demo, 'live_goto', lambda *args, **kwargs: calls.append('next goal') or True)
    used = [False]

    def fail_once(*args, **kwargs):
        if not used[0]:
            used[0] = True
            raise error
        calls.append('next goal')
        return True

    operation = {'plan': 'plan_goal', 'execute': 'execute_path', 'live': 'live_goto'}[phase]
    monkeypatch.setattr(demo, operation, fail_once)

    demo.main()

    assert calls == ['next goal', 'shutdown']
    assert 'Returning to goal selection' in capsys.readouterr().out


@pytest.mark.parametrize('planned', (False, True))
def test_balanced_goal_uses_automatic_reference_capture(monkeypatch, planned):
    controller = controller_fixture()
    controller.planner = SimpleNamespace(planner_config=PlannerConfig(frame_z_min=0.0))
    controller.shutdown = lambda: None
    controller.diagnostics = lambda: {'status': 'published'}
    events = []
    answers = ['y', 'y', '', 'left', '0.4 0.4 0.4 0 -30 0',
               'y' if planned else 'n', 'y']
    if planned:
        answers.extend(('y', 'y'))
    answers.append('n')
    responses = iter(answers)
    monkeypatch.setattr('builtins.input', lambda prompt: next(responses))
    monkeypatch.setattr(demo, 'load_controller_config', lambda name: {
        'mode': 'debug', 'topics': {'low_cmd': 'rt/safety/lowcmd_upper_in'},
        'reactive_counter_balance': {'weights': {'com': 1}},
    })
    monkeypatch.setattr(demo, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(demo, 'DemoController', lambda *args, **kwargs: controller)
    monkeypatch.setattr(demo, 'maybe_start_controller_logging', lambda c: None)
    monkeypatch.setattr(demo, 'startup_home', lambda c: None)
    monkeypatch.setattr(demo, 'select_planner', lambda *args: None)
    monkeypatch.setattr(demo, 'prepare_goal', lambda *args: (None, 'left_grasp_task'))
    monkeypatch.setattr(demo, 'plan_goal', lambda *args: np.zeros((2, 14)))
    monkeypatch.setattr(demo, 'check_path', lambda c, path: (path, np.zeros_like(path[1:])))
    monkeypatch.setattr(demo, 'execute_path', lambda c, path, balanced:
                        events.append(('execute', balanced)))
    monkeypatch.setattr(demo, 'wait_for_target', lambda *args, **kwargs: True)
    monkeypatch.setattr(demo, 'live_goto', lambda c, target, balanced, frame_task:
                        events.append(('live', balanced)) or True)
    monkeypatch.setattr(demo, 'recover_counter_arm', lambda c:
                        events.append(('recover', True)) or True)

    demo.main()

    assert events == [('execute' if planned else 'live', True), ('recover', True)]


def test_floor_choice_selects_separate_planner_without_changing_collision_checks(monkeypatch):
    controller = controller_fixture()
    original = SimpleNamespace(
        urdf_path='robot.urdf', urdf_sphere_path='spheres.urdf',
        srdf_sphere_path='spheres.srdf', handless=False,
        planner_config=PlannerConfig(frame_z_min=0.0, prewarm_worker=True),
    )
    controller.planner = original
    created = []
    warmed = []

    def planner_client(*args, planner_config, **kwargs):
        planner = SimpleNamespace(
            planner_config=planner_config,
            warmup_background=lambda: warmed.append(True),
        )
        created.append(planner)
        return planner

    monkeypatch.setattr(demo, 'PlannerClient', planner_client)
    planners = {True: original}

    demo.select_planner(controller, planners, False)
    assert controller.planner is created[0]
    assert controller.planner.planner_config.frame_z_min is None
    assert controller.planner.planner_config.frame_z_corridor_margin == 0.05
    assert original.planner_config.frame_z_min == 0.0
    demo.select_planner(controller, planners, True)
    assert controller.planner is original
    demo.select_planner(controller, planners, False)
    assert controller.planner is created[0]
    assert len(created) == 1
    assert warmed == [True]


def test_disabled_home_floor_is_chosen_before_warmup(monkeypatch):
    controller = controller_fixture()
    original = SimpleNamespace(
        urdf_path='robot.urdf', urdf_sphere_path='spheres.urdf',
        srdf_sphere_path='spheres.srdf', handless=False,
        planner_config=PlannerConfig(frame_z_min=0.0),
    )
    controller.planner = original
    original.shutdown = lambda: None
    events = []
    answers = iter(('y', 'y', 'q'))
    prompts = []

    def answer(prompt):
        prompts.append(prompt)
        return next(answers)

    monkeypatch.setattr('builtins.input', answer)
    monkeypatch.setattr(demo, 'load_controller_config', lambda name: {
        'mode': 'debug', 'topics': {'low_cmd': 'rt/safety/lowcmd_upper_in'},
        'reactive_counter_balance': {'weights': {'com': 1}},
    })
    monkeypatch.setattr(demo, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(demo, 'DemoController', lambda *args, **kwargs: controller)
    monkeypatch.setattr(demo, 'maybe_start_controller_logging', lambda c: None)

    def planner_client(*args, planner_config, **kwargs):
        return SimpleNamespace(
            planner_config=planner_config, shutdown=lambda: events.append('off shutdown'),
        )

    def home(c):
        events.append(('home floor', c.planner.planner_config.frame_z_min))

    monkeypatch.setattr(demo, 'PlannerClient', planner_client)
    monkeypatch.setattr(demo, 'startup_home', home)
    controller.shutdown = lambda: events.append('controller shutdown')

    demo.main()

    assert events == [('home floor', None), 'off shutdown', 'controller shutdown']
    assert controller.planner is original
    assert not any('minimum z' in prompt for prompt in prompts)


def test_goal_can_reenable_floor_after_disabled_home(monkeypatch):
    controller = controller_fixture()
    original = SimpleNamespace(
        urdf_path='robot.urdf', urdf_sphere_path='spheres.urdf',
        srdf_sphere_path='spheres.srdf', handless=False,
        planner_config=PlannerConfig(frame_z_min=0.0),
    )
    controller.planner = original
    events = []
    answers = iter(('y', 'y', 'home', 'y', 'n', 'y', 'n', 'q'))
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    monkeypatch.setattr(demo, 'load_controller_config', lambda name: {
        'mode': 'debug', 'topics': {'low_cmd': 'rt/safety/lowcmd_upper_in'},
        'reactive_counter_balance': {'weights': {'com': 1}},
    })
    monkeypatch.setattr(demo, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(demo, 'DemoController', lambda *args, **kwargs: controller)
    monkeypatch.setattr(demo, 'maybe_start_controller_logging', lambda c: None)
    monkeypatch.setattr(demo, 'startup_home', lambda c: events.append(('home', c.planner)))
    monkeypatch.setattr(demo, 'PlannerClient', lambda *args, planner_config, **kwargs:
                        SimpleNamespace(planner_config=planner_config, shutdown=lambda: None))
    monkeypatch.setattr(demo, 'prepare_goal', lambda *args: (np.zeros(14), None))
    monkeypatch.setattr(demo, 'plan_goal', lambda c, *args:
                        events.append(('goal', c.planner)) or np.zeros((2, 14)))
    monkeypatch.setattr(demo, 'check_path', lambda c, path: (path, np.zeros_like(path[1:])))
    controller.shutdown = lambda: None

    demo.main()

    assert events[0][0] == 'home' and events[0][1] is not original
    assert events[1] == ('goal', original)


def test_noop_warmup_starts_real_planner_worker_without_motion():
    from h12_ros2_controller.core.planner import PlannerClient, PlannerConfig

    assets = (demo.URDF_MAGPIE_PATH, demo.URDF_MAGPIE_SPHERE_PATH,
              demo.SRDF_MAGPIE_SPHERE_PATH)
    if not all(os.path.exists(path) for path in assets):
        pytest.skip('robot assets unavailable')
    planner = PlannerClient(*assets, planner_config=PlannerConfig())
    try:
        planner.warmup_background()
        planner.warmup()
        assert planner._process.is_alive()
        home = np.zeros(14)
        result = planner.plan_configuration(np.zeros(27), home, home)
        assert result.success, result.reason
        timings = result.metadata['client_timings']
        assert timings['total'] >= timings['startup'] >= 0.0
        assert timings['request_transport_and_worker'] >= result.planning_time
        assert np.array_equal(result.path[0], home)
        assert np.array_equal(result.path[-1], home)
        q = np.zeros(27)
        q[BODY_JOINTS.index('torso_joint')] = 0.03
        result = planner.plan_configuration(q, home, home)
        assert result.success, result.reason
        assert np.array_equal(result.path[-1], home)
        result = planner.plan_configuration(
            q, home, home, direct_only=True,
            local_frames=('left_grasp_frame',),
        )
        assert result.success, result.reason
        assert result.metadata['joint_path_ratio'] == pytest.approx(1.0)
    finally:
        planner.shutdown()
