'''Exercise real pacing methods with fake clocks and no ROS/DDS constructors'''

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1] / 'h12_ros2_controller'


class FakeTime:
    def __init__(self, rate=1.0):
        self.wall = 100.0
        self.sim = 10.0
        self.rate = rate
        self.sleeps = []

    def monotonic(self):
        return self.wall

    def time(self):
        return self.wall

    def advance(self, seconds):
        self.wall += seconds
        self.sim += seconds * self.rate

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.advance(seconds)


def load_timing(clock):
    source = ROOT / 'core/control_timing.py'
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    namespace = {'time': clock, 'math': math}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['ControlTiming']


def load_controller(clock):
    source = ROOT / 'core/controller/upper_controller.py'
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == 'UpperController')
    names = {'control_time', 'wait_control_tick', 'execute_path',
             '_move_to_reduced_configuration', '_move_torso_to_target'}
    cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name in names]
    namespace = {'time': clock, 'np': np, 'PathExecutionCancelled': InterruptedError,
                 'tqdm': lambda values, **kwargs: values,
                 'BODY_JOINTS': ['torso_joint']}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    controller = namespace['UpperController']()
    controller.control_timing = load_timing(clock)(lambda: clock.sim)
    controller.dt = 0.1
    controller.timeout = 0.5
    controller.check_state_fresh = lambda: None
    return controller


@pytest.mark.parametrize('rate', [0.37, 1.0, 2.0])
def test_fixed_dt_follows_simulation_rate_not_wall(rate):
    clock = FakeTime(rate)
    timing = load_timing(clock)(lambda: clock.sim)
    started = timing.now()
    for _ in range(10):
        assert timing.wait(timing.now(), 0.1, check=lambda: None, wall_deadline=110.0)
    assert clock.sim - started == pytest.approx(1.0, abs=1e-8)
    assert clock.wall - 100.0 == pytest.approx(1.0 / rate, abs=1e-7)


def test_default_is_monotonic_and_subtracts_work_elapsed():
    clock = FakeTime()
    timing = load_timing(clock)()
    started = timing.now()
    clock.advance(0.04)
    assert timing.wait(started, 0.1, check=lambda: None, wall_deadline=101.0)
    assert sum(clock.sleeps) == pytest.approx(0.06, abs=1e-8)
    assert timing.now() - started == pytest.approx(0.1, abs=1e-8)


def test_wait_rechecks_dynamic_elapsed_instead_of_reusing_remaining():
    clock = FakeTime(0.37)
    timing = load_timing(clock)(lambda: clock.sim)
    started = timing.now()
    clock.advance(0.06)
    checks = []

    def check():
        checks.append(clock.sim)
        if len(checks) == 3:
            clock.advance(0.07)

    assert timing.wait(started, 0.1, check=check, wall_deadline=101.0)
    assert clock.sim - started == pytest.approx(0.1, abs=1e-8)
    assert len(checks) > 3


def test_paused_clock_cancels_without_another_tick():
    clock = FakeTime(0.0)
    controller = load_controller(clock)
    assert not controller.wait_control_tick(
        controller.control_time(), cancel_check=lambda: clock.wall >= 100.03,
    )
    assert clock.sim == 10.0
    assert clock.wall <= 100.04


def test_paused_clock_retains_wall_timeout():
    clock = FakeTime(0.0)
    controller = load_controller(clock)
    with pytest.raises(TimeoutError, match='Control clock did not advance'):
        controller.wait_control_tick(controller.control_time())
    assert clock.wall == pytest.approx(100.5)
    assert not controller.wait_control_tick(
        controller.control_time(), wall_deadline=clock.wall + 0.03,
    )
    assert clock.wall == pytest.approx(100.53)


def test_paused_clock_still_checks_sdk_freshness():
    clock = FakeTime(0.0)
    controller = load_controller(clock)

    def stale():
        if clock.wall >= 100.02:
            raise RuntimeError('SDK stale')

    controller.check_state_fresh = stale
    with pytest.raises(RuntimeError, match='SDK stale'):
        controller.wait_control_tick(controller.control_time())
    assert clock.wall <= 100.03


def test_backward_clock_fails_closed():
    clock = FakeTime(-1.0)
    controller = load_controller(clock)
    with pytest.raises(RuntimeError, match='moved backwards'):
        controller.wait_control_tick(controller.control_time())


def test_playback_waits_on_sim_clock_and_cancels_during_pause():
    clock = FakeTime(0.37)
    controller = load_controller(clock)
    controller._as_reduced_path = np.asarray
    controller.robot_model = SimpleNamespace(
        state={'q': np.zeros(1)}, reduced_mask=np.array([True]),
    )
    commands = []
    controller._apply_joint_position = lambda q: commands.append((clock.sim, q.copy()))
    controller.update_robot_model = lambda: None
    controller.execute_path([[0.0], [0.1], [0.2]], validate=False)
    np.testing.assert_allclose([tick for tick, _ in commands], [10.0, 10.1, 10.2])
    clock.rate = 0.0
    cancel_at = clock.wall + 0.02
    with pytest.raises(InterruptedError):
        controller.execute_path([[0.2], [0.3]], validate=False,
                                cancel_check=lambda: clock.wall >= cancel_at)
    assert len(commands) == 4


def test_startup_arm_and_torso_steps_use_same_clock_without_threshold_changes(capsys):
    clock = FakeTime(0.37)
    controller = load_controller(clock)
    controller.robot_model = SimpleNamespace(
        state_reduced={'q': np.zeros(1)}, state={'q': np.zeros(1)},
    )
    controller.enabled_ids = [0]
    controller.low_cmd_handler = SimpleNamespace(
        q_cmd=np.zeros(1), _q_clip_limits=np.array([[-1.0, 1.0]]),
        set_joint_commands=lambda **kwargs: None,
    )
    controller.goto_reduced_configuration = lambda q: None
    controller.update_robot_model = lambda: None
    controller._move_to_reduced_configuration(np.array([0.2]), steps=2)
    assert clock.sim == pytest.approx(10.2)
    assert 'continuing startup' in capsys.readouterr().out
    controller._move_torso_to_target(0.0, 0.06)
    assert clock.sim == pytest.approx(10.4)


@pytest.mark.parametrize('filename', ['frame_task_server.py', 'dual_arm_server.py'])
def test_all_server_fixed_dt_loops_use_control_clock_and_wall_deadlines(filename):
    tree = ast.parse((ROOT / 'ros2' / filename).read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    loops = [node for node in ast.walk(cls) if isinstance(node, ast.While)]
    control_loops = [node for node in loops if 'start_time' in ast.unparse(node.test)]
    assert len(control_loops) == 4
    for loop in control_loops:
        calls = [node for node in ast.walk(loop) if isinstance(node, ast.Call)]
        waits = [node for node in calls if isinstance(node.func, ast.Attribute)
                 and node.func.attr == 'wait_control_tick']
        assert len(waits) == 1
        assert {'cancel_check', 'wall_deadline'} <= {kw.arg for kw in waits[0].keywords}
        assert any(isinstance(node.func, ast.Attribute) and node.func.attr == 'control_time'
                   for node in calls)
        assert 'time.monotonic()' in ast.unparse(loop.test)


class FakeHumbleExecutor:
    '''Model Humble retaining node.executor after remove_node'''

    def __init__(self):
        self.nodes = []

    def get_nodes(self):
        return list(self.nodes)

    def add_node(self, node):
        self.nodes.append(node)
        node.executor = self

    def remove_node(self, node):
        self.nodes.remove(node)


def test_ros_adapter_pumps_retained_executor_until_actual_main_ownership():
    clock = FakeTime(0.37)
    source = ROOT / 'ros2/control_timing.py'
    tree = ast.parse(source.read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef))
    spins = []
    observed = [0.0]
    startup_executor = FakeHumbleExecutor()
    node = SimpleNamespace(
        executor=None, context=object(), get_parameter=lambda name: SimpleNamespace(value=True),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(observed[0] * 1e9))),
    )

    def spin_once(node, **kwargs):
        startup_executor.add_node(node)
        try:
            spins.append(kwargs)
            # initial callbacks need not include /clock
            if len(spins) >= 3:
                observed[0] = clock.sim
        finally:
            startup_executor.remove_node(node)

    ros = SimpleNamespace(ok=lambda **kwargs: True, spin_once=spin_once)
    namespace = {'time': clock, 'rclpy': ros, 'ControlTiming': load_timing(clock)}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
    timing = namespace['node_control_timing'](node, 0.5)
    assert spins == [{'timeout_sec': 0.0}] * 3
    assert node.executor is startup_executor
    assert node not in startup_executor.get_nodes()
    sim_start, wall_start = timing.now(), clock.wall
    for _ in range(3):
        assert timing.wait(timing.now(), 0.1, check=lambda: None,
                           wall_deadline=clock.wall + 1.0)
    assert timing.now() - sim_start == pytest.approx(0.3, abs=1e-8)
    assert clock.wall - wall_start == pytest.approx(0.3 / 0.37, abs=1e-7)
    startup_spins = len(spins)
    assert startup_spins > 3

    main_executor = FakeHumbleExecutor()
    main_executor.add_node(node)
    sleep = clock.sleep

    def executor_clock_updates(seconds):
        sleep(seconds)
        observed[0] = clock.sim

    clock.sleep = executor_clock_updates
    for _ in range(3):
        assert timing.wait(timing.now(), 0.1, check=lambda: None,
                           wall_deadline=clock.wall + 1.0)
    assert node in main_executor.get_nodes()
    assert len(spins) == startup_spins
    node.get_parameter = lambda name: SimpleNamespace(value=False)
    assert namespace['node_control_timing'](node, 0.5) is None


def test_ros_adapter_missing_clock_times_out_before_controller_construction():
    source = ROOT / 'ros2/control_timing.py'
    tree = ast.parse(source.read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef))
    clock = FakeTime(0.0)
    executor = FakeHumbleExecutor()
    spins = []
    node = SimpleNamespace(
        executor=None, context=object(), get_parameter=lambda name: SimpleNamespace(value=True),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
    )
    def spin_once(node, **kwargs):
        executor.add_node(node)
        spins.append(kwargs)
        executor.remove_node(node)

    namespace = {'time': clock, 'ControlTiming': load_timing(clock),
                 'rclpy': SimpleNamespace(ok=lambda **kwargs: True, spin_once=spin_once)}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
    with pytest.raises(TimeoutError, match='No simulation clock received'):
        namespace['node_control_timing'](node, 0.03)
    assert clock.wall == pytest.approx(100.03)
    assert len(spins) >= 3
    assert node.executor is executor
    assert node not in executor.get_nodes()


@pytest.mark.parametrize('filename, method', [
    ('frame_task_server.py', '_hold_frame_task_callback'),
    ('frame_task_server.py', '_plan_named_config_callback'),
    ('dual_arm_server.py', '_plan_dual_arm_callback'),
    ('dual_arm_server.py', '_plan_named_config_callback'),
])
@pytest.mark.parametrize('paused', [False, True])
@pytest.mark.parametrize('cancel', [False, True])
def test_actual_endpoint_loops_do_not_wind_up_on_wall_clock(filename, method, paused, cancel):
    clock = FakeTime(0.0 if paused else 0.37)
    controller = load_controller(clock)
    ticks = []
    controller.init_steady_state = lambda: None
    controller.steady_state_step = lambda: ticks.append(clock.sim)
    controller.plan_to_configuration = lambda target: [[0.0]]
    controller.plan_to_ik_target = lambda: [[0.0]]
    controller.execute_path = lambda path, **kwargs: None
    controller.set_reduced_configuration_target = lambda target: None
    controller.steady_state_limited_joints = []
    controller.robot_model = SimpleNamespace(state_reduced={'q': np.array([0.2])})
    controller.left_ee_error = np.full(6, 0.2)
    controller.right_ee_error = np.full(6, 0.2)
    results = []
    goal = SimpleNamespace(is_cancel_requested=False,
                           canceled=lambda: results.append('cancel'),
                           abort=lambda: results.append('abort'),
                           succeed=lambda: results.append('success'))
    if cancel:
        sleep = clock.sleep

        def cancel_during_wait(seconds):
            sleep(seconds)
            if clock.wall >= 100.03:
                goal.is_cancel_requested = True

        clock.sleep = cancel_during_wait
    logger = SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    server = SimpleNamespace(
        controller=controller, get_logger=lambda: logger,
        threshold_joint=0.001, threshold_linear=0.001, threshold_angular=0.001,
        _visualize_planned_path=lambda path: None,
        _frame_task_errors=lambda: ([0.2], [0.2]),
        _dual_arm_errors=lambda: {key: 0.2 for key in (
            'left_linear', 'left_angular', 'right_linear', 'right_angular')},
        _publish_dual_arm_feedback=lambda *a: None,
        _publish_frame_task_feedback=lambda *a: None,
        _publish_named_config_feedback=lambda *a: None,
        _limit_block_reason=lambda: None, _log_hold_outcome=lambda *a: None,
    )
    source = ROOT / 'ros2' / filename
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    callback = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                    and node.name == method)
    namespace = {'time': clock, 'np': np, 'PathExecutionCancelled': InterruptedError,
                 'NamedConfig': SimpleNamespace(Result=SimpleNamespace),
                 'FrameTask': SimpleNamespace(Result=SimpleNamespace),
                 'DualArm': SimpleNamespace(Result=SimpleNamespace)}
    exec(compile(ast.Module(body=[callback], type_ignores=[]), str(source), 'exec'), namespace)
    args = (goal, np.zeros(1), 0.5) if 'named' in method else (goal, 0.5)
    result = namespace[method](server, *args)
    assert result.success is False
    assert results == ['cancel' if cancel else 'abort']
    np.testing.assert_allclose(ticks, [10.0] if paused or cancel else [10.0, 10.1])
    if cancel:
        assert clock.wall <= 100.04
    else:
        assert clock.wall == pytest.approx(100.5)
