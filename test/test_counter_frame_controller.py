import ast
import builtins
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_frame_controller import CounterFrameController
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.controller.upper_controller import (
    PathExecutionCancelled, UpperController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_goto_controller import (
    CounterGotoController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller import (
    CounterDDPVelocityRobustController,
)
from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.utility.path_definition import URDF_MAGPIE_PATH
from test_counter_ddp_velocity_robust_controller import _robust
from test_frame_pelvis_convention import load_server_methods


def server_factory():
    source = (Path(__file__).resolve().parents[1] / 'h12_ros2_controller' /
              'ros2' / 'frame_task_server.py')
    tree = ast.parse(source.read_text())
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'controller_class')
    namespace = {'FrameController': FrameController}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['controller_class'], tree


def test_factory_keeps_frame_default_and_imports_counter_only_when_selected(monkeypatch):
    factory, tree = server_factory()
    imports = []
    original = builtins.__import__

    def record_import(name, *args, **kwargs):
        imports.append(name)
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', record_import)
    assert factory('frame') is FrameController
    with pytest.raises(ValueError, match='frame or counter'):
        factory('other')
    assert imports == []
    assert factory('counter') is CounterFrameController
    assert imports == ['h12_ros2_controller.core.controller.counter_frame_controller']
    server = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    constructor = next(node for node in server.body if node.name == '__init__')
    text = ast.unparse(constructor)
    assert "declare_parameter('upper_controller', 'frame')" in text
    assert text.index('controller_class(selected)') < text.index('controller_type(')
    assert text.count('controller_type(') == 1


def test_startup_is_plain_before_base_initialization(monkeypatch):
    initialized = []

    def initialize(self, *args, **kwargs):
        assert self.plain_goto and not self.balanced_live
        assert not self.planned_balance
        initialized.append(kwargs)

    monkeypatch.setattr(CounterDDPVelocityRobustController, '__init__', initialize)
    CounterFrameController('robot', 'sphere', 'srdf', init=True)
    assert initialized == [{'init': True}]


def test_invalid_selector_rejects_before_controller_or_ros_publishers():
    factory, tree = server_factory()
    constructor = next(node for node in next(
        node for node in tree.body if isinstance(node, ast.ClassDef)
    ).body if node.name == '__init__')

    class Node:
        def __init__(self, name):
            pass

        def declare_parameter(self, name, default):
            return SimpleNamespace(value='invalid')

    server = ast.ClassDef(name='Server', bases=[ast.Name(id='Node', ctx=ast.Load())],
                          keywords=[], body=[constructor], decorator_list=[])
    namespace = {'Node': Node, 'controller_class': factory}
    module = ast.fix_missing_locations(ast.Module(body=[server], type_ignores=[]))
    exec(compile(module, '<server constructor>', 'exec'), namespace)
    with pytest.raises(ValueError, match='upper_controller'):
        namespace['Server']()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('suffix', [
    'graspgenx_frame', 'grasp_frame', 'wrist_yaw_link',
])
def test_model_chain_owns_grasp_and_wrist_frames(side, suffix):
    model_path = Path(URDF_MAGPIE_PATH)
    if not model_path.exists():
        model_path = Path(__file__).resolve().parents[4] / 'CL_Assets/ros_assets/h1_2_magpie.urdf'
    if not model_path.exists():
        pytest.skip('robot assets unavailable')
    robot = RobotModel(str(model_path))
    robot.init_reduced_model(ENABLED_JOINTS)
    controller = object.__new__(CounterFrameController)
    controller.robot_model = robot
    controller.motor_v_indices = np.arange(27)
    controller.arm_ids = list(range(13, 27))
    controller.moving_arm = None
    controller.prepare_frame_goal([f'{side}_{suffix}'])
    assert controller.moving_arm == side
    assert controller.counter_arm == ('right' if side == 'left' else 'left')
    assert not set(controller.moving_ids) & set(controller.counter_arm_ids)


def action_controller():
    controller = _robust()
    controller.__class__ = CounterFrameController
    controller.plain_goto = True
    controller.balanced_live = True
    controller.planned_balance = False
    controller.upper_ids = list(range(12, 27))
    controller.torso_target = 0.0
    controller.d_min = 0.02
    controller.robot_model.reduced_mask = np.r_[np.zeros(13, dtype=bool), np.ones(14, dtype=bool)]
    controller.robot_model.check_within_limits_reduced = lambda q: True
    controller.robot_model.check_collision_free = lambda q, **kwargs: True
    controller.clear_frame_tasks = Mock()
    controller._arm_for_frame = lambda name: name.split('_')[0]
    controller.check_state_fresh = Mock()
    writer = controller.low_cmd_handler.set_joint_commands
    controller.low_cmd_handler.set_joint_commands = (
        lambda q, dq, tau, joint_ids: writer(q=q, dq=dq, tau=tau, joint_ids=joint_ids)
    )
    return controller


def test_segmented_goals_keep_counter_references_and_named_goals_reset_without_motion():
    controller = action_controller()
    controller.q_counter_ref = reference = np.full(4, 0.2)
    controller._reference_captured = True
    controller.prepare_frame_goal(['left_graspgenx_frame'])
    assert controller.q_counter_ref is reference
    assert controller._reference_captured
    commanded = controller.low_cmd_handler.q_cmd.copy()
    controller.prepare_named_goal()
    assert controller.plain_goto and not controller.balanced_live
    assert controller.q_counter_ref is None
    assert not controller._reference_captured
    controller.clear_frame_tasks.assert_called_once()
    assert controller.low_cmd_handler.calls == []
    np.testing.assert_array_equal(controller.low_cmd_handler.q_cmd, commanded)
    controller.prepare_frame_goal(['right_graspgenx_frame'])
    assert controller.moving_arm == 'right'


def test_multi_frame_goal_rejected_without_changing_ownership():
    controller = action_controller()
    with pytest.raises(ValueError, match='exactly one'):
        controller.prepare_frame_goal(['left_graspgenx_frame', 'right_graspgenx_frame'])
    assert controller.moving_arm == 'left'
    assert controller.low_cmd_handler.calls == []


def test_first_same_arm_goal_after_named_prep_captures_post_motion_reference():
    controller = action_controller()
    controller.q_counter_ref = np.full(4, -0.2)
    controller._reference_captured = True
    controller.prepare_named_goal()
    assert not controller._reference_captured
    # named motion changes the measured and commanded counter-arm posture
    controller.robot_model.state['q'][controller.counter_arm_ids] = 0.35
    controller.low_cmd_handler.q_cmd[controller.counter_arm_ids] = 0.35
    controller.prepare_frame_goal(['left_graspgenx_frame'])
    assert not controller._reference_captured
    target = controller.robot_model.state['q'][controller.arm_ids].copy()
    controller.control_configuration_step(target, np.zeros(14))
    assert controller._reference_captured
    np.testing.assert_allclose(controller.q_counter_ref, 0.35)
    np.testing.assert_allclose(controller.counter_wrist_ref, 0.35)
    np.testing.assert_allclose(controller.low_cmd_handler.q_cmd[controller.counter_arm_ids], 0.35)


@pytest.mark.parametrize('planned', [False, True])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_balanced_command_runs_actual_nominal_core_with_one_mock_writer(planned, side):
    controller = action_controller()
    controller._set_arm_ownership(side)
    q = controller.robot_model.state['q'].copy()
    velocity = np.zeros(27)
    velocity[controller.moving_ids[:4]] = [0.1, -0.1, 0.1, -0.1]
    target = q + velocity * controller.dt
    if planned:
        controller.planned_balance = True
        controller._planned_previous = q[13:].copy()
        controller._publish_position_command(target, np.zeros(27), np.zeros(27))
    else:
        controller.ik_solver = SimpleNamespace(
            q=target, integrate=lambda unused: None,
        )
        controller._apply_velocity_command(velocity)
    assert len(controller.low_cmd_handler.calls) == 1
    command = controller.low_cmd_handler.q_cmd
    np.testing.assert_allclose(command[controller.moving_ids], target[controller.moving_ids])
    assert np.linalg.norm(command[controller.counter_ids] - q[controller.counter_ids]) > 0
    assert controller._latest_nominal_plan is not None
    assert np.all(controller.low_cmd_handler.dq_cmd[controller.moving_ids] == 0)


def test_hold_retains_counter_pose_and_finish_zeros_velocity_without_return():
    controller = action_controller()
    controller.low_cmd_handler.q_cmd[controller.counter_arm_ids] = 0.2
    controller.low_cmd_handler.dq_cmd[controller.counter_arm_ids] = 0.1
    controller._publish_position_command(np.full(27, 0.3), np.zeros(27), np.ones(27))
    np.testing.assert_allclose(controller.low_cmd_handler.q_cmd[controller.counter_arm_ids], 0.2)
    np.testing.assert_allclose(controller.low_cmd_handler.q_cmd[controller.moving_ids], 0.3)
    previous = controller.low_cmd_handler.q_cmd.copy()
    controller.finish_goal()
    np.testing.assert_array_equal(controller.low_cmd_handler.q_cmd, previous)
    assert not np.any(controller.low_cmd_handler.dq_cmd)
    count = len(controller.low_cmd_handler.calls)
    controller.low_cmd_handler._estopped = True
    controller.finish_goal()
    assert len(controller.low_cmd_handler.calls) == count


@pytest.mark.parametrize('invalid', ['limits', 'collision'])
def test_combined_planned_command_is_validated_before_write(invalid):
    controller = action_controller()
    controller.planned_balance = True
    controller.robot_model.check_within_limits_reduced = lambda q: invalid != 'limits'
    controller.robot_model.check_collision_free = lambda q, **kwargs: invalid != 'collision'
    with pytest.raises(ValueError, match='Combined planned/counter-arm'):
        controller._write_arm_command(np.zeros(14), np.zeros(14), np.zeros(14))
    assert controller.low_cmd_handler.calls == []


def test_planned_route_reuses_cancel_guard_and_restores_hold(monkeypatch):
    controller = action_controller()
    seen = []
    cancel_check = lambda: True

    def execute(self, path, validate=True, cancel_check=None):
        seen.append((self.planned_balance, validate, cancel_check()))
        raise PathExecutionCancelled('canceled')

    monkeypatch.setattr(UpperController, 'execute_path', execute)
    with pytest.raises(PathExecutionCancelled):
        controller.execute_path(np.zeros((2, 14)), cancel_check=cancel_check)
    assert seen == [(True, True, True)]
    assert not controller.planned_balance
    assert len(controller.low_cmd_handler.calls) == 1
    assert not np.any(controller.low_cmd_handler.dq_cmd)
    controller.prepare_named_goal()
    with pytest.raises(PathExecutionCancelled):
        controller.execute_path(np.zeros((2, 14)), cancel_check=cancel_check)
    assert seen[-1] == (False, True, True)


def test_example_and_action_adapter_share_routing_without_importing_ui():
    from h12_ros2_controller.example.frame_controller_counter_balance_goto import DemoController
    assert DemoController is CounterGotoController
    assert CounterFrameController.__bases__ == (CounterGotoController,)


@pytest.mark.parametrize('planned', [False, True])
def test_action_hooks_keep_lock_slow_mode_timeout_and_final_hold(planned):
    methods = load_server_methods()
    methods['_frame_task_callback'].__globals__['SLOW_MODE_SCALE'] = 0.25
    lock = threading.Lock()
    calls = []

    class Target:
        position = SimpleNamespace(x=0.4, y=0.35, z=0.25)
        orientation = SimpleNamespace(x=0.9659258263, y=0.2588190451, z=0.0, w=0.0)

        def copy(self):
            return self

    def prepare(names):
        assert lock.locked()
        calls.append(('prepare', names))
        return 'balanced moving=left, counter=right'

    controller = SimpleNamespace(
        robot_model=SimpleNamespace(
            state={'q': np.zeros(27)},
            model_body=SimpleNamespace(frames=[SimpleNamespace(name='left_graspgenx_frame')]),
        ),
        prepare_frame_goal=prepare, finish_goal=Mock(),
        check_state_fresh=Mock(), clear_frame_tasks=Mock(), add_frame_task=Mock(),
        update_ik_solver=Mock(), speed_scale=1.0,
    )

    def execute(goal, timeout):
        assert lock.locked()
        assert controller.speed_scale == 0.25
        assert timeout == 45.0
        calls.append(('execute', planned))
        raise PathExecutionCancelled('test cancellation')

    server = SimpleNamespace(
        controller=controller, _controller_lock=lock, timeout=5.0,
        get_logger=lambda: SimpleNamespace(info=Mock()),
        _resolve_target=lambda name, target, relative, measured_q: target,
        _plan_frame_task_callback=execute, _direct_frame_task_callback=execute,
    )
    goal = SimpleNamespace(request=SimpleNamespace(
        frame_names=['left_graspgenx_frame'], frame_targets=[Target()],
        relative=False, plan=planned, slow_mode=True,
        duration=SimpleNamespace(sec=45, nanosec=0),
    ))
    with pytest.raises(PathExecutionCancelled):
        methods['_frame_task_callback'](server, goal)
    assert calls == [('prepare', ['left_graspgenx_frame']), ('execute', planned)]
    assert controller.speed_scale == 1.0
    controller.finish_goal.assert_called_once()
    assert not lock.locked()
