'''Exercise startup methods without importing or constructing ROS/DDS endpoints'''

import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def controller_fixture():
    source = (Path(__file__).resolve().parents[1] / 'h12_ros2_controller' /
              'core/controller/upper_controller.py')
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == 'UpperController')
    names = {'_startup_routine', '_move_to_reduced_configuration',
             '_move_torso_to_target', '_publish_position_command'}
    cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name in names]
    joint_source = source.parents[2] / 'utility/joint_definition.py'
    joints = {}
    exec(compile(ast.parse(joint_source.read_text()), str(joint_source), 'exec'), joints)
    targets = {name: np.array([value]) for name, value in
               zip(('init_1', 'init_2', 'init_3', 'home'), (1.3, 1.2, 0.1, 0.0))}
    sleeps = []
    namespace = {'np': np, 'time': SimpleNamespace(sleep=sleeps.append),
                 'tqdm': lambda values, **kwargs: values, 'NAMED_CONFIGS': targets,
                 'BODY_JOINTS': joints['BODY_JOINTS'],
                 'UPPER_BODY_INDEX': joints['UPPER_BODY_INDEX']}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    controller = namespace['UpperController']()
    controller.dt = 0.02
    controller.timeout = 5.0
    controller.enabled_ids = [13]
    controller.robot_model = SimpleNamespace(
        state={'q': np.zeros(27)}, state_reduced={'q': np.zeros(1)},
    )
    commands = []

    def set_commands(q, dq=None, tau=None, *, joint_ids):
        controller.low_cmd_handler.q_cmd[joint_ids] = q
        commands.append((list(joint_ids), np.asarray(q).copy()))

    controller.low_cmd_handler = SimpleNamespace(
        q_cmd=np.zeros(27), _q_clip_limits=np.tile([-3.0, 3.0], (27, 1)),
        set_joint_commands=set_commands,
    )
    controller.update_robot_model = lambda: None
    return controller, targets, sleeps, commands


@pytest.mark.parametrize('timeout, dt, steps', [
    (5.0, 0.02, 250), (5.0, 1.0 / 30.0, 150),
    (5.01, 0.02, 251), (0.2, 0.02, 50),
])
def test_startup_uses_configured_budget_preserves_threshold_and_pose_order(timeout, dt, steps):
    controller, _, _, _ = controller_fixture()
    controller.timeout = timeout
    controller.dt = dt
    calls = []
    controller._move_to_reduced_configuration = lambda q, **kwargs: calls.append(kwargs)
    controller._move_torso_to_target = lambda *args: calls.append(args)

    controller._startup_routine(0.2, 0.0)

    assert [call['desc'] for call in calls[:-1]] == [
        'Moving init_1', 'Moving init_2', 'Moving init_3', 'Moving home',
    ]
    assert all(call['steps'] == steps and 'threshold' not in call for call in calls[:-1])
    assert calls[-1] == (0.2, 0.0)


@pytest.mark.parametrize('command_error', [0.0, 0.02])
def test_residual_above_threshold_warns_without_raising(capsys, command_error):
    controller, _, sleeps, _ = controller_fixture()
    controller.robot_model.state_reduced['q'][:] = 0.011
    controller.low_cmd_handler.q_cmd[13] = command_error
    controller.goto_reduced_configuration = lambda q: None

    controller._move_to_reduced_configuration(
        np.zeros(1), steps=2, threshold=0.01, desc='Moving home',
    )

    output = capsys.readouterr().out
    assert 'Warning: Moving home did not converge' in output
    assert f'measured error=0.0110 rad, command error={command_error:.4f} rad' in output
    assert 'threshold=0.0100 rad); continuing startup' in output
    assert sleeps == [controller.dt] * 2


def test_measured_convergence_preserves_legacy_early_exit(capsys):
    controller, _, sleeps, _ = controller_fixture()
    controller.low_cmd_handler.q_cmd[13] = 1.0
    controller.goto_reduced_configuration = lambda q: None

    controller._move_to_reduced_configuration(np.zeros(1), steps=50)

    assert not sleeps
    assert not capsys.readouterr().out


def test_all_nonconverging_arm_stages_reach_torso_zero_and_return(capsys):
    controller, targets, sleeps, commands = controller_fixture()
    stages = []

    def goto(target):
        stages.append(float(target[0]))
        controller.robot_model.state_reduced['q'] = target + 0.011
        controller.low_cmd_handler.q_cmd[13] = target[0]

    controller.goto_reduced_configuration = goto
    controller.robot_model.state['q'][12] = 0.2
    controller._startup_routine(0.2, 0.0)
    assert stages == [float(target[0]) for target in targets.values() for _ in range(250)]
    assert capsys.readouterr().out.count('continuing startup') == 4
    assert len(sleeps) >= 1000
    assert all(ids == [12] for ids, _ in commands)
    np.testing.assert_array_equal(commands[-1][1], [0.0])
    assert controller.low_cmd_handler.q_cmd[12] == 0.0
    # arm command generation must preserve the torso target, not measured yaw
    controller._publish_position_command(np.ones(27), np.ones(27), np.zeros(27))
    assert controller.low_cmd_handler.q_cmd[12] == 0.0


def test_server_endpoints_follow_controller_startup_without_live_construction():
    source = (Path(__file__).resolve().parents[1] / 'h12_ros2_controller' /
              'ros2/frame_task_server.py')
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == 'FrameTaskServer')
    init = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name == '__init__')
    assignments = {
        node.targets[0].attr: node for node in init.body
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Attribute)
    }
    controller = assignments['controller']
    assert any(keyword.arg == 'init' and keyword.value.value is True
               for keyword in controller.value.keywords)
    assert controller.lineno < assignments['action_server'].lineno
    assert controller.lineno < assignments['named_config_server'].lineno


def test_sdk_failure_still_propagates():
    controller, _, _, commands = controller_fixture()

    def goto(target):
        raise RuntimeError('SDK disconnected')

    controller.goto_reduced_configuration = goto
    with pytest.raises(RuntimeError, match='SDK disconnected'):
        controller._startup_routine(0.2, 0.0)
    assert not commands


def test_invalid_torso_target_still_rejected():
    controller, _, _, commands = controller_fixture()
    with pytest.raises(ValueError, match='Torso target must be finite'):
        controller._move_torso_to_target(0.2, np.nan)
    assert not commands
