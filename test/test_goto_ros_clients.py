import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from h12_ros2_controller.utility.goto_commands import (
    FrameGoal, GotoConsole, MultiFrameGoal, NamedGoal, Quit,
)


CLIENT_DIR = Path(__file__).resolve().parents[1] / 'h12_ros2_controller' / 'ros2'


def run_client(filename, commands, monkeypatch, plans=()):
    '''Run client entry point with mocked ROS actions and real console parsing'''
    source = CLIENT_DIR / filename
    tree = ast.parse(source.read_text())
    selected = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))]
    code = compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec')
    sent = []
    output = []
    spins = []
    inputs = iter(commands)
    plan_inputs = iter(plans)

    class Node:
        def __init__(self, name):
            self.name = name

        def get_logger(self):
            return SimpleNamespace(info=output.append, warn=output.append)

        def create_subscription(self, *args):
            return None

        def destroy_node(self):
            pass

    class Goal:
        pass

    class Pose:
        def __init__(self):
            self.position = SimpleNamespace(x=0., y=0., z=0.)
            self.orientation = SimpleNamespace(x=0., y=0., z=0., w=1.)

    class ActionClient:
        def __init__(self, node, action, name):
            self.name = name

        def wait_for_server(self):
            pass

        def send_goal_async(self, goal, feedback_callback):
            sent.append((self.name, goal, feedback_callback))
            handle = SimpleNamespace(
                accepted=True,
                get_result_async=lambda: SimpleNamespace(result=lambda: SimpleNamespace(
                    result=SimpleNamespace(success=True), status=4)),
            )
            return SimpleNamespace(result=lambda: handle)

    class Listener:
        def __init__(self, on_press):
            self.on_press = on_press

        def start(self):
            pass

        def stop(self):
            pass

    utility = CLIENT_DIR / 'utility.py'
    pose_functions = [node for node in ast.parse(utility.read_text()).body
                      if isinstance(node, ast.FunctionDef) and node.name in (
                          'list_to_pose', 'action_result_summary',
                      )]
    goal_status = SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6,
    )
    pose_namespace = {'Pose': Pose, 'R': R, 'GoalStatus': goal_status}
    exec(compile(ast.Module(body=pose_functions, type_ignores=[]), str(utility), 'exec'),
         pose_namespace)

    def console_factory(**kwargs):
        return GotoConsole(**kwargs, input_fn=lambda prompt: next(inputs),
                           output_fn=output.append)

    rclpy = SimpleNamespace(
        init=lambda args: None, ok=lambda: True, shutdown=lambda: None,
        spin_until_future_complete=lambda node, future: spins.append(future),
    )
    namespace = {
        'Node': Node, 'ActionClient': ActionClient, 'Pose': Pose,
        'PoseStamped': Goal, 'FrameTask': SimpleNamespace(Goal=Goal),
        'DualArm': SimpleNamespace(Goal=Goal), 'NamedConfig': SimpleNamespace(Goal=Goal),
        'keyboard': SimpleNamespace(Listener=Listener, Key=SimpleNamespace(backspace=object())),
        'rclpy': rclpy, 'NAMED_CONFIGS': {'home': None},
        'GotoConsole': console_factory, 'NamedGoal': NamedGoal, 'FrameGoal': FrameGoal,
        'MultiFrameGoal': MultiFrameGoal, 'Quit': Quit,
        'list_to_pose': pose_namespace['list_to_pose'],
        'action_result_summary': pose_namespace['action_result_summary'],
    }
    exec(code, namespace)
    monkeypatch.setattr('builtins.input', lambda prompt: next(plan_inputs))
    namespace['main']()
    return sent, output, spins


def test_frame_task_single_and_multiple_frames_reach_action(monkeypatch):
    sent, output, spins = run_client('frame_task_client.py', [
        'frame left_grasp_frame 0.4 0.2 0.3 0 90 0',
        'frame left_grasp_frame 0.4 0.2 0.3 0 90 0, '
        'right_grasp_frame 0.5 -0.2 0.3 0 0 180, head_frame 0 0 1 0 0 0',
        'quit',
    ], monkeypatch, plans=['n', 'y'])

    assert [name for name, _, _ in sent] == ['frame_task', 'frame_task']
    first, multiple = (item[1] for item in sent)
    assert first.frame_names == ['left_grasp_frame']
    assert first.frame_targets[0].position.x == pytest.approx(0.4)
    assert first.frame_targets[0].orientation.y == pytest.approx(np.sqrt(0.5))
    assert first.plan is False
    assert multiple.frame_names == ['left_grasp_frame', 'right_grasp_frame', 'head_frame']
    assert multiple.frame_targets[1].orientation.z == pytest.approx(1.)
    assert multiple.plan is True
    assert len(spins) == 4
    assert not any('not supported' in line for line in output)
    assert any('status=succeeded, success=True' in line for line in output)


def test_dual_arm_routes_reversed_pair_and_named_goal(monkeypatch):
    sent, _, spins = run_client('dual_arm_client.py', [
        'frame right 0.5 -0.2 0.3 0 0 180, '
        'left 0.4 0.2 0.3 0 90 0',
        'home', 'quit',
    ], monkeypatch, plans=['y', 'n'])

    assert [name for name, _, _ in sent] == ['dual_arm', 'named_config']
    assert sent[0][1].left_target.position.x == pytest.approx(0.4)
    assert sent[0][1].left_target.orientation.y == pytest.approx(np.sqrt(0.5))
    assert sent[0][1].right_target.orientation.z == pytest.approx(1.)
    assert sent[0][1].plan is True
    assert sent[1][1].config_name == 'home'
    assert sent[1][1].plan is False
    assert len(spins) == 4


@pytest.mark.parametrize('invalid', [
    'frame left 0 0 0 0 0 0',
    'frame left 0 0 0 0 0 0, head_frame 0 0 0 0 0 0',
    'frame left 0 0 0 0 0 0, right 0 0 0 0 0 0, '
    'head_frame 0 0 0 0 0 0',
    'frame left 0 0 0 0 0 0, left 0 0 0 0 0 0',
    'frame left_grasp_frame 0 0 0 0 0 0, right_grasp_frame 0 0 0 0 0 0',
])
def test_dual_arm_rejects_missing_extra_and_duplicate_frames(monkeypatch, invalid):
    sent, output, spins = run_client('dual_arm_client.py', [invalid, 'quit'], monkeypatch)

    assert sent == []
    assert spins == []
    assert any('requires exactly one' in line or 'distinct frame names' in line
               for line in output)


def test_action_summary_distinguishes_canceled_and_aborted():
    source = CLIENT_DIR / 'utility.py'
    function = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and
                    node.name == 'action_result_summary')
    namespace = {'GoalStatus': SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6,
    )}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'),
         namespace)

    for status, label in ((5, 'canceled'), (6, 'aborted')):
        response = SimpleNamespace(status=status,
                                   result=SimpleNamespace(success=False))
        assert namespace['action_result_summary'](response) == (
            f'status={label}, success=False'
        )
