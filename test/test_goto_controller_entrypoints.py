from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.example import arm_controller_goto as arm
from h12_ros2_controller.example import frame_controller_counter_balance_goto as counter
from h12_ros2_controller.example import frame_controller_goto as direct
from h12_ros2_controller.example import frame_controller_plan_goto as planned


LEFT = 'left_wrist_yaw_link'
RIGHT = 'right_wrist_yaw_link'
SINGLE = f'frame {LEFT} 1 2 3 90 0 0'
PAIR = f'{SINGLE}, {RIGHT} 4 5 6 0 0 -90'
ARM_SINGLE = 'frame left 1 2 3 90 0 0'
ARM_PAIR = f'{ARM_SINGLE}, right 4 5 6 0 0 -90'


def answers(monkeypatch, responses):
    values = iter(responses)
    monkeypatch.setattr('builtins.input', lambda prompt: next(values))


@pytest.mark.parametrize('command,count', [(SINGLE, 1), (PAIR, 2)])
def test_planned_frame_tasks_installed_before_single_plan(monkeypatch, command, count):
    calls = []
    controller = SimpleNamespace(
        config={'controller': {'timeout': 0, 'threshold_joint': 0}},
        clear_frame_tasks=lambda: calls.append(('clear',)),
        add_frame_task=lambda name, frame, pose: calls.append((name, frame, pose.copy())),
        plan_to_ik_target=lambda **kwargs: calls.append(('plan', kwargs)) or np.zeros((2, 14)),
        visualize_path=lambda path: calls.append(('visualize',)),
        execute_path=lambda path: calls.append(('execute',)),
        init_steady_state=lambda: None,
        shutdown=lambda: calls.append(('shutdown',)),
    )
    monkeypatch.setattr(planned, 'init_frame_controller', lambda config: controller)
    answers(monkeypatch, (command, '', 'n'))

    planned.main()

    assert [call[0] for call in calls] == [
        'clear', *([f'{name}_task' for name in (LEFT, RIGHT)[:count]]),
        'plan', 'visualize', 'execute', 'shutdown',
    ]
    assert calls[1][2][3] == pytest.approx(np.pi / 2)
    if count == 2:
        assert calls[2][2][5] == pytest.approx(-np.pi / 2)


@pytest.mark.parametrize('command,count', [(SINGLE, 1), (PAIR, 2)])
def test_direct_frame_tasks_installed_before_first_step(monkeypatch, command, count):
    calls = []
    controller = SimpleNamespace(
        clear_frame_tasks=lambda: calls.append('clear'),
        add_frame_task=lambda name, frame, pose: calls.append((frame, pose.copy())),
        update_ik_solver=lambda: calls.append('sync'),
        control_step_reduced=lambda com: calls.append('step') or np.zeros(14),
        get_frame_task_error=lambda name: np.zeros(6),
        init_steady_state=lambda: None,
        steady_state_step=lambda threshold: True,
        shutdown=lambda: None,
        dt=0,
    )
    monkeypatch.setattr(direct, 'load_controller_config', lambda name: {
        'controller': {'timeout': 1, 'threshold_ik': 1, 'threshold_joint': 1,
                       'threshold_linear': 1, 'threshold_angular': 1},
    })
    monkeypatch.setattr(direct, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(direct, 'FrameController', lambda *args, **kwargs: controller)
    monkeypatch.setattr(direct, 'maybe_start_controller_logging', lambda obj: None)
    monkeypatch.setattr(direct.time, 'sleep', lambda seconds: None)
    answers(monkeypatch, (command, '', 'n'))

    direct.main()

    assert calls[:count + 3] == [
        'clear', *[(name, pytest.approx([1, 2, 3, np.pi / 2, 0, 0]) if name == LEFT
                   else pytest.approx([4, 5, 6, 0, 0, -np.pi / 2]))
                  for name in (LEFT, RIGHT)[:count]], 'sync', 'step',
    ]


@pytest.mark.parametrize('command,count', [
    (ARM_SINGLE, 1), (ARM_PAIR, 2),
])
def test_arm_targets_installed_before_first_step(monkeypatch, command, count):
    calls = []

    class FakeArm:
        left_ee_name = LEFT
        right_ee_name = RIGHT
        left_ee_pose = np.zeros(6)
        right_ee_pose = np.zeros(6)
        left_ee_error = np.zeros(6)
        right_ee_error = np.zeros(6)
        dt = 0

        def __setattr__(self, name, value):
            if name.endswith('_target_pose'):
                calls.append((name, np.copy(value)))
            object.__setattr__(self, name, value)

        def update_ik_solver(self):
            calls.append('sync')

        def control_step_reduced(self):
            calls.append('step')
            return np.zeros(14)

        def init_steady_state(self):
            pass

        def steady_state_step(self, threshold):
            return True

        def shutdown(self):
            pass

    monkeypatch.setattr(arm, 'load_controller_config', lambda name: {
        'controller': {'timeout': 1, 'threshold_ik': 1, 'threshold_joint': 1,
                       'threshold_linear': 1, 'threshold_angular': 1},
    })
    monkeypatch.setattr(arm, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(arm, 'ArmController', lambda *args, **kwargs: FakeArm())
    monkeypatch.setattr(arm, 'maybe_start_controller_logging', lambda obj: None)
    monkeypatch.setattr(arm.time, 'sleep', lambda seconds: None)
    answers(monkeypatch, (command, '', 'n'))

    arm.main()

    assert [item[0] for item in calls[:2]] == [
        'left_ee_target_pose', 'right_ee_target_pose',
    ]
    assert np.allclose(calls[0][1], [1, 2, 3, np.pi / 2, 0, 0])
    assert np.allclose(calls[1][1], [4, 5, 6, 0, 0, -np.pi / 2] if count == 2
                       else np.zeros(6))
    assert calls[2:4] == ['sync', 'step']


@pytest.mark.parametrize('command,count', [(SINGLE, 1), (PAIR, 2)])
@pytest.mark.parametrize('plan', [False, True])
def test_counter_plain_frame_tasks_precede_live_or_plan(monkeypatch, command, count, plan):
    calls = []
    controller = SimpleNamespace(
        planner=SimpleNamespace(),
        clear_frame_tasks=lambda: calls.append('clear'),
        add_frame_task=lambda name, frame, pose: calls.append((frame, pose.copy())),
        shutdown=lambda: None,
        dt=0.1,
    )
    monkeypatch.setattr(counter, 'load_controller_config', lambda name: {
        'mode': 'debug', 'topics': {'low_cmd': 'rt/safety/lowcmd_upper_in'},
        'reactive_counter_balance': True,
    })
    monkeypatch.setattr(counter, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(counter, 'DemoController', lambda *args, **kwargs: controller)
    monkeypatch.setattr(counter, 'maybe_start_controller_logging', lambda obj: None)
    monkeypatch.setattr(counter, 'select_planner', lambda *args: None)
    monkeypatch.setattr(counter, 'startup_home', lambda obj: None)
    monkeypatch.setattr(counter, 'hold', lambda obj: None)
    monkeypatch.setattr(counter, 'ask_z_floor', lambda phase: True)
    monkeypatch.setattr(counter, 'plan_goal', lambda *args: calls.append('plan') or np.zeros((2, 14)))
    monkeypatch.setattr(counter, 'check_path', lambda obj, path: (path, np.zeros((1, 14))))
    monkeypatch.setattr(counter, 'live_goto', lambda *args, **kwargs: calls.append('step') or True)
    monkeypatch.setattr(counter, 'execute_path', lambda *args: calls.append('execute'))
    monkeypatch.setattr(counter, 'wait_for_target', lambda *args, **kwargs: True)
    responses = ['y', 'y', command, 'y' if plan else 'n', 'n']
    if plan:
        responses.append('y')
    responses.append('n')
    answers(monkeypatch, responses)

    counter.main()

    assert calls[:count + 2] == [
        'clear', *[(name, pytest.approx([1, 2, 3, np.pi / 2, 0, 0]) if name == LEFT
                   else pytest.approx([4, 5, 6, 0, 0, -np.pi / 2]))
                  for name in (LEFT, RIGHT)[:count]], 'plan' if plan else 'step',
    ]
    if plan:
        assert calls[-1] == 'execute'


def test_counter_rejects_balanced_pair_before_any_task_or_motion():
    from h12_ros2_controller.utility.goto_commands import parse_goto_command

    calls = []
    controller = SimpleNamespace(clear_frame_tasks=lambda: calls.append('clear'))
    with pytest.raises(ValueError, match='exactly one'):
        counter.prepare_frame_goals(controller, parse_goto_command(PAIR).tasks, True)
    assert calls == []


def test_counter_live_pair_requires_both_cartesian_tasks(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(counter.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(counter.time, 'sleep',
                        lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    controller = SimpleNamespace(
        config={'controller': {
            'timeout': 0.03, 'threshold_ik': 1.0, 'threshold_joint': 1.0,
            'threshold_linear': 0.001, 'threshold_angular': 0.002,
        }},
        dt=0.01, update_ik_solver=lambda: None,
        control_step_reduced=lambda com: np.zeros(14),
        init_steady_state=lambda: None,
        steady_state_step=lambda threshold: True,
        get_frame_task_error=lambda name: (
            np.zeros(6) if name == 'left_task' else
            np.array([0.1, 0., 0., 0., 0., 0.])
        ),
        plain_goto=False, balanced_live=False,
    )

    assert counter.live_goto(
        controller, None, balanced=False,
        frame_task=('left_task', 'right_task'),
    ) is False


def test_arm_refuses_unsupported_frame_without_stepping(monkeypatch, capsys):
    calls = []
    controller = SimpleNamespace(
        left_ee_name=LEFT, right_ee_name=RIGHT,
        update_ik_solver=lambda: calls.append('sync'),
        control_step_reduced=lambda: calls.append('step'),
        shutdown=lambda: None,
    )
    monkeypatch.setattr(arm, 'load_controller_config', lambda name: {
        'controller': {'timeout': 1, 'threshold_ik': 1, 'threshold_joint': 1,
                       'threshold_linear': 1, 'threshold_angular': 1},
    })
    monkeypatch.setattr(arm, 'init_channel_factory_guard', lambda config: None)
    monkeypatch.setattr(arm, 'ArmController', lambda *args, **kwargs: controller)
    monkeypatch.setattr(arm, 'maybe_start_controller_logging', lambda obj: None)
    answers(monkeypatch, ('frame left_grasp_frame 1 2 3 0 0 0', 'quit'))

    arm.main()

    assert not calls
    assert 'supports only left and right' in capsys.readouterr().out
