import os
import pty
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.utility.goto_commands import (
    FrameGoal, GotoConsole, LoadCase, NamedGoal, Quit, Replay, SaveCase,
    ShowCandidate, parse_goto_command,
)


def test_frame_command_converts_degrees_and_rejects_nonfinite_pose():
    command = parse_goto_command('frame left_grasp_frame 0.4 0.2 0.2 0 80 0')

    assert isinstance(command, FrameGoal)
    assert command.frame_name == 'left_grasp_frame'
    assert np.allclose(command.pose, [0.4, 0.2, 0.2, 0., np.deg2rad(80), 0.])
    with pytest.raises(ValueError, match='finite'):
        parse_goto_command('frame left_grasp_frame nan 0 0 0 0 0')


def test_named_and_optional_commands_are_typed_and_capability_checked():
    assert parse_goto_command('home', named_goals={'home'}) == NamedGoal('home')
    assert isinstance(parse_goto_command('quit'), Quit)
    assert isinstance(parse_goto_command('replay'), Replay)
    assert parse_goto_command('show 2') == ShowCandidate(2)
    assert parse_goto_command('save "my case.json"') == SaveCase('my case.json')
    assert parse_goto_command('load "my case.json"') == LoadCase('my case.json')
    assert parse_goto_command('   ') is None
    with pytest.raises(ValueError, match='Unknown named configuration'):
        parse_goto_command('missing', named_goals={'home'})
    with pytest.raises(ValueError, match='not supported'):
        parse_goto_command('replay', supported=(NamedGoal, FrameGoal, Quit))
    with pytest.raises(ValueError, match='nonnegative'):
        parse_goto_command('show -1')


def test_console_formats_attempts_candidates_and_planner_metadata():
    lines = []
    console = GotoConsole(
        title='Offline IK', named_goals=('home',), output_fn=lines.append,
    )
    failed = SimpleNamespace(
        seed_name='current', success=False, reason='timeout',
        linear_error=0.015, angular_error=0.01, elapsed_time=0.3,
    )
    selected = SimpleNamespace(seed_name='t_pose', success=True, score=1.25)
    ik = SimpleNamespace(
        success=True, seed_name='t_pose', linear_error=0.0,
        angular_error=0.0, attempts=[failed, selected], candidates=[selected],
    )
    plan = SimpleNamespace(
        success=True, reason='direct path', path=np.zeros((30, 14)),
        metadata={
            'path_kind': 'direct path', 'waypoint_count': 30,
            'duration': 0.967, 'effective_speed': 0.2,
            'path_length': 0.193, 'length_estimate_samples': 50,
            'frame_path_lengths': {'left_grasp_frame': 0.193},
            'frame_translation_lengths': {'left_grasp_frame': 0.15},
            'frame_rotation_lengths': {'left_grasp_frame': 0.043},
            'diagnostics': ['clamped start into joint limits'],
        },
    )
    result = SimpleNamespace(
        ik=ik, plan=plan, elapsed_time=0.6, selected_candidate=selected,
    )

    console.report(result, metrics_fn=lambda candidate: {
        'score': candidate.score, 'wrist_travel': 0.1, 'wrist_margin': 0.3,
    })

    text = '\n'.join(lines)
    assert '[Offline IK] IK: success=True, seed=t_pose' in text
    assert 'current: timeout' in text
    assert '[0] t_pose: score=1.250, wrist travel=0.100 rad' in text
    assert '[Offline IK] direct path: waypoints=30' in text
    assert '[Offline IK] clamped start into joint limits' in text
    assert '[planner]' not in text


def test_console_help_only_advertises_supported_commands():
    lines = []
    console = GotoConsole(
        title='ROS Goto', named_goals=('home',),
        supported=(NamedGoal, FrameGoal, Quit), output_fn=lines.append,
        input_fn=lambda prompt: 'home',
    )

    console.help()

    assert console.read_command() == NamedGoal('home')
    assert not any('replay' in line for line in lines)
    assert any('[ROS Goto] Commands:' in line for line in lines)


def test_arrow_key_edits_input_in_tty():
    master, slave = pty.openpty()
    script = (
        'from h12_ros2_controller.utility.goto_commands import GotoConsole; '
        'print(GotoConsole(named_goals=("home",)).read_command())'
    )
    process = subprocess.Popen(
        [sys.executable, '-c', script],
        stdin=slave, stdout=slave, stderr=slave,
        env={**os.environ, 'TERM': 'xterm'},
    )
    os.close(slave)
    try:
        # Move left twice before m, insert o: hme becomes home
        os.write(master, b'hme\x1b[D\x1b[Do\n')
        status = process.wait(timeout=10)
        output = bytearray()
        while True:
            try:
                chunk = os.read(master, 4096)
            except OSError:
                break
            if not chunk:
                break
            output.extend(chunk)
        assert status == 0, output.decode(errors='replace')
        assert b"NamedGoal(name='home')" in output, output.decode(errors='replace')
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        os.close(master)
