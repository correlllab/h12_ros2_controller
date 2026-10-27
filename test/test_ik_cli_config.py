import ast
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1] / 'h12_ros2_controller'
ENTRYPOINTS = (
    'example/frame_controller_plan_goto.py',
    'example/frame_controller_counter_balance_goto.py',
    'example/kinematic_plan_goto.py',
    'benchmark/ik_benchmark.py',
)


@pytest.mark.parametrize('filename', ENTRYPOINTS)
def test_ik_cli_has_no_timeout_or_alpha_override(filename):
    tree = ast.parse((PACKAGE / filename).read_text())
    flags = {
        argument.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'add_argument'
        for argument in node.args
        if isinstance(argument, ast.Constant)
    }

    assert '--ik-timeout' not in flags
    assert '--ik-alpha' not in flags


@pytest.mark.parametrize('filename', ENTRYPOINTS)
def test_entrypoint_main_has_no_hardcoded_ik_override(filename):
    tree = ast.parse((PACKAGE / filename).read_text())
    main = next(node for node in tree.body
                if isinstance(node, ast.FunctionDef) and node.name == 'main')
    defaults = dict(zip(
        (argument.arg for argument in main.args.args[-len(main.args.defaults):]),
        main.args.defaults,
    ))

    assert 'ik_alpha' not in defaults
    if filename == 'benchmark/ik_benchmark.py':
        assert ast.literal_eval(defaults['ik_timeout']) is None
    else:
        assert 'ik_timeout' not in defaults


@pytest.mark.parametrize('filename', (
    'ros2/frame_task_server.py', 'ros2/dual_arm_server.py',
))
def test_ros_planning_uses_configured_ik_defaults(filename):
    tree = ast.parse((PACKAGE / filename).read_text())
    calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'plan_to_ik_target'
    ]

    assert calls
    assert all(not call.args and not call.keywords for call in calls)
