import ast
import sys
import runpy
import hashlib
import subprocess
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock

import pytest

from h12_ros2_controller.utility import path_definition as resources
from h12_ros2_controller.utility.controller_config import load_controller_config


ROOT = Path(__file__).resolve().parents[1]


def _model_family(directory, handless=False):
    name = 'h1_2_handless' if handless else 'h1_2_magpie'
    paths = tuple(directory / filename for filename in (
        f'{name}.urdf', f'{name}_sphere.urdf', f'{name}_sphere_collision.srdf',
    ))
    directory.mkdir(parents=True, exist_ok=True)
    for path in paths:
        path.write_text(path.name)
    return tuple(str(path) for path in paths)


@pytest.fixture
def installed_shares(tmp_path, monkeypatch):
    controller = tmp_path / 'share' / 'h12_ros2_controller'
    model = tmp_path / 'share' / 'h12_ros2_model'
    (controller / 'config').mkdir(parents=True)
    (controller / 'config' / 'safety_split.yaml').write_text(
        'mode: debug\ngains: {kp: 1, kd: 0.1, ki: 0}\n'
        'topics: {low_state: rt/installed_state}\n',
    )
    (controller / 'config' / 'ik_benchmark_cases.json').write_text('[]')
    calls = []

    def get_share(name):
        calls.append(name)
        return str({'h12_ros2_controller': controller, 'h12_ros2_model': model}[name])

    package = ModuleType('ament_index_python')
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = get_share
    package.packages = packages
    monkeypatch.setitem(sys.modules, 'ament_index_python', package)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    monkeypatch.setattr(resources, 'PROJECT_ROOT', tmp_path / 'site-packages')
    return controller, model, calls


def test_installed_config_defaults_lookup_only_controller_share(installed_shares):
    controller, _, calls = installed_shares

    assert resources.get_config_dir() == controller / 'config'
    assert load_controller_config()['topics']['low_state'] == 'rt/installed_state'
    assert load_controller_config('config/safety_split')['topics']['low_state'] == 'rt/installed_state'
    assert calls == ['h12_ros2_controller'] * 3


def test_explicit_configs_do_not_lookup_any_share(installed_shares):
    controller, _, calls = installed_shares
    directory = controller / 'config'

    load_controller_config(directory / 'safety_split.yaml')
    load_controller_config('safety_split', config_dir=directory)
    load_controller_config('config/safety_split', config_dir=directory)

    assert calls == []


@pytest.mark.parametrize('handless', (False, True))
def test_installed_model_family_is_complete_and_on_demand(installed_shares, handless):
    _, model, calls = installed_shares
    expected = _model_family(model / 'assets' / 'h1_2', handless)

    assert calls == []
    assert resources.resolve_model_paths(handless) == expected
    assert calls == ['h12_ros2_model']


def test_partial_checkout_family_falls_back_without_mixing(installed_shares, monkeypatch, tmp_path):
    _, model, calls = installed_shares
    source = tmp_path / 'checkout'
    source.mkdir()
    for marker in ('setup.py', 'package.xml'):
        (source / marker).touch()
    checkout = _model_family(source / 'submodules' / 'CL_Assets' / 'ros_assets')
    Path(checkout[1]).unlink()
    installed = _model_family(model / 'assets' / 'h1_2')
    monkeypatch.setattr(resources, 'PROJECT_ROOT', source)

    assert resources.resolve_model_paths() == installed
    assert calls == ['h12_ros2_model']
    Path(installed[2]).unlink()
    with pytest.raises(FileNotFoundError, match='Incomplete installed model family'):
        resources.resolve_model_paths()


def test_unverified_root_does_not_use_coincidentally_named_resources(installed_shares):
    controller, model, calls = installed_shares
    (resources.PROJECT_ROOT / 'config').mkdir(parents=True)
    _model_family(resources.PROJECT_ROOT / 'submodules' / 'CL_Assets' / 'ros_assets')
    expected = _model_family(model / 'assets' / 'h1_2')

    assert resources.get_config_dir() == controller / 'config'
    assert resources.resolve_model_paths() == expected
    assert calls == ['h12_ros2_controller', 'h12_ros2_model']


def test_checkout_and_explicit_resources_need_no_ros_imports():
    script = '''
import builtins
original = builtins.__import__
def no_ros(name, *args, **kwargs):
    if name.startswith(('ament_index_python', 'rclpy')) or name.endswith('path_definition_ros'):
        raise AssertionError('unexpected ROS import: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = no_ros
from h12_ros2_controller.utility import path_definition as resources
from h12_ros2_controller.utility.controller_config import load_controller_config
assert load_controller_config()['mode'] == 'debug'
assert resources.resolve_model_paths()
from pathlib import Path
resources.PROJECT_ROOT = Path('/nonexistent/site-packages')
assert load_controller_config(Path(CONFIG))['mode'] == 'debug'
assert load_controller_config('config/safety_split', config_dir=Path(CONFIG).parent)['mode'] == 'debug'
'''
    result = subprocess.run(
        [sys.executable, '-c', script.replace('CONFIG', repr(str(ROOT / 'config' / 'safety_split.yaml')))],
        cwd=ROOT, capture_output=True, text=True, timeout=30,
    )

    assert result.returncode == 0, result.stderr


def test_corpus_is_declared_as_installed_data(monkeypatch):
    import setuptools

    setup = Mock()
    monkeypatch.setattr(setuptools, 'setup', setup)
    monkeypatch.chdir(ROOT)
    runpy.run_path(str(ROOT / 'setup.py'))

    files = dict(setup.call_args.kwargs['data_files'])
    assert 'config/ik_benchmark_cases.json' in files['share/h12_ros2_controller/config']
    assert (ROOT / 'config' / 'ik_benchmark_cases.json').is_file()
    assert 'joint_state_publisher = h12_ros2_controller.ros2.joint_state_publisher:main' in (
        setup.call_args.kwargs['entry_points']['console_scripts']
    )


def test_pipeline_example_worker_and_hashes_share_selected_family(installed_shares, monkeypatch):
    from h12_ros2_controller.benchmark import ik_benchmark as benchmark
    from h12_ros2_controller.example import planner_example as example
    from h12_ros2_controller.core.planner import kinematic_pipeline as pipeline_module

    controller, model, calls = installed_shares
    family = _model_family(model / 'assets' / 'h1_2')
    robot = Mock()
    robot_factory = Mock(return_value=robot)
    monkeypatch.setattr(pipeline_module, 'load_controller_config', lambda _: {
        'planner': {'timeout': 1.0}, 'controller': {'d_min': 0.02},
    })
    monkeypatch.setattr(pipeline_module, 'RobotModel', robot_factory)
    monkeypatch.setattr(pipeline_module, 'IKSolver', Mock())
    monkeypatch.setattr(pipeline_module, 'ReducedJointPlanner', Mock())
    pipeline = pipeline_module.KinematicPipeline()
    assert pipeline.model_paths == family
    robot_factory.assert_called_once_with(family[0], handless=False)
    robot.init_collision_model.assert_called_once_with(*family[1:])

    monkeypatch.setattr(example, 'RobotModel', robot_factory)
    robot.reset_mock()
    assert example.build_robot_model() is robot
    robot.init_collision_model.assert_called_once_with(*family[1:])

    client = Mock()
    client_factory = Mock(return_value=client)
    monkeypatch.setattr(benchmark, 'PlannerClient', client_factory)
    benchmark._MeasuredPlanner(pipeline, 'worker', seed=42)
    assert client_factory.call_args.args[:3] == family
    client.warmup.assert_called_once()

    monkeypatch.setattr(benchmark, 'resolve_model_paths', Mock(side_effect=AssertionError('re-resolved')))
    hashes = benchmark._asset_ids('safety_split', pipeline.model_paths)
    config = controller / 'config' / 'safety_split.yaml'
    assert hashes == {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in (config, *(Path(name) for name in family))}
    assert benchmark.benchmark_corpus_path() == controller / 'config' / 'ik_benchmark_cases.json'
    assert set(calls) == {'h12_ros2_model', 'h12_ros2_controller'}


@pytest.mark.parametrize('flags, expected', (
    (['--corpus'], 'packaged.json'),
    (['--cases', 'literal.json'], 'literal.json'),
    (['--cases', 'config/ik_benchmark_cases.json', '--corpus'], 'config/ik_benchmark_cases.json'),
    (['--corpus', '--cases', 'literal.json'], 'literal.json'),
    (['--planners'], 'packaged.json'),
))
def test_benchmark_cli_keeps_explicit_cases_literal(monkeypatch, flags, expected):
    import argparse

    path = ROOT / 'h12_ros2_controller' / 'benchmark' / 'ik_benchmark.py'
    tree = ast.parse(path.read_text())
    cli = next(node for node in tree.body if isinstance(node, ast.If)
               and ast.unparse(node.test) == "__name__ == '__main__'")
    main, compare = Mock(), Mock()
    namespace = {
        '__doc__': '', '__name__': '__main__', 'argparse': argparse,
        'COMPARISON_PLANNERS': ('RRTConnect',), 'COMPARISON_CASES': ('case',),
        'benchmark_corpus_path': lambda: 'packaged.json',
        'main': main, 'compare_planners': compare,
    }
    monkeypatch.setattr(sys, 'argv', ['ik_benchmark', *flags])
    exec(compile(ast.Module(body=[cli], type_ignores=[]), str(path), 'exec'), namespace)

    if '--planners' in flags:
        assert compare.call_args.args[0] == expected
    else:
        assert main.call_args.kwargs['cases_path'] == expected
