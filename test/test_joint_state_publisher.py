import sys
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest


ROOT = Path(__file__).resolve().parents[1]


def _module(monkeypatch, name, **attributes):
    module = ModuleType(name)
    module.__dict__.update(attributes)
    monkeypatch.setitem(sys.modules, name, module)
    return module


def _load(path):
    spec = importlib.util.spec_from_file_location('_resource_test_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def publisher_module(monkeypatch):
    class Node:
        def __init__(self, name):
            self.name = name
            self.get_logger = Mock(return_value=Mock())
            self.create_publisher = Mock()
            self.create_timer = Mock()
            self.destroy_node = Mock()

    ros = _module(monkeypatch, 'rclpy', init=Mock(), spin=Mock(),
                  ok=Mock(return_value=True), shutdown=Mock())
    _module(monkeypatch, 'rclpy.node', Node=Node)
    _module(monkeypatch, 'sensor_msgs')
    _module(monkeypatch, 'sensor_msgs.msg', JointState=Mock())
    _module(monkeypatch, 'std_msgs')
    _module(monkeypatch, 'std_msgs.msg', Header=Mock())
    _module(monkeypatch, 'h12_ros2_controller.core.robot_model', RobotModel=Mock())
    _module(monkeypatch, 'h12_ros2_controller.utility.path_definition_ros',
            URDF_HANDLESS_PIN_PATH='/installed/handless.urdf')
    module = _load(ROOT / 'h12_ros2_controller' / 'ros2' / 'joint_state_publisher.py')
    module.init_channel_factory = Mock()
    return module, ros


def test_publisher_subscribes_to_selected_low_state_topic(publisher_module):
    module, _ = publisher_module
    config = {'topics': {'low_state': 'rt/custom_lowstate'}}

    node = module.JointStatePublisher(config)

    module.RobotModel.assert_called_once_with('/installed/handless.urdf', handless=True)
    node.robot_model.init_subscriber.assert_called_once_with(low_state_topic='rt/custom_lowstate')
    node.create_publisher.assert_called_once_with(module.JointState, 'joint_states', 10)


@pytest.mark.parametrize('entrypoint', (True, False))
def test_main_parses_profile_and_preserves_ros_arguments(publisher_module, monkeypatch, entrypoint):
    module, ros = publisher_module
    config = {'topics': {'low_state': 'rt/selected'}}
    module.load_controller_config = Mock(return_value=config)
    node = Mock()
    module.JointStatePublisher = Mock(return_value=node)
    ros_args = ['--ros-args', '-r', '__node:=selected_publisher', '-p', 'use_sim_time:=true']
    arguments = ['--config', '/custom/profile.yaml', *ros_args]
    if entrypoint:
        monkeypatch.setattr(sys, 'argv', ['joint_state_publisher', *arguments])
        module.main()
    else:
        module.main(arguments)

    module.load_controller_config.assert_called_once_with('/custom/profile.yaml')
    module.init_channel_factory.assert_called_once_with(config)
    module.JointStatePublisher.assert_called_once_with(config)
    ros.init.assert_called_once_with(args=ros_args)
    ros.spin.assert_called_once_with(node)
    node.destroy_node.assert_called_once()
    ros.shutdown.assert_called_once()


def test_main_default_profile_uses_central_resolution(publisher_module):
    module, _ = publisher_module
    module.load_controller_config = Mock(return_value={'topics': {'low_state': 'rt/default'}})
    module.JointStatePublisher = Mock()

    module.main([])

    module.load_controller_config.assert_called_once_with(module.DEFAULT_CONFIG_NAME)


@pytest.mark.parametrize('filename, default', (
    ('full_launch.py', 'safety_split.yaml'),
    ('robot_launch.py', 'safety_split.yaml'),
    ('robot_safety_launch.py', 'safety_full.yaml'),
    ('robot_tf_launch.py', 'safety_split.yaml'),
))
def test_launch_forwards_selected_profile_to_publisher(monkeypatch, tmp_path, filename, default):
    def action(kind):
        return lambda *args, **kwargs: SimpleNamespace(kind=kind, args=args, **kwargs)

    _module(monkeypatch, 'launch', LaunchDescription=lambda actions: actions)
    _module(monkeypatch, 'launch.actions', DeclareLaunchArgument=action('declare'),
            TimerAction=action('timer'), OpaqueFunction=action('opaque'))
    _module(monkeypatch, 'launch.substitutions',
            LaunchConfiguration=lambda name: SimpleNamespace(name=name))
    _module(monkeypatch, 'launch_ros')
    _module(monkeypatch, 'launch_ros.actions', Node=action('node'))
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot name="test"/>')
    _module(monkeypatch, 'h12_ros2_controller.utility.path_definition_ros',
            PACKAGE_PATH=str(tmp_path), URDF_MAGPIE_ROS_PATH=str(urdf))
    launch = _load(ROOT / 'launch' / filename)
    actions = launch.generate_launch_description()
    declaration = next(item for item in actions
                       if item.kind == 'declare' and item.args == ('config',))
    assert declaration.default_value == default
    nodes = [item for item in actions if item.kind == 'node']
    nodes.extend(node for timer in actions if timer.kind == 'timer' for node in timer.actions)
    publisher = next(node for node in nodes if node.executable == 'joint_state_publisher')
    flag, selected = publisher.arguments
    assert flag == '--config'
    assert selected.name == 'config'
    servers = [node for node in nodes if node.executable == 'frame_task_server']
    assert all(node.arguments == publisher.arguments for node in servers)
