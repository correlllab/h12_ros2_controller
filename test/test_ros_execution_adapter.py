'''Test planned ROS action callbacks without requiring a ROS installation'''

import ast
import time
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np


class PathExecutionCancelled(RuntimeError):
    pass


class Result:
    def __init__(self):
        self.success = None


def load_planned_callbacks(server_file):
    '''Compile the actual callback methods without importing hardware dependencies'''
    source = Path(__file__).resolve().parents[1] / 'h12_ros2_controller' / 'ros2' / server_file
    tree = ast.parse(source.read_text())
    server = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    methods = [node for node in server.body if isinstance(node, ast.FunctionDef)
               and node.name.startswith('_plan_') and node.name.endswith('_callback')]
    namespace = {
        'PathExecutionCancelled': PathExecutionCancelled,
        'FrameTask': SimpleNamespace(Result=Result),
        'DualArm': SimpleNamespace(Result=Result),
        'NamedConfig': SimpleNamespace(Result=Result),
        'time': time,
    }
    for method in methods:
        module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
        exec(compile(module, str(source), 'exec'), namespace)
    return namespace


def load_direct_callbacks(server_file):
    '''Compile direct action loops without importing ROS or controller drivers'''
    source = Path(__file__).resolve().parents[1] / 'h12_ros2_controller' / 'ros2' / server_file
    tree = ast.parse(source.read_text())
    server = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    methods = [node for node in server.body if isinstance(node, ast.FunctionDef)
               and node.name.startswith('_direct_') and node.name.endswith('_callback')]
    namespace = {
        'FrameTask': SimpleNamespace(Result=Result),
        'DualArm': SimpleNamespace(Result=Result),
        'NamedConfig': SimpleNamespace(Result=Result),
        'time': time,
    }
    for method in methods:
        module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
        exec(compile(module, str(source), 'exec'), namespace)
    return namespace


def load_publisher_callback(server_file):
    '''Compile pose publishing without importing live ROS dependencies'''
    source = Path(__file__).resolve().parents[1] / 'h12_ros2_controller' / 'ros2' / server_file
    tree = ast.parse(source.read_text())
    server = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    method = next(node for node in server.body if isinstance(node, ast.FunctionDef)
                  and node.name == 'publisher_callback')
    namespace = {'matrix_to_pose': lambda transform: transform}
    module = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
    exec(compile(module, str(source), 'exec'), namespace)
    return namespace['publisher_callback']


class PlannedExecutionAdapterTest(unittest.TestCase):
    def setUp(self):
        self.cases = (
            ('frame_task_server.py', '_plan_frame_task_callback', False),
            ('frame_task_server.py', '_plan_named_config_callback', True),
            ('dual_arm_server.py', '_plan_dual_arm_callback', False),
            ('dual_arm_server.py', '_plan_named_config_callback', True),
        )

    def run_callback(self, filename, method, named, execution=None,
                     cancel_after_plan=False, timeout=1.0):
        namespace = load_planned_callbacks(filename)
        goal = SimpleNamespace(is_cancel_requested=False, canceled=Mock(), abort=Mock(),
                               succeed=Mock())
        controller = Mock()
        controller.plan_to_ik_target.return_value = [1, 2]
        controller.plan_to_configuration.return_value = [1, 2]
        if cancel_after_plan:
            def request_cancel(*args, **kwargs):
                goal.is_cancel_requested = True
                return [1, 2]
            if named:
                controller.plan_to_configuration.side_effect = request_cancel
            else:
                controller.plan_to_ik_target.side_effect = request_cancel
        if execution is not None:
            controller.execute_path.side_effect = lambda path, cancel_check: execution(
                goal, cancel_check,
            )
        server = SimpleNamespace(
            controller=controller, timeout=1.0, get_logger=lambda: Mock(),
            _visualize_planned_path=Mock(),
            _log_hold_outcome=Mock(),
        )
        args = (goal, [0], timeout) if named else (goal, timeout)
        result = namespace[method](server, *args)
        return goal, controller, result

    def test_cancel_after_planning_skips_playback(self):
        for filename, method, named in self.cases:
            with self.subTest(filename=filename, method=method):
                goal, controller, result = self.run_callback(
                    filename, method, named, cancel_after_plan=True,
                )
                controller.execute_path.assert_not_called()
                goal.canceled.assert_called_once()
                goal.abort.assert_not_called()
                goal.succeed.assert_not_called()
                self.assertIs(result.success, False)

    def test_direct_cancel_requested_before_step_sends_no_commands(self):
        for filename, planned_method, named in self.cases:
            direct_method = planned_method.replace('_plan_', '_direct_', 1)
            with self.subTest(filename=filename, method=direct_method):
                namespace = load_direct_callbacks(filename)
                goal = SimpleNamespace(is_cancel_requested=True, canceled=Mock(),
                                       abort=Mock(), succeed=Mock())
                controller = Mock()
                server = SimpleNamespace(controller=controller, get_logger=lambda: Mock())
                args = (goal, [0], 1.0) if named else (goal, 1.0)

                result = namespace[direct_method](server, *args)

                goal.canceled.assert_called_once()
                goal.abort.assert_not_called()
                controller.control_step_reduced.assert_not_called()
                controller.goto_reduced_configuration.assert_not_called()
                self.assertIs(result.success, False)

    def test_dual_pose_publishing_continues_during_action_lock(self):
        publisher = load_publisher_callback('dual_arm_server.py')
        model = SimpleNamespace(
            state={'q': np.zeros(27)},
            get_frame_transformation_body=Mock(return_value=np.eye(4)),
        )
        controller = SimpleNamespace(
            robot_model=model, left_ee_name='left_wrist_yaw_link',
            right_ee_name='right_wrist_yaw_link',
        )
        lock = threading.Lock()
        server = SimpleNamespace(
            controller=controller, _controller_lock=lock,
            _target_snapshot=(np.eye(4), np.eye(4)),
            _stamp_pose=lambda pose: pose,
            left_ee_pose_publisher=Mock(), right_ee_pose_publisher=Mock(),
            left_ee_target_publisher=Mock(), right_ee_target_publisher=Mock(),
        )

        with lock:
            publisher(server)

        self.assertEqual(model.get_frame_transformation_body.call_count, 2)
        server.left_ee_pose_publisher.publish.assert_called_once()
        server.right_ee_pose_publisher.publish.assert_called_once()
        server.left_ee_target_publisher.publish.assert_called_once()
        server.right_ee_target_publisher.publish.assert_called_once()

    def test_cancel_during_playback_uses_live_goal_flag(self):
        def cancel(goal, cancel_check):
            self.assertFalse(cancel_check())
            goal.is_cancel_requested = True
            self.assertTrue(cancel_check())
            raise PathExecutionCancelled('canceled at waypoint')

        for filename, method, named in self.cases:
            with self.subTest(filename=filename, method=method):
                goal, controller, result = self.run_callback(
                    filename, method, named, execution=cancel,
                )
                controller.execute_path.assert_called_once()
                goal.canceled.assert_called_once()
                goal.abort.assert_not_called()
                goal.succeed.assert_not_called()
                self.assertIs(result.success, False)

    def test_execution_failure_still_aborts(self):
        def fail(goal, cancel_check):
            raise ValueError('invalid path')

        for filename, method, named in self.cases:
            with self.subTest(filename=filename, method=method):
                goal, controller, result = self.run_callback(
                    filename, method, named, execution=fail,
                )
                controller.execute_path.assert_called_once()
                goal.abort.assert_called_once()
                goal.canceled.assert_not_called()
                goal.succeed.assert_not_called()
                self.assertIs(result.success, False)

    def test_hold_timeout_aborts_instead_of_succeeding_with_false_result(self):
        for filename, method, named in self.cases:
            with self.subTest(filename=filename, method=method):
                goal, controller, result = self.run_callback(
                    filename, method, named, timeout=0.0,
                )
                controller.execute_path.assert_called_once()
                goal.abort.assert_called_once()
                goal.succeed.assert_not_called()
                self.assertIs(result.success, False)

    def test_action_servers_assign_reentrant_callback_group(self):
        for filename, _, _ in self.cases[::2]:
            with self.subTest(filename=filename):
                source = (Path(__file__).resolve().parents[1] /
                          'h12_ros2_controller' / 'ros2' / filename)
                tree = ast.parse(source.read_text())
                servers = [node for node in ast.walk(tree)
                           if isinstance(node, ast.Call) and
                           isinstance(node.func, ast.Name) and
                           node.func.id == 'ActionServer']
                self.assertEqual(len(servers), 2)
                for server in servers:
                    self.assertIn('callback_group',
                                  [keyword.arg for keyword in server.keywords])
                self.assertIn('ReentrantCallbackGroup', source.read_text())


if __name__ == '__main__':
    unittest.main()
