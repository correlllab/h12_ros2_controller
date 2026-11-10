'''Exercise ROS comparison adapters without importing ROS, DDS or live controllers'''

import ast
import json
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np


SOURCE = (Path(__file__).resolve().parents[1] / 'h12_ros2_controller' /
          'ros2/frame_task_server.py')


class Pose:
    def __init__(self):
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class Message:
    def __init__(self, **kwargs):
        self.header = SimpleNamespace(stamp=None, frame_id='')
        self.__dict__.update(kwargs)


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = dict(self.overrides)
        self.logger = Mock()
        self.publishers = []

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)
        return self.get_parameter(name)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_logger(self):
        return self.logger

    def create_publisher(self, *args):
        publisher = Mock()
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, *args):
        return Mock()

    def add_on_set_parameters_callback(self, callback):
        self.parameter_callback = callback

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 123))


class PathExecutionCancelled(RuntimeError):
    pass


def pose_to_matrix(pose):
    q = pose.orientation
    quaternion = np.array([q.x, q.y, q.z, q.w])
    np.testing.assert_allclose(np.linalg.norm(quaternion), 1.0)
    transform = np.eye(4)
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return transform


def load_server():
    tree = ast.parse(SOURCE.read_text())
    definitions = [node for node in tree.body
                   if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and
                   getattr(node, 'name', '') != 'main']
    namespace = dict(Node=FakeNode, threading=threading, time=time, np=np, json=json,
                     os=os, tempfile=tempfile, Path=Path, Pose=Pose,
                     PoseArray=Message, PoseStamped=Message, StringArray=Message,
                     SetParametersResult=Message, GoalResponse=SimpleNamespace(ACCEPT=1, REJECT=2),
                     CancelResponse=SimpleNamespace(ACCEPT=1),
                     FrameTask=SimpleNamespace(Result=Message, Feedback=Message),
                     NamedConfig=SimpleNamespace(Result=Message, Feedback=Message),
                     NAMED_CONFIGS={'prep': [0], 'home': [1]}, CONFIG_DIR='config',
                     URDF_MAGPIE_PIN_PATH='pin', URDF_MAGPIE_SPHERE_PATH='sphere',
                     SRDF_MAGPIE_SPHERE_PATH='srdf',
                     pose_to_matrix=pose_to_matrix, matrix_to_pose=lambda matrix: matrix.copy(),
                     PathExecutionCancelled=PathExecutionCancelled,
                     ReentrantCallbackGroup=Mock(), ActionServer=Mock(),
                     maybe_start_controller_logging=Mock())
    config = {'controller': dict(timeout=1, threshold_ik=.1, threshold_joint=.1,
                                 threshold_linear=.01, threshold_angular=.01)}
    namespace['load_controller_config'] = Mock(return_value=config)
    exec(compile(ast.fix_missing_locations(ast.Module(body=definitions, type_ignores=[])),
                 str(SOURCE), 'exec'), namespace)
    return namespace


class ROSComparisonTests(unittest.TestCase):
    def setUp(self):
        self.namespace = load_server()
        self.controller = Mock()
        self.controller.robot_model = SimpleNamespace(
            model_body=SimpleNamespace(frames=[SimpleNamespace(name=name) for name in
                                             ('left_grasp_frame', 'right_grasp_frame', 'pelvis')]),
            state={'q': np.zeros(27)})
        self.controller.left_ee_name = 'left_grasp_frame'
        self.controller.right_ee_name = 'right_grasp_frame'
        self.controller.get_frame_transformation.return_value = np.eye(4)
        self.controller.get_frame_task_error.return_value = np.array([.01, 0, 0, .02, 0, 0])
        self.factory = Mock(return_value=self.controller)
        self.factory.__name__ = 'FakeController'
        self.namespace['controller_class'] = Mock(return_value=self.factory)
        self.runner = Mock()
        self.runner.run.return_value = {'success': True, 'cancelled': False,
                                        'reason': '', 'trials': [{'success': True}]}
        module = ModuleType('h12_ros2_controller.core.controller.planned_comparison')
        module.PlannedComparison = Mock(return_value=self.runner)
        self.helper = module.PlannedComparison
        self.module_patch = patch.dict(sys.modules, {module.__name__: module})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.environment = patch.dict(os.environ, {'ROS_LOG_DIR': self.directory.name})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        FakeNode.overrides = {'upper_controller': 'counter'}
        self.server = self.namespace['FrameTaskServer']()
        self.goal = SimpleNamespace(request=SimpleNamespace(
            plan=True, frame_names=['left_grasp_frame'], frame_targets=[Pose()],
            relative=True, slow_mode=True, duration=SimpleNamespace(sec=300, nanosec=0)),
            is_cancel_requested=False, abort=Mock(), succeed=Mock(), canceled=Mock(),
            publish_feedback=Mock())

    def execute(self):
        self.assertEqual(self.server.comparison_goal_callback(self.goal.request), 1)
        return self.server.compare_frame_task_callback(self.goal)

    def summaries(self):
        return [json.loads(path.read_text()) for path in Path(self.directory.name).glob('*.json')]

    def assert_released(self):
        self.assertFalse(self.server._comparison_active)
        self.assertTrue(self.server._controller_lock.acquire(blocking=False))
        self.server._controller_lock.release()
        self.assertEqual(self.server.normal_goal_callback(self.goal.request), 1)

    def test_one_controller_and_no_additional_publisher_for_counter_endpoint(self):
        self.factory.assert_called_once()
        self.helper.assert_called_once_with(self.controller)
        actions = self.namespace['ActionServer'].call_args_list
        self.assertEqual([call.args[2] for call in actions],
                         ['frame_task', 'named_config', 'compare_frame_task'])
        self.assertEqual(len(self.server.publishers), 7)
        self.assertTrue(all(call.args[0] is self.server for call in actions))
        self.namespace['ActionServer'].reset_mock()
        FakeNode.overrides = {}
        frame_server = self.namespace['FrameTaskServer']()
        self.assertFalse(hasattr(frame_server, 'compare_action_server'))
        self.assertEqual(self.namespace['ActionServer'].call_count, 2)
        self.assertEqual(self.helper.call_count, 1)

    def test_invalid_startup_parameters_fail_before_controller_construction(self):
        for parameters in ({'comparison_order': ''}, {'comparison_order': ['frame']},
                           {'comparison_order': 'frame,green'},
                           {'comparison_prepare_config': 'missing'}):
            self.factory.reset_mock()
            FakeNode.overrides = parameters
            with self.assertRaises(ValueError):
                self.namespace['FrameTaskServer']()
            self.factory.assert_not_called()

    def test_invalid_goals_abort_before_core_motion_and_release(self):
        invalid = [dict(plan=False), dict(frame_names=[]),
                   dict(frame_names=['left_grasp_frame', 'right_grasp_frame']),
                   dict(frame_names=['unknown']), dict(duration=SimpleNamespace(sec=-1, nanosec=0))]
        bad_pose = Pose()
        bad_pose.orientation.w = 0
        invalid.append(dict(frame_targets=[bad_pose]))
        nonfinite = Pose()
        nonfinite.position.x = float('nan')
        invalid.append(dict(frame_targets=[nonfinite]))
        original = vars(self.goal.request).copy()
        for values in invalid:
            self.goal.request = SimpleNamespace(**{**original, **values})
            result = self.execute()
            self.assertFalse(result.success)
            self.assert_released()
        self.runner.run.assert_not_called()
        self.assertTrue(all(not summary['success'] and summary['reason']
                            for summary in self.summaries()))

    def test_non_arm_known_frame_rejected_before_core(self):
        self.goal.request.frame_names = ['pelvis']
        self.controller._arm_for_frame.side_effect = ValueError('frame must depend on exactly one moving arm')
        self.assertFalse(self.execute().success)
        self.runner.run.assert_not_called()
        self.assert_released()

    def test_execution_revalidates_order_before_motion(self):
        self.server.parameters['comparison_order'] = 'frame,invalid'
        self.assertFalse(self.execute().success)
        self.runner.run.assert_not_called()
        self.assert_released()

    def test_summary_defaults_to_ros_home_log_not_source_directory(self):
        with patch.dict(os.environ, {'ROS_HOME': self.directory.name}):
            os.environ.pop('ROS_LOG_DIR', None)
            self.assertTrue(self.execute().success)
        summaries = list((Path(self.directory.name) / 'log').glob('planned-comparison-*.json'))
        self.assertEqual(len(summaries), 1)
        self.assertTrue(json.loads(summaries[0].read_text())['success'])

    def test_busy_and_admission_race_reject_without_waiting(self):
        with self.server._controller_lock:
            self.assertEqual(self.server.comparison_goal_callback(self.goal.request), 2)
        self.assertEqual(self.server.comparison_goal_callback(self.goal.request), 1)
        self.assertEqual(self.server.comparison_goal_callback(self.goal.request), 2)
        self.assertEqual(self.server.normal_goal_callback(self.goal.request), 2)
        with self.server._controller_lock:
            self.assertFalse(self.server.frame_task_callback(self.goal).success)
            self.assertFalse(self.server.named_config_callback(self.goal).success)
            self.assertFalse(self.server.compare_frame_task_callback(self.goal).success)
        self.runner.run.assert_not_called()
        self.assert_released()

    def test_order_validation_and_idle_only_updates(self):
        callback = self.server.parameter_callback
        for order in ('', 'frame,', 'frame,red', ['frame'], 'counter; touch /invalid'):
            self.assertFalse(callback([Message(name='comparison_order', value=order)]).successful)
        valid = 'counter,frame,frame,counter'
        self.assertTrue(callback([Message(name='comparison_order', value=valid)]).successful)
        self.server.parameters['comparison_order'] = valid
        self.assertFalse(callback([Message(name='upper_controller', value='frame')]).successful)

        def run(*args, **kwargs):
            self.assertFalse(callback([Message(name='comparison_order', value='frame')]).successful)
            self.assertFalse(callback([Message(name='comparison_prepare_config', value='home')]).successful)
            self.assertEqual(args[2], ('counter', 'frame', 'frame', 'counter'))
            self.assertTrue(self.server._controller_lock.locked())
            return {'success': True, 'trials': [], 'reason': ''}

        self.runner.run.side_effect = run
        self.assertTrue(self.execute().success)
        self.assertEqual(self.summaries()[0]['order'], valid.split(','))
        self.assert_released()

    def test_success_uses_original_cancel_flag_budget_and_absolute_snapshot(self):
        self.goal.request.frame_targets[0].orientation.w = 3
        absolute = np.eye(4)
        absolute[:3, 3] = [.4, .2, .1]

        def run(frame, target, order, **kwargs):
            self.assertEqual(frame, 'left_grasp_frame')
            self.assertEqual(order, ('frame', 'counter'))
            self.assertTrue(kwargs['relative'])
            self.assertTrue(kwargs['slow_mode'])
            self.assertGreater(kwargs['timeout'], 299)
            self.assertLessEqual(kwargs['timeout'], 300)
            self.assertFalse(kwargs['cancel_check']())
            event = kwargs['event']
            event({'type': 'phase', 'phase': 'preparation'})
            self.assertEqual(self.server._frame_publish_snapshot, ((), ()))
            event({'type': 'reference', 'target': absolute.tolist(), 'hash': 'abc'})
            event({'type': 'phase', 'phase': 'revalidation', 'hash': 'abc', 'index': 0, 'condition': 'frame'})
            event({'type': 'step', 'phase': 'playback', 'work_elapsed': .01, 'overrun': 0.0})
            self.assertEqual(self.server._frame_publish_snapshot[0], ('left_grasp_frame',))
            np.testing.assert_array_equal(self.server._frame_publish_snapshot[1][0], absolute)
            self.server.publisher_callback()
            np.testing.assert_array_equal(self.server.frame_targets_publisher.publish.call_args.args[0].poses[0], absolute)
            np.testing.assert_array_equal(self.server.frame_poses_publisher.publish.call_args.args[0].poses[0], np.eye(4))
            event({'type': 'phase', 'phase': 'preparation'})
            event({'type': 'step', 'phase': 'hold', 'work_elapsed': .01, 'overrun': 0.0})
            self.assertEqual(self.server._frame_publish_snapshot, ((), ()))
            return {'success': True, 'trials': [{'hash': 'abc', 'success': True}], 'reason': ''}

        self.runner.run.side_effect = run
        self.assertTrue(self.execute().success)
        self.goal.succeed.assert_called_once()
        feedback = self.goal.publish_feedback.call_args.args[0]
        self.assertEqual(feedback.errors_linear, [.01])
        self.assertEqual(feedback.errors_angular, [.02])
        self.assertEqual(self.summaries()[0]['trials'][0]['hash'], 'abc')
        self.assert_released()

    def test_cancellation_failure_and_exception_cleanup(self):
        for outcome in ('cancel', 'failure', 'exception'):
            self.goal.canceled.reset_mock()
            self.goal.abort.reset_mock()

            def run(*args, **kwargs):
                if outcome == 'cancel':
                    self.goal.is_cancel_requested = True
                    self.assertTrue(kwargs['cancel_check']())
                    return {'success': False, 'cancelled': True, 'reason': 'canceled', 'trials': []}
                if outcome == 'exception':
                    raise RuntimeError('solver failure')
                return {'success': False, 'reason': 'joint position limit', 'trials': []}

            self.goal.is_cancel_requested = False
            self.runner.run.side_effect = run
            self.assertFalse(self.execute().success)
            if outcome == 'cancel':
                self.goal.canceled.assert_called_once()
                self.goal.abort.assert_not_called()
            else:
                self.goal.abort.assert_called_once()
            self.assert_released()

    def test_late_cancel_status_and_saved_summary_agree(self):
        def run(*args, **kwargs):
            self.goal.is_cancel_requested = True
            return {'success': True, 'cancelled': False, 'reason': '', 'trials': []}

        self.runner.run.side_effect = run
        self.assertFalse(self.execute().success)
        self.goal.canceled.assert_called_once()
        self.assertFalse(self.summaries()[0]['success'])
        self.assertTrue(self.summaries()[0]['cancelled'])
        self.assert_released()

    def test_step_timing_separates_outer_phases_and_trials_without_step_logs(self):
        trials = [{'index': 0, 'condition': 'frame', 'success': True, 'hash': 'original'}]

        def run(*args, **kwargs):
            event = kwargs['event']
            common = dict(index=0, condition='frame')
            event(dict(common, type='phase', phase='preparation'))
            event(dict(common, type='step', phase='hold', work_elapsed=.002, overrun=0.0))
            event(dict(common, type='phase', phase='playback'))
            event(dict(common, type='step', phase='playback', work_elapsed=.03, overrun=.01))
            event(dict(common, type='step', phase='playback', work_elapsed=.01, overrun=0.0))
            event(dict(common, type='phase', phase='hold'))
            event(dict(common, type='step', phase='hold', work_elapsed=.04, overrun=.02))
            common = dict(index=1, condition='counter')
            event(dict(common, type='phase', phase='preparation'))
            event(dict(common, type='step', phase='hold', work_elapsed=.003, overrun=0.0))
            return dict(success=True, trials=trials, reason='')

        self.runner.run.side_effect = run
        self.assertTrue(self.execute().success)
        summary = self.summaries()[0]
        self.assertEqual(summary['trials'], trials)
        timing = summary['step_timing']
        self.assertEqual([(item['index'], item['condition'], item['phase']) for item in timing],
                         [(0, 'frame', 'preparation'), (0, 'frame', 'playback'),
                          (0, 'frame', 'hold'), (1, 'counter', 'preparation')])
        self.assertEqual([item['step_count'] for item in timing], [1, 2, 1, 1])
        self.assertEqual([item['overrun_count'] for item in timing], [0, 1, 1, 0])
        self.assertAlmostEqual(timing[1]['work_elapsed_sum'], .04)
        self.assertAlmostEqual(timing[1]['work_elapsed_max'], .03)
        self.assertAlmostEqual(timing[1]['overrun_max'], .01)
        self.assertAlmostEqual(timing[2]['overrun_max'], .02)
        self.assertIn('excludes observer callback overhead', summary['step_timing_scope'])
        self.assertIn('not a hard-real-time guarantee', summary['step_timing_scope'])
        json.dumps(summary, allow_nan=False)
        logs = [call.args[0] for call in self.server.logger.info.call_args_list]
        self.assertFalse(any('"type": "step"' in text for text in logs))
        self.assert_released()

    def test_step_timing_survives_exception_and_cancel_partial_batches(self):
        for cancelled in (False, True):
            def run(*args, **kwargs):
                event = kwargs['event']
                event(dict(type='phase', phase='preparation', index=0, condition='counter'))
                event(dict(type='step', phase='hold', index=0, condition='counter',
                           work_elapsed=.03, overrun=.01))
                if cancelled:
                    return dict(success=False, cancelled=True, reason='canceled', trials=[])
                raise RuntimeError('partial failure')

            self.runner.run.side_effect = run
            self.assertFalse(self.execute().success)
            summary = next(item for item in self.summaries()
                           if item['reason'] == ('canceled' if cancelled else 'partial failure'))
            self.assertEqual(summary['step_timing'], [dict(
                index=0, condition='counter', phase='preparation', step_count=1,
                work_elapsed_sum=.03, work_elapsed_max=.03, overrun_count=1, overrun_max=.01)])
            json.dumps(summary, allow_nan=False)
            self.assert_released()


if __name__ == '__main__':
    unittest.main()
