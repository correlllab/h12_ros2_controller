import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from pynput import keyboard

from custom_ros_messages.action import FrameTask, NamedConfig

sys.path.append(str(Path(__file__).resolve().parents[2]))

from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.goto_commands import (
    FrameGoal, GotoConsole, MultiFrameGoal, NamedGoal, Quit,
)
from h12_ros2_controller.ros2.utility import action_result_summary, list_to_pose

class FrameTaskClient(Node):
    def __init__(self):
        super().__init__('frame_task_client')

        # create the action client
        self.action_client = ActionClient(
            self,
            FrameTask,
            'frame_task'   # action name
        )
        self.named_config_client = ActionClient(
            self,
            NamedConfig,
            'named_config'
        )
        self.goal_handle = None

    def send_frame_task_goal(self, frame_names=None, frame_targets=None,
                             plan=False, relative=False):
        goal_msg = FrameTask.Goal()

        goal_msg.frame_names = frame_names if frame_names is not None else []
        goal_msg.frame_targets = frame_targets if frame_targets is not None else []
        goal_msg.plan = plan
        goal_msg.relative = relative

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # send action
        self.get_logger().info('Sending goal...')
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.frame_task_feedback
        )
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            return

        # start a cancel listener thread
        self.get_logger().info('Goal accepted, waiting for result...')
        self.get_logger().info('Press BACKSPACE to cancel the goal')
        listener = keyboard.Listener(
            on_press=self._keyboard_cancel
        )
        listener.start()

        # wait till finish
        future_result = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        self.get_logger().info(f'Final result: {action_result_summary(future_result.result())}')
        # stop the cancel listener thread
        listener.stop()

    def frame_task_feedback(self, feedback_msg):
        errors_linear = list(feedback_msg.feedback.errors_linear)
        errors_angular = list(feedback_msg.feedback.errors_angular)
        self.get_logger().info(f'Linear errors: {errors_linear}')
        self.get_logger().info(f'Angular errors: {errors_angular}')

    def send_named_config_goal(self, config_name: str, plan=False, slow_mode=False):
        goal_msg = NamedConfig.Goal()
        goal_msg.config_name = config_name
        goal_msg.plan = plan
        goal_msg.slow_mode = slow_mode

        self.get_logger().info('Waiting for named config action server...')
        self.named_config_client.wait_for_server()

        # send action
        self.get_logger().info(f'Sending named config goal: {config_name}...')
        future = self.named_config_client.send_goal_async(
            goal_msg,
            feedback_callback=self.named_config_feedback
        )
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Named config goal was rejected')
            return

        # start a cancel listener thread
        self.get_logger().info('Named config goal accepted, waiting for result...')
        self.get_logger().info('Press BACKSPACE to cancel the goal')
        listener = keyboard.Listener(
            on_press=self._keyboard_cancel
        )
        listener.start()

        # wait till finish
        future_result = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        self.get_logger().info(f'Final result: {action_result_summary(future_result.result())}')
        # stop the cancel listener thread
        listener.stop()

    def named_config_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Joint Error: {feedback.joint_error:.4f}')

    def _keyboard_cancel(self, key):
        if key == keyboard.Key.backspace:
            if self.goal_handle is not None:
                self.get_logger().info('Cancelling goal...')
                self.goal_handle.cancel_goal()


def input_plan():
    while True:
        choice = input('Use OMPL path planning? (y/n): ').strip().lower()
        if choice in ('y', 'n'):
            return choice == 'y'
        print('Please enter y or n')


def main(args=None):
    rclpy.init(args=args)
    node = FrameTaskClient()
    console = GotoConsole(
        title='Frame Task', named_goals=NAMED_CONFIGS,
        supported=(NamedGoal, FrameGoal, MultiFrameGoal, Quit),
    )
    console.help()

    try:
        while rclpy.ok():
            try:
                command = console.read_command()
            except ValueError as exc:
                console.say(str(exc))
                continue
            if isinstance(command, Quit):
                break
            if command is None:
                continue
            plan = input_plan()
            if isinstance(command, NamedGoal):
                node.send_named_config_goal(command.name, plan=plan)
            else:
                tasks = command.tasks if isinstance(command, MultiFrameGoal) else (command,)
                node.send_frame_task_goal(
                    [task.frame_name for task in tasks],
                    [list_to_pose(task.pose) for task in tasks], plan=plan,
                )
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
