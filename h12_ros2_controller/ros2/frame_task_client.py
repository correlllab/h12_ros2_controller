import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration

import numpy as np
from pynput import keyboard
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

from custom_ros_messages.action import FrameTask
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS

class FrameTaskClient(Node):
    def __init__(self):
        super().__init__('frame_task_client')

        # create the action client
        self.action_client = ActionClient(
            self,
            FrameTask,
            'frame_task'   # action name
        )

    def send_goal(self, keyword='', frame_names=None, frame_targets=None):
        goal_msg = FrameTask.Goal()

        goal_msg.frame_names = frame_names if frame_names is not None else []
        goal_msg.frame_targets = frame_targets if frame_targets is not None else []

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # send action
        self.get_logger().info('Sending goal...')
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
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
        result = future_result.result().result
        self.get_logger().info(f'Final result: success = {result.success}')
        # stop the cancel listener thread
        listener.stop()

    def feedback_callback(self, feedback_msg):
        errors_linear = list(feedback_msg.feedback.errors_linear)
        errors_angular = list(feedback_msg.feedback.errors_angular)
        self.get_logger().info(f'Linear errors: {errors_linear}')
        self.get_logger().info(f'Angular errors: {errors_angular}')

    def _keyboard_cancel(self, key):
        if key == keyboard.Key.backspace:
            if self.goal_handle is not None:
                self.get_logger().info('Cancelling goal...')
                self.goal_handle.cancel_goal_async()

def list_to_pose(values):
    assert(len(values) == 6), 'Please enter a pose of 6 elements'
    x, y, z, roll, pitch, yaw = values
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()  # Changed to degrees=False since we convert to radians earlier
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

def input_keyword_or_frame_task():
    '''
    Ask user for keyword command or manual frame task input
    Returns (keyword, frame_name, pose) tuple
    If keyword is provided, frame_name and pose will be None
    If no keyword, frame_name and pose will be provided
    '''
    print(f'Available keywords: {list(NAMED_CONFIGS.keys())}')

    choice = input('Enter keyword (or press Enter for manual frame task): ').strip()

    if choice:
        # user entered a keyword
        return choice, [], []
    else:
        # user wants manual frame task input
        frame_names = []
        frame_poses = []
        while True:
            frame_name, pose = input_frame_task()
            frame_names.append(frame_name)
            frame_poses.append(pose)
            cont = input('Do you want to add another frame task? (y/n): ').lower()
            if cont != 'y':
                break
        return '', frame_names, frame_poses

def input_frame_task():
    '''Get frame name and pose from user'''
    frame_name = input('Enter frame name: ')
    while True:
        input_pose = input('Enter x y z roll pitch yaw (separated by space): ')
        parts = input_pose.strip().split()

        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue
        try:
            values = [float(val) for val in parts]
            # degrees to radians for roll, pitch, yaw
            values[3:] = np.deg2rad(values[3:])
            return frame_name, list_to_pose(values)
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')
            continue

def main(args=None):
    rclpy.init(args=args)
    node = FrameTaskClient()

    try:
        while rclpy.ok():
            keyword, frame_names, frame_poses = input_keyword_or_frame_task()
            node.send_goal(keyword, frame_names, frame_poses)

            input('Press any key to continue...') # flush the input buffer
            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
