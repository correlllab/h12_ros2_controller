import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped

from pynput import keyboard
from scipy.spatial.transform import Rotation as R

from custom_ros_messages.action import DualArm

class DualArmClient(Node):
    def __init__(self):
        super().__init__('dual_arm_client')
        self.action_client = ActionClient(
            self,
            DualArm,
            'move_dual_arm'
        )
        self.goal_handle = None

        # subscriber of left and right end-effector poses
        self.left_ee_pose_subscriber = self.create_subscription(
            PoseStamped,
            'left_ee_pose',
            self.subscribe_left_ee_pose,
            10
        )
        self.right_ee_pose_subscriber = self.create_subscription(
            PoseStamped,
            'right_ee_pose',
            self.subscribe_right_ee_pose,
            10
        )
        # subscriber of left and right end-effector target poses
        self.left_ee_target_subscriber = self.create_subscription(
            PoseStamped,
            'left_ee_target',
            self.subscribe_left_ee_target,
            10
        )
        self.right_ee_target_subscriber = self.create_subscription(
            PoseStamped,
            'right_ee_target',
            self.subscribe_right_ee_target,
            10
        )
        self.left_ee_pose = None
        self.right_ee_pose = None
        self.left_ee_target = None
        self.right_ee_target = None

    def subscribe_left_ee_pose(self, msg):
        self.left_ee_pose = msg.pose

    def subscribe_right_ee_pose(self, msg):
        self.right_ee_pose = msg.pose

    def subscribe_left_ee_target(self, msg):
        self.left_ee_target = msg.pose

    def subscribe_right_ee_target(self, msg):
        self.right_ee_target = msg.pose

    def send_goal(self, left_target: Pose=Pose(), right_target: Pose=Pose(), keyword: str=''):
        goal_msg = DualArm.Goal()
        goal_msg.left_target = left_target
        goal_msg.right_target = right_target
        goal_msg.keyword = keyword

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
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Left Error Linear: {feedback.left_error_linear:.4f}, ' +
                               f'Left Error Angular: {feedback.left_error_angular:.4f}')
        self.get_logger().info(f'Right Error Linear: {feedback.right_error_linear:.4f}, ' +
                               f'Right Error Angular: {feedback.right_error_angular:.4f}')

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
    quat = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

def input_keyword_or_poses():
    '''
    Ask user for keyword command or manual pose input.
    Returns (keyword, left_pose, right_pose) tuple.
    If keyword is provided, poses will be None.
    If no keyword, poses will be provided.
    '''
    print('Available keywords: "home"')

    choice = input('Enter keyword (or press Enter for manual poses): ').strip()

    if choice:
        # user entered a keyword
        return choice, None, None
    else:
        # user wants manual pose input
        left_pose = input_pose('left')
        right_pose = input_pose('right')
        return '', left_pose, right_pose

def input_pose(side):
    print(f'{side.capitalize()} end-effector pose...')

    while True:
        input_pose = input('Enter x y z roll pitch yaw (separated by space): ')
        parts = input_pose.strip().split()

        if len(parts) != 6:
            print('Invalid input. Please enter exactly 6 values.')
            continue
        try:
            values = [float(val) for val in parts]
            return list_to_pose(values)
        except ValueError:
            print('Invalid input. Make sure all 6 values are numeric.')
            continue

def main(args=None):
    rclpy.init(args=args)
    node = DualArmClient()

    try:
        while rclpy.ok():
            keyword, left_pose, right_pose = input_keyword_or_poses()

            if keyword:
                # send keyword command
                node.send_goal(keyword=keyword)
            else:
                # send pose targets
                node.send_goal(left_target=left_pose, right_target=right_pose)

            input('Press any key to continue...') # flush the input buffer
            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
