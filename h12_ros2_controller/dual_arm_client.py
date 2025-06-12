import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped

from threading import Thread
from scipy.spatial.transform import Rotation as R

from controller_msgs.action import DualArm

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

    def send_goal(self, left_target: Pose, right_target: Pose):
        goal_msg = DualArm.Goal()
        goal_msg.left_target = left_target
        goal_msg.right_target = right_target

        self.action_client.wait_for_server()

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

        self.get_logger().info('Goal accepted, waiting for result...')
        self.get_logger().info('Press ENTER to cancel the goal')
        cencel_thread = Thread(target=self._keyboard_cancel)
        cencel_thread.start()

        future_result = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        result = future_result.result().result
        self.get_logger().info(f'Final result: success = {result.success}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Left Error: {feedback.left_error:.2f}; Right Error: {feedback.right_error:.2f}')

    def _keyboard_cancel(self):
        input('Press ENTER to cancel the goal...')
        self.get_logger().info('Cancelling goal...')
        if self.goal_handle is not None:
            future_cancel = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future_cancel)
            if future_cancel.result().accepted:
                self.get_logger().info('Goal cancelled successfully')
            else:
                self.get_logger().warn('Goal cancellation was rejected')
        else:
            self.get_logger().warn('No goal to cancel')

def input_pose():
    pose = Pose()
    pose.position.x = float(input('Enter x position: '))
    pose.position.y = float(input('Enter y position: '))
    pose.position.z = float(input('Enter z position: '))
    r = float(input('Enter roll (in degrees): '))
    p = float(input('Enter pitch (in degrees): '))
    y = float(input('Enter yaw (in degrees): '))
    quat = R.from_euler('xyz', [r, p, y], degrees=True).as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def main(args=None):
    rclpy.init(args=args)
    node = DualArmClient()

    # Example poses
    left_target = Pose()
    right_target = Pose()
    left_target.position.x = 0.5
    left_target.position.y = 0.2
    left_target.position.z = 0.1
    right_target.position.x = 0.3
    right_target.position.y = -0.2
    right_target.position.z = 0.1

    try:
        while rclpy.ok():
            left_pose = input_pose()
            right_pose = input_pose()
            node.send_goal(left_pose, right_pose)

            cont = input('Do you want to send another goal? (y/n): ').lower()
            if cont != 'y':
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
