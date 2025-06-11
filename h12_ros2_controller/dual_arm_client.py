import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped

from controller_msgs.action import DualArm

class DualArmClient(Node):
    def __init__(self):
        super().__init__('dual_arm_client')
        self.action_client = ActionClient(
            self,
            DualArm,
            'move_dual_arm'
        )

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

        return self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Left Error: {feedback.left_error:.2f}; Right Error: {feedback.right_error:.2f}')

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

    future = node.send_goal(left_target, right_target)

    def done_callback(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().warn('Goal was rejected')
            return

        node.get_logger().info('Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda r: node.get_logger().info(
            f'Final result: success = {r.result().result.success}'
        ))

    future.add_done_callback(done_callback)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
