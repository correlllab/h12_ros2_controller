import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration

from custom_ros_messages.action import FrameTask

class FrameTaskClient(Node):
    def __init__(self):
        super().__init__('frame_task_client')

        # create the action client
        self.action_client = ActionClient(
            self,
            FrameTask,
            'frame_task'   # action name
        )

    def send_goal(self):
        goal_msg = FrameTask.Goal()
        goal_msg.frame_names = ['left_wrist_yaw_link', 'right_wrist_yaw_link']

        pose_left = Pose()
        pose_left.position.x = 0.4
        pose_left.position.y = 0.4
        pose_left.position.z = 0.4
        pose_left.orientation.w = 1.0

        pose_right = Pose()
        pose_right.position.x = 0.4
        pose_right.position.y = -0.4
        pose_right.position.z = 0.4
        pose_right.orientation.w = 1.0

        goal_msg.frame_targets = [pose_left, pose_right]
        goal_msg.keyword = ''
        goal_msg.duration = Duration(sec=3, nanosec=0)

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Sending goal request...')
        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted by server.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: success={result.success}')

def main(args=None):
    rclpy.init(args=args)
    node = FrameTaskClient()

    try:
        node.send_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
