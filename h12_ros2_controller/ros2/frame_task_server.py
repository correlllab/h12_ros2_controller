import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose, PoseStamped

from custom_ros_messages.action import FrameTask

class FrameTaskServer(Node):
    def __init__(self):
        super().__init__('frame_task_server')

        # create the action server
        self._action_server = ActionServer(
            self,
            FrameTask,
            'frame_task',          # action name
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received new goal!')

        goal = goal_handle.request

        # print out the goal contents
        self.get_logger().info(f'Keyword: {goal.keyword}')
        self.get_logger().info(f'Duration: {goal.duration.sec}s {goal.duration.nanosec}ns')

        for name, pose in zip(goal.frame_names, goal.frame_targets):
            self.get_logger().info(f'Frame: {name}')
            self.get_logger().info(
                f'    Position: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
            )
            self.get_logger().info(
                f'    Orientation: ({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f})'
            )

        # set result
        result = FrameTask.Result()
        result.success = True

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FrameTaskServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
