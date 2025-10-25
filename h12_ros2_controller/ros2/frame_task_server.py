import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Pose, PoseStamped, PoseArray

import time
import asyncio
import numpy as np
from pyquaternion import Quaternion

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from custom_ros_messages.msg import StringArray
from custom_ros_messages.action import FrameTask
from h12_ros2_controller.core.frame_controller import FrameController
from h12_ros2_controller.utility.path_definition import URDF_PIN_PATH, URDF_SPHERE_PATH, SRDF_SPHERE_PATH

class FrameTaskServer(Node):
    def __init__(self,
                 dt=0.03,
                 timeout=10.0,
                 threshold_linear=5e-3,
                 threshold_angular=2e-2):
        super().__init__('frame_task_server')
        self.timeout = timeout
        self.threshold_linear = threshold_linear
        self.threshold_angular = threshold_angular
        # lists holding frame names and frame targets
        self.frame_names = []
        self.frame_targets = []

        ChannelFactoryInitialize()
        self.controller = FrameController(URDF_PIN_PATH,
                                          URDF_SPHERE_PATH,
                                          SRDF_SPHERE_PATH,
                                          dt=dt,
                                          v_lim=1.0,
                                          w_lim=2.0,
                                          dq_lim=2.0,
                                          d_min=0.02,
                                          visualize=False)

        # publisher publishing frame names and target poses
        self.frame_names_publisher = self.create_publisher(StringArray, 'frame_names', 10)
        self.frame_targets_publisher = self.create_publisher(PoseArray, 'frame_targets', 10)
        self.frame_poses_publisher = self.create_publisher(PoseArray, 'frame_poses', 10)
        self.publisher_timer = self.create_timer(1.0 / 100, self.publisher_callback)

        # create the action server
        self.action_server = ActionServer(
            self,
            FrameTask,
            'frame_task',          # action name
            self.execute_callback
        )

    def publisher_callback(self):
        # publish frame names
        frame_names = StringArray()
        frame_names.data = self.frame_names
        self.frame_names_publisher.publish(frame_names)

        # publish frame targets
        frame_targets = PoseArray()
        frame_targets.poses = self.frame_targets
        self.frame_targets_publisher.publish(frame_targets)

        # publish frame poses
        frame_poses = PoseArray()
        frame_poses.poses = [
            self._matrix_to_pose(self.controller.get_frame_transformation(name))
            for name in self.frame_names
        ]
        self.frame_poses_publisher.publish(frame_poses)

    @staticmethod
    def _pose_to_matrix(pose):
        position = pose.position
        orientation = pose.orientation
        rotation = Quaternion(
            [orientation.w, orientation.x, orientation.y, orientation.z]
        ).rotation_matrix
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = [position.x, position.y, position.z]
        return matrix

    @staticmethod
    def _matrix_to_pose(matrix):
        pose = Pose()
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        rotation = Quaternion(matrix=matrix[:3, :3])
        pose.orientation.w = rotation.w
        pose.orientation.x = rotation.x
        pose.orientation.y = rotation.y
        pose.orientation.z = rotation.z
        return pose

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received new goal!')

        goal = goal_handle.request

        # clear existing frame tasks
        self.controller.clear_frame_tasks()
        self.frame_names = goal.frame_names
        self.frame_targets = goal.frame_targets

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
            self.controller.add_frame_task(name, name, self._pose_to_matrix(pose))

        # update ik solver with current state
        self.controller.update_ik_solver()

        # main loop
        start_time = time.time()
        duration = goal.duration.sec + goal.duration.nanosec * 1e-9
        timeout = duration if duration > 0.0 else self.timeout
        while time.time() - start_time < timeout:
            frame_start_time = time.time()
            # control one step
            self.controller.control_step()

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                result = FrameTask.Result()
                result.success = False
                return result

            time.sleep(max(0.0, self.controller.dt - (time.time() - frame_start_time)))
            await asyncio.sleep(0)

        # set result
        result = FrameTask.Result()
        result.success = True
        self.get_logger().info('Goal succeeded')

        goal_handle.succeed()
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal')
        return CancelResponse.ACCEPT

def main(args=None):
    rclpy.init(args=args)
    node = FrameTaskServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
