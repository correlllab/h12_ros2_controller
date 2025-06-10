import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose

import time
import numpy as np
from pyquaternion import Quaternion

from controller_msgs.action import MoveDualArm
from h12_ros2_controller.controller import ArmController
from h12_ros2_controller.utility.path_definition import URDF_PIN_PATH

class MoveDualArmServer(Node):
    def __init__(self):
        super().__init__('move_dual_arm_server')
        self.controller = ArmController(URDF_PIN_PATH,
                                        dt=0.01,
                                        vlim=1.0,
                                        visualize=False)

        # publisher of left and right end-effector poses
        self.left_ee_pose_publisher = self.create_publisher(
            Pose,
            'left_ee_pose',
            10
        )
        self.right_ee_pose_publisher = self.create_publisher(
            Pose,
            'right_ee_pose',
            10
        )
        # publisher of left and right end-effector target poses
        self.left_ee_target_publisher = self.create_publisher(
            Pose,
            'left_ee_target_pose',
            10
        )
        self.right_ee_target_publisher = self.create_publisher(
            Pose,
            'right_ee_target_pose',
            10
        )
        self.left_ee_pose_timer = self.create_timer(1.0 / 100, self.publish_left_ee_pose)
        self.right_ee_pose_timer = self.create_timer(1.0 / 100, self.publish_right_ee_pose)
        self.left_ee_target_timer = self.create_timer(1.0 / 100, self.publish_left_ee_target)
        self.right_ee_target_timer = self.create_timer(1.0 / 100, self.publish_right_ee_target)

        # action server to control dual arms
        self.action_server = ActionServer(
            self,
            MoveDualArm,
            'move_dual_arm',
            self.execute_callback
        )

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

    def publish_left_ee_pose(self):
        left_ee_pose = self._matrix_to_pose(self.controller.left_ee_transformation)
        self.left_ee_pose_publisher.publish(left_ee_pose)

    def publish_right_ee_pose(self):
        right_ee_pose = self._matrix_to_pose(self.controller.right_ee_transformation)
        self.right_ee_pose_publisher.publish(right_ee_pose)

    def publish_left_ee_target(self):
        left_ee_target = self._matrix_to_pose(self.controller.left_ee_target_transformation)
        self.left_ee_target_publisher.publish(left_ee_target)

    def publish_right_ee_target(self):
        right_ee_target = self._matrix_to_pose(self.controller.right_ee_target_transformation)
        self.right_ee_target_publisher.publish(right_ee_target)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal')
        feedback_msg = MoveDualArm.Feedback()

        # set left and right target poses
        self.controller.left_ee_target_transformation = self._pose_to_matrix(
            goal_handle.request.left_target_pose
        )
        self.controller.right_ee_target_transformation = self._pose_to_matrix(
            goal_handle.request.right_target_pose
        )

        # for i in range(1, 11):
        #     feedback_msg.progress = i * 10.0
        #     goal_handle.publish_feedback(feedback_msg)
        #     await rclpy.sleep(0.02)  # ~2 sec total
        # TODO move

        goal_handle.succeed()
        result = MoveDualArm.Result()
        result.success = True

        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveDualArmServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
