import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Pose, PoseStamped

import time
import asyncio
import numpy as np
from pyquaternion import Quaternion

from custom_ros_messages.action import DualArm
from h12_ros2_controller.controller import ArmController
from h12_ros2_controller.utility.path_definition import URDF_PIN_PATH

class MoveDualArmServer(Node):
    def __init__(self, dt=0.01, vlim=1.0, threshold=0.1):
        super().__init__('move_dual_arm_server')
        self.threshold = threshold
        self.controller = ArmController(URDF_PIN_PATH,
                                        dt=dt,
                                        vlim=vlim,
                                        visualize=False)
        # publisher of left and right end-effector poses
        self.left_ee_pose_publisher = self.create_publisher(
            PoseStamped,
            'left_ee_pose',
            10
        )
        self.right_ee_pose_publisher = self.create_publisher(
            PoseStamped,
            'right_ee_pose',
            10
        )
        # publisher of left and right end-effector target poses
        self.left_ee_target_publisher = self.create_publisher(
            PoseStamped,
            'left_ee_target',
            10
        )
        self.right_ee_target_publisher = self.create_publisher(
            PoseStamped,
            'right_ee_target',
            10
        )
        self.left_ee_pose_timer = self.create_timer(1.0 / 100, self.publish_left_ee_pose)
        self.right_ee_pose_timer = self.create_timer(1.0 / 100, self.publish_right_ee_pose)
        self.left_ee_target_timer = self.create_timer(1.0 / 100, self.publish_left_ee_target)
        self.right_ee_target_timer = self.create_timer(1.0 / 100, self.publish_right_ee_target)

        # action server to control dual arms
        self.action_server = ActionServer(
            self,
            DualArm,
            'move_dual_arm',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Controller server initialized')

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

    def _stamp_pose(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'pelvis'
        pose_stamped.pose = pose
        return pose_stamped

    def publish_left_ee_pose(self):
        # sync and update robot model
        self.controller.sync_robot_model()
        self.controller.update_robot_model()
        # transform and publish the pose
        left_ee_pose = self._matrix_to_pose(self.controller.left_ee_transformation)
        self.left_ee_pose_publisher.publish(self._stamp_pose(left_ee_pose))

    def publish_right_ee_pose(self):
        # sync and update robot model
        self.controller.sync_robot_model()
        self.controller.update_robot_model()
        # transform and publish the pose
        right_ee_pose = self._matrix_to_pose(self.controller.right_ee_transformation)
        self.right_ee_pose_publisher.publish(self._stamp_pose(right_ee_pose))

    def publish_left_ee_target(self):
        # sync and update robot model
        self.controller.sync_robot_model()
        self.controller.update_robot_model()
        # transform and publish the pose
        left_ee_target = self._matrix_to_pose(self.controller.left_ee_target_transformation)
        self.left_ee_target_publisher.publish(self._stamp_pose(left_ee_target))

    def publish_right_ee_target(self):
        # sync and update robot model
        self.controller.sync_robot_model()
        self.controller.update_robot_model()
        # transform and publish the pose
        right_ee_target = self._matrix_to_pose(self.controller.right_ee_target_transformation)
        self.right_ee_target_publisher.publish(self._stamp_pose(right_ee_target))

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal')
        feedback_msg = DualArm.Feedback()

        # set left and right target poses
        self.controller.left_ee_target_transformation = self._pose_to_matrix(
            goal_handle.request.left_target
        )
        self.controller.right_ee_target_transformation = self._pose_to_matrix(
            goal_handle.request.right_target
        )
        start_time = time.time()
        while time.time() - start_time < 20:
            time_start = time.time()
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                result = DualArm.Result()
                result.success = False
                return result

            # check the error
            left_error = np.linalg.norm(self.controller.left_ee_error)
            right_error = np.linalg.norm(self.controller.right_ee_error)
            # publish feedback errors
            feedback_msg.left_error = left_error
            feedback_msg.right_error = right_error
            goal_handle.publish_feedback(feedback_msg)
            # check if the goal is reached
            if left_error < self.threshold and right_error < self.threshold:
                self.get_logger().info('Goal reached')
                break
            # control one step
            # self.controller.sim_dual_arm_step()
            self.controller.control_dual_arm_step()

            time.sleep(max(0, self.controller.dt - (time.time() - time_start)))
            await asyncio.sleep(0)

        goal_handle.succeed()
        result = DualArm.Result()
        result.success = True
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal')
        return CancelResponse.ACCEPT

def main(args=None):
    rclpy.init(args=args)
    node = MoveDualArmServer(threshold=0.05)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
