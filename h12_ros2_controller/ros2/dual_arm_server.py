import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Pose, PoseStamped

import time
import asyncio
import numpy as np
from pyquaternion import Quaternion
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from custom_ros_messages.action import DualArm
from h12_ros2_controller.core.controller.arm_controller import ArmController
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import URDF_PIN_PATH, URDF_SPHERE_PATH, SRDF_SPHERE_PATH, LOG_PATH

class DualArmServer(Node):
    def __init__(self,
                 dt=0.03,
                 timeout=10.0,
                 threshold_linear=5e-3,
                 threshold_angular=2e-2):
        super().__init__('dual_arm_server')
        self.timeout = timeout
        self.threshold_linear = threshold_linear
        self.threshold_angular = threshold_angular

        ChannelFactoryInitialize()
        self.controller = ArmController(URDF_PIN_PATH,
                                        URDF_SPHERE_PATH,
                                        SRDF_SPHERE_PATH,
                                        dt=dt,
                                        visualize=False,
                                        sport_mode=False)
        # start recording with background saving
        save_path = f'{LOG_PATH}/control_record'
        self.controller.start_recording(save_path=save_path, filename='record_ros2')

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
        self.publisher_timer = self.create_timer(1.0 / 100, self.publisher_callback)

        # action server to control dual arms
        self.action_server = ActionServer(
            self,
            DualArm,
            'move_dual_arm',
            execute_callback=self.dual_arm_callback,
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

    def publisher_callback(self):
        self.controller.update_robot_model()
        # transform and publish the pose
        left_ee_pose = self._matrix_to_pose(self.controller.left_ee_transformation)
        right_ee_pose = self._matrix_to_pose(self.controller.right_ee_transformation)
        left_ee_target = self._matrix_to_pose(self.controller.left_ee_target_transformation)
        right_ee_target = self._matrix_to_pose(self.controller.right_ee_target_transformation)
        self.left_ee_pose_publisher.publish(self._stamp_pose(left_ee_pose))
        self.right_ee_pose_publisher.publish(self._stamp_pose(right_ee_pose))
        self.left_ee_target_publisher.publish(self._stamp_pose(left_ee_target))
        self.right_ee_target_publisher.publish(self._stamp_pose(right_ee_target))

    async def dual_arm_callback(self, goal_handle):
        self.get_logger().info('Received goal')

        goal = goal_handle.request
        print(f'{goal.keyword=}')
        print(f'{goal.keyword in NAMED_CONFIGS=}')

        self.get_logger().info('Going to target end-effector poses')
        # set target
        self.controller.left_ee_target_transformation = self._pose_to_matrix(goal.left_target)
        self.controller.right_ee_target_transformation = self._pose_to_matrix(goal.right_target)

        # update ik solver with current state
        self.controller.update_ik_solver()

        # main loop
        start_time = time.time()
        duration = goal.duration.sec + goal.duration.nanosec * 1e-9
        timeout = duration if duration > 0.0 else self.timeout
        while time.time() - start_time < timeout:
            frame_start_time = time.time()
            # control one step
            self.controller.control_dual_arm_step()

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                result = DualArm.Result()
                result.success = False
                return result

            # compute errors
            left_error_linear = np.linalg.norm(self.controller.left_ee_error[:3])
            left_error_angular = np.linalg.norm(self.controller.left_ee_error[3:])
            right_error_linear = np.linalg.norm(self.controller.right_ee_error[:3])
            right_error_angular = np.linalg.norm(self.controller.right_ee_error[3:])
            # write feedback
            feedback_msg = DualArm.Feedback()
            feedback_msg.left_error_linear = left_error_linear
            feedback_msg.left_error_angular = left_error_angular
            feedback_msg.right_error_linear = right_error_linear
            feedback_msg.right_error_angular = right_error_angular
            goal_handle.publish_feedback(feedback_msg)

            # check if the goal is reached
            if (left_error_linear < self.threshold_linear and
                right_error_linear < self.threshold_linear and
                left_error_angular < self.threshold_angular and
                right_error_angular < self.threshold_angular):
                self.get_logger().info('Goal reached')
                break

            time.sleep(max(0.0, self.controller.dt - (time.time() - frame_start_time)))
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
    node = DualArmServer(dt=0.03,
                         timeout=10.0,
                         threshold_linear=5e-3,
                         threshold_angular=2e-2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.controller.shutdown()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
