import rclpy
import argparse
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import PoseStamped

import time
import threading
import numpy as np

from custom_ros_messages.action import DualArm, NamedConfig
from h12_ros2_controller.core.controller.arm_controller import ArmController
from h12_ros2_controller.utility.controller_config import (
    load_controller_config,
    initialize_channel_factory,
    maybe_start_controller_logging,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import URDF_PIN_PATH, URDF_SPHERE_PATH, SRDF_SPHERE_PATH
from h12_ros2_controller.ros2.utility import pose_to_matrix, matrix_to_pose

class DualArmServer(Node):
    def __init__(self,
                 dt=0.03,
                 timeout=10.0,
                 threshold_linear=5e-3,
                 threshold_angular=2e-2,
                 config='default.yaml'):
        super().__init__('dual_arm_server')
        self.timeout = timeout
        self.threshold_linear = threshold_linear
        self.threshold_angular = threshold_angular
        self.config_name = config

        config_data = load_controller_config(config)
        initialize_channel_factory(config_data)
        self.controller = ArmController(URDF_PIN_PATH,
                                        URDF_SPHERE_PATH,
                                        SRDF_SPHERE_PATH,
                                        dt=dt,
                                        visualize=False,
                                        config=config)
        maybe_start_controller_logging(self.controller)

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
        self._controller_lock = threading.Lock()
        self.dual_arm_server = ActionServer(
            self,
            DualArm,
            'dual_arm',
            execute_callback=self.dual_arm_callback,
            cancel_callback=self.cancel_callback
        )
        self.named_config_server = ActionServer(
            self,
            NamedConfig,
            'named_config',
            execute_callback=self.named_config_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Dual Arm Server initialized')

    def _stamp_pose(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'pelvis'
        pose_stamped.pose = pose
        return pose_stamped

    def publisher_callback(self):
        self.controller.update_robot_model()
        # transform and publish the pose
        left_ee_pose = matrix_to_pose(self.controller.left_ee_transformation)
        right_ee_pose = matrix_to_pose(self.controller.right_ee_transformation)
        left_ee_target = matrix_to_pose(self.controller.left_ee_target_transformation)
        right_ee_target = matrix_to_pose(self.controller.right_ee_target_transformation)
        self.left_ee_pose_publisher.publish(self._stamp_pose(left_ee_pose))
        self.right_ee_pose_publisher.publish(self._stamp_pose(right_ee_pose))
        self.left_ee_target_publisher.publish(self._stamp_pose(left_ee_target))
        self.right_ee_target_publisher.publish(self._stamp_pose(right_ee_target))

    def dual_arm_callback(self, goal_handle):
        with self._controller_lock:
            self.get_logger().info('Received goal')
            goal = goal_handle.request
            self.get_logger().info(str(goal))

            self.get_logger().info('Going to target end-effector poses')
            # set target
            self.controller.left_ee_target_transformation = pose_to_matrix(goal.left_target)
            self.controller.right_ee_target_transformation = pose_to_matrix(goal.right_target)

            # update ik solver with current state
            self.controller.update_ik_solver()

            # main loop
            start_time = time.time()
            duration = goal.duration.sec + goal.duration.nanosec * 1e-9
            timeout = duration if duration > 0.0 else self.timeout
            while time.time() - start_time < timeout:
                frame_start_time = time.time()
                # control one step
                self.controller.control_step_reduced()

                # handle cancel event
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
                # send feedback
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

            goal_handle.succeed()
            result = DualArm.Result()
            result.success = True
            return result

    def named_config_callback(self, goal_handle):
        with self._controller_lock:
            self.get_logger().info('Received named config goal')
            goal = goal_handle.request
            self.get_logger().info(str(goal))

            # check if the named config exists
            config_name = goal.config_name
            if config_name not in NAMED_CONFIGS:
                self.get_logger().warn(f'Named config "{config_name}" not found')
                goal_handle.abort()
                result = NamedConfig.Result()
                result.success = False
                return result

            self.get_logger().info(f'Going to named config: {config_name}')
            q_reduced = NAMED_CONFIGS[config_name]

            # update ik solver with current state
            self.controller.update_ik_solver()

            # main loop
            start_time = time.time()
            duration = goal.duration.sec + goal.duration.nanosec * 1e-9
            timeout = duration if duration > 0.0 else self.timeout
            while time.time() - start_time < timeout:
                frame_start_time = time.time()
                # control one step
                self.controller.goto_reduced_configuration(q_reduced)

                # handle cancel event
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal cancelled')
                    goal_handle.canceled()
                    result = NamedConfig.Result()
                    result.success = False
                    return result

                # compute error
                joint_error = np.max(np.abs(self.controller.reduced_configuration_error))
                # send feedback
                feedback_msg = NamedConfig.Feedback()
                feedback_msg.joint_error = joint_error
                goal_handle.publish_feedback(feedback_msg)

                # check if the goal is reached
                if joint_error < 1e-3:
                    self.get_logger().info('Named config reached')
                    break

                time.sleep(max(0.0, self.controller.dt - (time.time() - frame_start_time)))

            goal_handle.succeed()
            result = NamedConfig.Result()
            result.success = True
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal')
        return CancelResponse.ACCEPT

def main(args=None):
    parser = argparse.ArgumentParser(description='Dual arm ROS2 action server')
    parser.add_argument('--config', type=str, default='default.yaml', help='YAML file name under config/')
    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = DualArmServer(dt=0.03,
                         timeout=10.0,
                         threshold_linear=5e-3,
                         threshold_angular=2e-2,
                         config=parsed_args.config)
    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.controller.shutdown()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
