import rclpy
import argparse
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

import time
import threading
import numpy as np

from custom_ros_messages.msg import StringArray
from custom_ros_messages.action import FrameTask, NamedConfig
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.controller_config import load_controller_config, initialize_channel_factory
from h12_ros2_controller.utility.path_definition import (
    URDF_HANDLESS_PIN_PATH,
    URDF_HANDLESS_SPHERE_PATH,
    SRDF_HANDLESS_SPHERE_PATH,
    CONFIG_DIR
)
from h12_ros2_controller.ros2.utility import pose_to_matrix, matrix_to_pose

class FrameTaskServer(Node):
    def __init__(self,
                 timeout=None,
                 threshold_ik=None,
                 threshold_joint=None,
                 threshold_linear=None,
                 threshold_angular=None,
                 config_name='debug.yaml'):
        super().__init__('frame_task_server')
        config = load_controller_config(config_name, config_dir=CONFIG_DIR)
        controller_cfg = config['controller']
        self.timeout = controller_cfg['timeout'] if timeout is None else timeout
        self.threshold_ik = (
            controller_cfg['threshold_ik']
            if threshold_ik is None else threshold_ik
        )
        self.threshold_joint = (
            controller_cfg['threshold_joint']
            if threshold_joint is None else threshold_joint
        )
        self.threshold_linear = (
            controller_cfg['threshold_linear']
            if threshold_linear is None else threshold_linear
        )
        self.threshold_angular = (
            controller_cfg['threshold_angular']
            if threshold_angular is None else threshold_angular
        )
        # lists holding frame names and frame targets
        self.frame_names = []
        self.frame_targets = []

        initialize_channel_factory(config)
        self.controller = FrameController(
            URDF_HANDLESS_PIN_PATH,
            URDF_HANDLESS_SPHERE_PATH,
            SRDF_HANDLESS_SPHERE_PATH,
            handless=True,
            visualize=False,
            config=config
        )

        # publisher publishing frame names and target poses
        self.frame_names_publisher = self.create_publisher(StringArray, 'frame_names', 10)
        self.frame_targets_publisher = self.create_publisher(PoseArray, 'frame_targets', 10)
        self.frame_poses_publisher = self.create_publisher(PoseArray, 'frame_poses', 10)

        # convenience publishers for left and right end-effector poses
        self.left_ee_pose_publisher = self.create_publisher(PoseStamped, 'left_ee_pose', 10)
        self.right_ee_pose_publisher = self.create_publisher(PoseStamped, 'right_ee_pose', 10)
        self.left_ee_target_publisher = self.create_publisher(PoseStamped, 'left_ee_target', 10)
        self.right_ee_target_publisher = self.create_publisher(PoseStamped, 'right_ee_target', 10)

        self.publisher_timer = self.create_timer(1.0 / 100, self.publisher_callback)

        # create the action server
        self._controller_lock = threading.Lock()
        self.action_server = ActionServer(
            self,
            FrameTask,
            'frame_task',
            execute_callback=self.frame_task_callback,
            cancel_callback=self.cancel_callback
        )
        self.named_config_server = ActionServer(
            self,
            NamedConfig,
            'named_config',
            execute_callback=self.named_config_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Frame Task Server initialized')

    def _stamp_pose(self, pose):
        """Convert Pose to PoseStamped with current timestamp."""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'pelvis'
        pose_stamped.pose = pose
        return pose_stamped

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
            matrix_to_pose(self.controller.get_frame_transformation(name))
            for name in self.frame_names
        ]
        self.frame_poses_publisher.publish(frame_poses)

        # publish convenience left and right end-effector poses
        left_ee_pose = matrix_to_pose(self.controller.left_ee_transformation)
        right_ee_pose = matrix_to_pose(self.controller.right_ee_transformation)
        self.left_ee_pose_publisher.publish(self._stamp_pose(left_ee_pose))
        self.right_ee_pose_publisher.publish(self._stamp_pose(right_ee_pose))

        # publish left and right end-effector target poses if available
        try:
            # These properties exist in ArmController but frame_controller doesn't directly expose them
            # We need to access them through the ik_solver frame tasks
            left_ee_task = self.controller.ik_solver.frame_tasks.get('left_ee_task')
            right_ee_task = self.controller.ik_solver.frame_tasks.get('right_ee_task')
            if left_ee_task is not None:
                left_ee_target = matrix_to_pose(left_ee_task.transform_target_to_world.np)
                self.left_ee_target_publisher.publish(self._stamp_pose(left_ee_target))
            if right_ee_task is not None:
                right_ee_target = matrix_to_pose(right_ee_task.transform_target_to_world.np)
                self.right_ee_target_publisher.publish(self._stamp_pose(right_ee_target))
        except (AttributeError, KeyError):
            pass

    def frame_task_callback(self, goal_handle):
        with self._controller_lock:
            self.get_logger().info('Received goal')
            goal = goal_handle.request

            # clear existing frame tasks
            self.controller.clear_frame_tasks()
            self.frame_names = goal.frame_names
            self.frame_targets = goal.frame_targets

            # set frame tasks
            self.get_logger().info('Going to target frame poses')
            for frame_name, frame_target in zip(goal.frame_names, goal.frame_targets):
                task_name = f'{frame_name}_task'
                self.controller.add_frame_task(task_name, frame_name, pose_to_matrix(frame_target))

            # update ik solver with current state
            self.controller.update_ik_solver()

            # main loop
            start_time = time.time()
            duration = goal.duration.sec + goal.duration.nanosec * 1e-9
            timeout = duration if duration > 0.0 else self.timeout
            ik_converged = False
            steady_state_converged = False
            while time.time() - start_time < timeout:
                frame_start_time = time.time()
                if not ik_converged:
                    vel = self.controller.control_step_reduced()
                    vel_error = np.max(np.abs(vel))
                else:
                    steady_state_converged = self.controller.steady_state_step()

                # handle cancel event
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal cancelled')
                    goal_handle.canceled()
                    result = FrameTask.Result()
                    result.success = False
                    return result

                # compute errors
                errors_linear = []
                errors_angular = []
                for frame_name in self.frame_names:
                    error = self.controller.get_frame_task_error(f'{frame_name}_task')
                    errors_linear.append(np.linalg.norm(error[:3]))
                    errors_angular.append(np.linalg.norm(error[3:]))
                # send feedback
                feedback_msg = FrameTask.Feedback()
                feedback_msg.errors_linear = errors_linear
                feedback_msg.errors_angular = errors_angular
                goal_handle.publish_feedback(feedback_msg)

                if not ik_converged:
                    if vel_error < self.threshold_ik:
                        ik_converged = True
                        self.controller.init_steady_state()
                        self.get_logger().info(
                            'IK converged; entering steady-state hold'
                        )
                else:
                    steady_state_converged = steady_state_converged or (
                        len(errors_linear) > 0 and
                        len(errors_angular) > 0 and
                        max(errors_linear) < self.threshold_linear and
                        max(errors_angular) < self.threshold_angular
                    )
                    if steady_state_converged:
                        break

                time.sleep(max(
                    0.0,
                    self.controller.dt - (time.time() - frame_start_time)
                ))

            if not ik_converged:
                self.get_logger().warn('Timed out before IK convergence')
            elif not steady_state_converged:
                self.get_logger().warn('Timed out during steady-state hold')
            else:
                self.get_logger().info('Goal reached')

            # set result
            result = FrameTask.Result()
            result.success = True
            goal_handle.succeed()
            return result

    def named_config_callback(self, goal_handle):
        with self._controller_lock:
            self.get_logger().info('Received named config goal')
            goal = goal_handle.request

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
            ik_converged = False
            steady_state_converged = False
            while time.time() - start_time < timeout:
                frame_start_time = time.time()
                if not ik_converged:
                    vel = self.controller.goto_reduced_configuration(q_reduced)
                    vel_error = np.max(np.abs(vel))
                else:
                    steady_state_converged = self.controller.steady_state_step()

                # handle cancel event
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal cancelled')
                    goal_handle.canceled()
                    result = NamedConfig.Result()
                    result.success = False
                    return result

                # compute error
                joint_error = np.max(
                    np.abs(self.controller.reduced_configuration_error)
                )
                # send feedback
                feedback_msg = NamedConfig.Feedback()
                feedback_msg.joint_error = joint_error
                goal_handle.publish_feedback(feedback_msg)

                if not ik_converged:
                    if vel_error < self.threshold_ik:
                        ik_converged = True
                        self.controller.init_steady_state()
                        self.get_logger().info(
                            'IK converged; entering steady-state hold'
                        )
                else:
                    steady_state_converged = (
                        steady_state_converged or
                        joint_error < self.threshold_joint
                    )
                    if steady_state_converged:
                        self.get_logger().info('Named config reached')
                        break

                time.sleep(max(
                    0.0,
                    self.controller.dt - (time.time() - frame_start_time)
                ))

            if not ik_converged:
                self.get_logger().warn('Timed out before IK convergence')
            elif not steady_state_converged:
                self.get_logger().warn('Timed out during steady-state hold')
            else:
                self.get_logger().info('Goal reached')

            goal_handle.succeed()
            result = NamedConfig.Result()
            result.success = True
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal')
        return CancelResponse.ACCEPT

def main(args=None):
    parser = argparse.ArgumentParser(description='Frame task ROS2 action server')
    parser.add_argument('--config', type=str, default='debug.yaml', help='YAML file name under config/')
    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = FrameTaskServer(config_name=parsed_args.config)
    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.controller.shutdown()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
