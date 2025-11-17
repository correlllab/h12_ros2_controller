import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from geometry_msgs.msg import Pose, PoseArray

import time
import asyncio
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from custom_ros_messages.msg import StringArray
from custom_ros_messages.action import FrameTask, NamedConfig
from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.path_definition import URDF_PIN_PATH, URDF_SPHERE_PATH, SRDF_SPHERE_PATH
from h12_ros2_controller.ros2.utility import pose_to_matrix, matrix_to_pose

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
                                          visualize=False,
                                          sport_mode=False)

        # publisher publishing frame names and target poses
        self.frame_names_publisher = self.create_publisher(StringArray, 'frame_names', 10)
        self.frame_targets_publisher = self.create_publisher(PoseArray, 'frame_targets', 10)
        self.frame_poses_publisher = self.create_publisher(PoseArray, 'frame_poses', 10)
        self.publisher_timer = self.create_timer(1.0 / 100, self.publisher_callback)

        # create the action server
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

    async def frame_task_callback(self, goal_handle):
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
        while time.time() - start_time < timeout:
            frame_start_time = time.time()
            # control one step
            self.controller.control_step_reduced()

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

            # check if the goal is reached
            if len(errors_linear) > 0 and len(errors_angular) > 0:
                if (max(errors_linear) < self.threshold_linear and
                    max(errors_angular) < self.threshold_angular):
                    self.get_logger().info('Goal reached')
                    break

            time.sleep(max(0.0, self.controller.dt - (time.time() - frame_start_time)))
            await asyncio.sleep(0)

        # set result
        result = FrameTask.Result()
        result.success = True
        goal_handle.succeed()
        return result

    async def named_config_callback(self, goal_handle):
        self.get_logger().info('Received named config goal')
        goal = goal_handle.request

        # check if the named config exists
        config_name = goal.config_name
        if config_name not in NAMED_CONFIGS:
            self.get_logger().warn(f'Named config "{config_name}" not found')
            goal_handle.canceled()
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
            await asyncio.sleep(0)

        goal_handle.succeed()
        result = NamedConfig.Result()
        result.success = True
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal')
        return CancelResponse.ACCEPT

def main(args=None):
    rclpy.init(args=args)
    node = FrameTaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.controller.shutdown()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
