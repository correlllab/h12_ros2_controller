import rclpy
import argparse
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

import time
import json
import threading
import sys
from pathlib import Path

import numpy as np

from custom_ros_messages.msg import StringArray
from custom_ros_messages.action import FrameTask, NamedConfig

sys.path.append(str(Path(__file__).resolve().parents[2]))

from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.core.controller.upper_controller import (
    PathExecutionCancelled,
    StaleStateError,
)
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
from h12_ros2_controller.utility.controller_config import (
    init_channel_factory,
    load_controller_config,
)
from h12_ros2_controller.utility.path_definition_ros import (
    URDF_MAGPIE_PIN_PATH,
    URDF_MAGPIE_SPHERE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
    CONFIG_DIR
)
from h12_ros2_controller.ros2.utility import pose_to_matrix, matrix_to_pose

# Speed multiplier applied when a goal requests slow_mode. The controller
# applies it per execution mode: a direct move scales joint velocity, a planned
# move is planned at a proportionally lower grasp-frame speed (more waypoints,
# same control period) so the arm stays smooth instead of stepping.
SLOW_MODE_SCALE = 0.25

class FrameTaskServer(Node):
    def __init__(self,
                 timeout=None,
                 threshold_ik=None,
                 threshold_joint=None,
                 threshold_linear=None,
                 threshold_angular=None,
                 config_name='safety_split.yaml'):
        super().__init__('frame_task_server')
        self.config_name = config_name
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
        self._frame_publish_snapshot = ((), ())

        self.controller = FrameController(
            URDF_MAGPIE_PIN_PATH,
            URDF_MAGPIE_SPHERE_PATH,
            SRDF_MAGPIE_SPHERE_PATH,
            init=True,
            handless=False,
            visualize=False,
            config=config,
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
        self._action_callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            FrameTask,
            'frame_task',
            execute_callback=self.frame_task_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_callback_group,
        )
        self.named_config_server = ActionServer(
            self,
            NamedConfig,
            'named_config',
            execute_callback=self.named_config_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_callback_group,
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
        names, targets = self._frame_publish_snapshot
        measured_q = self.controller.robot_model.state['q']
        frame_names = StringArray()
        frame_names.data = list(names)
        self.frame_names_publisher.publish(frame_names)

        stamp = self.get_clock().now().to_msg()
        frame_targets = PoseArray()
        frame_targets.header.stamp = stamp
        frame_targets.header.frame_id = 'pelvis'
        frame_targets.poses = list(targets)
        self.frame_targets_publisher.publish(frame_targets)

        frame_poses = PoseArray()
        frame_poses.header.stamp = stamp
        frame_poses.header.frame_id = 'pelvis'
        frame_poses.poses = [
            matrix_to_pose(self.controller.get_frame_transformation(name, measured_q))
            for name in names
        ]
        self.frame_poses_publisher.publish(frame_poses)

        left_ee_pose = matrix_to_pose(self.controller.get_frame_transformation(
            self.controller.left_ee_name, measured_q,
        ))
        right_ee_pose = matrix_to_pose(self.controller.get_frame_transformation(
            self.controller.right_ee_name, measured_q,
        ))
        self.left_ee_pose_publisher.publish(self._stamp_pose(left_ee_pose))
        self.right_ee_pose_publisher.publish(self._stamp_pose(right_ee_pose))

        # target pairs are copied once while the action owns the controller lock
        targets_by_name = dict(zip(names, targets))
        left_target = targets_by_name.get(self.controller.left_ee_name)
        right_target = targets_by_name.get(self.controller.right_ee_name)
        if left_target is not None:
            self.left_ee_target_publisher.publish(self._stamp_pose(left_target))
        if right_target is not None:
            self.right_ee_target_publisher.publish(self._stamp_pose(right_target))

    def _abort_stale(self, goal_handle, result, err):
        '''Abort a goal because rt/lowstate went stale; the last command holds'''
        self.get_logger().error(f'Aborting goal: {err}')
        goal_handle.abort()
        result.success = False
        return result

    def frame_task_callback(self, goal_handle):
        try:
            return self._frame_task_callback(goal_handle)
        except StaleStateError as err:
            return self._abort_stale(goal_handle, FrameTask.Result(), err)
        except ValueError as err:
            self.get_logger().error(f'Aborting invalid frame goal: {err}')
            goal_handle.abort()
            result = FrameTask.Result()
            result.success = False
            return result

    def _frame_task_callback(self, goal_handle):
        with self._controller_lock:
            self.get_logger().info('Received goal')
            goal = goal_handle.request
            if not goal.frame_names or len(goal.frame_names) != len(goal.frame_targets):
                raise ValueError('frame_names and frame_targets must have matching nonempty lengths')
            if len(set(goal.frame_names)) != len(goal.frame_names):
                raise ValueError('Frame goal names must be distinct')
            frames = {frame.name for frame in self.controller.robot_model.model_body.frames}
            if any(name not in frames for name in goal.frame_names):
                raise ValueError('Frame goal contains an unknown robot frame')
            for target in goal.frame_targets:
                position = target.position
                orientation = target.orientation
                values = np.array([
                    position.x, position.y, position.z,
                    orientation.x, orientation.y, orientation.z, orientation.w,
                ], dtype=float)
                if (not np.all(np.isfinite(values)) or
                        np.linalg.norm(values[3:]) < 1e-8):
                    raise ValueError('Frame goal contains a nonfinite pose or invalid quaternion')
            # relative targets are resolved against the measured pose
            self.controller.check_state_fresh()
            measured_q = self.controller.robot_model.state['q']

            # resolve targets to absolute pelvis-frame poses before the
            # current tasks are cleared
            target_matrices = [
                self._resolve_target(name, pose_to_matrix(target), goal.relative, measured_q)
                for name, target in zip(goal.frame_names, goal.frame_targets)
            ]

            # clear existing frame tasks
            self.controller.clear_frame_tasks()
            self.frame_names = goal.frame_names
            self.frame_targets = [matrix_to_pose(m) for m in target_matrices]

            # set frame tasks
            self.get_logger().info(
                'Going to target frame poses'
                + (' (relative)' if goal.relative else '')
            )
            for frame_name, target_matrix in zip(goal.frame_names, target_matrices):
                task_name = f'{frame_name}_task'
                self.controller.add_frame_task(task_name, frame_name, target_matrix)

            # update ik solver with current state
            self.controller.update_ik_solver()
            self._frame_publish_snapshot = (
                tuple(self.frame_names), tuple(self.frame_targets),
            )

            duration = goal.duration.sec + goal.duration.nanosec * 1e-9
            timeout = duration if duration > 0.0 else self.timeout
            plan = goal.plan
            # slow the arm for this goal only; restore full speed afterwards
            self.controller.speed_scale = (
                SLOW_MODE_SCALE if goal.slow_mode else 1.0
            )
            if goal.slow_mode:
                self.get_logger().info(
                    f'Slow mode enabled (speed_scale={SLOW_MODE_SCALE})'
                )
            try:
                if plan:
                    return self._plan_frame_task_callback(goal_handle, timeout)
                else:
                    return self._direct_frame_task_callback(goal_handle, timeout)
            finally:
                self.controller.speed_scale = 1.0

    def _resolve_target(self, frame_name, target, relative, measured_q=None):
        '''
        Return the absolute pelvis-frame target for one frame.

        A relative target's translation is added to the frame's current
        position and its rotation is applied in pelvis-frame axes about the
        frame's current origin.
        '''
        if not relative:
            return target
        if measured_q is None:
            current = self.controller.get_frame_transformation(frame_name)
        else:
            current = self.controller.get_frame_transformation(frame_name, measured_q)
        absolute = np.eye(4)
        absolute[:3, :3] = target[:3, :3] @ current[:3, :3]
        absolute[:3, 3] = current[:3, 3] + target[:3, 3]
        return absolute

    def _direct_frame_task_callback(self, goal_handle, timeout):
        # main loop
        start_time = time.time()
        ik_converged = False
        steady_state_converged = False
        blocked = None
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FrameTask.Result()
                result.success = False
                return result
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
            errors_linear, errors_angular = self._frame_task_errors()
            # send feedback
            self._publish_frame_task_feedback(
                goal_handle,
                errors_linear,
                errors_angular,
            )

            if not ik_converged:
                if vel_error < self.threshold_ik:
                    ik_converged = True
                    self.controller.init_steady_state()
                    self.get_logger().info(
                        'IK converged; entering steady-state hold'
                    )
            else:
                blocked = self._limit_block_reason()
                if blocked is not None:
                    break
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
        else:
            self._log_hold_outcome(steady_state_converged, blocked)

        # set result
        success = ik_converged and steady_state_converged
        result = FrameTask.Result()
        result.success = bool(success)
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _visualize_planned_path(self, path):
        '''
        Draw the planned path in Meshcat, one triad per waypoint.

        Purely diagnostic: a dead or unreachable Meshcat server must never
        abort a motion goal, so every failure here is swallowed with a warning.
        '''
        if not self.controller.visualize:
            return
        try:
            self.controller.visualize_path(path)
        except Exception as err:  # noqa: BLE001 - visualization is best-effort
            self.get_logger().warn(f'Path visualization failed: {err}')

    def _plan_frame_task_callback(self, goal_handle, timeout):
        self.get_logger().info('Planning path to target frame poses')
        try:
            # solve ik, plan a reduced-joint path, then execute it
            path = self.controller.plan_to_ik_target()
            self._visualize_planned_path(path)
            self.get_logger().info(
                f'Executing planned path with {len(path)} waypoints'
            )
            if goal_handle.is_cancel_requested:
                raise PathExecutionCancelled('Goal canceled before path execution')
            execute_start_time = time.time()
            self.controller.execute_path(
                path, cancel_check=lambda: goal_handle.is_cancel_requested,
            )
            self.get_logger().info(
                f'Path execution finished in '
                f'{time.time() - execute_start_time:.2f}s; '
                f'holding steady state (timeout {timeout:.1f}s)'
            )
        except PathExecutionCancelled:
            self.get_logger().info('Goal cancelled')
            goal_handle.canceled()
            result = FrameTask.Result()
            result.success = False
            return result
        except Exception as err:
            self.get_logger().warn(f'Planning or execution failed: {err}')
            context = getattr(self.controller, 'last_planning_context', None)
            if isinstance(context, dict):
                case = {**context, 'config': self.config_name}
                self.get_logger().warn(f'Planning replay case: {json.dumps(case)}')
            goal_handle.abort()
            result = FrameTask.Result()
            result.success = False
            return result

        # hold the final target using the existing steady-state controller
        self.controller.init_steady_state()
        start_time = time.time()
        steady_state_converged = False
        blocked = None
        while time.time() - start_time < timeout:
            frame_start_time = time.time()

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                result = FrameTask.Result()
                result.success = False
                return result

            self.controller.steady_state_step()
            # publish feedback while waiting for final pose convergence
            errors_linear, errors_angular = self._frame_task_errors()
            self._publish_frame_task_feedback(
                goal_handle,
                errors_linear,
                errors_angular,
            )

            blocked = self._limit_block_reason()
            if blocked is not None:
                break
            steady_state_converged = (
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

        self._log_hold_outcome(steady_state_converged, blocked)

        if steady_state_converged:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = FrameTask.Result()
        result.success = bool(steady_state_converged)
        return result

    def _limit_block_reason(self):
        '''The joints pinning the steady-state hold at a position limit, or None'''
        limited = self.controller.steady_state_limited_joints
        if not limited:
            return None
        return ', '.join(
            f'{name} q={q:+.3f} (limit {limit:+.3f})' for name, q, limit in limited
        )

    def _log_hold_outcome(self, converged, blocked):
        if blocked is not None:
            self.get_logger().warn(f'Hold blocked at a joint position limit: {blocked}')
        elif converged:
            self.get_logger().info('Goal reached')
        else:
            self.get_logger().warn('Timed out during steady-state hold')

    def _frame_task_errors(self):
        errors_linear = []
        errors_angular = []
        for frame_name in self.frame_names:
            error = self.controller.get_frame_task_error(f'{frame_name}_task')
            errors_linear.append(float(np.linalg.norm(error[:3])))
            errors_angular.append(float(np.linalg.norm(error[3:])))
        return errors_linear, errors_angular

    def _publish_frame_task_feedback(self, goal_handle, errors_linear,
                                     errors_angular):
        feedback_msg = FrameTask.Feedback()
        feedback_msg.errors_linear = errors_linear
        feedback_msg.errors_angular = errors_angular
        goal_handle.publish_feedback(feedback_msg)

    def _named_config_joint_error(self):
        return float(np.max(
            np.abs(self.controller.reduced_configuration_error)
        ))

    def _publish_named_config_feedback(self, goal_handle, joint_error):
        feedback_msg = NamedConfig.Feedback()
        feedback_msg.joint_error = joint_error
        goal_handle.publish_feedback(feedback_msg)

    def named_config_callback(self, goal_handle):
        try:
            return self._named_config_callback(goal_handle)
        except StaleStateError as err:
            return self._abort_stale(goal_handle, NamedConfig.Result(), err)

    def _named_config_callback(self, goal_handle):
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

            duration = goal.duration.sec + goal.duration.nanosec * 1e-9
            timeout = duration if duration > 0.0 else self.timeout
            plan = goal.plan
            # slow the arm for this goal only; restore full speed afterwards
            self.controller.speed_scale = (
                SLOW_MODE_SCALE if goal.slow_mode else 1.0
            )
            if goal.slow_mode:
                self.get_logger().info(
                    f'Slow mode enabled (speed_scale={SLOW_MODE_SCALE})'
                )
            try:
                if plan:
                    return self._plan_named_config_callback(
                        goal_handle,
                        q_reduced,
                        timeout,
                    )
                else:
                    return self._direct_named_config_callback(
                        goal_handle,
                        q_reduced,
                        timeout,
                    )
            finally:
                self.controller.speed_scale = 1.0

    def _direct_named_config_callback(self, goal_handle, q_reduced, timeout):
        # main loop
        start_time = time.time()
        ik_converged = False
        steady_state_converged = False
        blocked = None
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NamedConfig.Result()
                result.success = False
                return result
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
            joint_error = self._named_config_joint_error()
            # send feedback
            self._publish_named_config_feedback(goal_handle, joint_error)

            if not ik_converged:
                if vel_error < self.threshold_ik:
                    ik_converged = True
                    self.controller.init_steady_state()
                    self.get_logger().info(
                        'IK converged; entering steady-state hold'
                    )
            else:
                blocked = self._limit_block_reason()
                if blocked is not None:
                    break
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
        else:
            self._log_hold_outcome(steady_state_converged, blocked)

        result = NamedConfig.Result()
        result.success = bool(ik_converged and steady_state_converged)
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _plan_named_config_callback(self, goal_handle, q_reduced, timeout):
        self.get_logger().info('Planning path to named configuration')
        try:
            # plan directly in reduced joint space for named configs
            path = self.controller.plan_to_configuration(q_reduced)
            self._visualize_planned_path(path)
            self.get_logger().info(
                f'Executing planned path with {len(path)} waypoints'
            )
            if goal_handle.is_cancel_requested:
                raise PathExecutionCancelled('Goal canceled before path execution')
            execute_start_time = time.time()
            self.controller.execute_path(
                path, cancel_check=lambda: goal_handle.is_cancel_requested,
            )
            self.get_logger().info(
                f'Path execution finished in '
                f'{time.time() - execute_start_time:.2f}s; '
                f'holding steady state (timeout {timeout:.1f}s)'
            )
        except PathExecutionCancelled:
            self.get_logger().info('Goal cancelled')
            goal_handle.canceled()
            result = NamedConfig.Result()
            result.success = False
            return result
        except Exception as err:
            self.get_logger().warn(f'Planning or execution failed: {err}')
            goal_handle.abort()
            result = NamedConfig.Result()
            result.success = False
            return result

        # match feedback/error target to the planned joint goal
        self.controller.set_reduced_configuration_target(q_reduced)

        # hold the final joint target using steady-state control
        self.controller.init_steady_state()
        start_time = time.time()
        steady_state_converged = False
        blocked = None
        while time.time() - start_time < timeout:
            frame_start_time = time.time()

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                result = NamedConfig.Result()
                result.success = False
                return result

            self.controller.steady_state_step()
            joint_error = float(np.max(np.abs(
                self.controller.robot_model.state_reduced['q'] - q_reduced
            )))

            # publish joint-space feedback during the hold phase
            self._publish_named_config_feedback(goal_handle, joint_error)

            blocked = self._limit_block_reason()
            if blocked is not None:
                break
            steady_state_converged = joint_error < self.threshold_joint
            if steady_state_converged:
                break

            time.sleep(max(
                0.0,
                self.controller.dt - (time.time() - frame_start_time)
            ))

        self._log_hold_outcome(steady_state_converged, blocked)

        if steady_state_converged:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = NamedConfig.Result()
        result.success = bool(steady_state_converged)
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal')
        return CancelResponse.ACCEPT


def main(args=None):
    parser = argparse.ArgumentParser(description='Frame task ROS2 action server')
    parser.add_argument('--config', type=str, default='safety_split.yaml', help='YAML file name under config/')
    parsed_args, ros_args = parser.parse_known_args(args=args)

    config = load_controller_config(parsed_args.config, config_dir=CONFIG_DIR)
    init_channel_factory(config)
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
