'''Streaming setpoint server (joint-space or frame-space).

Runs a single persistent control loop at the controller rate (ctrl_hz) that
tracks the latest setpoint, so callers can publish small successive targets at
their own rate (e.g. 10 Hz) and the loop tracks between them. The loop operates
in one of two modes, selected by whichever setpoint type was last received:

  - JOINT mode: track a 14-dim joint configuration (ENABLED_JOINTS order) via
    goto_reduced_configuration.
        ROS:  std_msgs/Float64MultiArray on "joint_stream"
        DDS:  unitree_go/MotorCmds_ on "rt/joint_stream" (i-th .q -> joint i)
  - FRAME mode: track one or more end-effector frame poses via frame tasks +
    control_step_reduced.
        ROS:  geometry_msgs/PoseStamped on "frame_stream"
              (header.frame_id names the frame, e.g. "left_ee" / "right_ee")

Both modes solve through the same PINK reduced-model QP, so the self-collision
barrier and joint/velocity limits stay ON in either case. A QP-IK collision
avoidance policy composes cleanly: it shapes the target, this server guarantees
self-collision and limit safety on the executed motion.

Frame streaming is ROS-only: Unitree's DDS IDL has no SE(3)/pose type, so there
is no clean straight-DDS carrier for poses. Joint streaming supports both
transports.
'''

import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_

from h12_ros2_controller.core.controller.frame_controller import FrameController
from h12_ros2_controller.utility.controller_config import (
    load_controller_config,
    initialize_channel_factory,
)
from h12_ros2_controller.utility.path_definition import (
    URDF_HANDLESS_PIN_PATH,
    URDF_HANDLESS_SPHERE_PATH,
    SRDF_HANDLESS_SPHERE_PATH,
    CONFIG_DIR,
)
from h12_ros2_controller.ros2.utility import pose_to_matrix

ROS_JOINT_TOPIC = 'joint_stream'
DDS_JOINT_TOPIC = 'rt/joint_stream'
ROS_FRAME_TOPIC = 'frame_stream'

MODE_JOINT = 'joint'
MODE_FRAME = 'frame'


class StreamServer(Node):
    def __init__(self, config_name='debug.yaml'):
        super().__init__('stream_server')

        config = load_controller_config(config_name, config_dir=CONFIG_DIR)
        initialize_channel_factory(config)
        self.controller = FrameController(
            URDF_HANDLESS_PIN_PATH,
            URDF_HANDLESS_SPHERE_PATH,
            SRDF_HANDLESS_SPHERE_PATH,
            handless=True,
            visualize=False,
            config=config,
        )

        # one lock serializes the control loop with the setpoint callbacks, so
        # frame-task mutations never race the QP solve
        self._lock = threading.Lock()
        # start in JOINT mode holding the current configuration
        self._mode = MODE_JOINT
        self._q_target = np.asarray(self.controller.robot_model.state_reduced['q'], dtype=float)
        self._num_joints = self._q_target.shape[0]
        self.controller.update_ik_solver()

        # joint setpoint inputs (ROS + straight DDS)
        self.create_subscription(Float64MultiArray, ROS_JOINT_TOPIC, self._ros_joint_callback, 10)
        self._dds_subscriber = ChannelSubscriber(DDS_JOINT_TOPIC, MotorCmds_)
        self._dds_subscriber.Init(self._dds_joint_callback, 10)
        # frame setpoint input (ROS only)
        self.create_subscription(PoseStamped, ROS_FRAME_TOPIC, self._ros_frame_callback, 10)

        # persistent control loop
        self._loop_running = threading.Event()
        self._loop_running.set()
        self._loop_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._loop_thread.start()

        self.get_logger().info(
            f'Stream Server initialized: {self._num_joints} joints | '
            f'joint ROS "{ROS_JOINT_TOPIC}" / DDS "{DDS_JOINT_TOPIC}" | '
            f'frame ROS "{ROS_FRAME_TOPIC}"'
        )

    # ------------------------------------------------------------------ joint
    def _set_joint_target(self, values, source):
        if len(values) != self._num_joints:
            self.get_logger().warn(
                f'Ignoring {source} joint setpoint with {len(values)} values '
                f'(expected {self._num_joints})'
            )
            return
        with self._lock:
            self._q_target = np.asarray(values, dtype=float)
            if self._mode != MODE_JOINT:
                # leaving frame mode: drop frame tasks and resync the IK solver
                self.controller.clear_frame_tasks()
                self.controller.update_ik_solver()
                self._mode = MODE_JOINT
                self.get_logger().info('Switched to JOINT mode')

    def _ros_joint_callback(self, msg):
        self._set_joint_target(list(msg.data), 'ROS')

    def _dds_joint_callback(self, msg: MotorCmds_):
        self._set_joint_target([cmd.q for cmd in msg.cmds[:self._num_joints]], 'DDS')

    # ------------------------------------------------------------------ frame
    def _ros_frame_callback(self, msg: PoseStamped):
        frame = msg.header.frame_id
        if not frame:
            self.get_logger().warn('Ignoring frame setpoint with empty header.frame_id')
            return
        task_name = f'{frame}_task'
        target = pose_to_matrix(msg.pose)
        with self._lock:
            if self._mode != MODE_FRAME:
                # entering frame mode: resync the IK solver to current state
                self.controller.update_ik_solver()
                self._mode = MODE_FRAME
                self.get_logger().info('Switched to FRAME mode')
            if task_name in self.controller.ik_solver.frame_tasks:
                self.controller.set_frame_task_transformation(task_name, target)
            else:
                self.controller.add_frame_task(task_name, frame, target)

    # ------------------------------------------------------------------- loop
    def _control_loop(self):
        self.get_logger().info('Stream control loop started')
        dt = self.controller.dt
        while self._loop_running.is_set() and rclpy.ok():
            start = time.time()
            with self._lock:
                if self._mode == MODE_JOINT:
                    self.controller.goto_reduced_configuration(self._q_target.copy())
                else:
                    self.controller.control_step_reduced()
            time.sleep(max(0.0, dt - (time.time() - start)))
        self.get_logger().info('Stream control loop stopped')

    def shutdown(self):
        self._loop_running.clear()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)
        self.controller.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='Streaming setpoint server (joint or frame; ROS + DDS)')
    parser.add_argument('--config', type=str, default='debug.yaml', help='YAML file name under config/')
    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = StreamServer(config_name=parsed_args.config)
    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
