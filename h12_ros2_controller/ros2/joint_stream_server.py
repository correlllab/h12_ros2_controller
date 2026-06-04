'''Joint setpoint streaming server.

Streams 14-dim joint setpoints (in ENABLED_JOINTS order) to the arms. A single
persistent control loop tracks the latest setpoint at the controller rate
(ctrl_hz), so callers can publish small successive targets at their own rate
(e.g. 10 Hz) and the loop interpolates/tracks between them.

Setpoints can arrive over either transport, into the same shared setpoint:
  - ROS:  std_msgs/Float64MultiArray on topic "joint_stream"
  - DDS:  unitree_go MotorCmds_ on channel "rt/joint_stream" (the i-th cmd's
          .q field is the target for ENABLED_JOINTS[i]); lets non-ROS callers
          publish straight over Unitree DDS.

The control loop uses goto_reduced_configuration, which keeps the IK
self-collision barrier and joint/velocity limits ON. This is one differential-IK
QP per tick toward the setpoint, so a caller whose own collision avoidance is
also a QP-IK composes cleanly with it rather than fighting a separate planner.
'''

import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

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

ROS_TOPIC = 'joint_stream'
DDS_TOPIC = 'rt/joint_stream'


class JointStreamServer(Node):
    def __init__(self, config_name='debug.yaml'):
        super().__init__('joint_stream_server')

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

        # setpoint protected by a lock; initialize to the current configuration
        # so the robot holds still until the first command arrives
        self._setpoint_lock = threading.Lock()
        self._q_target = np.asarray(self.controller.robot_model.state_reduced['q'], dtype=float)
        self._num_joints = self._q_target.shape[0]

        # sync the IK solver to the current robot state before streaming
        self.controller.update_ik_solver()

        # ROS setpoint input
        self.create_subscription(Float64MultiArray, ROS_TOPIC, self._ros_callback, 10)
        # straight DDS setpoint input (no ROS required on the publisher side)
        self._dds_subscriber = ChannelSubscriber(DDS_TOPIC, MotorCmds_)
        self._dds_subscriber.Init(self._dds_callback, 10)

        # persistent control loop tracking the latest setpoint
        self._loop_running = threading.Event()
        self._loop_running.set()
        self._loop_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._loop_thread.start()

        self.get_logger().info(
            f'Joint Stream Server initialized: tracking {self._num_joints} joints '
            f'(ROS topic "{ROS_TOPIC}", DDS channel "{DDS_TOPIC}")'
        )

    def _set_target(self, values, source):
        if len(values) != self._num_joints:
            self.get_logger().warn(
                f'Ignoring {source} setpoint with {len(values)} values '
                f'(expected {self._num_joints})'
            )
            return
        with self._setpoint_lock:
            self._q_target = np.asarray(values, dtype=float)

    def _ros_callback(self, msg):
        self._set_target(list(msg.data), 'ROS')

    def _dds_callback(self, msg: MotorCmds_):
        self._set_target([cmd.q for cmd in msg.cmds[:self._num_joints]], 'DDS')

    def _control_loop(self):
        self.get_logger().info('Joint stream control loop started')
        dt = self.controller.dt
        while self._loop_running.is_set() and rclpy.ok():
            start = time.time()
            with self._setpoint_lock:
                q_target = self._q_target.copy()
            # one differential-IK step toward the setpoint; keeps the IK
            # self-collision barrier and joint/velocity limits on
            self.controller.goto_reduced_configuration(q_target)
            time.sleep(max(0.0, dt - (time.time() - start)))
        self.get_logger().info('Joint stream control loop stopped')

    def shutdown(self):
        self._loop_running.clear()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)
        self.controller.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description='Joint setpoint streaming server (ROS + DDS)')
    parser.add_argument('--config', type=str, default='debug.yaml', help='YAML file name under config/')
    parsed_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = JointStreamServer(config_name=parsed_args.config)
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
