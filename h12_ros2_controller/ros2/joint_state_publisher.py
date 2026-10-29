import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.controller_config import (
    init_channel_factory,
    load_controller_config,
    DEFAULT_CONFIG_NAME,
)
from h12_ros2_controller.utility.path_definition_ros import URDF_HANDLESS_PIN_PATH
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

class JointStatePublisher(Node):
    def __init__(self, config):
        super().__init__('joint_state_publisher')

        # initialize robot model
        self.robot_model = RobotModel(URDF_HANDLESS_PIN_PATH, handless=True)
        self.robot_model.init_subscriber(low_state_topic=config['topics']['low_state'])
        self.get_logger().info('robot_model successfully initialized')

        # create publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0 / 100, self.publish_joint_states)

    def publish_joint_states(self):
        # update robot model
        self.robot_model.update_kinematics()
        # create JointState message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = BODY_JOINTS
        msg.position = self.robot_model.state['q'].tolist()

        self.publisher.publish(msg)

def main(args=None):
    parser = argparse.ArgumentParser(description='Robot joint state ROS2 publisher')
    parser.add_argument('--config', default=DEFAULT_CONFIG_NAME, help='Controller YAML profile')
    parsed_args, ros_args = parser.parse_known_args(args=args)
    config = load_controller_config(parsed_args.config)
    init_channel_factory(config)
    rclpy.init(args=ros_args)
    node = JointStatePublisher(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
