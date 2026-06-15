import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import numpy as np

from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.dds_init import init_channel_factory_from_env
from h12_ros2_controller.utility.path_definition import URDF_HANDLESS_PIN_PATH
from h12_ros2_controller.utility.joint_definition import BODY_JOINTS

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # initialize robot model
        init_channel_factory_from_env()
        self.robot_model = RobotModel(URDF_HANDLESS_PIN_PATH, handless=True)
        self.robot_model.init_subscriber()
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

def main():
    rclpy.init()
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
