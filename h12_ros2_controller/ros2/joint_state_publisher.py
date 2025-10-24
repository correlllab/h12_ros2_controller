import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from h12_ros2_controller.core.robot_model import RobotModel
from h12_ros2_controller.utility.path_definition import URDF_PIN_PATH
from h12_ros2_controller.utility.joint_definition import ALL_JOINTS, BODY_JOINTS

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # initialize robot model
        ChannelFactoryInitialize()
        self.robot_model = RobotModel(URDF_PIN_PATH)
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
        # publish 0 for non-body joints
        joint_pos = np.zeros(len(ALL_JOINTS))
        joint_pos[[ALL_JOINTS.index(joint) for joint in BODY_JOINTS]] = self.robot_model.state['q'].tolist()
        msg.name = ALL_JOINTS
        msg.position = joint_pos.tolist()

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
