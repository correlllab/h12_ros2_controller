import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from h12_ros2_controller.core.controller.hand_controller import HandController

'''
Finger angles definition
0 for close, 1 for open
#index 0:pinky
#index 1:ring
#index 2:middle
#index 3:index
#index 4:thumb
#index 5:thumb angle
'''

class HandControllerNode(Node):
    def __init__(self):
        super().__init__('hand_controller_node')
        ChannelFactoryInitialize()
        self.hand_controller = HandController()

        # subscribers for each hand cmd
        self.right_cmd_subscriber = self.create_subscription(
            Float64MultiArray,
            'right_hand_cmd',
            self.right_callback,
            10)
        self.left_cmd_subscriber = self.create_subscription(
            Float64MultiArray,
            'left_hand_cmd',
            self.left_callback,
            10)

        # publishers for each hand state
        self.right_state_publisher = self.create_publisher(
            Float64MultiArray,
            'right_hand_state',
            10)
        self.left_state_publisher = self.create_publisher(
            Float64MultiArray,
            'left_hand_state',
            10)
        self.create_timer(1.0 / 100.0, self.publish_states)

    def right_callback(self, msg):
        # check data validity
        if len(msg.data) != 6:
            self.get_logger().error(f'Right hand: expected 6 elements, got {len(msg.data)}')
            return
        if not all(0.0 <= val <= 1.0 for val in msg.data):
            self.get_logger().error('Right hand: values must be between 0.0 and 1.0')
            return
        right_arr = msg.data
        self.get_logger().info(f'Right hand cmd: {right_arr}')
        self.hand_controller.ctrl_right(right_arr)

    def left_callback(self, msg):
        # check data validity
        if len(msg.data) != 6:
            self.get_logger().error(f'Left hand: expected 6 elements, got {len(msg.data)}')
            return
        if not all(0.0 <= val <= 1.0 for val in msg.data):
            self.get_logger().error('Left hand: values must be between 0.0 and 1.0')
            return
        left_arr = msg.data
        self.get_logger().info(f'Left hand cmd: {left_arr}')
        self.hand_controller.ctrl_left(left_arr)

    def publish_states(self):
        # publish right hand state
        right_state = Float64MultiArray()
        right_state.data = self.hand_controller.q_right.tolist()
        self.right_state_publisher.publish(right_state)

        # publish left hand state
        left_state = Float64MultiArray()
        left_state.data = self.hand_controller.q_left.tolist()
        self.left_state_publisher.publish(left_state)

def main(args=None):
    rclpy.init(args=args)
    node = HandControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
