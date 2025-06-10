import rclpy
from rclpy.node import Node

class DualArmClient(Node):
    def __init__(self):
        super().__init__('dual_arm_client')


def main(args=None):
    rclpy.init(args=args)
    node = DualArmClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
