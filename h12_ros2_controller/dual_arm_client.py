import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped

from pynput import keyboard
from scipy.spatial.transform import Rotation as R

from custom_ros_messages.action import DualArm
import cv2
class DualArmClient(Node):
    def __init__(self):
        super().__init__('dual_arm_client')
        self.action_client = ActionClient(
            self,
            DualArm,
            'move_dual_arm'
        )
        self.goal_handle = None

        # subscriber of left and right end-effector poses
        self.left_ee_pose_subscriber = self.create_subscription(
            PoseStamped,
            'left_ee_pose',
            self.subscribe_left_ee_pose,
            10
        )
        self.right_ee_pose_subscriber = self.create_subscription(
            PoseStamped,
            'right_ee_pose',
            self.subscribe_right_ee_pose,
            10
        )
        # subscriber of left and right end-effector target poses
        self.left_ee_target_subscriber = self.create_subscription(
            PoseStamped,
            'left_ee_target',
            self.subscribe_left_ee_target,
            10
        )
        self.right_ee_target_subscriber = self.create_subscription(
            PoseStamped,
            'right_ee_target',
            self.subscribe_right_ee_target,
            10
        )
        self.left_ee_pose = None
        self.right_ee_pose = None
        self.left_ee_target = None
        self.right_ee_target = None

    def subscribe_left_ee_pose(self, msg):
        self.left_ee_pose = msg.pose

    def subscribe_right_ee_pose(self, msg):
        self.right_ee_pose = msg.pose

    def subscribe_left_ee_target(self, msg):
        self.left_ee_target = msg.pose

    def subscribe_right_ee_target(self, msg):
        self.right_ee_target = msg.pose

    def send_goal(self, left_target: Pose, right_target: Pose):
        goal_msg = DualArm.Goal()
        goal_msg.left_target = left_target
        goal_msg.right_target = right_target

        self.action_client.wait_for_server()

        # send action
        self.get_logger().info('Sending goal...')
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            return

        # start a cancel listener thread
        self.get_logger().info('Goal accepted, waiting for result...')
        self.get_logger().info('Press BACKSPACE to cancel the goal')
        listener = keyboard.Listener(
            on_press=self._keyboard_cancel
        )
        listener.start()

        # wait till finish
        future_result = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future_result)
        result = future_result.result().result
        self.get_logger().info(f'Final result: success = {result.success}')
        # stop the cancel listener thread
        listener.stop()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Left Error: {feedback.left_error:.2f}; Right Error: {feedback.right_error:.2f}')

    def _keyboard_cancel(self, key):
        if key == keyboard.Key.backspace:
            if self.goal_handle is not None:
                self.get_logger().info('Cancelling goal...')
                self.goal_handle.cancel_goal_async()

def input_pose():
    while True:
        input_pose = input("Enter x y z roll pitch yaw (separated by space): ")
        parts = input_pose.strip().split()

        if len(parts) != 6:
            print("Invalid input. Please enter exactly 6 values.")
            continue
        try:
            values = [float(val) for val in parts]
            break
        except ValueError:
            print("Invalid input. Make sure all 6 values are numeric.")
            continue

    x, y, z, roll, pitch, yaw = values
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

def main(args=None):
    rclpy.init(args=args)
    node = DualArmClient()

    left_home = Pose()
    left_home.position.x = 0.3
    left_home.position.y = 0.2
    left_home.position.z = 0.1
    right_home = Pose()
    right_home.position.x = 0.3
    right_home.position.y = -0.2
    right_home.position.z = 0.1

    # Slider range and defaults
    slider_range = (-1000, 1000)  # Represents -1.0 to 1.0 with 0.001 resolution
    scale = 0.001  # Convert integer slider value to float

    # Pose parameter names
    params = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

    # Create a window
    cv2.namedWindow("Pose Sliders", cv2.WINDOW_NORMAL)

    # Helper function to create trackbars
    def create_pose_sliders(prefix):
        zero_slider_val = int((slider_range[1] - slider_range[0])/2)  # Default position for sliders
        for param in params:
            name = f"{prefix}_{param}"
            cv2.createTrackbar(name, "Pose Sliders", 0, slider_range[1] - slider_range[0], lambda x: None)
            if param == "x":
                cv2.setTrackbarPos(name, "Pose Sliders", int(0.3 / scale) + zero_slider_val)
            elif param == "y":
                cv2.setTrackbarPos(name, "Pose Sliders", int(0.2 / scale) + zero_slider_val)
            elif param == "z":
                cv2.setTrackbarPos(name, "Pose Sliders", int(0.1 / scale)  + zero_slider_val)
            if prefix == "right" and param == "y":
                cv2.setTrackbarPos(name, "Pose Sliders", int(-0.2 / scale)  + zero_slider_val)
    def pose_array_to_message(pose_array):
        pose = Pose()
        pose.position.x = pose_array[0]
        pose.position.y = pose_array[1]
        pose.position.z = pose_array[2]
        quat = R.from_euler('xyz', pose_array[3:], degrees=True).as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose
    # Create sliders for left and right poses
    create_pose_sliders("left")
    create_pose_sliders("right")
    try:
        while rclpy.ok():
            cv2.waitKey(0)
            L_pose_arr = [
                (cv2.getTrackbarPos(f"left_{param}", "Pose Sliders") + slider_range[0]) * scale
                for param in params
            ]
            R_pose_arr = [
                (cv2.getTrackbarPos(f"right_{param}", "Pose Sliders") + slider_range[0]) * scale
                for param in params
            ]
            left_pose = pose_array_to_message(L_pose_arr)

            right_pose = pose_array_to_message(R_pose_arr)
            node.send_goal(left_pose, right_pose)

    finally:
        node.destroy_node()
        rclpy.shutdown()
