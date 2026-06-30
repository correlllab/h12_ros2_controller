from launch import LaunchDescription
from launch_ros.actions import Node

from h12_ros2_controller.utility.path_definition_ros import URDF_MAGPIE_ROS_PATH

def generate_launch_description():
    package_name = 'h12_ros2_controller'

    with open(URDF_MAGPIE_ROS_PATH, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'publish_frequency': 50.0}],
            output='screen'
        ),
        Node(
            package=package_name,
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ])
