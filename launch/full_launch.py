from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

import os
from h12_ros2_controller.utility.path_definition import PACKAGE_PATH, URDF_ROS_PATH

def generate_launch_description():
    package_name = 'h12_ros2_controller'

    with open(URDF_ROS_PATH, 'r') as urdf_file:
        robot_description = urdf_file.read()

    rviz_config_path = f'{PACKAGE_PATH}/rviz/default.rviz'

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package=package_name,
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package=package_name,
            executable='dual_arm_server',
            name='dual_arm_server',
            output='screen'
        ),
        Node(
            package=package_name,
            executable='dual_arm_client',
            name='dual_arm_client',
            output='screen'
        ),
        TimerAction(  # slight delay to ensure robot_description is set
            period=1.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
                    output='screen'
                )
            ]
        )
    ])
