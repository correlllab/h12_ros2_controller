from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from h12_ros2_controller.utility.path_definition import URDF_HANDLESS_ROS_PATH


def _config_suffix(config_name: str) -> str:
    """Return full or split suffix from config name"""
    stem = config_name.rsplit("/", 1)[-1]
    stem = stem.rsplit(".", 1)[0]
    if stem.endswith("_full"):
        return "full"
    if stem.endswith("_split"):
        return "split"
    raise ValueError(f'Config "{config_name}" must end with _full or _split')


def _validate_config_compatibility(context):
    config = LaunchConfiguration("config").perform(context)
    safety_config = LaunchConfiguration("safety_config").perform(context)
    config_suffix = _config_suffix(config)
    safety_suffix = _config_suffix(safety_config)
    if config_suffix != safety_suffix:
        raise ValueError(
            f'controller config "{config}" and safety config '
            f'"{safety_config}" are incompatible: suffixes must match'
        )
    return []


def generate_launch_description():
    package_name = "h12_ros2_controller"
    config = LaunchConfiguration("config")
    safety_config = LaunchConfiguration("safety_config")

    with open(URDF_HANDLESS_ROS_PATH, "r") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="safety_full.yaml",
                description="Controller config file name, must end with _full or _split",
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value="default_safety_full",
                description="Safety layer config name, must end with _full or _split",
            ),
            OpaqueFunction(function=_validate_config_compatibility),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package=package_name,
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="estop",
                executable="estop_node",
                name="estop_node",
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="h12_safety_layer",
                        executable="safety_node",
                        name="safety_node",
                        arguments=["--config", safety_config],
                        output="screen",
                    ),
                ],
            ),
            # TimerAction(
            #     period=2.0,
            #     actions=[
            #         Node(
            #             package=package_name,
            #             executable='dual_arm_server',
            #             name='dual_arm_server',
            #             arguments=['--config', config],
            #             output='screen'
            #         ),
            #     ]
            # ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package=package_name,
                        executable="frame_task_server",
                        name="frame_task_server",
                        arguments=["--config", config],
                        output="screen",
                    ),
                ],
            ),
            Node(
                package=package_name,
                executable="hand_controller_node",
                name="hand_controller_node",
                output="screen",
            ),
        ]
    )
