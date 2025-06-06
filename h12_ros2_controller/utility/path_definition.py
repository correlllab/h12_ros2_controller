from ament_index_python.packages import get_package_share_directory

# useful path
PACKAGE_PATH = get_package_share_directory('h12_ros2_controller')
ASSET_PATH = f'{PACKAGE_PATH}/assets'
# pinnochio query meshes relatively, ROS requires package:// prefix
URDF_PIN_PATH = f'{ASSET_PATH}/h1_2/h1_2.urdf'
URDF_ROS_PATH = f'{ASSET_PATH}/h1_2/h1_2_ros.urdf'
