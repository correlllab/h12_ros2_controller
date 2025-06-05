from ament_index_python.packages import get_package_share_directory

# useful path
PACKAGE_PATH = get_package_share_directory('h12_ros2_controller')
ASSET_PATH = f'{PACKAGE_PATH}/assets'
URDF_PATH = f'{ASSET_PATH}/h1_2/h1_2.urdf'
