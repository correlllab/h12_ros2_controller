from ament_index_python.packages import get_package_share_directory

# useful path
PACKAGE_PATH = get_package_share_directory('h12_ros2_controller')

MODEL_PACKAGE_PATH = get_package_share_directory('h12_ros2_model')
ASSETS_PATH = f'{MODEL_PACKAGE_PATH}/assets'
# pinnochio query meshes relatively, ROS requires package:// prefix
URDF_PIN_PATH = f'{MODEL_PACKAGE_PATH}/h1_2/h1_2.urdf'
URDF_ROS_PATH = f'{MODEL_PACKAGE_PATH}/h1_2/h1_2_ros.urdf'
