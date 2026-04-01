from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# useful path
PACKAGE_NAME = 'h12_ros2_controller'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

MODEL_PACKAGE_PATH = get_package_share_directory('h12_ros2_model')
MODEL_ASSETS_PATH = f'{MODEL_PACKAGE_PATH}/assets'
MODEL_H12_PATH = f'{MODEL_ASSETS_PATH}/h1_2'
# pinnochio query meshes relatively, ROS requires package:// prefix
URDF_PIN_PATH = f'{MODEL_H12_PATH}/h1_2.urdf'
URDF_ROS_PATH = f'{MODEL_H12_PATH}/h1_2_ros.urdf'
URDF_SPHERE_PATH = f'{MODEL_H12_PATH}/h1_2_sphere.urdf'
SRDF_SPHERE_PATH = f'{MODEL_H12_PATH}/h1_2_sphere_collision.srdf'
# data path
HOME = str(Path.home())
LOG_PATH = f'{HOME}/.ros/log/{PACKAGE_NAME}'
CONFIG_DIR = Path(f'{PACKAGE_PATH}/config')
