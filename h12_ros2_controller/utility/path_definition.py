from ament_index_python.packages import get_package_share_directory

# useful path
PACKAGE_PATH = get_package_share_directory('h12_ros2_controller')

MODEL_PACKAGE_PATH = get_package_share_directory('h12_ros2_model')
MODEL_ASSETS_PATH = f'{MODEL_PACKAGE_PATH}/assets'
MODEL_H12_PATH = f'{MODEL_ASSETS_PATH}/h1_2'
# pinnochio query meshes relatively, ROS requires package:// prefix
URDF_PIN_PATH = f'{MODEL_H12_PATH}/h1_2.urdf'
URDF_ROS_PATH = f'{MODEL_H12_PATH}/h1_2_ros.urdf'
URDF_SPHERE_PATH = f'{MODEL_H12_PATH}/h1_2_sphere.urdf'
SRDF_SPHERE_PATH = f'{MODEL_H12_PATH}/h1_2_sphere_collision.srdf'
