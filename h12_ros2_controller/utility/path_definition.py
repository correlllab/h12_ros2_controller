from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ASSETS_PATH = str(PROJECT_ROOT / 'submodules' / 'CL_Assets')
ROS_ASSETS_PATH = f'{ASSETS_PATH}/ros_assets'
CUROBO_ASSETS_PATH = f'{ASSETS_PATH}/curobo_assets'

URDF_MAGPIE_PATH = f'{ROS_ASSETS_PATH}/h1_2_magpie.urdf'
URDF_MAGPIE_SPHERE_PATH = f'{ROS_ASSETS_PATH}/h1_2_magpie_sphere.urdf'
SRDF_MAGPIE_SPHERE_PATH = f'{ROS_ASSETS_PATH}/h1_2_magpie_sphere_collision.srdf'
CUROBO_MAGPIE_CONFIG_PATH = f'{CUROBO_ASSETS_PATH}/h1_2_magpie_curobo.yml'

URDF_HANDLESS_PATH = f'{ROS_ASSETS_PATH}/h1_2_handless.urdf'
URDF_HANDLESS_SPHERE_PATH = f'{ROS_ASSETS_PATH}/h1_2_handless_sphere.urdf'
SRDF_HANDLESS_SPHERE_PATH = f'{ROS_ASSETS_PATH}/h1_2_handless_sphere_collision.srdf'
CUROBO_HANDLESS_CONFIG_PATH = f'{CUROBO_ASSETS_PATH}/h1_2_handless_curobo.yml'
