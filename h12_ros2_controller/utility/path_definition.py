from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ASSETS_PATH = str(PROJECT_ROOT / '../../../CL_Assets')
ROS_ASSETS_PATH = f'{ASSETS_PATH}/ros_assets'

URDF_MAGPIE_PATH = f'{ROS_ASSETS_PATH}/h1_2_magpie.urdf'
URDF_MAGPIE_SPHERE_PATH = f'{ROS_ASSETS_PATH}/h1_2_magpie_sphere.urdf'
SRDF_MAGPIE_SPHERE_PATH = f'{ROS_ASSETS_PATH}/h1_2_magpie_sphere_collision.srdf'

URDF_HANDLESS_PATH = f'{ROS_ASSETS_PATH}/h1_2_handless.urdf'
URDF_HANDLESS_SPHERE_PATH = f'{ROS_ASSETS_PATH}/h1_2_handless_sphere.urdf'
SRDF_HANDLESS_SPHERE_PATH = f'{ROS_ASSETS_PATH}/h1_2_handless_sphere_collision.srdf'


def _is_source_checkout():
    '''Verify the inferred root before using checkout resources'''
    return all((PROJECT_ROOT / name).is_file() for name in ('setup.py', 'package.xml'))


def _package_share(package_name):
    '''Look up an installed share only when checkout resources are unavailable'''
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package_name))


def get_config_dir():
    '''Choose checkout or installed controller configuration resources'''
    source = PROJECT_ROOT / 'config'
    if _is_source_checkout() and source.is_dir():
        return source
    return _package_share('h12_ros2_controller') / 'config'


def resolve_config_resource(name, config_dir=None):
    '''Resolve absolute paths first and config-prefixed names under the chosen root'''
    path = Path(name)
    if path.is_absolute():
        return path
    if path.parts and path.parts[0] == 'config':
        path = Path(*path.parts[1:])
    directory = get_config_dir() if config_dir is None else Path(config_dir)
    return directory / path


def resolve_model_paths(handless=False):
    '''Resolve a complete model family without mixing checkout and installed files'''
    model = 'h1_2_handless' if handless else 'h1_2_magpie'
    names = (f'{model}.urdf', f'{model}_sphere.urdf',
             f'{model}_sphere_collision.srdf')
    source = PROJECT_ROOT / 'submodules' / 'CL_Assets' / 'ros_assets'
    if _is_source_checkout() and all((source / name).is_file() for name in names):
        return tuple(str(source / name) for name in names)
    installed = _package_share('h12_ros2_model') / 'assets' / 'h1_2'
    paths = tuple(installed / name for name in names)
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError(f'Incomplete installed model family: {", ".join(missing)}')
    return tuple(str(path) for path in paths)
