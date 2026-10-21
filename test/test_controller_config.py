from textwrap import dedent

import pytest

from h12_ros2_controller.utility.joint_definition import UPPER_BODY_INDEX
from h12_ros2_controller.utility.controller_config import (
    DEFAULT_CONFIG_DIR,
    load_controller_config,
)


def _gain_list(value):
    values = ', '.join([str(value)] * len(UPPER_BODY_INDEX))
    return f'[{values}]'


def _write_config(tmp_path, extra_section):
    config_path = tmp_path / 'test.yaml'
    config_path.write_text(
        dedent(
            f'''
            mode: debug

            gains:
              kp: {_gain_list(1.0)}
              kd: {_gain_list(0.1)}
              ki: {_gain_list(0.0)}

            {extra_section}
            '''
        ),
        encoding='utf-8',
    )
    return config_path


def test_load_controller_config_preserves_empty_zmp_mapping(tmp_path):
    _write_config(tmp_path, 'zmp: {}')

    config = load_controller_config('test.yaml', config_dir=tmp_path)

    assert config['zmp'] == {}


@pytest.mark.parametrize('zmp_section', ('zmp: invalid',))
def test_load_controller_config_rejects_non_mapping_zmp(tmp_path, zmp_section):
    _write_config(tmp_path, zmp_section)

    with pytest.raises(
            ValueError, match='zmp must be a mapping when provided'):
        load_controller_config('test.yaml', config_dir=tmp_path)


def test_checked_in_balance_profile_keeps_empty_zmp_mapping():
    config = load_controller_config(
        'balance_safety_split.yaml', config_dir=DEFAULT_CONFIG_DIR)

    assert config['topics']['low_cmd'] == 'rt/safety/lowcmd_upper_in'
    assert config['zmp'] == {}


def test_point_cloud_profile_loads_with_safety_topic():
    config = load_controller_config(
        'pcd_debug.yaml', config_dir=DEFAULT_CONFIG_DIR)

    assert config['topics']['low_cmd'] == 'rt/safety/lowcmd_in'
    assert config['planner']['point_cloud_path'].endswith(
        'data/planner/pointcloud.npy'
    )
