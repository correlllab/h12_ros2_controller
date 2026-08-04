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


def test_load_controller_config_preserves_counter_balance_mapping(tmp_path):
    _write_config(
        tmp_path,
        'counter_balance:\n'
        '              maxiter: 7\n'
        '              w_com: 2.0\n'
        '              support_geometry:\n'
        '                front: 0.174',
    )

    config = load_controller_config('test.yaml', config_dir=tmp_path)

    assert config['counter_balance'] == {
        'maxiter': 7,
        'w_com': 2.0,
        'support_geometry': {'front': 0.174},
    }


def test_load_controller_config_rejects_non_mapping_counter_balance(tmp_path):
    _write_config(tmp_path, 'counter_balance: invalid')

    with pytest.raises(
            ValueError, match='counter_balance must be a mapping'):
        load_controller_config('test.yaml', config_dir=tmp_path)


@pytest.mark.parametrize('zmp_section', ('zmp: invalid',))
def test_load_controller_config_rejects_non_mapping_zmp(tmp_path, zmp_section):
    _write_config(tmp_path, zmp_section)

    with pytest.raises(
            ValueError, match='zmp must be a mapping when provided'):
        load_controller_config('test.yaml', config_dir=tmp_path)


@pytest.mark.parametrize(
    ('name', 'topic'),
    (
        ('balance_debug.yaml', 'rt/lowcmd'),
        ('balance_safety_full.yaml', 'rt/safety/lowcmd_in'),
        ('balance_safety_split.yaml', 'rt/safety/lowcmd_upper_in'),
        ('balance_sport.yaml', 'rt/arm_sdk'),
    ),
)
def test_checked_in_profiles_keep_empty_zmp_mapping(name, topic):
    config = load_controller_config(name, config_dir=DEFAULT_CONFIG_DIR)

    assert config['topics']['low_cmd'] == topic
    assert config['zmp'] == {}
