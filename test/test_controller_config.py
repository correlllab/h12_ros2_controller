from textwrap import dedent

import pytest

from h12_ros2_controller.utility.joint_definition import UPPER_BODY_INDEX
from h12_ros2_controller.utility.controller_config import (
    _resolve_config_path,
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
        'balance_safety_split.yaml')

    assert config['topics']['low_cmd'] == 'rt/safety/lowcmd_upper_in'
    assert config['zmp'] == {}


@pytest.mark.parametrize('profile, topic', (
    ('sim_safety_split.yaml', 'rt/safety/lowcmd_upper_in'),
    ('safety_split.yaml', 'rt/safety/lowcmd_upper_in'),
    ('safety_full.yaml', 'rt/safety/lowcmd_in'),
))
def test_supported_default_profiles_resolve_to_safety_inputs(profile, topic):
    config = load_controller_config(profile)

    assert config['topics']['low_cmd'] == topic


@pytest.mark.parametrize('controller', (
    'execution_start_tolerance: .nan',
    'command_seed_max_gap: 0',
    'plan_max_age: 0',
    'threshold_angular: .inf',
))
def test_load_controller_config_rejects_invalid_motion_policy(tmp_path, controller):
    _write_config(tmp_path, f'controller: {{{controller}}}')

    with pytest.raises(ValueError, match='must be finite and positive'):
        load_controller_config('test.yaml', config_dir=tmp_path)


@pytest.mark.parametrize('name', ('test', 'test.yaml', 'config/test', 'config/test.yaml'))
def test_explicit_config_dir_applies_to_bare_and_prefixed_names(tmp_path, name):
    expected = _write_config(tmp_path, 'topics: {low_state: rt/custom_state}')

    assert _resolve_config_path(name, tmp_path) == expected
    assert load_controller_config(name, config_dir=tmp_path)['topics']['low_state'] == 'rt/custom_state'


def test_absolute_config_precedes_explicit_config_dir(tmp_path):
    expected = _write_config(tmp_path, 'topics: {low_state: rt/absolute_state}')

    assert _resolve_config_path(expected, tmp_path / 'wrong') == expected
    assert load_controller_config(
        expected.with_suffix(''), config_dir=tmp_path / 'wrong',
    )['topics']['low_state'] == 'rt/absolute_state'
