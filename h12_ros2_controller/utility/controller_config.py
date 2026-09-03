import os
from pathlib import Path
from typing import Any

import yaml
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from h12_ros2_controller.utility.joint_definition import NUM_MOTOR, UPPER_BODY_INDEX
from h12_ros2_controller.utility.joint_limits import (
    JOINT_POSITION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    JOINT_TORQUE_LIMITS,
)

DEFAULT_CONFIG_NAME = 'debug.yaml'
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_DIR = REPO_ROOT / 'config'
def _ensure_yaml_extension(config_path: Path) -> Path:
    '''Append .yaml when config path has no suffix'''
    if config_path.suffix:
        return config_path
    return config_path.with_suffix('.yaml')

def _resolve_config_path(config_name: str,
                         config_dir: Path = DEFAULT_CONFIG_DIR) -> Path:
    '''Resolve config from either bare name or config/<name> path'''
    config_path = _ensure_yaml_extension(Path(config_name))
    if config_path.is_absolute():
        return config_path
    if config_path.parts and config_path.parts[0] == 'config':
        return REPO_ROOT / config_path
    return config_dir / config_path

def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as handle:
        loaded = yaml.safe_load(handle)
    if loaded is None:
        return {}
    if not isinstance(loaded, dict):
        raise ValueError('config root must be a mapping')
    return loaded

def _validate_mode(mode):
    if mode not in ('debug', 'sport'):
        raise ValueError(f"Config mode must be 'debug' or 'sport', got: {mode}")


def _validate_zmp(raw: dict[str, Any]) -> dict[str, Any]:
    if 'zmp' not in raw:
        return {}

    zmp = raw['zmp']
    if not isinstance(zmp, dict):
        raise ValueError('zmp must be a mapping when provided')

    return zmp


def _validate_reactive_counter_balance(
        raw: dict[str, Any]) -> dict[str, Any]:
    if 'reactive_counter_balance' not in raw:
        return {}

    reactive = raw['reactive_counter_balance']
    if not isinstance(reactive, dict):
        raise ValueError(
            'reactive_counter_balance must be a mapping when provided'
        )
    return reactive


def _validate_counter_ddp(raw: dict[str, Any]) -> dict[str, Any]:
    if 'counter_ddp' not in raw:
        return {}
    counter_ddp = raw['counter_ddp']
    if not isinstance(counter_ddp, dict):
        raise ValueError('counter_ddp must be a mapping when provided')
    return counter_ddp


def _validate_adaptive_authority(raw: dict[str, Any]) -> dict[str, Any]:
    if 'adaptive_authority' not in raw:
        return {}
    adaptive = raw['adaptive_authority']
    if not isinstance(adaptive, dict):
        raise ValueError('adaptive_authority must be a mapping when provided')
    return adaptive


def _validate_decoupled_feedback(raw: dict[str, Any]) -> dict[str, Any]:
    if 'decoupled_feedback' not in raw:
        return {}
    feedback = raw['decoupled_feedback']
    if not isinstance(feedback, dict):
        raise ValueError('decoupled_feedback must be a mapping when provided')
    return feedback


def _validate_iteration5_residual_probe(raw: dict[str, Any]) -> dict[str, Any]:
    if 'iteration5_residual_probe' not in raw:
        return {}
    probe = raw['iteration5_residual_probe']
    if not isinstance(probe, dict):
        raise ValueError(
            'iteration5_residual_probe must be a mapping when provided'
        )
    return probe


def _validate_iteration5_h2(raw: dict[str, Any]) -> dict[str, Any]:
    if 'iteration5_h2' not in raw:
        return {}
    h2 = raw['iteration5_h2']
    if not isinstance(h2, dict):
        raise ValueError('iteration5_h2 must be a mapping when provided')
    return h2


def _load_joint_config(value: Any, field_name: str) -> np.ndarray:
    if isinstance(value, (int, float)):
        return np.full((NUM_MOTOR,), float(value), dtype=np.float64)
    if isinstance(value, list) and len(value) == NUM_MOTOR:
        return np.asarray(value, dtype=np.float64)
    if isinstance(value, list) and len(value) == len(UPPER_BODY_INDEX):
        values = np.zeros((NUM_MOTOR,), dtype=np.float64)
        values[UPPER_BODY_INDEX] = np.asarray(value, dtype=np.float64)
        return values
    raise ValueError(
        f'{field_name} must be a number, a list with {NUM_MOTOR} entries, '
        f'or a list with {len(UPPER_BODY_INDEX)} upper-body entries'
    )

def _load_upper_body_gain_config(value: Any, field_name: str) -> np.ndarray:
    gains = np.zeros((NUM_MOTOR,), dtype=np.float64)
    if isinstance(value, (int, float)):
        gains[UPPER_BODY_INDEX] = float(value)
        return gains
    if isinstance(value, list) and len(value) == len(UPPER_BODY_INDEX):
        gains[UPPER_BODY_INDEX] = np.asarray(value, dtype=np.float64)
        return gains
    raise ValueError(
        f'{field_name} must be a number or a list with '
        f'{len(UPPER_BODY_INDEX)} upper-body entries'
    )

def _derive_q_limits(position_offset: np.ndarray) -> np.ndarray:
    q_limits = np.zeros((NUM_MOTOR, 2), dtype=np.float64)
    for i in range(NUM_MOTOR):
        low = float(JOINT_POSITION_LIMITS[i]['low']) + position_offset[i]
        high = float(JOINT_POSITION_LIMITS[i]['high']) - position_offset[i]
        if low >= high:
            raise ValueError(f'position_offset too large for joint {i}')
        q_limits[i, 0] = low
        q_limits[i, 1] = high
    return q_limits

def _process_clip_limits(policy: dict[str, Any]) -> dict[str, np.ndarray]:
    position_offset = _load_joint_config(policy.get('position_offset', 0.001), 'limits.clip.position_offset')
    velocity_ratio = _load_joint_config(policy.get('velocity_ratio', 0.10), 'limits.clip.velocity_ratio')
    torque_ratio = _load_joint_config(policy.get('torque_ratio', 0.25), 'limits.clip.torque_ratio')

    return {
        'q_limits': _derive_q_limits(position_offset),
        'dq_limits': np.asarray(JOINT_VELOCITY_LIMITS, dtype=np.float64) * velocity_ratio,
        'tau_limits': np.asarray(JOINT_TORQUE_LIMITS, dtype=np.float64) * torque_ratio,
    }

def _process_estop_limits(policy: dict[str, Any]) -> dict[str, np.ndarray]:
    position_offset = _load_joint_config(policy.get('position_offset', 0.01), 'limits.estop.position_offset')
    velocity_ratio = _load_joint_config(policy.get('velocity_ratio', 0.5), 'limits.estop.velocity_ratio')
    torque_ratio = _load_joint_config(policy.get('torque_ratio', 0.5), 'limits.estop.torque_ratio')

    return {
        'q_limits': _derive_q_limits(position_offset),
        'dq_limits': np.asarray(JOINT_VELOCITY_LIMITS, dtype=np.float64) * velocity_ratio,
        'tau_limits': np.asarray(JOINT_TORQUE_LIMITS, dtype=np.float64) * torque_ratio,
    }

def build_processed_limits(limits_cfg: dict[str, Any] | None = None) -> dict[str, np.ndarray]:
    limits_cfg = {} if limits_cfg is None else limits_cfg
    clip_policy = limits_cfg.get('clip', limits_cfg.get('enforce', {}))
    estop_policy = limits_cfg.get('estop', {})

    clip_limits = _process_clip_limits(clip_policy)
    estop_limits = _process_estop_limits(estop_policy)

    return {
        'q_clip_limits': clip_limits['q_limits'],
        'dq_clip_limits': clip_limits['dq_limits'],
        'tau_clip_limits': clip_limits['tau_limits'],
        'q_estop_limits': estop_limits['q_limits'],
        'dq_estop_limits': estop_limits['dq_limits'],
        'tau_estop_limits': estop_limits['tau_limits'],
    }

def get_publisher_clip_limits(config: dict[str, Any] | None = None) -> dict[str, Any]:
    config = {} if config is None else config
    limits_cfg = config.get('limits', {})

    if all(key in limits_cfg for key in ('q_clip_limits', 'dq_clip_limits', 'tau_clip_limits')):
        q_clip_limits = limits_cfg['q_clip_limits']
        dq_clip_limits = limits_cfg['dq_clip_limits']
        tau_clip_limits = limits_cfg['tau_clip_limits']
    else:
        processed = build_processed_limits(limits_cfg)
        q_clip_limits = processed['q_clip_limits']
        dq_clip_limits = processed['dq_clip_limits']
        tau_clip_limits = processed['tau_clip_limits']

    return {
        'position_clip': [
            {'low': float(limit[0]), 'high': float(limit[1])}
            for limit in q_clip_limits
        ],
        'velocity_clip': dq_clip_limits,
        'torque_clip': tau_clip_limits,
    }

def _process_gains(raw_gains: dict[str, Any]) -> dict[str, Any]:
    kp = raw_gains.get('kp')
    kd = raw_gains.get('kd')
    ki = raw_gains.get('ki')
    if kp is None or kd is None or ki is None:
        raise ValueError('gains.kp, gains.kd, and gains.ki must be provided')

    return {
        'kp': _load_upper_body_gain_config(kp, 'gains.kp'),
        'kd': _load_upper_body_gain_config(kd, 'gains.kd'),
        'ki': _load_upper_body_gain_config(ki, 'gains.ki'),
    }

def load_controller_config(config_name=DEFAULT_CONFIG_NAME,
                           config_dir=DEFAULT_CONFIG_DIR) -> dict[str, Any]:
    config_path = _resolve_config_path(config_name, config_dir=Path(config_dir))
    if not config_path.exists():
        raise FileNotFoundError(f'Config file not found: {config_path}')

    raw = _load_yaml(config_path)

    mode = str(raw.get('mode', 'debug'))
    _validate_mode(mode)

    topics = raw.get('topics', {})
    network = raw.get('network', {})
    controller = raw.get('controller', {})
    planner = raw.get('planner', {})
    frequency = raw.get('frequency', {})
    limits = raw.get('limits', {})
    gains = raw.get('gains', {})
    logging = raw.get('logging', {})
    if not isinstance(planner, dict):
        raise ValueError('planner must be a mapping')
    momentum_ddp = raw.get('momentum_ddp', {})
    zmp = _validate_zmp(raw)
    reactive_counter_balance = _validate_reactive_counter_balance(raw)
    counter_ddp = _validate_counter_ddp(raw)
    adaptive_authority = _validate_adaptive_authority(raw)
    decoupled_feedback = _validate_decoupled_feedback(raw)
    iteration5_residual_probe = _validate_iteration5_residual_probe(raw)
    iteration5_h2 = _validate_iteration5_h2(raw)

    ctrl_hz = float(frequency.get('ctrl_hz', 50.0))
    pub_hz = float(frequency.get('pub_hz', 500.0))
    check_hz = float(frequency.get('check_hz', 1000.0))
    if ctrl_hz <= 0.0:
        raise ValueError('frequency.ctrl_hz must be positive')
    if pub_hz <= 0.0:
        raise ValueError('frequency.pub_hz must be positive')
    if check_hz <= 0.0:
        raise ValueError('frequency.check_hz must be positive')

    processed_limits = build_processed_limits(limits)
    processed_gains = _process_gains(gains)

    v_lim = float(controller.get('v_lim', 1.0))
    w_lim = float(controller.get('w_lim', 2.0))
    dq_lim = float(controller.get('dq_lim', 1.0))
    d_min = float(controller.get('d_min', 0.02))
    timeout = float(controller.get('timeout', 10.0))
    torso_target = float(controller.get('torso_target', 0.0))
    threshold_ik = float(controller.get('threshold_ik', 0.001))
    threshold_joint = float(controller.get('threshold_joint', 0.01))
    threshold_linear = float(controller.get('threshold_linear', 0.002))
    threshold_angular = float(controller.get('threshold_angular', 0.01))

    return {
        'mode': mode,
        'topics': {
            'low_cmd': str(topics.get('low_cmd', 'rt/lowcmd')),
            'low_state': str(topics.get('low_state', 'rt/lowstate')),
        },
        'network': {
            'domain_id': int(network.get('domain_id', 0)),
            'interface': None if network.get('interface') in (None, '') else str(network.get('interface')),
        },
        'controller': {
            'v_lim': v_lim,
            'w_lim': w_lim,
            'dq_lim': dq_lim,
            'd_min': d_min,
            'timeout': timeout,
            'torso_target': torso_target,
            'threshold_ik': threshold_ik,
            'threshold_joint': threshold_joint,
            'threshold_linear': threshold_linear,
            'threshold_angular': threshold_angular,
        },
        'planner': dict(planner),
        'frequency': {
            'ctrl_hz': ctrl_hz,
            'pub_hz': pub_hz,
            'check_hz': check_hz,
        },
        'gains': {
            'kp': None if processed_gains['kp'] is None else processed_gains['kp'].astype(np.float32),
            'kd': None if processed_gains['kd'] is None else processed_gains['kd'].astype(np.float32),
            'ki': None if processed_gains['ki'] is None else processed_gains['ki'].astype(np.float32),
        },
        'limits': {
            'q_clip_limits': processed_limits['q_clip_limits'],
            'dq_clip_limits': processed_limits['dq_clip_limits'],
            'tau_clip_limits': processed_limits['tau_clip_limits'],
            'q_estop_limits': processed_limits['q_estop_limits'],
            'dq_estop_limits': processed_limits['dq_estop_limits'],
            'tau_estop_limits': processed_limits['tau_estop_limits'],
        },
        'logging': {
            'enabled': bool(logging.get('enabled', False)),
            'base_dir': str(logging.get('base_dir', logging.get('save_path', 'data/control_record'))),
            'filename': str(logging.get('filename', logging.get('save_filename', 'record'))),
            'record_interval': float(logging.get('record_interval', 1.0)),
        },
        'momentum_ddp': momentum_ddp,
        'zmp': zmp,
        'reactive_counter_balance': reactive_counter_balance,
        'counter_ddp': counter_ddp,
        'adaptive_authority': adaptive_authority,
        'decoupled_feedback': decoupled_feedback,
        'iteration5_residual_probe': iteration5_residual_probe,
        'iteration5_h2': iteration5_h2,
    }

def _channel_factory_settings(config: dict[str, Any]) -> tuple[int, str | None]:
    '''Get DDS channel factory settings from environment and config'''
    env_domain = os.environ.get('ROS_DOMAIN_ID')
    network = config['network']
    domain_id = (
        int(env_domain)
        if env_domain is not None
        else int(network['domain_id'])
    )
    return domain_id, network['interface']


def init_channel_factory(config: dict[str, Any]) -> None:
    '''Initialize the DDS channel factory from environment and config'''
    domain_id, interface = _channel_factory_settings(config)
    if interface:
        ChannelFactoryInitialize(domain_id, interface)
    else:
        ChannelFactoryInitialize(domain_id)


def init_channel_factory_guard(config: dict[str, Any]) -> None:
    '''Confirm before initializing the real robot DDS domain'''
    domain_id, _ = _channel_factory_settings(config)
    if domain_id == 0:
        try:
            confirmation = input(
                'WARNING: ROS_DOMAIN_ID=0 -> DDS domain 0 is the REAL ROBOT '
                'command bus.\n'
                '         Nodes will publish/subscribe on the live robot.\n'
                'Proceed on DDS domain 0 (real robot)? [y/N] '
            )
        except EOFError:
            confirmation = ''
        if confirmation.strip().lower() != 'y':
            raise SystemExit('DDS channel factory initialization cancelled')
    init_channel_factory(config)

def resolve_sport_mode(config):
    return config.get('mode', 'debug') == 'sport'

def maybe_start_controller_logging(controller):
    logging_cfg = controller.config.get('logging', {})
    if not logging_cfg.get('enabled', False):
        return

    filename = logging_cfg.get('filename', '').strip()
    if not filename:
        raise ValueError('logging.filename must be set when logging.enabled is true')

    controller.start_recording(
        save_path=logging_cfg.get('base_dir', 'data/control_record'),
        filename=filename,
        record_interval=float(logging_cfg.get('record_interval', 1.0)),
    )
