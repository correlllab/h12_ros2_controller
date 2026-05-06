from pathlib import Path
from typing import Any

import yaml
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from h12_ros2_controller.utility.joint_definition import NUM_MOTOR
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

def _load_joint_config(value: Any, field_name: str) -> np.ndarray:
    if isinstance(value, (int, float)):
        return np.full((NUM_MOTOR,), float(value), dtype=np.float64)
    if isinstance(value, list) and len(value) == NUM_MOTOR:
        return np.asarray(value, dtype=np.float64)
    raise ValueError(f'{field_name} must be a number or a list with {NUM_MOTOR} entries')

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
    position_offset = _load_joint_config(policy.get('position_offset', 0.02), 'limits.clip.position_offset')
    velocity_ratio = _load_joint_config(policy.get('velocity_ratio', 0.08), 'limits.clip.velocity_ratio')
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
    if kp is None and kd is None:
        return {'kp': None, 'kd': None}
    if kp is None or kd is None:
        raise ValueError('gains.kp and gains.kd must be provided together')
    return {
        'kp': _load_joint_config(kp, 'gains.kp'),
        'kd': _load_joint_config(kd, 'gains.kd'),
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
    frequency = raw.get('frequency', {})
    limits = raw.get('limits', {})
    gains = raw.get('gains', {})
    logging = raw.get('logging', {})

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
    torso_q = float(controller.get('torso_q', 0.0))

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
            'torso_q': torso_q,
        },
        'frequency': {
            'ctrl_hz': ctrl_hz,
            'pub_hz': pub_hz,
            'check_hz': check_hz,
        },
        'gains': {
            'kp': None if processed_gains['kp'] is None else processed_gains['kp'].astype(np.float32),
            'kd': None if processed_gains['kd'] is None else processed_gains['kd'].astype(np.float32),
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
            'record_interval': float(logging.get('record_interval', 0.01)),
        },
    }

def initialize_channel_factory(config):
    network = config.get('network', {})
    domain_id = int(network.get('domain_id', 0))
    interface = network.get('interface')
    if interface:
        ChannelFactoryInitialize(domain_id, interface)
    else:
        ChannelFactoryInitialize(domain_id)

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
        record_interval=float(logging_cfg.get('record_interval', 0.01)),
    )
