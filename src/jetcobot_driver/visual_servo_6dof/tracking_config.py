"""Load the shared tracking configuration without opening devices."""

from pathlib import Path
import hashlib
import json

import numpy as np
import yaml


def package_file(package, relative):
    """Resolve installed resources, with a source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory(package)) / relative
        if path.exists():
            return path
    except (ImportError, LookupError):
        pass
    return Path(__file__).resolve().parents[2] / package / relative


def default_config():
    """Return the default editable YAML path."""
    source = Path(__file__).resolve().parent / 'config/visual_servo.yaml'
    return source if source.exists() else package_file(
        'jetcobot_driver', 'visual_servo_6dof/config/visual_servo.yaml')


def load_config(path=None):
    """Read and validate parameters shared by planning and execution."""
    path = Path(path or default_config()).expanduser().resolve()
    config = yaml.safe_load(path.read_text(encoding='utf-8'))['tracking']
    config['_path'] = str(path)
    model = config['model']
    model['urdf'] = str(package_file('jetcobot_description', 'urdf/jetcobot.urdf'))
    model['srdf'] = str(package_file('jetcobot_moveit', 'config/jetcobot.srdf'))
    for section, names in {
            'runtime': ['control_hz', 'command_hz', 'log_period_s', 'timeout_s', 'tf_sync_tolerance_s'],
            'camera': ['square_side_mm', 'desired_depth_mm', 'min_radius_px'],
            'pid': ['max_speed_m_s', 'max_acceleration_m_s2'],
            'serial': ['update_hz', 'timeout_s', 'max_step_rad']}.items():
        for name in names:
            value = config[section][name]
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{section}.{name} must be finite and positive')
    if config['runtime']['command_hz'] > config['runtime']['control_hz']:
        raise ValueError('command_hz must not exceed control_hz')
    if config['runtime']['tf_sync_tolerance_s'] > config['runtime']['timeout_s']:
        raise ValueError('tf_sync_tolerance_s must not exceed timeout_s')
    for name in ['kp', 'ki', 'kd', 'deadband_m', 'integral_limit_m_s']:
        a = np.asarray(config['pid'][name], dtype=float)
        if a.shape != (3,) or not np.all(np.isfinite(a)) or np.any(a < 0):
            raise ValueError(f'pid.{name} requires three finite nonnegative values')
    for name in ['max_velocity_rad_s', 'max_acceleration_rad_s2']:
        a = np.asarray(model[name], dtype=float)
        if a.shape != (6,) or not np.all(np.isfinite(a)) or np.any(a <= 0):
            raise ValueError(f'model.{name} requires six positive values')
    if not 1 <= config['serial']['speed_max'] <= 15:
        raise ValueError('serial.speed_max must be in 1..15')
    if config['serial']['mode'] not in ('preview', 'execute'):
        raise ValueError('serial.mode must be preview or execute')
    if config['ik']['backend'] not in ('moveit_servo', 'weighted_dls'):
        raise ValueError('invalid ik.backend')
    for name in ['reference_length_m', 'epsilon', 'lambda_max', 'weight_max', 'rank_tolerance']:
        if not np.isfinite(config['ik'][name]) or config['ik'][name] <= 0:
            raise ValueError(f'ik.{name} must be positive')
    for name in ['input_tau_s', 'depth_tau_s', 'derivative_tau_s']:
        if not np.isfinite(config['pid'][name]) or config['pid'][name] <= 0:
            raise ValueError(f'pid.{name} must be positive')
    ratio = config['pid'].get('settle_exit_ratio', 1.5)
    if not np.isfinite(ratio) or ratio < 1.:
        raise ValueError('pid.settle_exit_ratio must be finite and >= 1')
    for name in ['speed_filter_tau_s', 'speed_change_interval_s']:
        value = config['serial'].get(name, .25)
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f'serial.{name} must be finite and positive')
    safety = config['safety']
    if safety.get('collision_backend', 'capsules') not in ('urdf_mesh', 'capsules'):
        raise ValueError('invalid collision_backend')
    if (safety.get('max_tracking_excursion_m') is not None and
            (not np.isfinite(safety['max_tracking_excursion_m']) or safety['max_tracking_excursion_m'] <= 0)):
        raise ValueError('max_tracking_excursion_m must be positive or null')
    if (len(safety['capsule_radii_m']) != 7 or
            not np.all(np.isfinite(safety['capsule_radii_m'])) or
            np.any(np.array(safety['capsule_radii_m']) <= 0)):
        raise ValueError('seven positive capsule radii are required')
    if (not 0 < config['model']['margin_deg'] < 30 or
            not 0 < config['serial']['max_step_rad'] <= .075 or
            not 1. / config['serial']['update_hz'] < config['serial']['timeout_s'] <= 1.):
        raise ValueError('invalid joint margin, step or serial timing')
    if safety['table_z_m'] is not None and not np.isfinite(safety['table_z_m']):
        raise ValueError('table_z_m must be finite or null')
    for name in ['workspace_min_m', 'workspace_max_m', 'radius_origin_m']:
        a = np.asarray(safety[name], dtype=float)
        if a.shape != (3,) or not np.all(np.isfinite(a)):
            raise ValueError(f'safety.{name} requires three finite values')
    if np.any(np.asarray(safety['workspace_min_m']) >= safety['workspace_max_m']):
        raise ValueError('workspace minimum must be below maximum')
    if not np.isfinite(safety['max_radius_m']) or safety['max_radius_m'] <= 0:
        raise ValueError('max_radius_m must be positive')
    if not isinstance(safety['step_samples'], int) or safety['step_samples'] < 2:
        raise ValueError('step_samples must be at least two')
    for name in ['base_from_sdk_base', 'sdk_tool_from_model_ee']:
        transform = np.asarray(config['fk_check'][name], dtype=float)
        if (transform.shape != (4, 4) or not np.all(np.isfinite(transform)) or
                not np.allclose(transform[3], [0, 0, 0, 1]) or
                not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1.e-6) or
                not np.isclose(np.linalg.det(transform[:3, :3]), 1.)):
            raise ValueError(f'fk_check.{name} must be a rigid transform in metres')
    canonical = {k: v for k, v in config.items() if not k.startswith('_')}
    digest = hashlib.sha256(json.dumps(canonical, sort_keys=True).encode())
    for resource in (model['urdf'], model['srdf'], config['camera']['calibration_file']):
        digest.update(Path(resource).expanduser().read_bytes())
    if safety.get('collision_backend') == 'urdf_mesh':
        import xml.etree.ElementTree as ET
        for mesh in ET.parse(model['urdf']).getroot().findall('.//collision/geometry/mesh'):
            package, relative = mesh.get('filename')[len('package://'):].split('/', 1)
            digest.update(package_file(package, relative).read_bytes())
    config['_digest'] = digest.hexdigest()
    return config
