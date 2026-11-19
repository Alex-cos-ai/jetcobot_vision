"""Validate Servo positions before converting them to serial commands."""

import numpy as np

JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]
MODEL_LIMITS = np.array([
    [-3.05, 3.05], [-1.57, 1.57], [-1.57, 1.57],
    [-1.57, 1.57], [-3.05, 3.05], [-1.57, 1.57],
])


def checked_target(names, positions, current, max_step):
    """Require exactly six unique joints and a nearby finite valid target."""
    if (len(names) != 6 or len(set(names)) != 6 or
            set(names) != set(JOINT_NAMES) or len(positions) != 6):
        raise ValueError('Trajectory must contain the six named arm joints')
    by_name = dict(zip(names, positions))
    target = np.asarray([by_name[name] for name in JOINT_NAMES], dtype=float)
    if (not np.all(np.isfinite(target)) or
            not np.all(np.isfinite(current))):
        raise ValueError('Non-finite joint target or feedback')
    if np.any(target < MODEL_LIMITS[:, 0]) or np.any(
            target > MODEL_LIMITS[:, 1]):
        raise ValueError('Target exceeds URDF position limits')
    if np.max(np.abs(target - current)) > max_step:
        raise ValueError('Target is too far from actual joint feedback')
    return target


def model_radians(hardware_angles):
    """Apply the existing joint-six calibration to actual SDK feedback."""
    result = np.radians(np.asarray(hardware_angles, dtype=float))
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError('Hardware did not return six finite joint angles')
    result[5] += np.pi / 4
    return result


def hardware_degrees(model_angles):
    """Invert the joint-six offset before sending a command to the SDK."""
    result = np.degrees(np.asarray(model_angles, dtype=float)).copy()
    result[5] -= 45.
    return result.tolist()
