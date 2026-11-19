"""Camera aiming and apparent range control with velocity as the LQR input."""

import numpy as np
from scipy.linalg import solve_discrete_are


def aiming_axes(center_ray):
    """Build two camera rotation axes perpendicular to the desired view ray."""
    center_ray = center_ray / np.linalg.norm(center_ray)
    right = np.cross([0., 1., 0.], center_ray)
    right /= np.linalg.norm(right)
    return np.column_stack((right, np.cross(center_ray, right)))


def aiming_error(center_ray, target_ray, axes, range_error):
    """Return the rotation from image centre to target and the range residual."""
    cross = np.cross(center_ray, target_ray)
    sine = np.linalg.norm(cross)
    angle = np.arctan2(sine, np.dot(center_ray, target_ray))
    rotation = cross * (angle / sine) if sine > 0. else np.zeros(3)
    return np.r_[axes.T @ rotation, range_error]


def velocity_model(dt):
    """Locally, turning toward the target and approaching it reduce the errors."""
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('The sample period must be positive')
    return np.eye(3), -dt * np.eye(3)


def lqr_gain(dt, q, r):
    """Solve LQR for two camera rotation rates and line-of-sight speed."""
    q, r = np.asarray(q, dtype=float), np.asarray(r, dtype=float)
    if (q.shape != (3,) or r.shape != (3,) or
            not np.all(np.isfinite(np.r_[q, r])) or
            np.any(q <= 0) or np.any(r <= 0)):
        raise ValueError('Q and R must each have three positive finite weights')
    a, b = velocity_model(dt)
    p = solve_discrete_are(a, b, np.diag(q), np.diag(r))
    gain = np.linalg.solve(np.diag(r) + b.T @ p @ b, b.T @ p @ a)
    if np.max(np.abs(np.linalg.eigvals(a - b @ gain))) >= 1:
        raise ValueError('The discrete LQR is not stable')
    return gain


def camera_to_tool_twist(command, tool_from_camera):
    """Shift camera-origin velocity to the calibrated tool origin."""
    rotation = tool_from_camera[:3, :3]
    translation = tool_from_camera[:3, 3]
    angular = rotation @ command[3:]
    linear = rotation @ command[:3] + np.cross(translation, angular)
    return np.r_[linear, angular]


def norm_limit(vector, limit):
    """Bound a vector's norm while preserving its direction."""
    norm = np.linalg.norm(vector)
    return vector * min(1., limit / max(norm, 1.e-12))
