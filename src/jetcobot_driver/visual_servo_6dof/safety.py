"""Workspace, table, capsule collision and final-step constraints."""

import numpy as np
import json


def segment_distance(a, b, c, d):
    """Compute capsule centreline distance, including degenerate segments."""
    u, v, w = b - a, d - c, a - c
    uu, vv, uv = u @ u, v @ v, u @ v
    candidates = []
    for t in (0., 1.):
        s = np.clip(((a + t * u - c) @ v) / max(vv, 1.e-15), 0., 1.)
        candidates.append(np.linalg.norm(a + t * u - c - s * v))
    for s in (0., 1.):
        t = np.clip(((c + s * v - a) @ u) / max(uu, 1.e-15), 0., 1.)
        candidates.append(np.linalg.norm(a + t * u - c - s * v))
    determinant = uu * vv - uv**2
    if determinant > 1.e-15:
        t = (uv * (v @ w) - vv * (u @ w)) / determinant
        s = (uu * (v @ w) - uv * (u @ w)) / determinant
        if 0 <= t <= 1 and 0 <= s <= 1:
            candidates.append(np.linalg.norm(w + t * u - s * v))
    return float(min(candidates))


def project_halfspaces(value, a, b, iterations=100):
    """Project onto intersecting halfspaces a*x>=b using Dykstra iterations."""
    value = np.asarray(value, dtype=float).copy()
    corrections = np.zeros_like(a)
    for _ in range(iterations):
        previous = value.copy()
        for i, (normal, threshold) in enumerate(zip(a, b)):
            y = value + corrections[i]
            amount = max(0., threshold - normal @ y) / max(normal @ normal, 1.e-20)
            value = y + amount * normal
            corrections[i] = y - value
        if np.max(b - a @ value) <= 1.e-9 and np.linalg.norm(value - previous) < 1.e-9:
            return value
    return value if np.max(b - a @ value) <= 1.e-7 else None


class SafetyGuard:
    """Validate the actual candidate, not only its requested Cartesian speed."""

    def __init__(self, model, config):
        """Load geometry and limits; unverified geometry never authorizes execution."""
        self.model, self.config = model, config
        self.previous_velocity = np.zeros(6)
        self.reason = 'OK'
        self.tracking_origin = None
        self.constraint_cache = {}
        self.gradient_q = self.gradient = None
        self.mesh = None
        if config['safety'].get('collision_backend') == 'urdf_mesh':
            from .mesh_guard import MeshGuard
            self.mesh = MeshGuard(model, config)

    @property
    def execution_ready(self):
        """Require explicit physical model and table verification before real motion."""
        return bool(self.config['model']['verified'] and
                    self.config['fk_check']['alignment_verified'] and
                    self.config['safety']['verified'] and
                    self.mesh is not None and
                    self.config['safety']['table_z_m'] is not None)

    def constraints(self, q):
        """Cache repeated measured/intermediate states; configuration changes invalidate the key."""
        q = np.asarray(q)
        if self.tracking_origin is None:
            self.tracking_origin = self.model.fk(q).camera[:3, 3].copy()
        key = (q.tobytes(), json.dumps(self.config['safety'], sort_keys=True), self.tracking_origin.tobytes())
        if key not in self.constraint_cache:
            if len(self.constraint_cache) >= 64:
                self.constraint_cache.pop(next(iter(self.constraint_cache)))
            values, names = self._constraints_uncached(q)
            if self.mesh is not None:
                extra, labels = self.mesh.constraints(q)
                values, names = np.r_[values, extra], names + labels
            self.constraint_cache[key] = values, names
        return self.constraint_cache[key]

    def _constraints_uncached(self, q):
        """Return signed clearances and labels; nonnegative is feasible."""
        c = self.config['safety']
        pose = self.model.fk(q)
        p = pose.camera[:3, 3]
        if self.tracking_origin is None:
            self.tracking_origin = p.copy()
        lo, hi = np.asarray(c['workspace_min_m']), np.asarray(c['workspace_max_m'])
        values = list(p - lo) + list(hi - p)
        names = [f'workspace_min_{x}' for x in 'xyz'] + [f'workspace_max_{x}' for x in 'xyz']
        values.append(c['max_radius_m'] - np.linalg.norm(p - c['radius_origin_m']))
        names.append('workspace_radius')
        excursion = c.get('max_tracking_excursion_m')
        if excursion is not None:
            values.append(excursion - np.linalg.norm(p - self.tracking_origin))
            names.append('startup_tracking_radius')
        if c['table_z_m'] is not None:
            for i, segment in enumerate(pose.segments):
                if i not in c['table_exempt_segments']:
                    values.append(float(np.min(segment[:, 2])) - c['capsule_radii_m'][i] -
                                  c['table_z_m'] - c['table_clearance_m'])
                    names.append(f'table_segment_{i}')
        ignored = {tuple(sorted(pair)) for pair in c['allowed_collision_pairs']}
        for i in range(len(pose.segments)):
            for j in range(i + 2, len(pose.segments)):
                if (i, j) in ignored:
                    continue
                distance = segment_distance(*pose.segments[i], *pose.segments[j])
                values.append(distance - c['capsule_radii_m'][i] -
                              c['capsule_radii_m'][j] - c['self_clearance_m'])
                names.append(f'capsule_{i}_{j}')
        return np.asarray(values), names

    def valid(self, q):
        """Check joint bounds and all configured geometric constraints."""
        q = np.asarray(q)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            return False, 'invalid_joint_vector'
        if np.any(q < self.model.limits[:, 0] - 1.e-8) or np.any(q > self.model.limits[:, 1] + 1.e-8):
            return False, 'joint_position_limit'
        values, names = self.constraints(q)
        worst = int(np.argmin(values))
        return (True, 'OK') if values[worst] >= -1.e-7 else (False, names[worst])

    def project_task(self, q, velocity, dt):
        """Remove outward Cartesian velocity at box and radius boundaries."""
        c = self.config['safety']
        p = self.model.fk(q).camera[:3, 3]
        a = [*np.eye(3), *-np.eye(3)]
        b = [*(np.asarray(c['workspace_min_m']) - p) / dt,
             *(p - np.asarray(c['workspace_max_m'])) / dt]
        radial = p - c['radius_origin_m']
        length = np.linalg.norm(radial)
        if length > 1.e-9:
            a.append(-radial / length)
            b.append((length - c['max_radius_m']) / dt)
        if self.tracking_origin is not None and c.get('max_tracking_excursion_m') is not None:
            radial = p - self.tracking_origin
            length = np.linalg.norm(radial)
            if length > 1.e-9:
                a.append(-radial / length)
                b.append((length - c['max_tracking_excursion_m']) / dt)
        result = project_halfspaces(velocity, np.asarray(a), np.asarray(b))
        return np.zeros(3) if result is None else result

    def filter_velocity(self, q, requested, dt):
        """Enforce velocity/acceleration and linearized link constraints jointly."""
        valid, reason = self.valid(q)
        if not valid:
            self.previous_velocity[:] = 0.
            self.reason = reason
            return np.zeros(6)
        vmax = np.asarray(self.config['model']['max_velocity_rad_s'])
        amax = np.asarray(self.config['model']['max_acceleration_rad_s2'])
        max_step = self.config['serial']['max_step_rad']
        lower = np.maximum.reduce((-vmax, self.previous_velocity - amax * dt,
                                   (self.model.limits[:, 0] - q) / dt,
                                   np.full(6, -max_step / dt)))
        upper = np.minimum.reduce((vmax, self.previous_velocity + amax * dt,
                                   (self.model.limits[:, 1] - q) / dt,
                                   np.full(6, max_step / dt)))
        a, b = [*np.eye(6), *-np.eye(6)], [*lower, *-upper]
        values, names = self.constraints(q)
        close = np.flatnonzero(values < self.config['safety']['gradient_distance_m'])
        if len(close):
            if self.gradient_q is None or np.max(np.abs(q - self.gradient_q)) > .005:
                h = 1.e-5
                self.gradient = np.column_stack([(self.constraints(q + np.eye(6)[i] * h)[0] -
                                                 self.constraints(q - np.eye(6)[i] * h)[0]) / (2 * h)
                                                for i in range(6)])
                self.gradient_q = q.copy()
            gradient = self.gradient
            for i in close:
                a.append(gradient[i])
                b.append(-values[i] / dt)
        result = project_halfspaces(requested, np.asarray(a), np.asarray(b))
        if result is None:
            self.reason = 'infeasible_constraints'
            self.previous_velocity[:] = 0.
            return np.zeros(6)
        ok, reason = self.validate_step(q, q + result * dt)
        if not ok:
            self.reason = reason
            self.previous_velocity[:] = 0.
            return np.zeros(6)
        self.reason = 'projected' if np.linalg.norm(result - requested) > 1.e-7 else 'OK'
        self.previous_velocity = result.copy()
        return result

    def validate_step(self, current, target):
        """Check intermediate states as well as the final joint target."""
        for fraction in np.linspace(0., 1., self.config['safety']['step_samples']):
            ok, reason = self.valid(current + fraction * (target - current))
            if not ok:
                return False, reason
        return True, 'OK'
