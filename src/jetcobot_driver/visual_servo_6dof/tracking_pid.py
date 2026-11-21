"""PID and continuous tracking states, using metres and seconds internally."""

from dataclasses import dataclass

import numpy as np


def norm_limit(vector, limit):
    """Limit a vector without changing its direction."""
    vector = np.asarray(vector, dtype=float)
    return vector * min(1., limit / max(float(np.linalg.norm(vector)), 1.e-12))


class PIDChannel:
    """Use conditional integration and filtered measurement derivative."""

    def __init__(self, kp, ki, kd, deadband, integral_limit,
                 derivative_tau=.1, integral_decay_s=.4):
        """Store gains; error is measurement minus reference."""
        self.kp, self.ki, self.kd = kp, ki, kd
        self.deadband, self.integral_limit = deadband, integral_limit
        self.derivative_tau = derivative_tau
        self.integral_decay_s = integral_decay_s
        self.reset()

    def reset(self):
        """Clear all histories, including the last output."""
        self.integral = self.derivative = self.output = 0.
        self.previous_measurement = None

    def update(self, error, measurement, dt, limit, measurement_dt=None,
               integrate=True):
        """Compute velocity; positive motion reduces positive error."""
        if not np.all(np.isfinite([error, measurement, dt])) or dt <= 0:
            raise ValueError('PID input and dt must be finite; dt must be positive')
        if measurement_dt is not None:
            if self.previous_measurement is not None and measurement_dt > 0:
                raw = (measurement - self.previous_measurement) / measurement_dt
                alpha = measurement_dt / (self.derivative_tau + measurement_dt)
                self.derivative += alpha * (raw - self.derivative)
            self.previous_measurement = measurement
        old = self.integral
        inside = abs(error) <= self.deadband
        if inside:
            self.integral *= np.exp(-dt / self.integral_decay_s)
            e = 0.
        else:
            # 连续死区：越过边界时从零输出平滑增加，避免Kp*deadband阶跃。
            e = float(np.sign(error) * (abs(error)-self.deadband))
            if integrate and self.ki > 0:
                self.integral = float(np.clip(
                    old + e * dt, -self.integral_limit, self.integral_limit))
        # 本任务 e=测量-参考，正相机速度使 e 减小；微分项因此为正号。
        raw = self.kp * e + self.ki * self.integral + self.kd * self.derivative
        if abs(raw) > limit and raw * e > 0 and not inside:
            self.integral = old
            raw = self.kp * e + self.ki * old + self.kd * self.derivative
        self.output = float(np.clip(raw, -limit, limit))
        return self.output


class TrackingPID:
    """Convert enclosing-circle observations into camera translation."""

    def __init__(self, config, focal, reference_normalized):
        """Use rectified equal-focal-length pixels and a z=1 reference ray."""
        self.config, self.focal = config, float(focal)
        self.reference = np.asarray(reference_normalized, dtype=float)
        p = config['pid']
        self.channels = [PIDChannel(
            p['kp'][i], p['ki'][i], p['kd'][i], p['deadband_m'][i],
            p['integral_limit_m_s'][i], p['derivative_tau_s']) for i in range(3)]
        self.reset()

    def reset(self):
        """Clear pixel/depth filters and all PID histories."""
        for channel in self.channels:
            channel.reset()
        self.filtered = self.depth = self.measurement = None
        self.error = np.zeros(3)
        self.previous = np.zeros(3)
        self.stamp = self.sample_dt = None
        self.new_sample = False
        self.partial = False
        self.settled = False

    def restart_output(self):
        """暂停后仅清控制历史，保留仍然新鲜的图像测量，避免等待下一帧的反馈循环。"""
        for channel in self.channels:
            channel.reset()
        self.previous[:] = 0.
        self.new_sample = True

    def observe(self, cx, cy, radius, stamp, principal, partial=False):
        """Filter each new image once; return whether the distance is valid."""
        values = np.array([cx, cy, radius, stamp], dtype=float)
        c, p = self.config['camera'], self.config['pid']
        if not np.all(np.isfinite(values)) or radius < c['min_radius_px']:
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        dt = .05 if self.stamp is None else stamp - self.stamp
        if self.stamp is not None and dt > self.config['runtime']['timeout_s']:
            self.reset()
        if partial != self.partial:
            # 残缺/完整切换时清空半径滤波，避免残缺半径污染恢复后的测距。
            self.filtered = None
            self.channels[2].reset()
        self.partial = partial
        alpha = dt / (p['input_tau_s'] + dt)
        self.filtered = values[:3] if self.filtered is None else (
            self.filtered + alpha * (values[:3] - self.filtered))
        # 正方形外接圆物理直径为 sqrt(2)*边长，不能直接使用边长。
        if not partial:
            depth = self.focal * np.sqrt(2.) * c['square_side_mm'] / (
                2000. * self.filtered[2])
            if not c['depth_range_m'][0] <= depth <= c['depth_range_m'][1]:
                return False
            a = dt / (p['depth_tau_s'] + dt)
            self.depth = depth if self.depth is None else self.depth + a * (depth - self.depth)
        # 未获得完整圆时，只用上次有效深度或期望深度作横向速度的比例尺度。
        depth_for_xy = self.depth if self.depth is not None else c['desired_depth_mm']/1000.
        ray = (self.filtered[:2] - np.asarray(principal)) / self.focal
        self.measurement = np.r_[depth_for_xy * (ray - self.reference), depth_for_xy]
        self.error = self.measurement - np.array([0., 0., c['desired_depth_mm'] / 1000.])
        if partial:
            self.error[2] = 0.
            self.settled = False
        else:
            deadband = np.asarray(p['deadband_m'])
            ratio = p.get('settle_exit_ratio', 1.5)
            if self.settled:
                self.settled = bool(np.all(np.abs(self.error) <= ratio*deadband))
            else:
                self.settled = bool(np.all(np.abs(self.error) <= deadband))
        self.stamp, self.sample_dt, self.new_sample = stamp, dt, True
        return True

    def tick(self, dt, scale=1., blocked=False):
        """Update control at its own rate; return camera velocity in m/s."""
        if self.measurement is None:
            return np.zeros(3)
        p = self.config['pid']
        limit = p['max_speed_m_s'] * scale
        old_integrals = [channel.integral for channel in self.channels]
        v = np.array([channel.update(
            self.error[i], self.measurement[i], dt, limit,
            self.sample_dt if self.new_sample else None, not blocked)
            for i, channel in enumerate(self.channels)])
        if self.partial:
            v[2] = 0.
        elif self.settled:
            v[:] = 0.
        # 抵消目标参考射线不位于光学主点时，前后运动带来的横纵耦合。
        v[:2] += self.reference * v[2]
        limited = norm_limit(v, limit)
        if np.linalg.norm(v - limited) > 1.e-12 or blocked:
            for channel, old in zip(self.channels, old_integrals):
                channel.integral = old
        self.previous += norm_limit(
            limited - self.previous, p['max_acceleration_m_s2'] * dt)
        if self.partial:
            self.previous[2] = 0.
        self.new_sample = False
        return self.previous.copy()


@dataclass
class TrackingStateMachine:
    """Keep sensing through convergence and target loss."""

    soft_start_s: float = 1.
    lost_reset_s: float = .5
    lost_decay_s: float = .15
    state: str = 'IDLE'
    last_seen: float = None
    entered: float = 0.

    def step(self, found, healthy, now):
        """Return state, output scale and a request to reset PID histories."""
        old = self.state
        if not healthy:
            self.state = 'HOLD'
            return self.state, 0., old != 'HOLD'
        if found:
            if old != 'TRACKING':
                self.entered = now
            self.last_seen, self.state = now, 'TRACKING'
            return self.state, min(1., max(0., (now - self.entered) / self.soft_start_s)), old != 'TRACKING'
        elapsed = np.inf if self.last_seen is None else now - self.last_seen
        self.state = 'LOST' if elapsed < self.lost_reset_s else 'HOLD'
        return self.state, float(np.exp(-elapsed / self.lost_decay_s)), (
            self.state == 'HOLD' and old != 'HOLD')
