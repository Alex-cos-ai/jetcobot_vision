"""Bound the command lead over measured joints to overcome serial-servo deadband."""

import numpy as np


class JointCommandIntegrator:
    """Integrate short velocity steps while retaining a finite following-error budget."""

    def __init__(self):
        self.target = None

    def reset(self):
        """Discard the reference on loss, restart, or a rejected command."""
        self.target = None

    def step(self, current, velocity, dt, max_gap):
        """Keep a continuous command reference; never follow encoder noise or gravity sag."""
        current, velocity = np.asarray(current), np.asarray(velocity)
        reference = current.copy() if self.target is None else self.target.copy()
        delta = velocity*dt
        # 统一缩放，避免一个关节跟随受限时其他关节继续累积、破坏末端运动方向。
        active = np.abs(delta) > 1.e-12
        budget = np.where(delta >= 0, current+max_gap-reference, current-max_gap-reference)
        scale = min(1., max(0., float(np.min(budget[active]/delta[active])))) if np.any(active) else 0.
        target = reference + delta*scale
        self.target = target.copy()
        return target
