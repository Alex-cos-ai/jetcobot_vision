"""Final hardware gate, usable with a fake SDK in tests."""

import numpy as np

from .trajectory_guard import hardware_degrees, model_radians


class ArmController:
    """Keep all motion writes behind an explicit, fault-latching mode gate."""

    def __init__(self, robot, model, guard, config, mode='preview'):
        """Do not write any SDK command while constructing the controller."""
        if mode not in ('preview', 'execute'):
            raise ValueError('mode must be preview or execute')
        self.robot, self.model, self.guard = robot, model, guard
        self.config, self.mode = config, mode
        self.current = None
        self.feedback_time = None
        self.enabled = self.faulted = self.commanding = False
        self.fault_reason = None
        self.enabled_at = 0.
        self.last_sent = None
        self.accepted_ns = self.sent_ns = 0
        self.reason = 'waiting_for_feedback'
        self.previous_velocity = np.zeros(6)
        self.previous_generation_ns = 0
        self.previous_target = None
        self.feedback_failures = self.consecutive_feedback_failures = 0
        self.filtered_speed_fraction = 0.
        self.sent_speed = 1
        self.speed_changed_at = 0.

    def feedback(self, now):
        """Read six actual angles; failed reads never refresh feedback age."""
        try:
            value = self.robot.get_angles()
            if value is None:
                raise ValueError('no reply to get_angles')
            current = model_radians(value)
        except Exception as exc:
            self.feedback_failures += 1
            self.consecutive_feedback_failures += 1
            reason = f'feedback_fault:{exc}'
            age = float('inf') if self.feedback_time is None else now-self.feedback_time
            if age <= self.config['serial']['timeout_s'] and self.consecutive_feedback_failures < 3:
                self.pause(reason, now)
            else:
                self.stop(reason, latch=self.enabled)
            return False
        self.current, self.feedback_time = current, now
        self.consecutive_feedback_failures = 0
        return True

    def stop(self, reason, latch=False):
        """Stop only a previously enabled real executor; preview stays read-only."""
        self.faulted |= latch
        if latch and self.fault_reason is None:
            self.fault_reason = reason
        stop_error = None
        if self.mode == 'execute' and (self.enabled or self.commanding):
            try:
                self.robot.stop()
            except Exception as exc:
                # 停止调用失败也必须锁定；不能因finally先清enabled而丢掉故障。
                self.faulted = True
                stop_error = str(exc)
            finally:
                self.enabled = self.commanding = False
        self.reason = reason if stop_error is None else f'{reason};stop_failed:{stop_error}'
        self.last_sent = None
        self.previous_velocity[:] = 0.
        self.previous_generation_ns = 0
        self.previous_target = None
        self.filtered_speed_fraction = 0.
        self.sent_speed = 1
        self.speed_changed_at = 0.

    def enable(self, now):
        """Enable once external freshness checks passed; require a later target."""
        if self.mode != 'execute' or self.faulted or not self.guard.execution_ready:
            return False
        try:
            self.robot.set_fresh_mode(1)
            if self.robot.get_fresh_mode() != 1:
                raise RuntimeError('fresh mode not confirmed')
        except Exception as exc:
            self.stop(f'fresh_mode_fault:{exc}', latch=True)
            return False
        self.enabled, self.enabled_at = True, now
        self.reason = 'enabled_waiting_for_new_target'
        return True

    def pause(self, reason, now):
        """Temporarily stop on a vision gap; later fresh observations may soft-start again."""
        if self.mode == 'execute' and self.commanding:
            try:
                self.robot.stop()
            except Exception as exc:
                self.stop(f'pause_stop_failed:{exc}', latch=True)
                return
        self.commanding = False
        self.enabled_at = now
        self.previous_velocity[:] = 0.
        # 保留最后的命令位置，避免视觉短暂停顿后用下垂的编码器读数重新锚定。
        # 同时保留生成时间，恢复后的命令变化率按整个暂停间隔计算。
        self.reason = reason

    def accept(self, target, velocity, horizon, generation_ns, received, now):
        """Validate the exact goal, then optionally send a small SDK step."""
        c = self.config['serial']
        if self.current is None or self.feedback_time is None:
            raise ValueError('feedback_unavailable')
        if now - self.feedback_time > c['timeout_s']:
            raise ValueError('feedback_stale')
        target, velocity = np.asarray(target), np.asarray(velocity)
        if target.shape != (6,) or velocity.shape != (6,) or not np.all(np.isfinite(np.r_[target, velocity])):
            raise ValueError('invalid_target')
        if not .01 <= horizon <= .5:
            raise ValueError('invalid_command_horizon')
        delta = target - self.current
        if np.max(np.abs(delta)) > c['max_step_rad'] + 1.e-8:
            raise ValueError('target_feedback_gap')
        vmax = np.asarray(self.config['model']['max_velocity_rad_s'])
        if np.any(np.abs(velocity) > vmax + 1.e-6):
            raise ValueError('joint_velocity_limit')
        velocity_dt = horizon if not self.previous_generation_ns else (
            generation_ns - self.previous_generation_ns) / 1.e9
        if velocity_dt <= 0:
            raise ValueError('nonincreasing_target_time')
        reference = self.current if self.previous_target is None else self.previous_target
        active = (np.abs(velocity) > 1.e-9) & ((reference - self.current) * velocity >= 0.)
        if np.any(np.abs(target - reference)[active] > vmax[active] * velocity_dt + 1.e-5):
            raise ValueError(f'joint_command_rate_limit:dt={velocity_dt:.4f};'
                             f'change_deg={np.degrees(target-reference).round(4).tolist()}')
        amax = np.asarray(self.config['model']['max_acceleration_rad_s2'])
        if np.any(np.abs(velocity - self.previous_velocity) > amax * min(velocity_dt, .35) + 1.e-5):
            raise ValueError('joint_acceleration_limit')
        ok, reason = self.guard.validate_step(self.current, target)
        if not ok:
            raise ValueError(reason)
        self.accepted_ns = generation_ns
        self.previous_velocity = velocity.copy()
        self.previous_generation_ns = generation_ns
        self.previous_target = target.copy()
        if self.mode == 'preview':
            self.reason = 'preview_target_accepted'
            return False
        if not self.enabled or self.faulted or received <= self.enabled_at:
            return False
        # 只在新目标通过检查后写入。SDK speed 是等级，不能当角速度。
        if np.max(np.abs(delta)) < .001 or np.max(np.abs(velocity)) < 1.e-9:
            if self.commanding:
                self.robot.stop()
            self.commanding = False
            self.reason = 'holding'
            return False
        fraction = float(np.max(np.abs(velocity) / vmax))
        if self.last_sent is None:
            self.speed_changed_at = now
        tau = c.get('speed_filter_tau_s', .25)
        self.filtered_speed_fraction += (min(velocity_dt, .35)/(tau+min(velocity_dt, .35)) *
                                         (fraction-self.filtered_speed_fraction))
        desired_speed = max(1, min(c['speed_max'], int(np.ceil(c['speed_max'] * self.filtered_speed_fraction))))
        if now-self.speed_changed_at >= c.get('speed_change_interval_s', .25):
            self.sent_speed += int(np.sign(desired_speed-self.sent_speed))
            self.sent_speed = max(1, min(c['speed_max'], self.sent_speed))
            self.speed_changed_at = now
        speed = self.sent_speed
        self.robot.send_angles(hardware_degrees(target), speed)
        self.commanding, self.sent_ns = True, generation_ns
        self.last_sent, self.reason = target.copy(), 'executing'
        return True
