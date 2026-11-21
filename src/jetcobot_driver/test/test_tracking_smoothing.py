"""Regressions for unnecessary stops and discontinuities in streaming tracking."""

import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import rclpy

from visual_servo_6dof.arm_controller import ArmController
from visual_servo_6dof.kinematics import Kinematics
from visual_servo_6dof.safety import SafetyGuard
from visual_servo_6dof.tracking_config import load_config
from visual_servo_6dof.tracking_pid import PIDChannel, TrackingPID
from visual_servo_6dof.trajectory_guard import hardware_degrees
from visual_servo_6dof.visual_servo_6dof import Detection, VisualServo6Dof


def test_pid_deadband_has_no_proportional_step_at_boundary():
    channel = PIDChannel(.4, 0., 0., .003, .1)
    inside = channel.update(.003, .003, .05, .01)
    outside = channel.update(.003001, .003001, .05, .01)
    assert inside == 0.
    assert 0 < outside < 1.e-6
    assert np.isclose(channel.update(-.003001, -.003001, .05, .01), -outside)


def test_settling_hysteresis_does_not_restart_on_edge_noise():
    config = load_config()
    config['pid']['input_tau_s'] = config['pid']['depth_tau_s'] = 1.e-8
    pid = TrackingPID(config, 960., [0, 0])
    radius = 960.*np.sqrt(2)*.03/(2*.2)
    def observe(x_mm, t):
        return pid.observe(320+x_mm/1000.*960/.2, 240., radius, t, [320, 240])
    assert observe(2., 1.) and pid.settled
    for i, error in enumerate([3.2, 2.9, 3.5, 4.4, 3.1]):
        assert observe(error, 1.05+i*.05) and pid.settled
        assert np.linalg.norm(pid.tick(.05)) == 0.
    assert observe(4.6, 1.4) and not pid.settled
    assert pid.tick(.05)[0] > 0.


def test_abnormal_timer_dt_does_not_fake_target_loss_or_clear_state():
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)),
                           last_tick=time.monotonic(),
                           config={'runtime': {'dt_range_s': [.005, .2]}})
    # 无需其他节点状态：异常dt应在调用状态机或PID之前跳过。
    VisualServo6Dof.control_callback(node)


def test_reset_can_reuse_latest_fresh_detection_without_waiting_for_new_frame():
    rclpy.init(args=[])
    node = VisualServo6Dof()
    try:
        node.backend = 'weighted_dls'
        node.config['ik']['manipulability_gain'] = 0.
        node.joints = np.array(node.config['simulation']['initial_angles_rad'])
        node.info = SimpleNamespace(width=640, height=480)
        node.pid = TrackingPID(node.config, 960., [0, 0])
        node.rect_k = np.array([[960., 0., 320.], [0., 960., 240.], [0., 0., 1.]])
        node.joint_stamp = node.get_clock().now().nanoseconds-30_000_000
        node.image_stamp = node.joint_stamp-30_000_000
        node.joint_time = node.last_image = time.monotonic()
        node.last_tick = time.monotonic()-.05
        node.found = True
        node.last_detection = Detection(8000., 'blue', 370., 240., 100., (320, 190, 100, 100), False)
        assert not node.pending and node.pid.stamp is None
        node.control_callback()
        assert node.pid.stamp == node.image_stamp/1.e9
        assert node.machine.state == 'TRACKING'
        assert node.reason in ('OK', 'projected')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_sdk_speed_level_changes_gradually_without_stop_between_streamed_steps():
    config = load_config()
    config['model']['max_velocity_rad_s'] = [.1]*6
    config['model']['max_acceleration_rad_s2'] = [1.]*6
    config['serial']['speed_max'] = 5
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'])
    initial = np.array(config['simulation']['initial_angles_rad'])
    class Robot:
        def __init__(self):
            self.q, self.writes = initial.copy(), []
        def get_angles(self):
            return hardware_degrees(self.q)
        def send_angles(self, angles, speed):
            self.writes.append(('send', speed))
        def stop(self):
            self.writes.append(('stop',))
    robot = Robot()
    arm = ArmController(robot, model, SafetyGuard(model, config), config, 'execute')
    arm.enabled = True
    speeds, change_times = [], []
    for i in range(1, 21):
        now = i*.1
        arm.feedback(now)
        target = initial.copy()
        target[0] += i*.005
        arm.accept(target, np.array([.05, 0, 0, 0, 0, 0]), .1, i*100000000, now, now)
        robot.q = target.copy()
        speeds.append(arm.sent_speed)
        if i == 1 or speeds[-1] != speeds[-2]:
            change_times.append(now)
    assert all(w[0] == 'send' for w in robot.writes)
    assert np.max(np.abs(np.diff(speeds))) <= 1
    assert np.all(np.diff(change_times) >= .25-1.e-6)
    assert 2 <= speeds[-1] <= 5
