"""Numerical and execution regressions for colour PID tracking; no hardware I/O."""

import json
from pathlib import Path
import time

import cv2
import numpy as np
import pytest

from visual_servo_6dof.arm_controller import ArmController
from visual_servo_6dof.kinematics import DifferentialIK, Kinematics, check_sdk_fk, sdk_pose_transform
from visual_servo_6dof.safety import SafetyGuard, project_halfspaces, segment_distance
from visual_servo_6dof.tracking_config import load_config
from visual_servo_6dof.tracking_pid import PIDChannel, TrackingPID, TrackingStateMachine
from visual_servo_6dof.trajectory_guard import hardware_degrees
from visual_servo_6dof.visual_servo_6dof import detect_target


@pytest.fixture
def config():
    """Load editable project configuration."""
    result = load_config()
    # 测试明确控制验证门控，不依赖现场调试配置目前是否已验证。
    result['model']['verified'] = result['safety']['verified'] = False
    return result


@pytest.fixture
def model(config):
    """Use the same URDF and hand-eye matrix as runtime."""
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    return Kinematics(config['model']['urdf'], calibration['T_gripper_camera'],
                      np.radians(config['model']['margin_deg']))


def test_circle_is_not_half_bounding_box():
    """A square's enclosing diameter is its diagonal."""
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(image, (280, 180), (340, 240), (255, 0, 0), -1)
    detection = detect_target(image, 'blue')
    assert abs(detection.radius - 30 * np.sqrt(2)) < .1


@pytest.mark.parametrize('depth', [.15, .2, .3, .5])
def test_physical_square_circle_depth(config, depth):
    """Known-size projected corners recover depth in consistent rectified pixels."""
    f = 960.
    principal = np.array([356., 191.])
    half_side = config['camera']['square_side_mm'] / 2000.
    points = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * half_side * f / depth + principal
    (cx, cy), radius = cv2.minEnclosingCircle(points.astype(np.float32))
    pid = TrackingPID(config, f, [0, 0])
    assert pid.observe(cx, cy, radius, 1., principal)
    assert abs(pid.depth - depth) < 1.e-5


def test_pid_antiwindup_and_measurement_derivative():
    """Saturated I does not wind up; decreasing depth supplies braking D."""
    channel = PIDChannel(1., 1., .2, .001, .1)
    for _ in range(100):
        channel.update(1., 1., .05, .01, .05)
    assert channel.integral == 0.
    channel.reset()
    channel.update(.1, .3, .05, 1., .05)
    value = channel.update(.09, .29, .05, 1., .05)
    assert value < .09 + channel.integral
    previous = channel.derivative
    channel.update(.09, .29, .05, 1., None)
    assert channel.derivative == previous
    channel.integral = .1
    channel.update(0., .2, .05, 1., .05)
    assert channel.integral < .1
    channel.reset()
    assert channel.integral == channel.derivative == channel.output == 0.


def test_pid_variable_dt_acceleration_and_forward_coupling(config):
    """Actual dt bounds velocity changes and compensates off-principal reference."""
    pid = TrackingPID(config, 960., [-.0375, .05])
    radius = 960. * np.sqrt(2.) * .03 / (2 * .3)
    assert pid.observe(320, 240, radius, 1., [356, 192])
    previous = np.zeros(3)
    for dt in [.02, .08, .04, .1]:
        v = pid.tick(dt)
        assert np.linalg.norm(v - previous) <= config['pid']['max_acceleration_m_s2'] * dt + 1.e-9
        assert np.linalg.norm(v) <= config['pid']['max_speed_m_s'] + 1.e-9
        previous = v
    assert v[2] > 0 and v[0] < 0 and v[1] > 0


def test_lost_holds_and_reacquires_without_terminating():
    """Convergence/loss is a state transition, not program completion."""
    machine = TrackingStateMachine()
    assert machine.step(True, True, 1.) == ('TRACKING', 0., True)
    assert machine.step(False, True, 1.1)[0] == 'LOST'
    assert machine.step(False, True, 2.)[0] == 'HOLD'
    assert machine.step(True, True, 3.) == ('TRACKING', 0., True)
    assert machine.step(True, False, 3.1)[0] == 'HOLD'


@pytest.mark.parametrize('q', [np.zeros(6), np.array([0, .45, -1.25, -.3, .1, 0])])
def test_camera_jacobian_matches_fk(model, q):
    """Camera offset must be included in the linear Jacobian."""
    h = 1.e-6
    numerical = np.column_stack([
        (model.fk(q + np.eye(6)[i] * h).camera[:3, 3] -
         model.fk(q - np.eye(6)[i] * h).camera[:3, 3]) / (2 * h)
        for i in range(6)])
    assert np.allclose(model.jacobian(q)[:3], numerical, atol=1.e-7)


@pytest.mark.parametrize('q', [np.zeros(6), np.array([0, .45, -1.25, -.3, .1, 0])])
def test_dls_finite_at_singular_pose_and_bounded(model, config, q):
    """No blow-up or axis overspeed even at a full-pose singularity."""
    ik = DifferentialIK(model, config)
    result = ik.solve(q, np.array([.005, -.003, .002]), model.fk(q).camera[:3, :3], .1, True)
    assert np.all(np.isfinite(result))
    assert np.all(np.abs(result) <= np.array(config['model']['max_velocity_rad_s']) + 1.e-9)


def test_nullspace_secondary_preserves_position(model, config):
    """Changing only secondary preferences cannot change a rank-three position task."""
    q = np.array(config['simulation']['initial_angles_rad'])
    jacobian = model.jacobian(q)[:3]
    result = DifferentialIK(model, config).solve(q, np.zeros(3), model.fk(q).camera[:3, :3], .1)
    assert np.linalg.norm(jacobian @ result) < 1.e-10


def test_intersecting_boundary_projection():
    """A second projection must not violate a previously active wall."""
    a = np.array([[-1., 0., 0.], [-1., -1., 0.]])
    b = np.zeros(2)
    v = project_halfspaces(np.array([1., 2., 3.]), a, b)
    assert v is not None and np.all(a @ v >= b - 1.e-7)
    assert np.isclose(v[2], 3.)


def test_segment_distance_handles_crossing_and_points():
    """Capsules must detect crossings and zero-length link geometry."""
    assert segment_distance(np.array([-1., 0, 0]), np.array([1., 0, 0]),
                            np.array([0., -1, 0]), np.array([0., 1, 0])) < 1.e-12
    assert np.isclose(segment_distance(np.zeros(3), np.zeros(3), np.ones(3), np.ones(3)), np.sqrt(3))


def test_safety_detects_table_and_smooths_velocity(model, config):
    """Protect link geometry, final targets and acceleration together."""
    guard = SafetyGuard(model, config)
    q = np.array(config['simulation']['initial_angles_rad'])
    assert guard.valid(q)[0]
    velocity = guard.filter_velocity(q, np.ones(6), .1)
    assert np.all(np.abs(velocity) <= .03 + 1.e-6)
    assert guard.validate_step(q, q + velocity * .1)[0]
    config['safety']['table_z_m'] = .4
    assert not guard.valid(q)[0]
    assert np.all(guard.filter_velocity(q, velocity, .1) == 0.)


def test_invalid_initial_pose_rejects_motion(model, config):
    """A preexisting violation cannot authorize an unplanned escape step."""
    guard = SafetyGuard(model, config)
    q = np.zeros(6)
    assert not guard.valid(q)[0]
    assert np.array_equal(guard.filter_velocity(q, np.ones(6) * .01, .1), np.zeros(6))


class FakeRobot:
    """SDK test double records every possible motion/mode write."""

    def __init__(self, q):
        """Start from model angles but return SDK degrees."""
        self.q, self.writes = q.copy(), []

    def get_angles(self):
        """Return actual-feedback stand-ins."""
        return hardware_degrees(self.q)

    def send_angles(self, values, speed):
        """Record a motion call."""
        self.writes.append(('send', values, speed))

    def stop(self):
        """Record a stop call."""
        self.writes.append(('stop',))

    def set_fresh_mode(self, value):
        """Record a mode write."""
        self.writes.append(('mode', value))

    def get_fresh_mode(self):
        """Confirm replacement mode."""
        return 1


def test_preview_never_writes_during_tracking_and_exit(model, config):
    """Even enable, valid goals, invalid goals and shutdown keep preview read-only."""
    q = np.array(config['simulation']['initial_angles_rad'])
    robot = FakeRobot(q)
    arm = ArmController(robot, model, SafetyGuard(model, config), config)
    assert arm.feedback(1.)
    assert not arm.enable(1.)
    for i in range(1, 6):
        arm.feedback(1. + .1 * i)
        assert not arm.accept(q + .001, np.full(6, .01), .1, i * 100000000, 1. + .1 * i, 1. + .1 * i)
    with pytest.raises(ValueError):
        arm.accept(q + 1, np.ones(6), .1, 700000000, 1.7, 1.7)
    arm.stop('target_lost')
    arm.stop('shutdown')
    assert robot.writes == []


def test_execute_requires_verified_model_and_new_target(model, config):
    """A mode flag alone is insufficient; pre-enable targets are never replayed."""
    q = np.array(config['simulation']['initial_angles_rad'])
    robot = FakeRobot(q)
    guard = SafetyGuard(model, config)
    arm = ArmController(robot, model, guard, config, 'execute')
    arm.feedback(1.)
    assert not arm.enable(1.) and not robot.writes
    config['model']['verified'] = config['safety']['verified'] = True
    config['fk_check']['alignment_verified'] = True
    config['safety']['table_z_m'] = -1.
    assert arm.enable(1.)
    arm.accept(q + .001, np.full(6, .01), .1, 100000000, .9, 1.)
    assert not any(w[0] == 'send' for w in robot.writes)
    arm.accept(q + .001, np.full(6, .01), .1, 200000000, 1.1, 1.1)
    assert any(w[0] == 'send' for w in robot.writes)
    arm.stop('heartbeat_expired', latch=True)
    assert arm.faulted and not arm.enabled
    assert not arm.enable(1.2)


def test_feedback_failure_never_refreshes_age(model, config):
    """Retain the old age and prevent a stale measurement authorizing output."""
    robot = FakeRobot(np.array(config['simulation']['initial_angles_rad']))
    arm = ArmController(robot, model, SafetyGuard(model, config), config)
    arm.feedback(1.)
    robot.get_angles = lambda: None
    assert not arm.feedback(1.2)
    assert arm.feedback_time == 1.
    with pytest.raises(ValueError, match='feedback_stale'):
        arm.accept(arm.current, np.zeros(6), .1, 1, 2., 2.)


def test_stop_failure_still_latches_and_disables(model, config):
    """A serial exception during stop cannot accidentally clear the execution fault."""
    robot = FakeRobot(np.array(config['simulation']['initial_angles_rad']))
    arm = ArmController(robot, model, SafetyGuard(model, config), config, 'execute')
    arm.enabled = arm.commanding = True

    def fail_stop():
        raise OSError('serial disconnected')

    robot.stop = fail_stop
    arm.stop('watchdog', latch=True)
    assert arm.faulted and not arm.enabled and not arm.commanding
    assert 'stop_failed' in arm.reason
    assert not arm.enable(1.)


@pytest.mark.parametrize('mode', ['preview', 'execute'])
def test_executor_handshake_and_heartbeat_loss(config, mode):
    """Exercise the real sync node with an injected SDK; never open a serial port."""
    import rclpy
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from jetcobot_driver.sync_plan import SyncExecutor
    from visual_servo_6dof.tracking_protocol import encode
    from visual_servo_6dof.trajectory_guard import JOINT_NAMES
    rclpy.init(args=[])
    q = np.array(config['simulation']['initial_angles_rad'])
    robot = FakeRobot(q)
    if mode == 'execute':
        config['model']['verified'] = config['safety']['verified'] = True
        config['fk_check']['alignment_verified'] = True
        config['safety']['table_z_m'] = -1.
    node = SyncExecutor(config, mode, robot=robot)
    try:
        node.tick()  # no planner yet; actual feedback must already be available
        assert node.arm.current is not None and robot.writes == []
        generation = node.get_clock().now().nanoseconds
        session = 'test-session'
        node.heartbeat_callback(String(data=encode(dict(stamp_ns=generation, session=session, alive=True))))
        node.planner_callback(String(data=encode(dict(
            stamp_ns=generation, session=session, healthy=True, state='TRACKING',
            backend='weighted_dls', feedback_source='hardware', target_ns=generation,
            config_digest=config['_digest'], execution_ready=(mode == 'execute')))))
        target = JointTrajectory()
        target.header.stamp.sec, target.header.stamp.nanosec = divmod(generation, 1000000000)
        target.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint(positions=(q + .001).tolist(), velocities=[.01] * 6)
        point.time_from_start.nanosec = 100000000
        target.points = [point]
        node.target_callback(target)
        node.tick()
        assert node.arm.accepted_ns == generation
        if mode == 'preview':
            assert node.arm.reason == 'preview_target_accepted' and not robot.writes
        else:
            assert node.arm.enabled and not any(w[0] == 'send' for w in robot.writes)
            generation += 1000000
            node.planner['target_ns'] = generation
            target.header.stamp.sec, target.header.stamp.nanosec = divmod(generation, 1000000000)
            node.target_callback(target)
            node.tick()
            assert any(w[0] == 'send' for w in robot.writes)
        node.heartbeat_time -= 1.
        node.tick()
        assert node.arm.reason == 'waiting_for_main_heartbeat'
        assert node.latest is None
        if mode == 'preview':
            assert robot.writes == []
        else:
            assert not node.arm.enabled and node.arm.faulted and robot.writes[-1] == ('stop',)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


def test_sdk_pose_mapping_and_read_only_fk(model, config):
    """Frame alignment and stationary sampling are explicit; no diagnostic writes."""
    alignment = config['fk_check']
    alignment['base_from_sdk_base'] = np.eye(4).tolist()
    alignment['sdk_tool_from_model_ee'] = np.eye(4).tolist()
    alignment['alignment_verified'] = False
    pose = sdk_pose_transform([100, 200, 300, 0, 0, 90], alignment)
    assert np.allclose(pose[:3, 3], [.1, .2, .3])
    assert np.allclose(pose[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1.e-8)
    robot = FakeRobot(np.array(config['simulation']['initial_angles_rad']))
    robot.get_coords = lambda: [0., 0., 0., 0., 0., 0.]
    result = check_sdk_fk(robot, model, config)
    assert not result['alignment_verified'] and not result['ok']
    assert result['position_error_m'] > 0 and robot.writes == []
    first = robot.q.copy()
    samples = iter([hardware_degrees(first), hardware_degrees(first + .1)])
    robot.get_angles = lambda: next(samples)
    with pytest.raises(ValueError, match='stationary'):
        check_sdk_fk(robot, model, config)


def test_long_loss_releases_instance_lock_for_reacquisition(config):
    """A same-colour object reappearing far away must not leave tracking stuck forever."""
    from types import SimpleNamespace
    import rclpy
    from visual_servo_6dof.visual_servo_6dof import VisualServo6Dof, detect_target
    rclpy.init(args=[])
    node = VisualServo6Dof()
    try:
        now = time.monotonic()
        node.joints = np.array(config['simulation']['initial_angles_rad'])
        node.joint_time = node.last_image = now
        node.last_tick = now - .05
        node.info = SimpleNamespace(width=640, height=480)
        yy, xx = np.indices((480, 640), dtype=np.float32)
        node.maps = xx, yy
        node.pid = TrackingPID(config, 960., [0, 0])
        node.machine.state, node.machine.last_seen = 'LOST', now - 1.
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(image, (70, 200), (130, 260), (255, 0, 0), -1)
        node.colour, node.locked = 'blue', detect_target(image, 'blue')
        node.control_callback()
        assert node.machine.state == 'HOLD' and node.locked is None
        image[:] = 0
        cv2.rectangle(image, (470, 200), (530, 260), (255, 0, 0), -1)
        message = node.bridge.cv2_to_imgmsg(image, 'bgr8')
        message.header.stamp = node.get_clock().now().to_msg()
        node.image_callback(message)
        assert node.found and node.locked.cx > 450
    finally:
        node.destroy_node()
        rclpy.shutdown()
