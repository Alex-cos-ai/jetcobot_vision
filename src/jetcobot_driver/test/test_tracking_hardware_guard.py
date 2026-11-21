"""Offline regressions for mesh clearance and serial-servo command continuity."""

import json
from pathlib import Path

import numpy as np

from visual_servo_6dof import _fcl_guard
from visual_servo_6dof.arm_controller import ArmController
from visual_servo_6dof.joint_command import JointCommandIntegrator
from visual_servo_6dof.kinematics import Kinematics
from visual_servo_6dof.mesh_guard import MeshGuard
from visual_servo_6dof.safety import SafetyGuard
from visual_servo_6dof.tracking_config import load_config
from visual_servo_6dof.trajectory_guard import hardware_degrees


def test_fcl_detects_separation_overlap_and_containment():
    """GJK must treat a solid contained inside another solid as a collision."""
    a = _fcl_guard.make_box(.1, .1, .1)
    b = _fcl_guard.make_box(.02, .02, .02)
    matrices = np.stack([np.eye(4), np.eye(4)])
    matrices[1, 0, 3] = .2
    assert np.isclose(_fcl_guard.distances([a, b], matrices.tobytes(), [(0, 1)])[0], .14)
    matrices[1, 0, 3] = .05
    assert _fcl_guard.distances([a, b], matrices.tobytes(), [(0, 1)])[0] <= 0
    matrices[1, 0, 3] = 0.
    assert _fcl_guard.distances([a, b], matrices.tobytes(), [(0, 1)])[0] <= 0


def test_full_mesh_table_clearance_and_config_change():
    """The lowest mesh vertex protects the entire moving body, including the gripper."""
    config = load_config()
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'])
    guard = MeshGuard(model, config)
    q = np.radians([2.28, 1.31, -.26, -4.48, 5.27, 1.85])
    gaps, names = guard.constraints(q)
    assert np.min(gaps) > 0
    assert 'mesh_table_jiazhua_Link' in names and 'mesh_table_camera_link' in names
    config['safety']['table_z_m'] = .4
    gaps, names = guard.constraints(q)
    assert gaps[names.index('mesh_table_jiazhua_Link')] < 0


def test_hardware_and_model_joint_limits_intersect_without_urdf_changes():
    """The SDK J1 limit is narrower than the official model, even before margin."""
    config = load_config()
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'], np.radians(5),
                       config['model']['hardware_limits_deg'])
    assert np.isclose(np.degrees(model.limits[0, 1]), 163.)
    assert np.isclose(np.degrees(model.limits[5, 1]), np.degrees(1.57) - 5.)


def test_small_steps_cross_deadband_but_never_accumulate_past_feedback_budget():
    """A stuck joint can acquire at most the configured 1.72-degree command lead."""
    integrator = JointCommandIntegrator()
    current = np.zeros(6)
    velocity = np.array([.02, 0., 0., 0., 0., 0.])
    targets = [integrator.step(current, velocity, .1, .03) for _ in range(100)]
    assert np.isclose(targets[-1][0], .03)
    assert np.max(np.abs(np.diff(targets, axis=0))) <= .002000001
    reverse = integrator.step(current, -velocity, .1, .03)
    assert np.isclose(reverse[0], .028)
    assert np.array_equal(integrator.step(current, np.zeros(6), .1, .03), reverse)
    # 编码器越过参考值时也不将参考重置为编码器读数。
    current[0] = .04
    following = integrator.step(current, velocity, .1, .03)
    assert np.isclose(following[0], .03)
    integrator.reset()
    assert integrator.target is None


def test_executor_accepts_bounded_lead_and_rejects_command_jumps():
    """Rate validation follows successive commands, while feedback-gap protection stays active."""
    config = load_config()
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'])
    q = np.array(config['simulation']['initial_angles_rad'])
    class Robot:
        def get_angles(self):
            return hardware_degrees(q)
    arm = ArmController(Robot(), model, SafetyGuard(model, config), config)
    velocity = np.array([.01, 0, 0, 0, 0, 0])
    integrator = JointCommandIntegrator()
    for i in range(1, 25):
        arm.feedback(i*.1)
        target = integrator.step(q, velocity, .1, .03)
        arm.accept(target, velocity, .1, i*100000000, i*.1, i*.1)
    assert arm.accepted_ns == 2400000000
    import pytest
    with pytest.raises(ValueError, match='joint_command_rate_limit'):
        arm.accept(q + np.array([.055, 0, 0, 0, 0, 0]), velocity, .1, 2500000000, 2.5, 2.5)


def test_partial_target_moves_toward_visible_edge_without_false_depth_control():
    """A cropped circle may guide XY search; only a full face can set range error."""
    from visual_servo_6dof.tracking_pid import TrackingPID
    config = load_config()
    pid = TrackingPID(config, 960., [0., 0.])
    assert pid.observe(620, 350, 30, 1., [320, 240], partial=True)
    for _ in range(10):
        velocity = pid.tick(.05)
    assert velocity[0] > 0 and velocity[1] > 0 and velocity[2] == 0
    assert pid.depth is None and pid.error[2] == 0
    radius = 960.*np.sqrt(2)*.03/(2*.3)
    assert pid.observe(330, 240, radius, 1.05, [320, 240], partial=False)
    assert abs(pid.depth-.3) < 1.e-10
    assert pid.tick(.05)[2] > 0


def test_quad_tracking_center_is_diagonal_intersection_not_circle_center():
    """Perspective makes a square's projected centre differ from enclosing-circle centre."""
    import cv2
    from visual_servo_6dof.visual_servo_6dof import detect_target
    image = np.zeros((480, 640, 3), np.uint8)
    vertices = np.array([[200, 140], [380, 170], [350, 320], [220, 290]])
    cv2.fillConvexPoly(image, vertices, (255, 0, 0))
    detection = detect_target(image, 'blue')
    a = np.column_stack((vertices[2]-vertices[0], -(vertices[3]-vertices[1])))
    alpha = np.linalg.solve(a, vertices[1]-vertices[0])[0]
    expected = vertices[0]+alpha*(vertices[2]-vertices[0])
    assert np.linalg.norm(np.array([detection.cx, detection.cy])-expected) < 3.
    assert np.linalg.norm(np.array(detection.circle_center)-expected) > 3.


def test_one_missing_feedback_pauses_but_stale_feedback_latches():
    """Neither a single missing packet nor a later successful read can refresh the old sample time."""
    config = load_config()
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'])
    q = np.array(config['simulation']['initial_angles_rad'])
    class Robot:
        def get_angles(self):
            return hardware_degrees(q)
        def stop(self):
            pass
    robot = Robot()
    arm = ArmController(robot, model, SafetyGuard(model, config), config, 'execute')
    assert arm.feedback(1.)
    arm.enabled = arm.commanding = True
    robot.get_angles = lambda: None
    assert not arm.feedback(1.1)
    assert arm.enabled and not arm.commanding and not arm.faulted
    assert arm.feedback_time == 1.
    assert not arm.feedback(1.5)
    assert arm.faulted and not arm.enabled and arm.feedback_time == 1.


def test_partial_aim_holds_camera_position_and_rotates_toward_target():
    """The view-recovery task must not request downward translation to find a bottom-edge square."""
    from visual_servo_6dof.kinematics import DifferentialIK
    config = load_config()
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'])
    q = np.radians([.52, -40.69, -29.26, -1.58, .61, 40.61])
    pose = model.fk(q)
    angular = pose.camera[:3, :3] @ [-.02, .01, 0.]
    velocity = DifferentialIK(model, config).solve(q, np.zeros(3), pose.camera[:3, :3], .1,
                                                  aim_angular=angular)
    jacobian = model.jacobian(q)
    assert np.linalg.norm(jacobian[:3] @ velocity) < 1.e-8
    actual = pose.camera[:3, :3].T @ jacobian[3:] @ velocity
    assert np.allclose(actual[:2], [-.02, .01], atol=1.e-5)


def test_following_error_scales_all_joint_commands_together():
    """A lagging shoulder must not leave other axes integrating independently."""
    integrator = JointCommandIntegrator()
    integrator.target = np.array([.029, .01, 0, 0, 0, 0])
    old = integrator.target.copy()
    result = integrator.step(np.zeros(6), np.array([.02, .01, 0, 0, 0, 0]), .1, .03)
    assert np.allclose(result[:2]-old[:2], [.001, .0005])


def test_pause_preserves_command_and_elapsed_time_for_rate_checks():
    """Resume uses the actual interval between accepted commands, including a visual pause."""
    config = load_config()
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'])
    q = np.array(config['simulation']['initial_angles_rad'])
    class Robot:
        def get_angles(self):
            return hardware_degrees(q)
    arm = ArmController(Robot(), model, SafetyGuard(model, config), config)
    velocity = np.array([.01, 0, 0, 0, 0, 0])
    arm.feedback(1.)
    arm.accept(q+.1*velocity, velocity, .1, 1000000000, 1., 1.)
    target = arm.previous_target.copy()
    arm.pause('waiting_for_fresh_target', 1.1)
    assert np.array_equal(arm.previous_target, target)
    assert arm.previous_generation_ns == 1000000000
    arm.feedback(1.6)
    arm.accept(target+.6*velocity, velocity, .1, 1600000000, 1.6, 1.6)
    assert arm.accepted_ns == 1600000000


def test_executor_matches_delayed_target_with_its_original_metadata():
    """A newer planner state arriving before its trajectory must not invalidate the previous pair."""
    import rclpy
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from visual_servo_6dof.tracking_protocol import encode
    from visual_servo_6dof.trajectory_guard import JOINT_NAMES
    from jetcobot_driver.sync_plan import SyncExecutor
    config = load_config()
    q = np.array(config['simulation']['initial_angles_rad'])
    class Robot:
        def get_angles(self):
            return hardware_degrees(q)
    rclpy.init(args=[])
    node = SyncExecutor(config, 'preview', robot=Robot())
    try:
        generation = node.get_clock().now().nanoseconds
        node.heartbeat_callback(String(data=encode(dict(stamp_ns=generation, session='pair-test', alive=True))))
        for target_ns in (generation, generation+1000000):
            node.planner_callback(String(data=encode(dict(stamp_ns=generation, session='pair-test',
                healthy=True, state='TRACKING', backend='weighted_dls', feedback_source='hardware',
                target_ns=target_ns, config_digest=config['_digest'], execution_ready=False))))
        message = JointTrajectory()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(generation, 1000000000)
        message.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint(positions=(q+.001).tolist(), velocities=[.01]*6)
        point.time_from_start.nanosec = 100000000
        message.points = [point]
        node.target_callback(message)
        node.tick()
        assert node.arm.accepted_ns == generation
        assert not node.arm.faulted
        # 主进程心跳仍新鲜，但规划状态生成时间过期：停止等待，不接受旧目标。
        node.arm.enabled = True
        node.planner = dict(node.planner, stamp_ns=generation-1000000000)
        node.tick()
        assert node.arm.enabled and not node.arm.faulted
        assert node.latest is None and node.arm.reason == 'stale_planner_metadata'
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


def test_executor_pause_signal_restarts_pid_without_discarding_joint_reference():
    """Both sides soft-start together after a visual pause, preserving the bounded angle reference."""
    import rclpy
    from std_msgs.msg import String
    from visual_servo_6dof.tracking_protocol import encode
    from visual_servo_6dof.tracking_pid import TrackingPID
    from visual_servo_6dof.visual_servo_6dof import VisualServo6Dof
    rclpy.init(args=[])
    node = VisualServo6Dof()
    try:
        node.pid = TrackingPID(node.config, 960., [0., 0.])
        assert node.pid.observe(420, 300, 90., 1., [320, 240])
        node.command_integrator.target = np.ones(6)*.01
        node.guard.previous_velocity[:] = .02
        node.machine.state = 'TRACKING'
        node.sync_state = dict(mode='execute', enabled=True, reason='executing')
        node.sync_callback(String(data=encode(dict(mode='execute', enabled=True,
                                                  reason='waiting_for_joint_acceleration'))))
        assert node.pid.stamp == 1.0  # 暂停不能丢掉刚收到的有效测量。
        assert np.all(node.guard.previous_velocity == 0)
        assert node.machine.state == 'TRACKING'
        assert np.array_equal(node.pid.previous, np.zeros(3))
        assert np.allclose(node.command_integrator.target, .01)
    finally:
        node.destroy_node()
        rclpy.shutdown()
