"""Exercise the serial motion gate using a fake SDK, without hardware I/O."""

from types import MethodType, SimpleNamespace
import time

import numpy as np
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from visual_servo_6dof.servo_serial_bridge import ServoSerialBridge
from visual_servo_6dof.trajectory_guard import JOINT_NAMES, model_radians


class FakeRobot:
    """Record SDK writes so tests never open or move real hardware."""

    def __init__(self):
        """Start with a stationary set of measured hardware angles."""
        self.sent = []
        self.stops = 0

    def get_angles(self):
        """Return actual-feedback stand-ins, never commanded angles."""
        return [0., 20., -60., -20., 0., -45.]

    def send_angles(self, values, speed):
        """Record a hardware command."""
        self.sent.append((values, speed))

    def stop(self):
        """Record a stop request."""
        self.stops += 1

    def set_fresh_mode(self, value):
        """Accept the replacement mode used by streaming commands."""
        self.mode = value

    def get_fresh_mode(self):
        """Confirm the last mode setting."""
        return self.mode


def make_bridge():
    """Bind execution logic to an SDK double with no ROS publishers."""
    robot = FakeRobot()
    bridge = SimpleNamespace(
        robot=robot, enabled=False, was_commanding=False,
        latest=None, current=None, enabled_at=0.,
        feedback_time=None, feedback_error='waiting for first serial reading',
        timeout=.35, max_step=.035, speed=10,
        servo_status=0, servo_status_time=time.monotonic(), statuses=[],
        get_publishers_info_by_topic=lambda topic: [],
        get_name=lambda: 'servo_serial_bridge',
        get_logger=lambda: SimpleNamespace(error=lambda *a, **k: None,
                                           info=lambda *a, **k: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(time.time() * 1.e9))),
        joint_publisher=SimpleNamespace(publish=lambda message: None),
    )
    bridge.publish_status = bridge.statuses.append
    bridge.halt = MethodType(ServoSerialBridge.halt, bridge)
    return bridge


def fresh_target(bridge, delta=.01):
    """Build one nearby Servo point in the model's joint convention."""
    trajectory = JointTrajectory()
    trajectory.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = (
        model_radians(bridge.robot.get_angles()) + delta).tolist()
    trajectory.points = [point]
    return trajectory, time.monotonic()


def test_feedback_only_never_sends_motion():
    """An available trajectory cannot bypass the disabled motion gate."""
    bridge = make_bridge()
    bridge.latest = fresh_target(bridge)
    ServoSerialBridge.tick(bridge)
    assert bridge.current is not None
    assert bridge.robot.sent == []
    assert bridge.robot.stops == 0


def test_feedback_loss_blocks_enable_even_when_servo_is_ready():
    """Reject a missing serial reply and recover feedback without re-enabling motion."""
    bridge = make_bridge()
    ServoSerialBridge.tick(bridge)
    request = SetBool.Request(data=True)
    assert ServoSerialBridge.enable_callback(bridge, request, SetBool.Response()).success
    read_angles = bridge.robot.get_angles
    bridge.robot.get_angles = lambda: None
    ServoSerialBridge.tick(bridge)
    response = ServoSerialBridge.enable_callback(bridge, request, SetBool.Response())
    assert not response.success
    assert 'no reply to get_angles()' in response.message
    assert bridge.robot.stops == 1
    bridge.robot.get_angles = read_angles
    ServoSerialBridge.tick(bridge)
    assert bridge.current is not None
    assert bridge.statuses[-1] == 'feedback_only_motion_disabled'
    assert bridge.robot.sent == []


def test_enable_rejects_old_feedback():
    """Require a recent hardware reading before writing any mode command."""
    bridge = make_bridge()
    ServoSerialBridge.tick(bridge)
    bridge.feedback_time -= 1.
    response = ServoSerialBridge.enable_callback(
        bridge, SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    assert 'stale' in response.message
    assert not hasattr(bridge.robot, 'mode')


def test_enable_requires_a_new_trajectory_and_applies_the_joint_six_offset():
    """Discard pre-enable targets, then execute a fresh nearby target."""
    bridge = make_bridge()
    ServoSerialBridge.tick(bridge)
    bridge.latest = fresh_target(bridge)
    response = ServoSerialBridge.enable_callback(
        bridge, SetBool.Request(data=True), SetBool.Response())
    assert response.success
    assert bridge.latest is None
    bridge.latest = fresh_target(bridge)
    ServoSerialBridge.tick(bridge)
    command, speed = bridge.robot.sent[0]
    assert np.isclose(command[5], -45. + np.degrees(.01))
    assert speed == 10


def test_command_timeout_stops_and_disables_execution():
    """A stopped publisher must stop the arm and clear the last goal."""
    bridge = make_bridge()
    bridge.enabled = True
    target, received = fresh_target(bridge)
    bridge.latest = target, received - 1.
    ServoSerialBridge.tick(bridge)
    assert bridge.robot.sent == []
    assert bridge.robot.stops == 1
    assert not bridge.enabled


def test_singularity_or_bad_point_stops_and_disables_execution():
    """Servo protection and oversized steps must both prevent SDK writes."""
    for status, delta in [(2, .01), (0, .2)]:
        bridge = make_bridge()
        bridge.enabled = True
        bridge.servo_status = status
        bridge.latest = fresh_target(bridge, delta)
        ServoSerialBridge.tick(bridge)
        assert bridge.robot.sent == []
        assert bridge.robot.stops == 1
        assert not bridge.enabled
