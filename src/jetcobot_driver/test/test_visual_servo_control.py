"""Check the physical meaning and execution boundaries of visual tracking."""

import cv2
import numpy as np
import pytest
from collections import deque
from scipy.spatial.transform import Rotation
from types import SimpleNamespace

from visual_servo_6dof.control import (
    aiming_axes, aiming_error, camera_to_tool_twist, lqr_gain,
)
from visual_servo_6dof.trajectory_guard import (
    checked_target, hardware_degrees, JOINT_NAMES, model_radians,
)
from visual_servo_6dof.visual_servo_6dof import detect_target, VisualServo6Dof


def test_explicit_blue_selection_ignores_a_larger_red_object():
    """Selecting blue must ignore the larger red background object."""
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 30), (390, 350), (0, 0, 255), -1)
    cv2.rectangle(image, (455, 325), (620, 465), (255, 0, 0), -1)
    assert detect_target(image, 'auto')[1] == 'red'
    blue = detect_target(image, 'blue')
    assert blue is not None
    assert blue[1] == 'blue'
    assert blue[2] > 455


@pytest.mark.parametrize('age', [-.02, .36])
def test_invalid_image_timestamp_clears_previous_tracking_data(age):
    """An old or future frame must discard all targets waiting for TF."""
    node = SimpleNamespace(
        info=SimpleNamespace(width=640, height=480),
        optical_frame='camera_optical_frame',
        p=lambda name: .35,
        pending=deque([('previous_target', 'previous_stamp')]),
        observation='previous_target', filtered_error=np.ones(3),
        get_logger=lambda: SimpleNamespace(warning=lambda *a, **k: None))
    image = SimpleNamespace(width=640, height=480,
                            header=SimpleNamespace(frame_id='camera_optical_frame'))
    VisualServo6Dof.queue_detection(node, (1000, 'blue', 400, 250, 140), image, None, age)
    assert node.status == 'stale_image'
    assert not node.pending
    assert node.observation is None
    assert node.filtered_error is None


@pytest.mark.parametrize('target', [[.28, .21, .78], [-.17, -.12, .3]])
def test_camera_motion_centres_the_target_and_regulates_range(target):
    """Move the camera around a fixed target and check its new projection."""
    center_ray = np.array([-.0375, .051, 1.])
    axes = aiming_axes(center_ray)
    gain = lqr_gain(.05, [4., 4., 1.], [4., 4., 16.])
    point = np.array(target)
    initial_offset = np.linalg.norm(point[:2] / point[2] - center_ray[:2])
    for _ in range(400):
        distance = np.linalg.norm(point)
        ray = point / distance
        turn_x, turn_y, speed = -gain @ aiming_error(center_ray, ray, axes, distance - .45)
        angular = axes @ [turn_x, turn_y]
        point = Rotation.from_rotvec(-.05 * angular).apply(point - .05 * speed * ray)
    offset = np.linalg.norm(point[:2] / point[2] - center_ray[:2])
    assert offset < initial_offset * 1.e-4
    assert abs(np.linalg.norm(point) - .45) < .004


def test_twist_origin_shift_preserves_the_camera_velocity():
    """The hand-eye translation matters when angular velocity is nonzero."""
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', [.2, -.6, .4]).as_matrix()
    transform[:3, 3] = [.04, -.05, -.003]
    camera = np.array([.02, -.01, .03, .04, .02, -.03])
    tool = camera_to_tool_twist(camera, transform)
    recovered = transform[:3, :3].T @ (
        tool[:3] + np.cross(tool[3:], transform[:3, 3]))
    assert np.allclose(recovered, camera[:3])


def test_correct_size_off_center_calls_for_rotation_about_the_camera():
    """When range is correct, turn toward the object without approaching it."""
    center_ray = np.array([0., 0., 1.])
    axes = aiming_axes(center_ray)
    gain = lqr_gain(.05, [4., 4., 1.], [4., 4., 16.])
    aim = -gain @ aiming_error(center_ray, [.2, .1, 1.], axes, 0.)
    angular = axes @ aim[:2]
    assert angular[0] < 0.  # Down in an optical frame.
    assert angular[1] > 0.  # Right in an optical frame.
    assert np.isclose(aim[2], 0.)


def test_joint_names_are_reordered_and_offset_round_trips():
    """Serial output must invert the joint-six feedback offset exactly."""
    current = np.array([0, .4, -.8, .2, 0, .3])
    target = current + .01
    ordered = checked_target(JOINT_NAMES[::-1], target[::-1], current, .035)
    assert np.allclose(ordered, target)
    assert np.allclose(model_radians(hardware_degrees(target)), target)


@pytest.mark.parametrize(
    'kind', ['duplicate', 'missing', 'nan', 'jump', 'limit'])
def test_invalid_targets_cannot_reach_the_sdk(kind):
    """Reject invalid and distant targets before serial output."""
    names = JOINT_NAMES.copy()
    current = np.zeros(6)
    target = current.copy()
    if kind == 'duplicate':
        names[0] = names[1]
    elif kind == 'missing':
        names.pop()
    elif kind == 'nan':
        target[0] = np.nan
    elif kind == 'jump':
        target[1] = .1
    elif kind == 'limit':
        current[1] = target[1] = 1.7
    with pytest.raises(ValueError):
        checked_target(names, target, current, .035)
