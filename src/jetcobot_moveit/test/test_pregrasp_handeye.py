"""Check new hand-eye results against the existing, independent single-tag target."""

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
from handeye_solve import as_transform
from pregrasp_move import (Pregrasp, apriltag_spec, base_tag_pose,
                          camera_model_matches, checked_path, command_waypoint_indices,
                          follow_waypoint, next_command_index, JOINTS)


class CommandPathTest(unittest.TestCase):
    def path(self, degrees):
        values = np.zeros((len(degrees), 6))
        values[:, 3] = np.radians(degrees)
        return values

    def test_dense_straight_points_merge_without_changing_geometry(self):
        path = self.path(np.arange(0., 10.01, .5))
        indices = command_waypoint_indices(path)
        self.assertEqual(indices, [0, 6, 12, 18, 20])
        self.assertLessEqual(np.max(np.abs(np.degrees(np.diff(path[indices], axis=0)))), 3.+1e-8)

    def test_every_turn_and_reversal_is_preserved(self):
        path = np.zeros((7, 6))
        path[:, 0] = np.radians([0., .5, 1., 1., 1., .5, 0.])
        path[:, 1] = np.radians([0., 0., 0., .5, 1., 1., 1.])
        self.assertEqual(command_waypoint_indices(path), [0, 2, 4, 6])
        reversal = self.path([0., .5, 1., .5, 0.])
        self.assertEqual(command_waypoint_indices(reversal), [0, 2, 4])

    def test_curved_segments_are_not_shortcut(self):
        path = self.path([0., .5, 1., 1.5, 2.])
        path[:, 0] = np.radians([0., .01, .04, .09, .16])
        self.assertEqual(command_waypoint_indices(path), [0, 1, 2, 3, 4])

    def test_actual_lag_splits_next_command_before_four_degree_guard(self):
        path = self.path(np.arange(0., 10.01, .5))
        indices = command_waypoint_indices(path)
        current = path[6].copy(); current[3] -= np.radians(1.5)
        index = next_command_index(path, indices, 6, current)
        self.assertEqual(index, 10)
        self.assertLess(np.max(np.abs(np.degrees(path[index]-current))), 4.)

    def test_no_forward_command_when_feedback_is_too_far_from_path(self):
        path = self.path([0., .5, 1.])
        current = path[0].copy();current[0] = np.radians(5.)
        with self.assertRaisesRegex(RuntimeError, 'No forward path command'):
            next_command_index(path, command_waypoint_indices(path), 0, current)


class WaypointFeedbackTest(unittest.TestCase):
    def run_waypoint(self, feedback, moving, timeout=4.):
        elapsed = [0.]
        def pause(seconds):
            elapsed[0] += seconds
        robot = Mock()
        robot.is_moving.side_effect = lambda: moving(robot.send_angles.call_count, elapsed[0])
        diagnostics = {}
        result = follow_waypoint(
            robot, lambda: np.radians(feedback(robot.send_angles.call_count, elapsed[0])),
            lambda: None, np.zeros(6), 3, speed=5, timeout=timeout,
            clock=lambda: elapsed[0], pause=pause, diagnostics=diagnostics)
        return robot, diagnostics, result, elapsed[0]

    def test_does_not_advance_while_controller_still_moving_inside_tolerance(self):
        _, _, (_, error), elapsed = self.run_waypoint(
            lambda count, t: [.5] * 6, lambda count, t: int(t < .8))
        self.assertGreaterEqual(elapsed, .9)
        self.assertAlmostEqual(error, .5)

    def test_stopped_short_move_retries_same_command_and_requires_original_tolerance(self):
        robot, diagnostics, (_, error), _ = self.run_waypoint(
            lambda count, t: [0., 0., 0., 2.13 if count == 1 else .8, 0., 0.],
            lambda count, t: 0)
        self.assertEqual(robot.send_angles.call_count, 2)
        self.assertEqual(robot.send_angles.call_args_list[0], robot.send_angles.call_args_list[1])
        self.assertEqual(diagnostics['retry_count'], 1)
        self.assertLess(error, 2.)

    def test_permanent_stall_does_not_skip_waypoint_or_relax_two_degree_tolerance(self):
        with self.assertRaisesRegex(RuntimeError, 'J4.*error=2.13.*retries=2'):
            self.run_waypoint(lambda count, t: [0., 0., 0., 2.13, 0., 0.], lambda count, t: 0)

    def test_four_degree_tracking_limit_still_aborts_immediately(self):
        with self.assertRaisesRegex(RuntimeError, 'exceeds 4 degrees'):
            self.run_waypoint(lambda count, t: [0., 0., 0., 4.1, 0., 0.], lambda count, t: 0)

    def test_invalid_motion_feedback_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'invalid controller motion state'):
            self.run_waypoint(lambda count, t: [0.] * 6, lambda count, t: -1)

    def test_unstable_feedback_is_not_accepted_as_stopped(self):
        with self.assertRaisesRegex(RuntimeError, 'feedback timeout'):
            self.run_waypoint(lambda count, t: [(-1.) ** int(round(t*10))] * 6,
                              lambda count, t: 0)


class PregraspHandeyeTest(unittest.TestCase):
    def setUp(self):
        self.calibration = {
            'target_type': 'charuco', 'board': {'marker_length_m': .021},
            'T_gripper_camera': as_transform(
                cv2.Rodrigues(np.array([.1, -.4, .2]))[0], [.0404, -.0485, -.0032]).tolist(),
            'base_frame': 'dummy', 'parent_frame': '6_Link',
            'child_frame': 'camera_optical_frame',
            # Deliberately unrelated: a fresh target must not use this origin.
            'T_base_board': as_transform(np.eye(3), [9., 9., 9.]).tolist(),
            'camera_model': {'width': 640, 'height': 480,
                             'K': [800., 0., 320., 0., 800., 240., 0., 0., 1.],
                             'D': [0.] * 5, 'distortion_model': 'plumb_bob'},
        }

    def info(self):
        model = self.calibration['camera_model']
        info = CameraInfo()
        info.width, info.height = model['width'], model['height']
        info.header.frame_id = self.calibration['child_frame']
        info.k, info.d = model['K'], model['D']
        info.distortion_model = model['distortion_model']
        return info

    def test_charuco_calibration_uses_independent_20mm_apriltag(self):
        spec = apriltag_spec(self.calibration)
        self.assertEqual(spec['tag_id'], 1)
        self.assertEqual(spec['marker_length_m'], .020)
        self.assertEqual(apriltag_spec(self.calibration, 7, .035)['marker_length_m'], .035)
        old = {'tag_id': 3, 'marker_length_m': .027}
        self.assertEqual(apriltag_spec(old)['tag_id'], 3)
        self.assertEqual(apriltag_spec(old)['marker_length_m'], .027)
        for size in (0., -.01, np.nan):
            with self.assertRaises(ValueError):
                apriltag_spec(self.calibration, marker_length_m=size)

    def test_camera_changes_rejected(self):
        info = self.info()
        self.assertTrue(camera_model_matches(info, self.calibration))
        info.k[0] += 1.
        self.assertFalse(camera_model_matches(info, self.calibration))
        info = self.info(); info.width = 1280
        self.assertFalse(camera_model_matches(info, self.calibration))

    def test_base_point_uses_fresh_pose_and_correct_transform_direction(self):
        g = as_transform(cv2.Rodrigues(np.array([-.3, .1, .2]))[0], [.1, .03, .2])
        x = np.array(self.calibration['T_gripper_camera'])
        c = as_transform(cv2.Rodrigues(np.array([.4, -.1, .6]))[0], [-.02, .01, .3])
        result = base_tag_pose(g, x, cv2.Rodrigues(c[:3, :3])[0], c[:3, 3])
        np.testing.assert_allclose(result, g @ x @ c, atol=1e-12)

    def test_path_rejects_joint6_outside_hardware_range(self):
        bad = np.zeros((2, 6)); bad[:, 5] = -1.5
        with self.assertRaisesRegex(ValueError, 'hardware limits'):
            checked_path(JOINTS, bad)

    def test_failed_new_observation_invalidates_old_plan(self):
        rclpy.init()
        node = Pregrasp()
        try:
            with tempfile.TemporaryDirectory() as directory:
                calibration = Path(directory) / 'calibration.json'
                plan = Path(directory) / 'plan.json'
                calibration.write_text(json.dumps(self.calibration))
                plan.write_text(json.dumps({'status': 'planned', 'path_validated': True,
                                            'path_radians': [[0.] * 6, [.1] * 6]}))
                args = SimpleNamespace(calibration=calibration, plan=plan,
                                       tag_id=1, marker_length=.02)
                with patch.object(node, 'observe', side_effect=RuntimeError('No live tag')):
                    with self.assertRaisesRegex(RuntimeError, 'No live tag'):
                        node.plan(args)
                result = json.loads(plan.read_text())
                self.assertFalse(result['path_validated'])
                self.assertNotIn('path_radians', result)
                with self.assertRaisesRegex(RuntimeError, 'not passed'):
                    node.execute(args)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_execution_checks_actual_feedback_fk_before_reporting_reached(self):
        rclpy.init()
        node = Pregrasp()
        try:
            for x, expected in ((.003, 'reached'), (.025, 'stopped')):
                with self.subTest(position_error_m=x), tempfile.TemporaryDirectory() as directory:
                    plan = Path(directory) / 'plan.json'
                    start, goal = np.zeros(6), np.zeros(6)
                    goal[3] = np.radians(3.)
                    plan.write_text(json.dumps({
                        'status': 'planned', 'path_validated': True, 'motion_sent': False,
                        'planned_at_sec': node.get_clock().now().nanoseconds / 1e9,
                        'joint_names': JOINTS, 'path_radians': [start.tolist(), goal.tolist()],
                        'base_frame': 'dummy', 'link': 'jiazhua_Link',
                        'target_position_m': [0., 0., 0.],
                    }))
                    args = SimpleNamespace(plan=plan, port='/dev/synthetic', speed=10)
                    robot = Mock()
                    response = SimpleNamespace(error_code=SimpleNamespace(val=1),
                        pose_stamped=[SimpleNamespace(pose=SimpleNamespace(
                            position=SimpleNamespace(x=x, y=0., z=0.)))])
                    def follow(_robot, _read, _spin, target, _index, **_kwargs):
                        return target.copy(), 0.
                    with patch('pymycobot.mycobot.MyCobot', return_value=robot), \
                            patch('pregrasp_move.subprocess.run', return_value=SimpleNamespace(returncode=1)), \
                            patch('pregrasp_move.follow_waypoint', side_effect=follow), \
                            patch.object(node, 'read_robot', return_value=start), \
                            patch.object(node, 'create_client', return_value=Mock()), \
                            patch.object(node, 'wait', return_value=response):
                        if expected == 'stopped':
                            with self.assertRaisesRegex(RuntimeError, 'position error 25.00 mm'):
                                node.execute(args)
                            robot.stop.assert_called_once()
                        else:
                            node.execute(args)
                            robot.stop.assert_not_called()
                    report = json.loads(plan.read_text())
                    self.assertEqual(report['execution_status'], expected)
                    self.assertAlmostEqual(report['final_feedback_fk_position_error_mm'], x*1000.)
                    robot._serial_port.close.assert_called_once()
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_real_single_tag_observation_with_charuco_handeye(self):
        image = np.full((480, 640), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        image[200:280, 280:360] = cv2.aruco.generateImageMarker(dictionary, 1, 80)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        rclpy.init()
        node = Pregrasp()
        try:
            now = node.get_clock().now().nanoseconds - 300000000
            counter = 0
            def provide_frame(*_args, **_kwargs):
                nonlocal counter
                stamp = rclpy.time.Time(nanoseconds=now + counter * 1000000).to_msg()
                counter += 1
                msg = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
                msg.header.stamp = stamp
                msg.header.frame_id = 'camera_optical_frame'
                joints = JointState(); joints.header.stamp = stamp
                joints.name, joints.position = JOINTS, [0.] * 6
                node.latest.update(image=msg, info=self.info(), joints=joints)
                tf = TransformStamped(); tf.header.stamp = stamp
                tf.header.frame_id, tf.child_frame_id = 'dummy', '6_Link'
                tf.transform.rotation.w = 1.
                node.buffer.set_transform(tf, 'synthetic-test')
            with patch('pregrasp_move.rclpy.spin_once', side_effect=provide_frame):
                center, _, _, spread = node.observe(self.calibration, apriltag_spec(self.calibration))
            x = np.array(self.calibration['T_gripper_camera'])
            expected = (x @ np.array([0., 0., .2, 1.]))[:3]
            np.testing.assert_allclose(center, expected, atol=.001)
            self.assertLess(spread, .001)
            self.assertEqual(node.observation_report['sample_count'], 10)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
