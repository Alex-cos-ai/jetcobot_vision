"""Exercise real ChArUco detection, geometry, and independent validation."""

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
from charuco_board import BoardSpec, CharucoPoseEstimator
import handeye_solve as solver


class CharucoPoseTest(unittest.TestCase):
    def setUp(self):
        self.spec = BoardSpec()
        self.estimator = CharucoPoseEstimator(self.spec)
        self.matrix = np.array([[1100., 0, 640], [0, 1100., 480], [0, 0, 1]])
        self.distortion = np.zeros(5)
        self.rvec = np.array([0.25, -0.35, 0.08])
        self.tvec = np.array([-0.07, -0.098, 0.55])
        self.objects = self.estimator.board.getChessboardCorners()
        self.ids = np.arange(len(self.objects), dtype=np.int32).reshape(-1, 1)
        self.markers = self.estimator.board.getIds()
        self.pixels = cv2.projectPoints(
            self.objects, self.rvec, self.tvec, self.matrix, self.distortion)[0]

    def estimate(self, pixels=None, ids=None):
        return self.estimator.estimate(
            self.pixels if pixels is None else pixels,
            self.ids if ids is None else ids, self.markers,
            self.matrix, self.distortion, (1280, 960))

    def test_recovers_metric_pose_and_direction(self):
        pose = self.estimate()
        np.testing.assert_allclose(pose['tvec'].reshape(3), self.tvec, atol=1e-6)
        np.testing.assert_allclose(pose['R_target2cam'], cv2.Rodrigues(self.rvec)[0], atol=1e-5)
        self.assertEqual(pose['quality']['inlier_count'], 24)

    def test_detects_perspective_rendered_board(self):
        board_image = self.estimator.board.generateImage((1000, 1400))
        outer = np.array([[0, 0, 0], [.14, 0, 0], [.14, .196, 0], [0, .196, 0]])
        destination = cv2.projectPoints(
            outer, self.rvec, self.tvec, self.matrix, self.distortion)[0].reshape(4, 2)
        homography = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [999, 0], [999, 1399], [0, 1399]]),
            destination.astype(np.float32))
        image = cv2.warpPerspective(board_image, homography, (1280, 960), borderValue=255)
        corners, ids, _, markers = self.estimator.detect_corners(
            image, self.matrix, self.distortion)
        pose = self.estimator.estimate(corners, ids, markers,
                                       self.matrix, self.distortion, (1280, 960))
        self.assertGreaterEqual(pose['quality']['inlier_count'], 20)
        np.testing.assert_allclose(pose['tvec'].reshape(3), self.tvec, atol=.001)
        error = cv2.Rodrigues(cv2.Rodrigues(self.rvec)[0].T @ pose['R_target2cam'])[0]
        self.assertLess(np.linalg.norm(error), np.radians(.5))

    def test_rejects_clustered_corners_and_gross_outliers(self):
        with self.assertRaisesRegex(ValueError, '覆盖率'):
            self.estimate(self.pixels[:8], self.ids[:8])
        bad = self.pixels.copy()
        bad[:6, 0] += np.array([40., -35.])
        with self.assertRaisesRegex(ValueError, '内点不足'):
            self.estimate(bad)

    def test_partial_board_with_distributed_corners_is_usable(self):
        keep = np.array([0, 1, 2, 3, 4, 7, 8, 11, 12, 15, 20, 21, 22, 23])
        pose = self.estimate(self.pixels[keep], self.ids[keep])
        np.testing.assert_allclose(pose['tvec'].reshape(3), self.tvec, atol=1e-6)

    def test_wrong_physical_size_scales_depth(self):
        larger = CharucoPoseEstimator(BoardSpec(square_length_m=.030, marker_length_m=.0225))
        pose = larger.estimate(self.pixels, self.ids, self.markers,
                              self.matrix, self.distortion, (1280, 960))
        np.testing.assert_allclose(pose['tvec'].reshape(3), self.tvec * 30 / 28, atol=1e-6)


class CharucoCalibrationTest(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(923)
        self.spec = BoardSpec()
        self.handeye = solver.as_transform(cv2.Rodrigues(np.array([.2, -.4, .1]))[0], [.03, -.02, .04])
        self.fixed_board = solver.as_transform(np.eye(3), [.25, -.1, .02])
        self.gripper = np.array([solver.as_transform(
            cv2.Rodrigues(rng.normal(0, .5, 3))[0], rng.uniform(-.2, .3, 3)) for _ in range(30)])
        self.target = np.linalg.inv(self.handeye) @ np.linalg.inv(self.gripper) @ self.fixed_board
        self.data = {
            'target_type': 'charuco', 'board': self.spec.metadata(),
            'camera_model': {'width': 1280, 'height': 960,
                             'K': [1100., 0, 640, 0, 1100., 480, 0, 0, 1],
                             'D': [0.] * 5, 'distortion_model': 'plumb_bob'},
            'samples': [{
                'base_frame': 'dummy', 'gripper_frame': '6_Link',
                'camera_frame': 'camera_optical_frame', 'target_signature': self.spec.signature(),
                'stamp': {'sec': 1000 + i, 'nanosec': 0},
                'R_gripper2base': g[:3, :3].tolist(), 't_gripper2base': g[:3, 3].tolist(),
                'R_target2cam': c[:3, :3].tolist(), 't_target2cam': c[:3, 3].tolist(),
            } for i, (g, c) in enumerate(zip(self.gripper, self.target))],
        }
        training = dict(self.data, samples=self.data['samples'][:24])
        self.result = solver.calibrate(training, self.gripper[:24], self.target[:24], 'PARK')
        self.fresh = dict(self.data, samples=self.data['samples'][24:])

    def test_recovery_and_fresh_validation(self):
        np.testing.assert_allclose(self.result['T_gripper_camera'], self.handeye, atol=1e-9)
        np.testing.assert_allclose(self.result['T_base_board'], self.fixed_board, atol=1e-9)
        report = solver.validate(self.fresh, self.gripper[24:], self.target[24:], self.result)
        self.assertLess(report['metrics']['translation_rms_mm'], 1e-7)
        self.assertEqual(report['board'], self.spec.metadata())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'samples.json'
            path.write_text(json.dumps(self.data), encoding='utf-8')
            _, gripper, target = solver.load_samples(path)
            np.testing.assert_allclose(gripper, self.gripper)
            np.testing.assert_allclose(target, self.target)

    def test_rejects_changed_board_intrinsics_and_sample_identity(self):
        for key in ('board', 'camera_model'):
            wrong = copy.deepcopy(self.result)
            wrong[key] = {}
            with self.assertRaisesRegex(ValueError, key):
                solver.validate(self.fresh, self.gripper[24:], self.target[24:], wrong)
        wrong = copy.deepcopy(self.fresh)
        wrong['samples'][0]['target_signature'] = 'another-board'
        with self.assertRaisesRegex(ValueError, '签名'):
            solver.validate(wrong, self.gripper[24:], self.target[24:], self.result)

    def test_old_single_tag_transform_comparison_does_not_refit(self):
        baseline = {key: self.result[key] for key in (
            'T_gripper_camera', 'base_frame', 'parent_frame', 'child_frame')}
        baseline['T_gripper_camera'] = copy.deepcopy(baseline['T_gripper_camera'])
        baseline['T_gripper_camera'][0][3] += .02
        report = solver.compare_calibration(self.fresh, self.gripper[24:], self.target[24:], baseline)
        self.assertGreater(report['batch_scatter']['translation_rms_mm'], 1.)
        self.assertEqual(report['T_gripper_camera'], baseline['T_gripper_camera'])


if __name__ == '__main__':
    unittest.main()
