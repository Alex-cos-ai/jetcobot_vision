"""Numerical checks for transform direction and fresh-pose validation."""

import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / 'script' / 'handeye_solve.py'
SPEC = importlib.util.spec_from_file_location('handeye_solve', SCRIPT)
solver = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(solver)


class HandEyeSolveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rng = np.random.default_rng(71)
        cls.expected = solver.as_transform(
            cv2.Rodrigues(np.array([0.3, -0.7, 0.2]))[0], [0.04, -0.03, 0.02])
        cls.fixed_tag = solver.as_transform(
            cv2.Rodrigues(np.array([-0.3, 0.1, 0.5]))[0], [0.5, 0.1, 0.03])
        cls.gripper, cls.target = cls.make_poses(25, cls.fixed_tag)
        cls.data = cls.make_data(cls.gripper, cls.target, 100)
        cls.result = solver.calibrate(cls.data, cls.gripper, cls.target, 'PARK')

    @classmethod
    def make_poses(cls, count, fixed_tag):
        gripper = np.array([
            solver.as_transform(cv2.Rodrigues(cls.rng.normal(0, 0.6, 3))[0],
                                cls.rng.uniform(-0.2, 0.3, 3))
            for _ in range(count)
        ])
        target = np.linalg.inv(cls.expected) @ np.linalg.inv(gripper) @ fixed_tag
        return gripper, target

    @staticmethod
    def make_data(gripper, target, first_second):
        return {'marker_length_m': 0.02, 'samples': [
            {'base_frame': 'dummy', 'gripper_frame': '6_Link',
             'camera_frame': 'camera_optical_frame', 'tag_id': 1,
             'stamp': {'sec': first_second + i, 'nanosec': 0},
             'R_gripper2base': g[:3, :3].tolist(),
             't_gripper2base': g[:3, 3].tolist(),
             'R_target2cam': t[:3, :3].tolist(),
             't_target2cam': t[:3, 3].tolist()}
            for i, (g, t) in enumerate(zip(gripper, target))]}

    def test_recovers_camera_to_gripper_direction(self):
        np.testing.assert_allclose(
            self.result['T_gripper_camera'], self.expected, atol=1e-9)
        self.assertLess(self.result['methods']['PARK']['heldout'][
            'translation_rms_mm'], 1e-7)

    def test_new_poses_pass_without_refitting(self):
        g, t = self.make_poses(6, self.fixed_tag)
        report = solver.validate(self.make_data(g, t, 200), g, t, self.result)
        self.assertLess(report['metrics']['translation_rms_mm'], 1e-7)

    def test_shifted_tag_is_measured_against_original_reference(self):
        shifted = self.fixed_tag.copy()
        shifted[0, 3] += 0.01
        g, t = self.make_poses(6, shifted)
        report = solver.validate(self.make_data(g, t, 300), g, t, self.result)
        self.assertAlmostEqual(report['metrics']['translation_rms_mm'], 10.0, 6)
        self.assertAlmostEqual(report['reference_shift']['translation_norm_mm'], 10.0, 6)
        np.testing.assert_allclose(report['reference_shift']['translation_xyz_mm'],
                                   [10.0, 0.0, 0.0], atol=1e-7)
        self.assertLess(report['batch_scatter']['translation_rms_mm'], 1e-7)

    def test_reused_samples_rejected(self):
        with self.assertRaisesRegex(ValueError, '相同'):
            solver.validate(self.data, self.gripper, self.target, self.result)
        with self.assertRaisesRegex(ValueError, '时间戳'):
            solver.validate(self.make_data(self.gripper[:1], self.target[:1], 100),
                            self.gripper[:1], self.target[:1], self.result)

    def test_previous_batch_reference_after_base_moved(self):
        shifted = self.fixed_tag.copy()
        shifted[0, 3] += 0.15
        g1, t1 = self.make_poses(6, shifted)
        report = solver.validate(self.make_data(g1, t1, 500), g1, t1, self.result)
        g2, t2 = self.make_poses(6, shifted)
        checked = solver.validate(self.make_data(g2, t2, 600), g2, t2,
                                  self.result, report)
        self.assertLess(checked['metrics']['translation_rms_mm'], 1e-7)
        self.assertEqual(checked['reference_source'], 'previous_validation_batch')
        with self.assertRaisesRegex(ValueError, '重叠'):
            solver.validate(self.make_data(g1, t1, 500), g1, t1, self.result, report)
        report['T_gripper_camera'][0][3] += 0.01
        with self.assertRaisesRegex(ValueError, '手眼变换'):
            solver.validate(self.make_data(g2, t2, 600), g2, t2, self.result, report)

    def test_frame_mismatch_rejected(self):
        g, t = self.make_poses(1, self.fixed_tag)
        data = self.make_data(g, t, 400)
        data['samples'][0]['camera_frame'] = 'camera_link'
        with self.assertRaisesRegex(ValueError, 'camera_frame'):
            solver.validate(data, g, t, self.result)

    def test_invalid_rotation_and_nonfinite_values_rejected(self):
        for rotation, translation in (
                (np.eye(3) * 2, [0, 0, 0]), (np.eye(3), [np.nan, 0, 0])):
            with self.assertRaises(ValueError):
                solver.as_transform(rotation, translation)


if __name__ == '__main__':
    unittest.main()
