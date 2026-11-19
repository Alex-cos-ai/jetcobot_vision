"""ROS callback checks with synthetic images/TF; no robot is connected."""

import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'script'))
from charuco_handeye_collect import CharucoHandEyeCollector


class CharucoCollectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.path = Path(cls.directory.name) / 'samples.json'
        rclpy.init(args=['--ros-args', '-p', f'output_file:={cls.path}'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        cls.directory.cleanup()

    def setUp(self):
        self.path.unlink(missing_ok=True)
        self.node = CharucoHandEyeCollector()

    def tearDown(self):
        self.node.destroy_node()

    def fill_window(self):
        now = self.node.get_clock().now().nanoseconds
        for index in range(6):
            self.node.window.append({
                'stamp_ns': now - 450000000 + 80000000 * index,
                'gripper': np.eye(4), 'target': np.eye(4),
                'quality': {'reprojection_rms_px': .2 - .01 * index},
            })

    def test_stale_or_moving_robot_and_duplicate_pose_are_rejected(self):
        self.fill_window()
        sample = self.node.make_sample()
        self.node.samples = [sample]
        with self.assertRaisesRegex(ValueError, '太接近'):
            self.node.make_sample()
        self.node.samples = []
        self.node.window[-1]['gripper'][0, 3] = .002
        with self.assertRaisesRegex(ValueError, 'gripper 尚不稳定'):
            self.node.make_sample()
        self.node.window[-1]['gripper'][0, 3] = 0
        for frame in self.node.window:
            frame['stamp_ns'] -= 2000000000
        with self.assertRaisesRegex(ValueError, '不够新'):
            self.node.make_sample()

    def test_invalid_camera_info_clears_previous_valid_window(self):
        self.fill_window()
        self.node.info_callback(CameraInfo())
        self.assertEqual(len(self.node.window), 0)
        response = self.node.save_callback(Trigger.Request(), Trigger.Response())
        self.assertFalse(response.success)

    def test_image_tf_save_and_resume_end_to_end(self):
        info = CameraInfo()
        info.header.frame_id = 'camera_optical_frame'
        info.width, info.height = 1280, 960
        info.distortion_model = 'plumb_bob'
        info.k = [1100., 0., 640., 0., 1100., 480., 0., 0., 1.]
        info.d = [0.] * 5
        self.node.info_callback(info)
        matrix = np.array(info.k).reshape(3, 3)
        outer = np.array([[0., 0, 0], [.14, 0, 0], [.14, .196, 0], [0, .196, 0]])
        pixels = cv2.projectPoints(outer, np.array([.25, -.35, .08]),
                                   np.array([-.07, -.098, .55]), matrix, np.zeros(5))[0]
        homography = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [999, 0], [999, 1399], [0, 1399]]),
            pixels.reshape(4, 2).astype(np.float32))
        image = cv2.warpPerspective(self.node.estimator.board.generateImage((1000, 1400)),
                                    homography, (1280, 960), borderValue=255)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        # Allow processing time while keeping all exposure stamps in the past.
        start = self.node.get_clock().now().nanoseconds - 500000000
        for index in range(6):
            stamp = rclpy.time.Time(nanoseconds=start + index * 80000000).to_msg()
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id, tf.child_frame_id = 'dummy', '6_Link'
            tf.transform.rotation.w = 1.
            self.node.buffer.set_transform(tf, 'synthetic-test')
            msg = self.node.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.stamp, msg.header.frame_id = stamp, info.header.frame_id
            self.node.image_callback(msg)
        response = self.node.save_callback(Trigger.Request(), Trigger.Response())
        self.assertTrue(response.success, response.message)
        data = json.loads(self.path.read_text(encoding='utf-8'))
        self.assertEqual(len(data['samples']), 1)
        self.assertEqual(data['samples'][0]['target_signature'], self.node.spec.signature())
        np.testing.assert_allclose(data['samples'][0]['t_target2cam'], [-.07, -.098, .55], atol=.001)
        resumed = CharucoHandEyeCollector()
        try:
            self.assertEqual(len(resumed.samples), 1)
            self.assertEqual(resumed.camera_model, self.node.camera_model)
        finally:
            resumed.destroy_node()


if __name__ == '__main__':
    unittest.main()
