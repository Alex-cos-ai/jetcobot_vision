#!/usr/bin/env python3
"""Collect stationary, timestamp-paired eye-in-hand ChArUco observations.

This node observes images and TF only; it never commands robot motion.
"""

from collections import deque
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
import tf2_ros

from charuco_board import BoardSpec, CharucoPoseEstimator
from handeye_solve import as_transform, average_pose, quaternion_xyzw


def pose_delta(a, b):
    translation = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    rotation = math.degrees(float(np.linalg.norm(
        cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])[0])))
    return translation, rotation


class CharucoHandEyeCollector(Node):
    def __init__(self):
        super().__init__('charuco_handeye_collector')
        defaults = {
            'image_topic': '/wrist_camera/image_raw',
            'camera_info_topic': '/wrist_camera/camera_info',
            'base_frame': 'dummy', 'gripper_frame': '6_Link',
            'camera_frame': 'camera_optical_frame', 'board_frame': 'base_chessboard',
            'squares_x': 5, 'squares_y': 7, 'square_length_m': 0.028,
            'marker_length_m': 0.021, 'dictionary': 'DICT_5X5_100',
            'output_file': '/home/jetson/charuco_handeye_samples.json',
            'calibration_file': '', 'min_markers': 4, 'min_corners': 8,
            'min_board_coverage': 0.20, 'min_image_coverage': 0.01,
            'max_reprojection_rms_px': 0.8,
            'max_image_age_sec': 1.0, 'stability_window_sec': 0.8,
            'min_stable_duration_sec': 0.35, 'min_stable_frames': 6,
            'max_robot_translation_m': 0.001, 'max_robot_rotation_deg': 0.5,
            'max_board_translation_m': 0.004, 'max_board_rotation_deg': 1.5,
            'min_translation_m': 0.01, 'min_rotation_deg': 5.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key in defaults:
            if isinstance(defaults[key], float):
                if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                    raise ValueError(f'{key} 必须为正有限数')
        if (self.p['min_stable_frames'] < 2 or self.p['min_stable_duration_sec']
                >= self.p['stability_window_sec']):
            raise ValueError('稳定帧数至少为 2，稳定时长须小于窗口时长')
        if (len({self.p['base_frame'], self.p['gripper_frame'], self.p['board_frame']}) < 3
                or self.p['board_frame'] == self.p['camera_frame']):
            raise ValueError('board_frame 必须独立，base_frame 与 gripper_frame 不得相同')
        self.spec = BoardSpec(**{key: self.p[key] for key in (
            'squares_x', 'squares_y', 'square_length_m', 'marker_length_m', 'dictionary')})
        self.estimator = CharucoPoseEstimator(self.spec, **{key: self.p[key] for key in (
            'min_markers', 'min_corners', 'min_board_coverage',
            'min_image_coverage', 'max_reprojection_rms_px')})
        self.output_file = Path(self.p['output_file']).expanduser()
        self.camera_model = None
        self.samples = self.load_samples()
        self.handeye = self.load_calibration()
        self.window = deque(maxlen=120)
        self.last_stamp_ns = None
        self.last_status = None
        self.last_status_time = 0.0
        self.bridge = CvBridge()
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.debug = self.create_publisher(Image, '/charuco_handeye/debug_image', 1)
        self.subs = [
            self.create_subscription(CameraInfo, self.p['camera_info_topic'],
                                     self.info_callback, qos_profile_sensor_data),
            self.create_subscription(Image, self.p['image_topic'],
                                     self.image_callback, qos_profile_sensor_data),
        ]
        self.service_handles = [
            self.create_service(Trigger, '/charuco_handeye/save_sample', self.save_callback),
            self.create_service(Trigger, '/charuco_handeye/undo_sample', self.undo_callback),
        ]
        self.get_logger().info(
            f'ChArUco {self.spec.squares_x}x{self.spec.squares_y}，'
            f'格长 {self.spec.square_length_m*1000:.2f} mm，'
            f'tag {self.spec.marker_length_m*1000:.2f} mm，{self.spec.dictionary}；'
            f'已有 {len(self.samples)} 组，输出 {self.output_file}')
        self.get_logger().info('固定标定板，手动改变末端姿态并停稳，然后调用 '
                               '/charuco_handeye/save_sample。此程序只采集，不控制运动。')

    def load_samples(self):
        if not self.output_file.exists():
            return []
        data = json.loads(self.output_file.read_text(encoding='utf-8'))
        if data.get('target_type') != 'charuco' or data.get('board') != self.spec.metadata():
            raise ValueError('已有采集文件的板参数不一致，请指定新的 output_file')
        for key in ('base_frame', 'gripper_frame', 'camera_frame'):
            if data.get(key) != self.p[key]:
                raise ValueError(f'已有采集文件的 {key} 不一致')
        self.camera_model = data.get('camera_model')
        samples = data['samples']
        for sample in samples:
            if sample.get('target_signature') != self.spec.signature():
                raise ValueError('已有样本的标定板签名不一致')
            for key in ('base_frame', 'gripper_frame', 'camera_frame'):
                if sample.get(key) != self.p[key]:
                    raise ValueError(f'已有样本的 {key} 不一致')
        if samples and self.camera_model is None:
            raise ValueError('已有文件缺少相机内参，不能继续追加')
        return samples

    def load_calibration(self):
        if not self.p['calibration_file']:
            return None
        path = Path(self.p['calibration_file']).expanduser()
        if path.resolve() == self.output_file.resolve():
            raise ValueError('output_file 不得覆盖 calibration_file')
        result = json.loads(path.read_text(encoding='utf-8'))
        for key, result_key in (('base_frame', 'base_frame'),
                                ('gripper_frame', 'parent_frame'), ('camera_frame', 'child_frame')):
            if self.p[key] != result[result_key]:
                raise ValueError(f'标定文件的 {result_key} 不一致')
        if result.get('target_type') == 'charuco' and result.get('board') != self.spec.metadata():
            raise ValueError('标定文件的板参数不一致')
        self.calibration_camera_model = result.get('camera_model')
        matrix = np.asarray(result['T_gripper_camera'], dtype=float)
        return as_transform(matrix[:3, :3], matrix[:3, 3])

    def status(self, message):
        # Error values can change every image; cap logging at once per 2 s.
        now = time.monotonic()
        if message != self.last_status and now - self.last_status_time > 2.0:
            self.get_logger().info(message)
            self.last_status, self.last_status_time = message, now

    def invalidate(self, message):
        self.window.clear()
        self.status(message)

    def info_callback(self, msg):
        matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        if (not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.isfinite(msg.d).all() or msg.width <= 0 or msg.height <= 0):
            self.invalidate('相机内参无效')
            self.active_camera_model = None
            return
        if msg.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            self.invalidate('只支持 plumb_bob / rational_polynomial 原始图像内参')
            self.active_camera_model = None
            return
        if msg.header.frame_id != self.p['camera_frame']:
            self.invalidate('camera_info frame_id 与 camera_frame 不一致')
            self.active_camera_model = None
            return
        model = {'width': int(msg.width), 'height': int(msg.height),
                 'K': list(msg.k), 'D': list(msg.d), 'distortion_model': msg.distortion_model}
        if ((self.camera_model is not None and model != self.camera_model)
                or (self.handeye is not None and self.calibration_camera_model is not None
                    and model != self.calibration_camera_model)):
            self.invalidate('相机内参已改变，请使用新的采集文件或匹配的标定结果')
            self.active_camera_model = None
            return
        self.camera_model = self.active_camera_model = model

    def image_callback(self, msg):
        if getattr(self, 'active_camera_model', None) is None:
            self.invalidate('等待匹配的有效 camera_info')
            return
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
        if stamp.nanoseconds <= 0 or not 0 <= age <= self.p['max_image_age_sec']:
            self.invalidate('图像时间戳过期或时钟不一致')
            return
        if self.last_stamp_ns is not None and stamp.nanoseconds <= self.last_stamp_ns:
            self.invalidate('图像时间戳重复或倒退')
            self.last_stamp_ns = stamp.nanoseconds
            return
        self.last_stamp_ns = stamp.nanoseconds
        model = self.camera_model
        if (msg.header.frame_id != self.p['camera_frame']
                or (msg.width, msg.height) != (model['width'], model['height'])):
            self.invalidate('图像 frame_id 或分辨率与 camera_info 不匹配')
            return
        image = None
        label = 'Waiting for board'
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            matrix = np.asarray(model['K'], dtype=float).reshape(3, 3)
            distortion = np.asarray(model['D'], dtype=float)
            corners, ids, markers, marker_ids = self.estimator.detect_corners(
                cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), matrix, distortion)
            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(image, markers, marker_ids)
            if ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(image, corners, ids)
            pose = self.estimator.estimate(
                corners, ids, marker_ids, matrix, distortion, (msg.width, msg.height))
            cv2.drawFrameAxes(image, matrix, distortion, pose['rvec'], pose['tvec'], 0.05)
            # Query the exact exposure stamp. Falling back to latest TF would
            # turn camera/robot latency into a systematic calibration error.
            tf = self.buffer.lookup_transform(
                self.p['base_frame'], self.p['gripper_frame'], stamp,
                timeout=Duration(seconds=0.08))
            q, t = tf.transform.rotation, tf.transform.translation
            gripper = as_transform(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                                   [t.x, t.y, t.z])
            target = as_transform(pose['R_target2cam'], pose['tvec'])
            self.window.append({'stamp_ns': stamp.nanoseconds, 'gripper': gripper,
                                'target': target, 'quality': pose['quality']})
            while self.window and (stamp.nanoseconds - self.window[0]['stamp_ns']) / 1e9 > self.p['stability_window_sec']:
                self.window.popleft()
            if self.handeye is not None:
                base_board = gripper @ self.handeye @ target
                output = TransformStamped()
                output.header.stamp = msg.header.stamp
                output.header.frame_id = self.p['base_frame']
                output.child_frame_id = self.p['board_frame']
                output.transform.translation.x, output.transform.translation.y, output.transform.translation.z = base_board[:3, 3].tolist()
                quaternion = quaternion_xyzw(base_board[:3, :3])
                output.transform.rotation.x, output.transform.rotation.y, output.transform.rotation.z, output.transform.rotation.w = quaternion
                self.broadcaster.sendTransform(output)
            quality = pose['quality']
            label = (f'{len(self.samples)} saved | {quality["inlier_count"]} corners | '
                     f'RMS {quality["reprojection_rms_px"]:.2f}px')
            self.status(f'整板识别有效，{len(self.window)} 帧，'
                        f'重投影 RMS={quality["reprojection_rms_px"]:.3f} px')
        except (ValueError, cv2.error, tf2_ros.TransformException) as exc:
            self.invalidate(str(exc))
            label = 'Rejected: see collector log'
        except Exception as exc:
            self.invalidate(f'图像处理失败: {exc}')
            label = 'Image processing failed'
        if image is not None:
            cv2.putText(image, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 200, 255), 2)
            debug = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            debug.header = msg.header
            self.debug.publish(debug)

    def make_sample(self):
        frames = list(self.window)
        if len(frames) < self.p['min_stable_frames']:
            raise ValueError('有效稳定帧不足，请保持板可见、末端停稳后重试')
        age = (self.get_clock().now().nanoseconds - frames[-1]['stamp_ns']) / 1e9
        duration = (frames[-1]['stamp_ns'] - frames[0]['stamp_ns']) / 1e9
        if not 0 <= age <= self.p['max_image_age_sec'] or duration < self.p['min_stable_duration_sec']:
            raise ValueError('图像不够新或稳定时间不足')
        for key, translation_limit, rotation_limit in (
                ('gripper', 'max_robot_translation_m', 'max_robot_rotation_deg'),
                ('target', 'max_board_translation_m', 'max_board_rotation_deg')):
            # Compare every pair, not just adjacent frames; slow motion must
            # not pass merely because its per-frame displacement is small.
            for i, a in enumerate(frames):
                for b in frames[i+1:]:
                    translation, rotation = pose_delta(a[key], b[key])
                    if translation > self.p[translation_limit] or rotation > self.p[rotation_limit]:
                        raise ValueError(f'{key} 尚不稳定: {translation*1000:.2f} mm / {rotation:.2f} deg')
        # Keep each pose pair intact. The averaged pose is used only for
        # diagnostics, never paired with a different exposure's robot TF.
        best = min(frames, key=lambda f: f['quality']['reprojection_rms_px'])
        for previous in self.samples:
            old = as_transform(previous['R_gripper2base'], previous['t_gripper2base'])
            translation, rotation = pose_delta(old, best['gripper'])
            if translation < self.p['min_translation_m'] and rotation < self.p['min_rotation_deg']:
                raise ValueError('与已有样本姿态太接近，请换一个位置或倾角')
        g, c = best['gripper'], best['target']
        target_mean = average_pose(np.array([f['target'] for f in frames]))
        jitter = [pose_delta(target_mean, f['target'])[0] * 1000 for f in frames]
        return {
            'stamp': {'sec': best['stamp_ns'] // 10**9, 'nanosec': best['stamp_ns'] % 10**9},
            **{key: self.p[key] for key in ('base_frame', 'gripper_frame', 'camera_frame')},
            'target_signature': self.spec.signature(),
            'R_gripper2base': g[:3, :3].tolist(), 't_gripper2base': g[:3, 3].tolist(),
            'R_target2cam': c[:3, :3].tolist(), 't_target2cam': c[:3, 3].tolist(),
            'quality': dict(best['quality'], stable_frames=len(frames),
                            stable_duration_sec=duration, target_jitter_rms_mm=float(
                                np.sqrt(np.mean(np.square(jitter))))),
        }

    def write_samples(self, samples):
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        data = {'schema_version': 1, 'target_type': 'charuco', 'board': self.spec.metadata(),
                'camera_model': self.camera_model, 'opencv_version': cv2.__version__,
                **{key: self.p[key] for key in (
                    'base_frame', 'gripper_frame', 'camera_frame', 'image_topic', 'camera_info_topic')},
                'collection_settings': self.p, 'samples': samples}
        temporary = self.output_file.with_suffix(self.output_file.suffix + '.tmp')
        temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n',
                             encoding='utf-8')
        temporary.replace(self.output_file)
        self.samples = samples

    def save_callback(self, _request, response):
        try:
            sample = self.make_sample()
            self.write_samples([*self.samples, sample])
            response.success = True
            response.message = f'已保存第 {len(self.samples)} 组，{self.output_file}'
        except (ValueError, OSError) as exc:
            response.success = False
            response.message = str(exc)
        self.get_logger().info(response.message)
        return response

    def undo_callback(self, _request, response):
        try:
            if not self.samples:
                raise ValueError('没有可撤销的样本')
            self.write_samples(self.samples[:-1])
            response.success = True
            response.message = f'已撤销最后一组，剩余 {len(self.samples)} 组'
        except (ValueError, OSError) as exc:
            response.success = False
            response.message = str(exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = CharucoHandEyeCollector()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
