#!/usr/bin/env python3

import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from dt_apriltags import Detector
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
import tf2_ros


class HandEyeCollector(Node):
    """Collect synchronized gripper-to-base and marker-to-camera samples."""

    def __init__(self):
        super().__init__('handeye_collector')

        self.declare_parameter('image_topic', '/wrist_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/wrist_camera/camera_info')
        self.declare_parameter('base_frame', 'dummy')
        self.declare_parameter('gripper_frame', '6_Link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('marker_id', 1)
        self.declare_parameter('marker_family', 'auto')
        self.declare_parameter('marker_length_m', 0.02)
        self.declare_parameter(
            'output_file', '/home/jetson/handeye_samples.json'
        )
        self.declare_parameter('min_translation_m', 0.01)
        self.declare_parameter('min_rotation_deg', 5.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.gripper_frame = self.get_parameter('gripper_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_family = self.get_parameter('marker_family').value
        self.marker_length_m = float(
            self.get_parameter('marker_length_m').value
        )
        self.output_file = Path(self.get_parameter('output_file').value)
        self.min_translation_m = float(
            self.get_parameter('min_translation_m').value
        )
        self.min_rotation_deg = float(
            self.get_parameter('min_rotation_deg').value
        )

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_sample = None
        self.samples = self.load_samples()
        self.last_report = None
        self.last_status = None

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.apriltag_detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            '/wrist_camera/handeye_debug',
            10,
        )
        self.save_service = self.create_service(
            Trigger,
            '/handeye/save_sample',
            self.save_sample_callback,
        )
        self.clear_service = self.create_service(
            Trigger,
            '/handeye/clear_samples',
            self.clear_samples_callback,
        )

        self.get_logger().info(
            f'采集器已启动，目标标志 ID={self.marker_id}，'
            f'标志外边长={self.marker_length_m:.4f} m，'
            f'识别家族={self.marker_family}'
        )
        self.get_logger().info(
            '每到一个稳定姿态调用: ros2 service call '
            '/handeye/save_sample std_srvs/srv/Trigger {}'
        )
        self.get_logger().info(
            f'当前已有 {len(self.samples)} 组数据，保存到 {self.output_file}'
        )

    def camera_info_callback(self, msg):
        matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if not np.any(matrix):
            return
        self.camera_matrix = matrix
        self.dist_coeffs = np.asarray(msg.d, dtype=np.float64)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.report_status('等待有效的相机内参')
            self.publish_debug(msg, None, '等待有效 camera_info')
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'图像转换失败: {exc}')
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.detector_parameters,
        )
        detected_ids = [] if ids is None else [
            int(marker_id) for marker_id in ids.flatten()
        ]
        april_tags = self.apriltag_detector.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )
        april_ids = [int(tag.tag_id) for tag in april_tags]

        marker_kind, marker_corners, marker_index = self.find_marker(
            corners, ids, april_tags
        )
        if marker_kind is None:
            self.latest_sample = None
            self.report_status(
                f'未检测到目标 ID {self.marker_id}，当前 ArUco IDs='
                f'{detected_ids}，AprilTag IDs={april_ids}'
            )
            self.publish_debug(
                msg,
                image,
                f'目标 ID={self.marker_id}, ArUco={detected_ids}, '
                f'AprilTag={april_ids}',
                corners,
                ids,
                april_tags,
            )
            return

        if marker_kind == 'aruco':
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners],
                self.marker_length_m,
                self.camera_matrix,
                self.dist_coeffs,
            )
            rvec = np.asarray(rvecs[0], dtype=np.float64).reshape(3, 1)
            tvec = np.asarray(tvecs[0], dtype=np.float64).reshape(3, 1)
        else:
            object_points = np.array([
                [-self.marker_length_m / 2, self.marker_length_m / 2, 0.0],
                [self.marker_length_m / 2, self.marker_length_m / 2, 0.0],
                [self.marker_length_m / 2, -self.marker_length_m / 2, 0.0],
                [-self.marker_length_m / 2, -self.marker_length_m / 2, 0.0],
            ], dtype=np.float64)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                np.asarray(marker_corners, dtype=np.float64),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                self.latest_sample = None
                self.report_status('检测到 AprilTag，但位姿求解失败')
                return
        rotation_target_camera, _ = cv2.Rodrigues(rvec)

        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.gripper_frame,
                stamp,
                timeout=Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.latest_sample = None
            self.report_status(f'已检测到 ID {self.marker_id}，但 TF 查询失败: {exc}')
            self.publish_debug(msg, image, f'TF 不可用: {exc}')
            return

        rotation_gripper_base, translation_gripper_base = (
            transform_to_matrix_parts(transform)
        )
        self.latest_sample = {
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
            'tag_id': self.marker_id,
            'marker_family': marker_kind,
            'base_frame': self.base_frame,
            'gripper_frame': self.gripper_frame,
            'camera_frame': msg.header.frame_id or self.camera_frame,
            # OpenCV calibrateHandEye inputs: gripper->base and target->camera.
            'R_gripper2base': rotation_gripper_base.tolist(),
            't_gripper2base': translation_gripper_base.reshape(3).tolist(),
            'R_target2cam': rotation_target_camera.tolist(),
            't_target2cam': tvec.reshape(3).tolist(),
        }
        self.report_status(
            f'已获得可保存样本：{marker_kind} ID {self.marker_id}，'
            f'距离 {float(np.linalg.norm(tvec)):.3f} m'
        )
        self.publish_debug(
            msg,
            image,
            f'ID={self.marker_id}, 可保存 {len(self.samples)} 组',
            [marker_corners] if marker_kind == 'aruco' else None,
            np.array([[self.marker_id]], dtype=np.int32)
            if marker_kind == 'aruco' else None,
            april_tags if marker_kind == 'apriltag' else None,
            rvec,
            tvec,
        )

    def find_marker(self, corners, ids, april_tags):
        if self.marker_family in ('aruco', 'auto') and ids is not None:
            for index, marker_id in enumerate(ids.flatten()):
                if int(marker_id) == self.marker_id:
                    return 'aruco', corners[index], index
        if self.marker_family in ('apriltag', 'auto'):
            for tag in april_tags:
                if int(tag.tag_id) == self.marker_id:
                    return 'apriltag', tag.corners, None
        return None, None, None

    def report_status(self, status):
        if status != self.last_status:
            self.last_status = status
            self.get_logger().info(status)

    def save_sample_callback(self, _request, response):
        if self.latest_sample is None:
            response.success = False
            response.message = '当前没有同时获得 Tag 位姿和末端 TF。'
            return response

        if self.samples and not self.sample_is_different(self.latest_sample):
            response.success = False
            response.message = (
                '当前姿态与上一组数据太接近，请移动机械臂的位置或姿态后再保存。'
            )
            return response

        self.samples.append(self.latest_sample)
        self.write_samples()
        count = len(self.samples)
        response.success = True
        response.message = f'已保存第 {count} 组手眼标定数据。'
        self.get_logger().info(response.message)
        return response

    def clear_samples_callback(self, _request, response):
        self.samples = []
        self.write_samples()
        response.success = True
        response.message = '已清空手眼标定数据。'
        self.get_logger().warn(response.message)
        return response

    def sample_is_different(self, sample):
        previous = self.samples[-1]
        old_t = np.asarray(previous['t_gripper2base'], dtype=np.float64)
        new_t = np.asarray(sample['t_gripper2base'], dtype=np.float64)
        translation_delta = np.linalg.norm(new_t - old_t)

        old_r = np.asarray(previous['R_gripper2base'], dtype=np.float64)
        new_r = np.asarray(sample['R_gripper2base'], dtype=np.float64)
        relative_r = old_r.T @ new_r
        cosine = np.clip((np.trace(relative_r) - 1.0) / 2.0, -1.0, 1.0)
        rotation_delta = math.degrees(math.acos(cosine))
        return (
            translation_delta >= self.min_translation_m
            or rotation_delta >= self.min_rotation_deg
        )

    def load_samples(self):
        if not self.output_file.exists():
            return []
        try:
            with self.output_file.open('r', encoding='utf-8') as stream:
                data = json.load(stream)
            return data.get('samples', [])
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'无法读取已有采集文件: {exc}')
            return []

    def write_samples(self):
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'marker_family': self.marker_family,
            'marker_id': self.marker_id,
            'marker_length_m': self.marker_length_m,
            'image_topic': self.image_topic,
            'camera_info_topic': self.camera_info_topic,
            'base_frame': self.base_frame,
            'gripper_frame': self.gripper_frame,
            'samples': self.samples,
        }
        temporary_file = self.output_file.with_suffix('.tmp')
        with temporary_file.open('w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2)
        temporary_file.replace(self.output_file)

    def publish_debug(
        self,
        source_msg,
        image,
        status,
        marker_corners=None,
        marker_ids=None,
        april_tags=None,
        rvec=None,
        tvec=None,
    ):
        if image is None:
            return
        debug_image = image.copy()
        if marker_corners is not None and marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(
                debug_image,
                marker_corners,
                marker_ids,
            )
        if april_tags:
            for tag in april_tags:
                tag_corners = tag.corners.astype(int)
                cv2.polylines(
                    debug_image,
                    [tag_corners.reshape((-1, 1, 2))],
                    True,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    debug_image,
                    f'AprilTag {tag.tag_id}',
                    tuple(tag.center.astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
        if rvec is not None and tvec is not None:
            cv2.drawFrameAxes(
                debug_image,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.marker_length_m * 0.5,
            )
        cv2.putText(
            debug_image,
            status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = source_msg.header
            self.debug_publisher.publish(debug_msg)
        except Exception as exc:
            self.get_logger().error(f'调试图发布失败: {exc}')


def transform_to_matrix_parts(transform_stamped: TransformStamped):
    transform = transform_stamped.transform
    qx = transform.rotation.x
    qy = transform.rotation.y
    qz = transform.rotation.z
    qw = transform.rotation.w
    rotation = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
         2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
         2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
         1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)
    translation = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ], dtype=np.float64).reshape(3, 1)
    return rotation, translation


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCollector()
    # The TF listener has its own callback group. Let it receive transforms
    # while the image callback waits for the transform at the image stamp.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
