"""Publish the calibrated wrist camera with V4L2 acquisition timestamps."""

from math import isfinite
from pathlib import Path
import time
from urllib.parse import unquote, urlparse

import cv2
from cv_bridge import CvBridge
import rclpy
from .tracking_protocol import init_ros
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import yaml


def load_calibration(url, frame):
    """Read an existing ROS CameraInfo YAML calibration from a local file."""
    path = urlparse(url)
    if path.scheme != 'file' or path.netloc not in ('', 'localhost'):
        raise ValueError('camera_info_url must be a local file URL')
    data = yaml.safe_load(Path(unquote(path.path)).read_text(encoding='utf-8'))
    info = CameraInfo()
    info.header.frame_id = frame
    info.width, info.height = data['image_width'], data['image_height']
    info.distortion_model = data['distortion_model']
    info.d = data['distortion_coefficients']['data']
    info.k = data['camera_matrix']['data']
    info.r = data['rectification_matrix']['data']
    info.p = data['projection_matrix']['data']
    return info


class WristCamera(Node):
    """Capture a fresh YUYV frame and pair it with its calibrated CameraInfo."""

    def __init__(self):
        """Open the camera at the calibrated resolution with one capture buffer."""
        super().__init__('wrist_camera')
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('camera_info_url', (
            Path.home() / '.ros/camera_info/wrist_camera.yaml').as_uri())
        self.declare_parameter('frame_id', 'camera_optical_frame')
        device = self.get_parameter('video_device').value
        self.info = load_calibration(self.get_parameter('camera_info_url').value,
                                     self.get_parameter('frame_id').value)
        self.bridge = CvBridge()
        cv2.setNumThreads(1)
        self.last_capture_ns = 0
        self.camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not self.camera.isOpened():
                raise RuntimeError(f'Cannot open wrist camera {device}')
            for prop, value in (
                    (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV')),
                    (cv2.CAP_PROP_FRAME_WIDTH, self.info.width),
                    (cv2.CAP_PROP_FRAME_HEIGHT, self.info.height),
                    (cv2.CAP_PROP_FPS, 30), (cv2.CAP_PROP_BUFFERSIZE, 1)):
                if not self.camera.set(prop, value):
                    raise RuntimeError(f'Cannot configure {device} (property {prop})')
            self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            self.camera.set(cv2.CAP_PROP_CONTRAST, 32)
            self.camera.set(cv2.CAP_PROP_GAIN, 0)
            self.image_pub = self.create_publisher(
                Image, 'image_raw', qos_profile_sensor_data)
            self.info_pub = self.create_publisher(
                CameraInfo, 'camera_info', qos_profile_sensor_data)
            self.create_timer(1. / 30., self.capture)
        except Exception:
            self.camera.release()
            raise

    def capture(self):
        """Translate the kernel capture time to this host's ROS clock."""
        ok, frame = self.camera.read()
        if not ok or frame.shape[:2] != (self.info.height, self.info.width):
            self.get_logger().warning('No calibrated wrist camera frame',
                                      throttle_duration_sec=2.)
            return
        # The OpenCV V4L2 backend exposes the buffer's CLOCK_MONOTONIC time
        # through POS_MSEC. Sample both clocks together to project into ROS time.
        capture_ms = self.camera.get(cv2.CAP_PROP_POS_MSEC)
        now = self.get_clock().now()
        if not isfinite(capture_ms):
            return
        capture_ns = round(capture_ms * 1.e6)
        age_ns = time.monotonic_ns() - capture_ns
        if capture_ns <= self.last_capture_ns or not 0 <= age_ns <= 250_000_000:
            self.get_logger().warning('Invalid or stale V4L2 frame timestamp',
                                      throttle_duration_sec=2.)
            return
        self.last_capture_ns = capture_ns
        self.info.header.stamp = Time(nanoseconds=now.nanoseconds - age_ns).to_msg()
        image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        image.header = self.info.header
        self.info_pub.publish(self.info)
        self.image_pub.publish(image)


def main(args=None):
    """Run the wrist camera and release its device on shutdown."""
    init_ros(args=args)
    node = None
    try:
        node = WristCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.camera.release()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
