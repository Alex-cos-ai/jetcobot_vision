#!/usr/bin/env python3
"""
Turn the wrist camera toward a coloured target and regulate its apparent size.

Discrete LQR controls the two bearing errors and apparent range. The
calibrated hand-eye transform maps camera motion to the six-axis arm's tool.
"""

import json
from collections import deque
from pathlib import Path
import signal
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .control import aiming_axes, aiming_error, camera_to_tool_twist, lqr_gain, norm_limit


def colour_masks(hsv):
    """Mask red, green and blue in OpenCV's HSV convention."""
    masks = {
        'red': (cv2.inRange(hsv, (0, 80, 50), (10, 255, 255)) |
                cv2.inRange(hsv, (170, 80, 50), (180, 255, 255))),
        'green': cv2.inRange(hsv, (35, 60, 40), (85, 255, 255)),
        'blue': cv2.inRange(hsv, (90, 60, 40), (140, 255, 255)),
    }
    kernel = np.ones((5, 5), np.uint8)
    return {name: cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            for name, mask in masks.items()}


def detect_target(image, target_colour='auto', min_area_px=250.):
    """Choose the largest valid blob, optionally restricted to one colour."""
    masks = colour_masks(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
    detections = []
    for name, mask in masks.items():
        if target_colour != 'auto' and name != target_colour:
            continue
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area_px:
                x, y, width, height = cv2.boundingRect(contour)
                detections.append((
                    area, name, x + width / 2., y + height / 2.,
                    max(width, height)))
    return max(detections) if detections else None


class VisualServo6Dof(Node):
    """Use fresh calibrated images and measured tool TF for visual feedback."""

    def __init__(self):
        """Load hand-eye calibration and the six-axis controller."""
        super().__init__('visual_servo_6dof')
        defaults = {
            'image_topic': '/wrist_camera/image_raw',
            'camera_info_topic': '/wrist_camera/camera_info',
            'command_topic': '/servo_node/delta_twist_cmds',
            'base_frame': 'base_link', 'command_frame': '6_Link',
            'target_colour': 'auto',
            'calibration_file':
                '/home/jetson/charuco_handeye_result_filtered.json',
            'control_frequency': 20., 'target_timeout': .35,
            'nominal_depth_m': .45, 'target_width_px': 140.,
            'min_area_px': 250., 'max_linear_velocity': .01,
            'max_angular_velocity': .04, 'max_linear_acceleration': .03,
            'max_angular_acceleration': .1, 'filter_alpha': .35,
            'deadband_u_px': 4., 'deadband_v_px': 4.,
            'deadband_width_px': 5.,
            'publish_debug_image': True,
            'q_aim': [4., 4., 1.],
            'r_aim': [4., 4., 16.],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {
            name: self.get_parameter(name).value for name in defaults}
        self.p = self.params.__getitem__
        positive = (
            'control_frequency', 'target_timeout', 'nominal_depth_m',
            'target_width_px', 'min_area_px', 'max_linear_velocity',
            'max_angular_velocity', 'max_linear_acceleration',
            'max_angular_acceleration',
        )
        if any(not np.isfinite(self.p(k)) or self.p(k) <= 0
               for k in positive):
            raise ValueError('Controller periods and limits must be positive')
        if not 0 < self.p('filter_alpha') <= 1:
            raise ValueError('filter_alpha must be in (0, 1]')
        if self.p('target_colour') not in ('auto', 'red', 'green', 'blue'):
            raise ValueError('target_colour must be auto, red, green or blue')
        calibration = json.loads(
            Path(self.p('calibration_file')).expanduser().read_text())
        if calibration['parent_frame'] != self.p('command_frame'):
            raise ValueError('Hand-eye parent and command_frame must match')
        self.optical_frame = calibration['child_frame']
        self.handeye = np.asarray(calibration['T_gripper_camera'], dtype=float)
        if (self.handeye.shape != (4, 4) or
                not np.all(np.isfinite(self.handeye)) or
                not np.allclose(self.handeye[3], [0, 0, 0, 1]) or
                not np.allclose(self.handeye[:3, :3].T @ self.handeye[:3, :3],
                                np.eye(3), atol=1.e-5) or
                not np.isclose(np.linalg.det(self.handeye[:3, :3]), 1.)):
            raise ValueError('Invalid calibrated hand-eye transform')
        self.calibrated_camera = calibration.get('camera_model')
        self.bridge = CvBridge()
        self.buffer = Buffer(cache_time=Duration(seconds=5.))
        self.listener = TransformListener(self.buffer, self)
        self.info = None
        self.center_ray = None
        self.axes = None
        self.gain = lqr_gain(
            1. / self.p('control_frequency'), self.p('q_aim'), self.p('r_aim'))
        self.observation = None
        self.pending = deque(maxlen=12)
        self.filtered_error = None
        self.previous_command = np.zeros(6)
        self.last_tick = time.monotonic()
        self.status = 'waiting_for_image'
        self.twist_pub = self.create_publisher(
            TwistStamped, self.p('command_topic'), 10)
        self.state_pub = self.create_publisher(
            Float32MultiArray, '~/visual_state', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self.create_subscription(CameraInfo, self.p('camera_info_topic'),
                                 self.info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.p('image_topic'),
                                 self.image_callback, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset_reference', self.reset_reference)
        self.create_timer(
            1. / self.p('control_frequency'), self.control_callback)
        self.get_logger().info(
            f'Calibrated six-axis velocity controller: {self.optical_frame} '
            f'-> {self.p("command_frame")}; aiming at the target')

    def reset_reference(self, request, response):
        """Clear tracking history so the controller uses a new observation."""
        self.observation = None
        self.pending.clear()
        self.filtered_error = None
        self.previous_command = np.zeros(6)
        self.status = 'waiting_for_image'
        response.success = True
        response.message = 'Tracking history cleared'
        return response

    def info_callback(self, message):
        """Require the camera model used by hand-eye calibration."""
        if (message.header.frame_id != self.optical_frame or
                message.k[0] <= 0 or message.k[4] <= 0 or
                message.distortion_model not in (
                    'plumb_bob', 'rational_polynomial')):
            self.info = None
            self.center_ray = None
            self.status = 'camera_info_mismatch'
            return
        model = self.calibrated_camera
        if model is not None and (
                (message.width, message.height) != (
                    model['width'], model['height']) or
                len(message.d) != len(model['D']) or
                not np.allclose(message.k, model['K'], rtol=0, atol=1.e-7) or
                not np.allclose(message.d, model['D'], rtol=0, atol=1.e-7)):
            self.info = None
            self.center_ray = None
            self.status = 'calibration_intrinsics_mismatch'
            return
        self.info = message
        if self.center_ray is not None:
            return
        self.center_ray = self.pixel_ray(message.width / 2., message.height / 2.)
        self.axes = aiming_axes(self.center_ray)

    def pixel_ray(self, u, v):
        """Get a unit camera ray through a calibrated image pixel."""
        k = np.asarray(self.info.k).reshape(3, 3)
        pixel = np.array([[[u, v]]], dtype=float)
        ray = np.r_[cv2.undistortPoints(pixel, k, np.asarray(self.info.d)).ravel(), 1.]
        return ray / np.linalg.norm(ray)

    def image_callback(self, message):
        """Detect a coloured target in a fresh calibrated image."""
        try:
            image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except CvBridgeError:
            self.status = 'invalid_image'
            return
        detection = detect_target(
            image, self.p('target_colour'), self.p('min_area_px'))
        stamp = Time.from_msg(message.header.stamp)
        age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1.e9
        self.queue_detection(detection, message, stamp, age)
        self.publish_debug(image, message.header, detection, age)

    def queue_detection(self, detection, message, stamp, age):
        """Accept only calibrated, recent observations for TF matching."""
        if detection is None:
            self.status = 'target_lost'
            self.pending.clear()
            self.observation = None
            self.filtered_error = None
            return
        if self.info is None:
            self.status = 'waiting_for_matching_camera_info'
            return
        if (message.header.frame_id != self.optical_frame or
                (message.width, message.height) != (
                    self.info.width, self.info.height)):
            self.status = 'image_camera_info_mismatch'
            return
        if not 0 <= age <= self.p('target_timeout'):
            self.status = 'stale_image'
            self.pending.clear()
            self.observation = None
            self.filtered_error = None
            self.get_logger().warning(
                f'Image timestamp age {age * 1000:.0f} ms; allowed '
                f'0..{self.p("target_timeout") * 1000:.0f} ms. '
                f'Check the camera publisher with: ros2 run jetcobot_driver check_camera_sync',
                throttle_duration_sec=3.)
            return
        # Feedback TF can arrive after its corresponding camera image. Retain
        # a short queue so interpolation can succeed on a later tick.
        self.pending.append((detection, stamp))

    def update_observation(self, detection, stamp):
        """Calculate how far to turn and how far to approach or retreat."""
        _, colour, u, v, width = detection
        depth = self.p('nominal_depth_m') * self.p('target_width_px') / width
        if not .08 <= depth <= 1.5:
            self.status = 'target_scale_out_of_range'
            return
        ray = self.pixel_ray(u, v)
        middle_u, middle_v = self.info.width / 2., self.info.height / 2.
        if abs(u - middle_u) < self.p('deadband_u_px'):
            u = middle_u
        if abs(v - middle_v) < self.p('deadband_v_px'):
            v = middle_v
        range_error = depth - self.p('nominal_depth_m')
        if abs(width - self.p('target_width_px')) < self.p('deadband_width_px'):
            range_error = 0.
        error = aiming_error(self.center_ray, self.pixel_ray(u, v), self.axes, range_error)
        alpha = self.p('filter_alpha')
        self.filtered_error = (
            error if self.filtered_error is None else
            alpha * error + (1. - alpha) * self.filtered_error)
        self.observation = (
            self.filtered_error.copy(), ray, stamp)
        self.status = f'tracking_{colour}'

    def publish_debug(self, image, header, detection, age):
        """Display the image centre and selected colour blob."""
        if not self.p('publish_debug_image'):
            return
        h, w = image.shape[:2]
        colour = (0, 0, 255) if self.status == 'stale_image' else (255, 255, 255)
        cv2.putText(image, f'{self.status} | stamp age {age * 1000:.0f} ms',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, colour, 1)
        cv2.drawMarker(image, (w // 2, h // 2), (255, 255, 255),
                       cv2.MARKER_CROSS, 24, 2)
        if detection is not None:
            _, name, u, v, width = detection
            cv2.arrowedLine(image, (w // 2, h // 2), (int(u), int(v)),
                            (0, 255, 255), 2, tipLength=.15)
            cv2.circle(image, (int(u), int(v)), int(width / 2),
                       (0, 255, 255), 2)
            cv2.putText(image, name, (int(u), max(20, int(v) - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
        debug = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        debug.header = header
        self.debug_pub.publish(debug)

    def control_callback(self):
        """Publish a rate-limited tool twist, or zero for invalid feedback."""
        now = time.monotonic()
        dt = min(max(now - self.last_tick, .001), .1)
        self.last_tick = now
        command = np.zeros(6)
        if self.center_ray is not None:
            for detection, stamp in reversed(self.pending):
                age = (
                    self.get_clock().now().nanoseconds - stamp.nanoseconds
                ) / 1.e9
                if not 0 <= age <= self.p('target_timeout'):
                    continue
                try:
                    self.buffer.lookup_transform(
                        self.p('base_frame'), self.p('command_frame'), stamp)
                except TransformException:
                    continue
                self.update_observation(detection, stamp)
                self.pending = deque(
                    [(d, t) for d, t in self.pending
                     if t.nanoseconds > stamp.nanoseconds], maxlen=12)
                break
            else:
                if self.pending and self.observation is None:
                    self.status = 'waiting_for_timestamped_tool_tf'
        valid = self.observation is not None
        if valid:
            error, ray, image_stamp = self.observation
            age = (
                self.get_clock().now().nanoseconds - image_stamp.nanoseconds
            ) / 1.e9
            valid = 0 <= age <= self.p('target_timeout')
        if valid and self.center_ray is not None and self.status.startswith('tracking_'):
            aim_velocity = -self.gain @ error
            linear = norm_limit(ray * aim_velocity[2], self.p('max_linear_velocity'))
            angular = norm_limit(self.axes @ aim_velocity[:2], self.p('max_angular_velocity'))
            optical = np.r_[linear, angular]
            command = camera_to_tool_twist(optical, self.handeye)
            # Scale both together so a rotation at the camera keeps its pivot
            # after shifting the velocity to the tool's origin.
            command *= min(1., self.p('max_linear_velocity') /
                           max(np.linalg.norm(command[:3]), 1.e-12))
            change = command - self.previous_command
            fraction = min(
                1., dt * self.p('max_linear_acceleration') /
                max(np.linalg.norm(change[:3]), 1.e-12),
                dt * self.p('max_angular_acceleration') /
                max(np.linalg.norm(change[3:]), 1.e-12))
            command = self.previous_command + fraction * change
            state = Float32MultiArray()
            state.data = error.tolist()
            self.state_pub.publish(state)
        else:
            if self.status.startswith('tracking_'):
                self.status = 'target_timeout'
            self.filtered_error = None
        self.previous_command = command
        self.publish_twist(command)
        status = String()
        status.data = self.status
        self.status_pub.publish(status)

    def publish_twist(self, command):
        """Publish tool-origin velocity in the calibrated tool frame."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.p('command_frame')
        linear, angular = message.twist.linear, message.twist.angular
        linear.x, linear.y, linear.z = command[:3].tolist()
        angular.x, angular.y, angular.z = command[3:].tolist()
        self.twist_pub.publish(message)


def main(args=None):
    """Run the calibrated colour follower."""
    rclpy.init(args=args)
    node = None
    try:
        node = VisualServo6Dof()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch can repeat SIGINT while the ROS node is being destroyed.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.publish_twist(np.zeros(6))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
