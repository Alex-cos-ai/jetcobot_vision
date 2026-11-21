#!/usr/bin/env python3
"""Calibrated colour PID planner; serial execution lives in sync_plan.py."""

from collections import deque
import json
from pathlib import Path
import signal
import time
from typing import NamedTuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
import numpy as np
import rclpy
from .tracking_protocol import init_ros
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .control import camera_to_tool_twist
from .kinematics import DifferentialIK, Kinematics, rotation_error
from .joint_command import JointCommandIntegrator
from .safety import SafetyGuard
from .tracking_config import load_config
from .tracking_pid import norm_limit, TrackingPID, TrackingStateMachine
from .tracking_protocol import decode, encode, LATEST, stamp_ns
from .trajectory_guard import hardware_degrees, JOINT_NAMES


class Detection(NamedTuple):
    """One real contour, its enclosing circle, and a display rectangle."""

    area: float
    colour: str
    cx: float
    cy: float
    radius: float
    bbox: tuple
    clipped: bool
    square_like: bool = True
    circle_center: tuple = None


def colour_masks(hsv):
    """Reuse the project's red/green/blue HSV thresholds."""
    masks = {
        'red': cv2.inRange(hsv, (0, 80, 50), (10, 255, 255)) |
        cv2.inRange(hsv, (170, 80, 50), (180, 255, 255)),
        'green': cv2.inRange(hsv, (35, 60, 40), (85, 255, 255)),
        'blue': cv2.inRange(hsv, (90, 60, 40), (140, 255, 255))}
    kernel = np.ones((5, 5), np.uint8)
    return {name: cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            for name, mask in masks.items()}


def detect_objects(image, min_area_px=250., border=2):
    """Extract actual minimum enclosing circles, not half a bbox width."""
    result = []
    height, width = image.shape[:2]
    for colour, mask in colour_masks(cv2.cvtColor(image, cv2.COLOR_BGR2HSV)).items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area_px:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            circle_center = (cx, cy)
            x, y, w, h = cv2.boundingRect(contour)
            clipped = x <= border or y <= border or x + w >= width - border or y + h >= height - border
            (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
            square_like = bool(min(rw, rh) > 0 and area / max(rw*rh, 1.) >= .6 and
                               max(rw, rh) / min(rw, rh) <= 1.8)
            if clipped:
                square_like = bool(min(rw, rh) > 0 and area/max(rw*rh, 1.) >= .4)
            if not clipped:
                polygon = cv2.approxPolyDP(contour, .025*cv2.arcLength(contour, True), True)
                if len(polygon) == 4 and cv2.isContourConvex(polygon):
                    corners = polygon[:, 0, :].astype(float)
                    # 平面方块投影中心为两条对角线的交点；不等同于外接圆圆心。
                    matrix = np.column_stack((corners[2]-corners[0], -(corners[3]-corners[1])))
                    if abs(np.linalg.det(matrix)) > 1.e-6:
                        alpha = np.linalg.solve(matrix, corners[1]-corners[0])[0]
                        cx, cy = corners[0] + alpha*(corners[2]-corners[0])
            result.append(Detection(area, colour, cx, cy, radius, (x, y, w, h), clipped,
                                    square_like, circle_center))
    return sorted(result, key=lambda item: item.area, reverse=True)


def detect_target(image, target_colour='auto', min_area_px=250.):
    """Select the largest requested colour; retained as a public detection helper."""
    return next((d for d in detect_objects(image, min_area_px)
                 if target_colour == 'auto' or d.colour == target_colour), None)


class VisualServo6Dof(Node):
    """Sense continuously and publish only checked single-step model joint goals."""

    def __init__(self):
        """Load shared configuration; no serial port or camera is opened here."""
        super().__init__('visual_servo_6dof')
        for name, value in [('config_file', ''), ('target_colour', ''),
                            ('session', ''), ('feedback_source', 'hardware'), ('ik_backend', '')]:
            self.declare_parameter(name, value)
        self.config = load_config(self.get_parameter('config_file').value or None)
        self.session = self.get_parameter('session').value
        self.feedback_source = self.get_parameter('feedback_source').value
        c = self.config['camera']
        self.colour = self.get_parameter('target_colour').value or c['target_colour']
        self.backend = self.get_parameter('ik_backend').value or self.config['ik']['backend']
        if self.colour not in ('auto', 'red', 'green', 'blue'):
            raise ValueError('target_colour must be auto/red/green/blue')
        if self.backend not in ('moveit_servo', 'weighted_dls'):
            raise ValueError('invalid IK backend')
        calibration = json.loads(Path(c['calibration_file']).expanduser().read_text())
        if calibration['parent_frame'] != '6_Link' or calibration['child_frame'] != 'camera_optical_frame':
            raise ValueError('hand-eye frames must be 6_Link <- camera_optical_frame')
        self.handeye = np.asarray(calibration['T_gripper_camera'])
        self.calibrated_camera = calibration['camera_model']
        self.model = Kinematics(self.config['model']['urdf'], self.handeye,
                                np.radians(self.config['model']['margin_deg']),
                                self.config['model'].get('hardware_limits_deg'))
        self.guard = SafetyGuard(self.model, self.config)
        self.ik = DifferentialIK(self.model, self.config)
        self.command_integrator = JointCommandIntegrator()
        self.machine = TrackingStateMachine(
            self.config['runtime']['soft_start_s'], self.config['runtime']['lost_reset_s'],
            self.config['runtime']['lost_decay_s'])
        self.bridge = CvBridge()
        cv2.setNumThreads(1)  # Jetson上避免多个图像/控制进程各自占满CPU线程。
        self.buffer = Buffer(cache_time=Duration(seconds=5.))
        self.listener = TransformListener(self.buffer, self)
        self.info = self.rect_k = self.maps = self.pid = None
        self.pending = deque(maxlen=3)
        self.joints = self.reference_rotation = self.home = None
        self.joint_time = 0.
        self.joint_stamp = 0
        self.last_image = 0.
        self.image_stamp = 0
        self.found = False
        self.locked = None
        self.last_detection = None
        self.raw_candidate = None
        self.servo_code, self.servo_time = None, 0.
        self.servo_started = False
        self.servo_future = None
        self.last_start_attempt = 0.
        self.sync_state = {}
        self.sync_time = 0.
        self.scene_state, self.scene_time = {}, 0.
        self.last_tick = time.monotonic()
        self.last_command = self.last_log = 0.
        self.last_velocity_cam = np.zeros(3)
        self.last_aim_cam = np.zeros(3)
        self.blocked = False
        self.reason = 'WAIT_FEEDBACK'
        self.target_ns = 0
        self.target = self.velocity = None
        self.image_age = 0.
        self.tf_exact_count = 0
        self.tf_fallback_count = 0
        self.tf_gap_s = float('inf')
        self.home_active = False
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', LATEST)
        self.target_pub = self.create_publisher(JointTrajectory, '/tracking/joint_target', LATEST)
        self.state_pub = self.create_publisher(String, '/tracking/planner_state', LATEST)
        self.status_pub = self.create_publisher(String, '~/status', LATEST)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self.create_subscription(CameraInfo, c['camera_info_topic'], self.info_callback, qos_profile_sensor_data)
        latest_image = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, c['image_topic'], self.image_callback, latest_image)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, LATEST)
        self.create_subscription(JointTrajectory, '/arm_group_controller/joint_trajectory',
                                 self.candidate_callback, LATEST)
        from std_msgs.msg import Int8
        self.create_subscription(Int8, '/servo_node/status', self.servo_callback, LATEST)
        self.create_subscription(String, '/tracking/sync_state', self.sync_callback, LATEST)
        self.create_subscription(String, '/tracking/scene_state', self.scene_callback, LATEST)
        self.create_subscription(String, '/tracking/select_colour', self.select_callback, LATEST)
        self.start_client = self.create_client(Trigger, '/servo_node/start_servo')
        self.create_service(Trigger, '~/reset_reference', self.reset_reference)
        self.create_timer(1. / self.config['runtime']['control_hz'], self.control_callback)
        self.get_logger().info(f'PID planner backend={self.backend} source={self.feedback_source}; '
                               f'config={self.config["_path"]}')

    def reset_reference(self, request, response):
        """Reset control state without changing actual joint feedback."""
        if self.pid:
            self.pid.reset()
        self.reference_rotation = None
        self.command_integrator.reset()
        self.guard.previous_velocity[:] = 0.
        self.pending.clear()
        self.locked = None
        response.success, response.message = True, 'PID and pose reference cleared'
        return response

    def info_callback(self, message):
        """Validate calibration and precompute a consistent equal-focal rectification."""
        model = self.calibrated_camera
        if (message.header.frame_id != 'camera_optical_frame' or
                (message.width, message.height) != (model['width'], model['height']) or
                len(message.d) != len(model['D']) or
                not np.allclose(message.k, model['K'], atol=1.e-7, rtol=0) or
                not np.allclose(message.d, model['D'], atol=1.e-7, rtol=0)):
            self.info = None
            self.reason = 'camera_calibration_mismatch'
            return
        self.info = message
        if self.maps is not None:
            return
        k = np.array(message.k).reshape(3, 3)
        focal = (k[0, 0] + k[1, 1]) / 2.
        self.rect_k = k.copy()
        self.rect_k[0, 0] = self.rect_k[1, 1] = focal
        self.maps = cv2.initUndistortRectifyMap(
            k, np.array(message.d), None, self.rect_k, (message.width, message.height), cv2.CV_32FC1)
        reference = (np.array(self.config['camera']['reference_pixel']) - self.rect_k[:2, 2]) / focal
        self.pid = TrackingPID(self.config, focal, reference)

    def joint_callback(self, message):
        """Use actual named model joints; never consume planned positions here."""
        if len(message.name) != 6 or set(message.name) != set(JOINT_NAMES) or len(message.position) != 6:
            return
        q = np.array([message.position[message.name.index(name)] for name in JOINT_NAMES])
        age = (self.get_clock().now().nanoseconds - stamp_ns(message.header.stamp)) / 1.e9
        if np.all(np.isfinite(q)) and 0 <= age <= self.config['runtime']['timeout_s']:
            self.joints, self.joint_time = q, time.monotonic()
            self.joint_stamp = stamp_ns(message.header.stamp)
            if self.home is None:
                self.home = q.copy()

    def candidate_callback(self, message):
        """Retain a Servo candidate only at its original arrival instant."""
        self.raw_candidate = message, time.monotonic()

    def servo_callback(self, message):
        """Track Servo protections, independent of its trajectory stream."""
        self.servo_code, self.servo_time = message.data, time.monotonic()

    def sync_callback(self, message):
        """Show the executor's actual mode and the exact accepted/sent target."""
        state = decode(message)
        restart = (state.get('mode') == 'execute' and self.sync_state.get('mode') != 'execute') or (
            state.get('enabled') and not self.sync_state.get('enabled'))
        pauses = ('WAIT_FEEDBACK_OR_FRESH_IMAGE', 'waiting_for_timestamped_tool_tf',
                  'waiting_for_fresh_target', 'waiting_for_joint_acceleration',
                  'waiting_for_planner', 'stale_planner_metadata')
        if state.get('reason') in pauses and self.sync_state.get('reason') not in pauses:
            if self.pid:
                self.pid.restart_output()
            self.guard.previous_velocity[:] = 0.
            if self.machine.state == 'TRACKING':
                self.machine.entered = time.monotonic()
        self.sync_state, self.sync_time = state, time.monotonic()
        if restart:
            if self.pid:
                self.pid.reset()
            self.command_integrator.reset()
            self.guard.previous_velocity[:] = 0.
            self.machine.state = 'IDLE'
            self.reference_rotation = None

    def scene_callback(self, message):
        """Require an acknowledged table scene before hardware Servo execution."""
        self.scene_state, self.scene_time = decode(message), time.monotonic()

    def select_callback(self, message):
        """Switch colour without blocking image capture or control."""
        if message.data in ('auto', 'red', 'green', 'blue'):
            self.colour, self.locked = message.data, None
            self.reference_rotation = None
            c = self.config['runtime']
            self.machine = TrackingStateMachine(c['soft_start_s'], c['lost_reset_s'], c['lost_decay_s'])
            if self.pid:
                self.pid.reset()

    def image_callback(self, message):
        """Rectify and detect; the display remains independent of joint feedback."""
        try:
            image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except CvBridgeError:
            return
        age = (self.get_clock().now().nanoseconds - stamp_ns(message.header.stamp)) / 1.e9
        self.image_age = age
        fresh = 0 <= age <= self.config['runtime']['timeout_s']
        if self.info is not None and self.maps is not None and image.shape[:2] == (self.info.height, self.info.width):
            image = cv2.remap(image, *self.maps, cv2.INTER_LINEAR)
            detections = detect_objects(image, self.config['camera']['min_area_px'],
                                        self.config['camera']['border_margin_px'])
            candidates = [d for d in detections if d.square_like and
                          (self.colour == 'auto' or d.colour == self.colour)]
            chosen = None
            if self.locked is None:
                chosen = candidates[0] if candidates else None
            else:
                same = [d for d in candidates if d.colour == self.locked.colour]
                if same:
                    nearest = min(same, key=lambda d: np.hypot(d.cx - self.locked.cx, d.cy - self.locked.cy))
                    distance = np.hypot(nearest.cx - self.locked.cx, nearest.cy - self.locked.cy)
                    if distance <= self.config['camera']['lock_distance_px']:
                        chosen = nearest
            if chosen is not None:
                self.locked = chosen
            self.last_detection = chosen
            self.found = bool(fresh and chosen)
            if fresh:
                self.last_image, self.image_stamp = time.monotonic(), stamp_ns(message.header.stamp)
                if self.found:
                    self.pending.append((chosen, Time.from_msg(message.header.stamp)))
                else:
                    self.pending.clear()
            else:
                self.pending.clear()
                self.found = False
            self.publish_debug(image, message.header, detections, chosen)
        else:
            self.found = False
            self.publish_debug(image, message.header, [], None)

    def publish_debug(self, image, header, detections, chosen):
        """Render colour rectangles, the actual circle, and camera-motion arrows."""
        for detection in detections:
            x, y, w, h = detection.bbox
            active = chosen is detection
            colour = (0, 255, 255) if active else (150, 150, 150)
            cv2.rectangle(image, (x, y), (x + w, y + h), colour, 2 if active else 1)
            cv2.putText(image, detection.colour, (x, max(16, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, .5, colour, 1)
            if active:
                circle = detection.circle_center or (detection.cx, detection.cy)
                cv2.circle(image, tuple(round(v) for v in circle), round(detection.radius), colour, 2)
                cv2.drawMarker(image, (round(detection.cx), round(detection.cy)),
                               (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        centre = tuple(round(x) for x in self.config['camera']['reference_pixel'])
        cv2.drawMarker(image, centre, (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        velocity = self.last_velocity_cam * 1000.
        partial = bool(self.last_detection and self.last_detection.clipped)
        arrow = np.array([self.last_aim_cam[1], -self.last_aim_cam[0]])*2000 if partial else velocity[:2]*8
        end = (round(centre[0] + arrow[0]), round(centre[1] + arrow[1]))
        cv2.arrowedLine(image, centre, end, (0, 255, 0), 2)
        depth = ('n/a (partial target: AIM only)' if partial else
                 ('n/a' if self.pid is None or self.pid.depth is None else f'{self.pid.depth * 1000:.1f}'))
        error = np.zeros(3) if self.pid is None else self.pid.error * 1000.
        mode = self.sync_state.get('mode', 'SIMULATED' if self.feedback_source == 'simulation' else 'WAIT_SYNC')
        motion_line = (f'Camera aim deg/s: {np.degrees(self.last_aim_cam[0]):+.1f} '
                       f'{np.degrees(self.last_aim_cam[1]):+.1f}' if partial else
                       f'Camera motion mm/s: {velocity[0]:+.1f} {velocity[1]:+.1f} {velocity[2]:+.1f}')
        lines = [f'{mode} | {self.machine.state} | {self.reason}',
                 f'Depth estimate={depth}mm desired={self.config["camera"]["desired_depth_mm"]:.0f}mm',
                 f'Error camera mm: {error[0]:+.1f} {error[1]:+.1f} {error[2]:+.1f}',
                 motion_line,
                 f'Image age={self.image_age * 1000:.0f}ms '
                 f'TFgap={self.tf_gap_s * 1000:.0f}ms | colour={self.colour}',
                 'r/g/b/a: colour | click box: lock colour | q/Esc: exit']
        for i, text in enumerate(lines):
            cv2.putText(image, text, (7, 18 + i * 19), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
        result = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        result.header = header
        self.debug_pub.publish(result)

    def start_servo(self, now):
        """Start solving automatically, without enabling serial motion."""
        if self.backend != 'moveit_servo' or self.servo_started:
            return
        if self.servo_future is not None:
            if self.servo_future.done():
                response = self.servo_future.result()
                self.servo_started = bool(response and response.success)
                self.servo_future = None
            return
        if self.joints is not None and self.start_client.service_is_ready() and now - self.last_start_attempt > 1.:
            self.servo_future = self.start_client.call_async(Trigger.Request())
            self.last_start_attempt = now

    def publish_twist(self, command):
        """Publish a tool-origin Twist in tool coordinates."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = '6_Link'
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = command[:3].tolist()
        message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = command[3:].tolist()
        self.twist_pub.publish(message)

    def publish_target(self, target, velocity, horizon, generation_ns=None):
        """Publish a freshly validated target in the model-radian convention."""
        message = JointTrajectory()
        if generation_ns is None:
            generation_ns = self.get_clock().now().nanoseconds
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(generation_ns, 1000000000)
        message.header.frame_id = 'base_link'
        message.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions, point.velocities = target.tolist(), velocity.tolist()
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(horizon * 1.e9), 1000000000)
        message.points = [point]
        self.target_pub.publish(message)
        self.target_ns = stamp_ns(message.header.stamp)
        self.target, self.velocity = target.copy(), velocity.copy()

    def control_callback(self):
        """Run PID at 20Hz and final joint targets at 10Hz, independently of logs."""
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
        dt, self.last_tick = now - self.last_tick, now
        # executor负载后可能紧接着补调timer；异常dt只跳过，不伪造丢失并重置PID。
        if not self.config['runtime']['dt_range_s'][0] <= dt <= self.config['runtime']['dt_range_s'][1]:
            return
        self.start_servo(now)
        timeout = self.config['runtime']['timeout_s']
        healthy = (self.joints is not None and now - self.joint_time <= timeout and
                   self.info is not None and now - self.last_image <= timeout)
        if len(self.get_publishers_info_by_topic('/joint_states')) > 1:
            healthy = False
        state, scale, reset = self.machine.step(self.found, healthy, now)
        if reset and self.pid:
            self.pid.reset()
            if not self.sync_state.get('enabled', False):
                self.command_integrator.reset()
            self.guard.previous_velocity[:] = 0.
            self.reference_rotation = None
            if state == 'HOLD':
                # 丢失后的实例锁超时；同色目标在新位置重现时允许重新捕获。
                self.locked = None
        if not self.found and self.machine.last_seen and now - self.machine.last_seen > timeout:
            # TF/目标丢失时只衰减，不把旧位置误差继续积分。
            self.pending.clear()
        # 控制历史重置时可再次装入最新的有效图像，不必空等下一帧。
        observations = list(reversed(self.pending))
        if (self.pid and self.pid.stamp is None and self.found and self.last_detection is not None and
                self.image_stamp):
            observations.append((self.last_detection, Time(nanoseconds=self.image_stamp)))
        for detection, stamp in observations:
            age = (ros_now - stamp.nanoseconds) / 1.e9
            if not 0 <= age <= timeout:
                continue
            # 相机30Hz与关节10Hz通常不会共享完全相同的时间戳。
            # 先查精确TF；若缓存尚未覆盖该帧，但最新实测关节仍在允许时差内，
            # 当前FK会使用这组实测关节，避免正常采样差异造成目标丢失。
            tf_available = False
            self.tf_gap_s = ((stamp.nanoseconds - self.joint_stamp) / 1.e9
                             if self.joint_stamp else float('inf'))
            # 自研DLS使用当前实测q的FK；像素PID不需要额外等待TF发布线程。
            tolerance = self.config['runtime']['tf_sync_tolerance_s']
            if self.joint_stamp and abs(self.tf_gap_s) <= tolerance:
                self.tf_fallback_count += 1
                tf_available = True
            elif self.backend == 'moveit_servo':
                try:
                    self.buffer.lookup_transform('base_link', '6_Link', stamp)
                    self.tf_exact_count += 1
                    tf_available = True
                except TransformException:
                    pass
            was_partial = self.pid.partial
            if tf_available and self.pid.observe(
                    detection.cx, detection.cy, detection.radius,
                    stamp.nanoseconds / 1.e9, self.rect_k[:2, 2], partial=detection.clipped):
                if was_partial and not detection.clipped and self.joints is not None:
                    self.reference_rotation = self.model.fk(self.joints).camera[:3, :3].copy()
                self.pending.clear()
            break
        velocity_cam = np.zeros(3)
        self.last_aim_cam[:] = 0.
        self.reason = self.guard.reason if healthy else 'WAIT_FEEDBACK_OR_FRESH_IMAGE'
        observation_ready = bool(self.pid and self.pid.stamp is not None and
                                 ros_now / 1.e9 - self.pid.stamp <= timeout)
        if healthy and self.pid and state == 'TRACKING' and self.pid.stamp is not None:
            if ros_now / 1.e9 - self.pid.stamp <= timeout:
                velocity_cam = self.pid.tick(dt, scale, self.blocked)
            else:
                self.reason = 'waiting_for_timestamped_tool_tf'
        elif healthy and state == 'LOST':
            velocity_cam = self.last_velocity_cam * np.exp(-dt / self.config['runtime']['lost_decay_s'])
        self.last_velocity_cam = velocity_cam
        pose = self.model.fk(self.joints) if self.joints is not None else None
        if pose is not None and self.reference_rotation is None:
            self.reference_rotation = pose.camera[:3, :3].copy()
        command = np.zeros(6)
        if healthy and pose is not None:
            base_linear = pose.camera[:3, :3] @ velocity_cam
            base_linear = self.guard.project_task(self.joints, base_linear, 1. / self.config['runtime']['command_hz'])
            velocity_cam = pose.camera[:3, :3].T @ base_linear
            if state == 'TRACKING':
                angular_base = norm_limit(self.config['ik']['orientation_gain'] * rotation_error(
                    self.reference_rotation, pose.camera[:3, :3]), self.config['ik']['max_angular_speed_rad_s'])
            else:
                angular_base = np.zeros(3)
            optical = np.r_[velocity_cam, pose.camera[:3, :3].T @ angular_base]
            command = camera_to_tool_twist(optical, self.handeye)
        if self.backend == 'moveit_servo':
            self.publish_twist(command)
        horizon = 1. / self.config['runtime']['command_hz']
        home_requested = bool(
            healthy and not self.found and self.config['runtime']['home_on_lost'] and
            self.home is not None and self.machine.last_seen is not None and
            now - self.machine.last_seen > self.config['runtime']['home_delay_s'])
        if home_requested:
            state = 'HOME'
        solver_healthy = (self.backend != 'moveit_servo' or
                          (self.servo_code in (0, 1, 3, 6) and now - self.servo_time <= timeout))
        can_publish = (healthy and solver_healthy and (state != 'TRACKING' or observation_ready) and
                       now - self.last_command >= horizon * .95)
        if can_publish:
            horizon = (min(.2, max(.01, now-self.last_command)) if self.last_command else horizon)
            self.last_command = now
            requested = None
            if state == 'HOME':
                requested = norm_limit((self.home - self.joints) / horizon,
                                       self.config['runtime']['home_speed_rad_s'])
            elif state in ('HOLD', 'IDLE'):
                requested = np.zeros(6)
            elif self.backend == 'weighted_dls':
                aim_angular = None
                if self.pid.partial:
                    depth = self.pid.depth or self.config['camera']['desired_depth_mm']/1000.
                    ex, ey = self.pid.error[:2]/depth
                    angular_cam = norm_limit(self.config['camera']['partial_aim_gain'] *
                                             np.array([-ey, ex, 0.]),
                                             self.config['ik']['max_angular_speed_rad_s']) * scale
                    aim_angular = pose.camera[:3, :3] @ angular_cam
                    self.last_aim_cam = angular_cam
                    # 画面箭头显示搜索的倾转方向；此时不声称执行了平移/有效测距。
                    self.last_velocity_cam[:] = 0.
                requested = self.ik.solve(self.joints, pose.camera[:3, :3] @ velocity_cam,
                                          self.reference_rotation, horizon, self.config['ik']['full_pose'],
                                          aim_angular=aim_angular)
                if not self.pid.partial and self.pid.settled:
                    requested[:] = 0.  # 对准后保持，目标移动后继续实时跟踪。
            elif self.raw_candidate is not None:
                candidate, received = self.raw_candidate
                self.raw_candidate = None
                if now - received <= .2 and self.servo_code in (0, 1, 3, 6) and now - self.servo_time <= timeout:
                    if (len(candidate.points) == 1 and len(candidate.joint_names) == 6 and
                            set(candidate.joint_names) == set(JOINT_NAMES)):
                        point = candidate.points[0]
                        candidate_horizon = point.time_from_start.sec + point.time_from_start.nanosec / 1.e9
                        if len(point.positions) == 6 and .01 <= candidate_horizon <= .5:
                            order = [candidate.joint_names.index(n) for n in JOINT_NAMES]
                            target = np.asarray(point.positions)[order]
                            requested = (target - self.joints) / candidate_horizon
            if requested is not None and np.all(np.isfinite(requested)):
                safe_velocity = self.guard.filter_velocity(self.joints, requested, horizon)
                self.blocked = np.linalg.norm(requested - safe_velocity) > 1.e-6
                self.reason = self.guard.reason
                if not self.sync_state.get('enabled', False):
                    self.command_integrator.reset()
                previous_reference = (None if self.command_integrator.target is None else
                                      self.command_integrator.target.copy())
                target = self.command_integrator.step(
                    self.joints, safe_velocity, horizon, self.config['serial']['max_step_rad'])
                valid, reason = self.guard.validate_step(self.joints, target)
                if valid:
                    self.publish_target(target, safe_velocity, horizon, ros_now)
                else:
                    self.command_integrator.target = previous_reference
                    self.reason = reason
            else:
                self.reason = 'waiting_for_safe_servo_candidate'
        ready = bool(healthy and solver_healthy and (state != 'TRACKING' or observation_ready) and
                     self.target is not None and
                     self.reason in ('OK', 'projected') and ros_now - self.target_ns < timeout * 1.e9)
        if not ready and self.reason in ('OK', 'projected'):
            if state == 'TRACKING' and not observation_ready:
                self.reason = 'waiting_for_timestamped_tool_tf'
            elif not solver_healthy:
                self.reason = 'waiting_for_safe_servo_candidate'
            else:
                self.reason = 'waiting_for_fresh_target'
        execution_ready = bool(self.guard.execution_ready and (
            self.backend != 'moveit_servo' or
            (self.scene_state.get('ready') and now - self.scene_time <= timeout)))
        payload = dict(stamp_ns=ros_now, session=self.session, feedback_source=self.feedback_source,
                       config_digest=self.config['_digest'],
                       backend=self.backend, healthy=ready, state=state, reason=self.reason,
                       target_ns=self.target_ns, feedback_ns=self.joint_stamp,
                       execution_ready=execution_ready,
                       sigma_min=self.ik.diagnostics.get('sigma_min', 0.))
        payload.update(image_age_s=self.image_age, joint_age_s=now-self.joint_time,
                       control_dt_s=dt, found=self.found, colour=self.colour,
                       depth_m=None if self.pid is None else self.pid.depth,
                       error_m=None if self.pid is None else self.pid.error.tolist(),
                       camera_velocity_m_s=self.last_velocity_cam.tolist())
        payload['partial_target'] = bool(self.last_detection and self.last_detection.clipped)
        payload['camera_angular_velocity_rad_s'] = self.last_aim_cam.tolist()
        payload['settled'] = bool(self.pid and self.pid.settled)
        self.state_pub.publish(String(data=encode(payload)))
        self.status_pub.publish(String(data=f'{state}:{self.reason}'))
        if now - self.last_log >= self.config['runtime']['log_period_s']:
            self.last_log = now
            self.print_joints()

    def print_joints(self):
        """Print a single complete table every two seconds, never sleep in control."""
        mode = self.sync_state.get('mode', 'SIMULATED' if self.feedback_source == 'simulation' else 'WAIT_SYNC')
        lines = [f'[{mode} | {self.machine.state}] {self.reason} backend={self.backend}']
        if (self.joints is None or self.target is None or
                time.monotonic() - self.joint_time > self.config['runtime']['timeout_s'] or
                self.get_clock().now().nanoseconds - self.target_ns > self.config['runtime']['timeout_s'] * 1.e9):
            lines.append('关节反馈/有效规划尚不可用；画面继续识别。')
        else:
            lines.append('轴    实测SDK°    规划SDK°    已发送SDK°   目标-实测°   角速度°/s')
            actual, target = hardware_degrees(self.joints), hardware_degrees(self.target)
            sent = self.sync_state.get('sent_sdk_deg')
            for i in range(6):
                sent_value = f'{sent[i]:+10.3f}' if sent is not None else '       n/a'
                lines.append(f'J{i+1} {actual[i]:+10.3f} {target[i]:+10.3f} {sent_value} '
                             f'{target[i]-actual[i]:+11.3f} {np.degrees(self.velocity[i]):+11.3f}')
            lines.append(f'sync accepted={self.sync_state.get("accepted_ns", 0)} '
                         f'sent={self.sync_state.get("sent_ns", 0)} '
                         f'target={self.target_ns}; motion enabled={self.sync_state.get("enabled", False)}')
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    """Run the planning node; serial is deliberately absent."""
    init_ros(args=args)
    node = None
    try:
        node = VisualServo6Dof()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.publish_twist(np.zeros(6))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
