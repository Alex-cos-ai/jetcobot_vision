#!/usr/bin/env python3

"""Local OpenCV camera window and six-axis JetCobot controller.

The camera image comes from a ROS Image topic. Joint commands go directly to
the MyCobot serial API, so this tool does not use the web controller or MoveIt.
Moving a slider only changes the pending value. Press ``s`` or Space to send
all six values to the arm.
"""

import argparse
import math
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from pymycobot.mycobot import MyCobot
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]
# The hardware driver's joint-6 command is 45 degrees below the URDF value.
JOINT6_URDF_OFFSET_DEG = 45.0
# Conservative limits matching the MoveIt model in this workspace.
JOINT_LIMITS = [
    (-174.7, 174.7),
    (-90.0, 90.0),
    (-90.0, 90.0),
    (-90.0, 90.0),
    (-174.7, 174.7),
    (-90.0, 90.0),
]
TRACKBAR_MAX = 1000


class LocalJointController(Node):
    def __init__(self, args):
        super().__init__('local_joint_camera_control')
        self.args = args
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.frame_id = ''
        self.selected_joint = 0
        # The GUI starts from the model zero pose. It does not send this pose
        # automatically; press Space or ``s`` after checking the preview.
        self.target_degrees = self.clamp_angles(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.last_sent = None
        self.status = '就绪：拖动滑块后按 s 或空格发送'
        self.mc = None

        self.subscription = self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.joint_state_publisher = self.create_publisher(
            JointState,
            args.joint_state_topic,
            10,
        )
        self.joint_state_published = False
        self.last_joint_state_error = None
        self.last_joint_state_error_time = 0.0

        self.warn_if_port_busy(args.port)
        try:
            self.mc = MyCobot(args.port, args.baud)
            self.status = (
                f'已连接 {args.port}；实际角度将发布到 {args.joint_state_topic}'
            )
            self.joint_state_timer = self.create_timer(
                args.joint_state_period,
                self.publish_joint_state,
            )
        except Exception as exc:
            self.status = f'串口连接失败: {exc}'
            self.get_logger().error(self.status)

    def warn_if_port_busy(self, port):
        try:
            result = subprocess.run(
                ['fuser', port],
                check=False,
                capture_output=True,
                text=True,
            )
            owners = (result.stdout + ' ' + result.stderr).strip()
            if owners:
                self.get_logger().warn(
                    f'{port} 已被其他进程占用 ({owners})；请停止 yahboom_app.service '
                    '后再控制机械臂。'
                )
        except (OSError, ValueError):
            pass

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'图像转换失败: {exc}')
            return
        with self.frame_lock:
            self.latest_frame = frame
            self.frame_id = msg.header.frame_id

    def clamp_angles(self, values):
        return [
            min(max(float(value), limits[0]), limits[1])
            for value, limits in zip(values, JOINT_LIMITS)
        ]

    def send_angles(self):
        if self.mc is None:
            self.status = '没有串口连接，无法发送'
            return
        values = self.clamp_angles(self.target_degrees)
        try:
            self.mc.send_angles(values, int(self.args.speed))
            self.target_degrees = values
            self.last_sent = list(values)
            self.status = (
                '已发送: ' + ' / '.join(
                    f'J{index + 1}={value:.1f}°'
                    for index, value in enumerate(values)
                )
            )
            self.get_logger().info(self.status)
        except Exception as exc:
            self.status = f'发送失败: {exc}'
            self.get_logger().error(self.status)

    def read_angles(self):
        if self.mc is None:
            return
        try:
            values = self.mc.get_angles()
            if values and len(values) == 6:
                self.target_degrees = self.clamp_angles(
                    [float(value) for value in values]
                )
                self.status = '已读取机械臂当前角度（仅更新滑块）'
                update_trackbars(self.target_degrees)
        except Exception as exc:
            self.status = f'读取角度失败: {exc}'
            self.get_logger().error(self.status)

    def publish_joint_state(self):
        if self.mc is None:
            return
        try:
            degrees = self.mc.get_angles()
            if (not degrees or len(degrees) != 6 or
                    not all(math.isfinite(float(value)) for value in degrees)):
                raise ValueError(f'机械臂没有返回有效六轴角度: {degrees!r}')

            positions = [math.radians(float(value)) for value in degrees]
            positions[5] += math.radians(JOINT6_URDF_OFFSET_DEG)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = positions
            self.joint_state_publisher.publish(msg)
            if self.last_joint_state_error is not None:
                self.get_logger().info('真实关节反馈已恢复，继续发布 /joint_states')
            self.last_joint_state_error = None
            if not self.joint_state_published:
                self.joint_state_published = True
                self.get_logger().info(
                    f'正在将真实机械臂角度发布到 {self.args.joint_state_topic}'
                )
        except Exception as exc:
            message = f'读取并发布关节状态失败: {exc}'
            now = time.monotonic()
            if (message != self.last_joint_state_error or
                    now - self.last_joint_state_error_time >= 5.0):
                self.last_joint_state_error = message
                self.last_joint_state_error_time = now
                self.get_logger().error(message)


def angle_to_slider(angle, joint_index):
    low, high = JOINT_LIMITS[joint_index]
    ratio = (angle - low) / (high - low)
    return int(round(min(max(ratio, 0.0), 1.0) * TRACKBAR_MAX))


def slider_to_angle(value, joint_index):
    low, high = JOINT_LIMITS[joint_index]
    return low + (high - low) * float(value) / TRACKBAR_MAX


def make_trackbar_callback(node, joint_index):
    def callback(value):
        node.target_degrees[joint_index] = slider_to_angle(value, joint_index)
        node.selected_joint = joint_index
        node.status = (
            f'待发送 J{joint_index + 1} = '
            f'{node.target_degrees[joint_index]:.1f}°；按 s 或空格发送'
        )

    return callback


def update_trackbars(values):
    for index, value in enumerate(values):
        cv2.setTrackbarPos(
            f'J{index + 1}',
            'JetCobot Joint Control',
            angle_to_slider(value, index),
        )


def draw_overlay(frame, node):
    canvas = frame.copy()
    height, width = canvas.shape[:2]
    cv2.rectangle(canvas, (0, 0), (width, 112), (0, 0, 0), -1)
    cv2.putText(
        canvas,
        'JetCobot local joint control',
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        canvas,
        's/SPACE: send   r: read angles   1-6: select joint   a/d: +/-1 deg   q/ESC: quit',
        (12, 49),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    cv2.putText(
        canvas,
        f'selected J{node.selected_joint + 1} | {node.status}',
        (12, 76),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 220, 0),
        1,
    )
    values = '  '.join(
        f'J{index + 1}:{value:.1f}'
        for index, value in enumerate(node.target_degrees)
    )
    cv2.putText(
        canvas,
        values,
        (12, 101),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )
    return canvas


def parse_args():
    parser = argparse.ArgumentParser(
        description='JetCobot 本地相机画面和六轴角度控制'
    )
    parser.add_argument('--image-topic', default='/wrist_camera/image_raw')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=1000000)
    parser.add_argument('--speed', type=int, default=30)
    parser.add_argument('--joint-state-topic', default='/joint_states')
    parser.add_argument('--joint-state-period', type=float, default=0.1)
    return parser.parse_args()


def main(args=None):
    cli_args = parse_args()
    rclpy.init(args=args)
    node = LocalJointController(cli_args)

    cv2.namedWindow('JetCobot Camera', cv2.WINDOW_NORMAL)
    cv2.namedWindow('JetCobot Joint Control', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('JetCobot Joint Control', 480, 260)
    for index in range(6):
        cv2.createTrackbar(
            f'J{index + 1}',
            'JetCobot Joint Control',
            angle_to_slider(node.target_degrees[index], index),
            TRACKBAR_MAX,
            make_trackbar_callback(node, index),
        )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            with node.frame_lock:
                frame = None if node.latest_frame is None else node.latest_frame.copy()
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    frame,
                    f'等待 {cli_args.image_topic} 图像...',
                    (30, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                )
            cv2.imshow('JetCobot Camera', draw_overlay(frame, node))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if ord('1') <= key <= ord('6'):
                node.selected_joint = key - ord('1')
            elif key in (ord('a'), ord('A')):
                index = node.selected_joint
                node.target_degrees[index] = max(
                    JOINT_LIMITS[index][0], node.target_degrees[index] - 1.0
                )
                update_trackbars(node.target_degrees)
            elif key in (ord('d'), ord('D')):
                index = node.selected_joint
                node.target_degrees[index] = min(
                    JOINT_LIMITS[index][1], node.target_degrees[index] + 1.0
                )
                update_trackbars(node.target_degrees)
            elif key in (ord('s'), ord('S'), 32):
                node.send_angles()
            elif key in (ord('r'), ord('R')):
                node.read_angles()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
