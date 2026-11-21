"""Mock feedback and optional synthetic camera; never imports a hardware SDK."""

import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from .tracking_protocol import init_ros
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from .kinematics import Kinematics
from .tracking_config import load_config
from .tracking_protocol import encode, LATEST, stamp_ns
from .trajectory_guard import JOINT_NAMES


class MockArm(Node):
    """Advance bounded simulated joints, then project a world-frame target."""

    def __init__(self):
        """Configure simulation without serial access."""
        super().__init__('tracking_mock')
        self.declare_parameter('config_file', '')
        self.declare_parameter('synthetic_camera', False)
        self.config = load_config(self.get_parameter('config_file').value or None)
        cal = json.loads(Path(self.config['camera']['calibration_file']).read_text())
        self.model = Kinematics(self.config['model']['urdf'], cal['T_gripper_camera'])
        self.q = np.array(self.config['simulation']['initial_angles_rad'])
        self.target = self.q.copy()
        self.last_goal = 0.
        self.last_ns = 0
        self.last_tick = time.monotonic()
        self.started = self.last_tick
        pose = self.model.fk(self.q).camera
        self.world_centre = pose[:3, 3] + pose[:3, :3] @ np.array([0., 0., .2])
        side = self.config['camera']['square_side_mm'] / 2000.
        self.world_corners = np.array([[-side, -side, 0], [side, -side, 0],
                                       [side, side, 0], [-side, side, 0]]) @ pose[:3, :3].T
        self.world_axes = pose[:3, :3]
        self.k = np.array(cal['camera_model']['K']).reshape(3, 3)
        self.distortion = np.array(cal['camera_model']['D'])
        self.camera_model = cal['camera_model']
        self.pub = self.create_publisher(JointState, '/joint_states', LATEST)
        self.status = self.create_publisher(String, '/tracking/sync_state', LATEST)
        self.create_subscription(JointTrajectory, '/tracking/joint_target', self.command, LATEST)
        self.create_timer(.1, self.tick)
        if self.get_parameter('synthetic_camera').value:
            self.bridge = CvBridge()
            self.image = self.create_publisher(Image, self.config['camera']['image_topic'], 1)
            self.info = self.create_publisher(CameraInfo, self.config['camera']['camera_info_topic'], 1)
            self.create_timer(1. / 30., self.camera)

    def command(self, message):
        """Consume only small named simulated targets."""
        if len(message.points) != 1 or message.joint_names != JOINT_NAMES:
            return
        goal = np.asarray(message.points[0].positions)
        if goal.shape == (6,) and np.all(np.isfinite(goal)) and np.max(np.abs(goal - self.q)) <= .035:
            self.target, self.last_goal = goal, time.monotonic()
            self.last_ns = stamp_ns(message.header.stamp)

    def tick(self):
        """Move with lag and bounds, then publish simulated actual positions."""
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        if now - self.last_goal < .35:
            velocity = (self.target - self.q) / self.config['simulation']['lag_s']
            vmax = np.array(self.config['model']['max_velocity_rad_s'])
            self.q += np.clip(velocity, -vmax, vmax) * dt
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name, message.position = JOINT_NAMES, self.q.tolist()
        self.pub.publish(message)
        self.status.publish(String(data=encode(dict(mode='SIMULATED', enabled=False,
                                                    accepted_ns=self.last_ns, sent_ns=0,
                                                    feedback_source='simulation'))))

    def camera(self):
        """Generate images from FK and a moving world target, forming a closed loop."""
        elapsed = time.monotonic() - self.started
        offset = self.world_axes @ np.array([.015 * np.sin(elapsed * .3),
                                             .01 * np.sin(elapsed * .2), .01 * np.sin(elapsed * .15)])
        pose = self.model.fk(self.q).camera
        corners = (self.world_corners + self.world_centre + offset - pose[:3, 3]) @ pose[:3, :3]
        image = np.zeros((self.camera_model['height'], self.camera_model['width'], 3), dtype=np.uint8)
        if np.all(corners[:, 2] > .02):
            pixels = cv2.projectPoints(corners, np.zeros(3), np.zeros(3), self.k, self.distortion)[0]
            cv2.fillConvexPoly(image, pixels.astype(np.int32), (255, 0, 0))
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = 'camera_optical_frame'
        info.height, info.width = image.shape[:2]
        info.distortion_model = self.camera_model['distortion_model']
        info.k, info.d = self.k.ravel().tolist(), self.distortion.tolist()
        self.info.publish(info)
        message = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        message.header = info.header
        self.image.publish(message)


def main(args=None):
    """Run the no-serial simulated feedback node."""
    init_ros(args=args)
    node = MockArm()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
