"""Execute small Servo targets and publish measured JetCobot joint states."""

import fcntl
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from pymycobot.mycobot import MyCobot
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from .trajectory_guard import (
    checked_target, hardware_degrees, JOINT_NAMES, model_radians,
)


class ServoSerialBridge(Node):
    """Keep serial ownership, feedback and commands in one executor."""

    def __init__(self):
        """Open the serial port for feedback with motion disabled."""
        super().__init__('servo_serial_bridge')
        defaults = {
            'port': '/dev/ttyUSB0', 'baud': 1000000,
            'update_rate': 10.0, 'speed': 10,
            'command_timeout': 0.35, 'max_step_rad': 0.035,
            'trajectory_topic': '/arm_group_controller/joint_trajectory',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value
        self.period = 1. / float(get('update_rate'))
        self.timeout = float(get('command_timeout'))
        self.max_step = float(get('max_step_rad'))
        self.speed = int(get('speed'))
        if (not 1 <= self.speed <= 15 or not .05 <= self.period <= .5 or
                not .005 <= self.max_step <= .035 or
                not self.period < self.timeout <= 1.):
            raise ValueError('Invalid serial speed, period, step or timeout')
        port = str(get('port'))
        self.serial_lock = open(
            Path('/tmp') / f'jetcobot-servo-{Path(port).name}.lock', 'a')
        fcntl.flock(self.serial_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        owners = subprocess.run(
            ['fuser', port], capture_output=True, text=True, timeout=2)
        if owners.stdout.strip():
            raise RuntimeError(f'{port} already belongs to {owners.stdout}')
        self.robot = MyCobot(port, int(get('baud')), timeout=.15)
        self.enabled = False
        self.was_commanding = False
        self.latest = None
        self.current = None
        self.feedback_time = None
        self.feedback_error = 'waiting for first serial reading'
        self.servo_status = None
        self.servo_status_time = 0.
        self.enabled_at = 0.
        self.joint_publisher = self.create_publisher(
            JointState, '/joint_states', 10)
        self.status_publisher = self.create_publisher(String, '~/status', 10)
        self.create_subscription(
            JointTrajectory, str(get('trajectory_topic')),
            self.trajectory_callback, 1)
        self.create_subscription(Int8, '/servo_node/status',
                                 self.servo_status_callback, 1)
        self.create_service(SetBool, '~/enable_motion', self.enable_callback)
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            'Reading actual joints; motion disabled. Enable explicitly via '
            '/servo_serial_bridge/enable_motion.')

    def servo_status_callback(self, message):
        """Keep the Servo protection status for the execution gate."""
        self.servo_status = message.data
        self.servo_status_time = time.monotonic()

    def trajectory_callback(self, message):
        """Retain only the latest single-step Servo trajectory."""
        self.latest = (message, time.monotonic())

    def enable_callback(self, request, response):
        """Enable only fresh Servo streaming with one real feedback source."""
        if not request.data:
            self.halt('disabled_by_user')
            response.success = True
            return response
        others = [endpoint.node_name for endpoint in
                  self.get_publishers_info_by_topic('/joint_states')
                  if endpoint.node_name != self.get_name()]
        if others:
            response.message = f'Conflicting /joint_states publishers: {others}'
        elif self.current is None:
            response.message = f'No valid hardware joint feedback: {self.feedback_error}'
        elif time.monotonic() - self.feedback_time > self.timeout:
            response.message = 'Hardware joint feedback is stale'
        elif (time.monotonic() - self.servo_status_time > self.timeout or
              self.servo_status not in (0, 1, 3, 6)):
            response.message = (
                f'Servo stopped or unavailable (status={self.servo_status})')
        else:
            # Fresh mode replaces the previous tiny target rather than queuing
            # an unbounded backlog of camera tracking commands.
            try:
                self.robot.set_fresh_mode(1)
                if self.robot.get_fresh_mode() != 1:
                    raise RuntimeError('Cannot confirm fresh mode')
            except Exception as exc:
                self.halt(f'cannot_enable: {exc}')
                response.message = str(exc)
                return response
            self.enabled = True
            self.enabled_at = time.monotonic()
            self.latest = None
            response.success = True
            response.message = 'Enabled; waiting for a new command'
        return response

    def halt(self, reason):
        """Stop active hardware movement and require explicit re-enabling."""
        if self.enabled or self.was_commanding:
            try:
                self.robot.stop()
            except Exception as exc:
                self.get_logger().error(f'Serial stop failed: {exc}')
        self.enabled = False
        self.was_commanding = False
        self.latest = None
        self.publish_status(reason)

    def publish_status(self, value):
        """Publish an execution status suitable for terminal inspection."""
        if not self.context.ok():
            return
        message = String()
        message.data = value
        self.status_publisher.publish(message)

    def tick(self):
        """Read feedback, then execute at most one checked fresh target."""
        before = self.get_clock().now().nanoseconds
        try:
            angles = self.robot.get_angles()
            if angles is None:
                raise ValueError('no reply to get_angles()')
            current = model_radians(angles)
        except Exception as exc:
            self.current = None
            self.feedback_time = None
            self.feedback_error = f'{type(exc).__name__}: {exc}'
            self.halt(f'feedback_fault: {self.feedback_error}')
            self.get_logger().error(
                f'Serial joint read failed: {self.feedback_error}',
                throttle_duration_sec=2.)
            return
        after = self.get_clock().now().nanoseconds
        recovering = self.current is None
        self.current = current
        self.feedback_time = time.monotonic()
        self.feedback_error = None
        if recovering:
            self.get_logger().info('Receiving valid joint feedback from the serial controller')
        try:
            message = JointState()
            # Approximate the sampling instant within the serial round trip.
            stamp = (before + after) // 2
            message.header.stamp.sec = int(stamp // 1000000000)
            message.header.stamp.nanosec = int(stamp % 1000000000)
            message.name = JOINT_NAMES
            message.position = self.current.tolist()
            self.joint_publisher.publish(message)
            others = [x.node_name for x in
                      self.get_publishers_info_by_topic('/joint_states')
                      if x.node_name != self.get_name()]
            if others:
                self.halt('conflicting_joint_state_publishers')
                return
            if not self.enabled:
                self.publish_status('feedback_only_motion_disabled')
                return
            now = time.monotonic()
            if (now - self.servo_status_time > self.timeout or
                    self.servo_status not in (0, 1, 3, 6)):
                self.halt(f'servo_protection_{self.servo_status}')
                return
            if self.latest is None:
                if now - self.enabled_at > self.timeout:
                    self.halt('command_timeout')
                return
            trajectory, received = self.latest
            if now - received > self.timeout:
                self.halt('command_timeout')
                return
            if len(trajectory.points) != 1:
                raise ValueError('Only a single-step Servo target is accepted')
            stamp = trajectory.header.stamp
            if stamp.sec or stamp.nanosec:
                age = self.get_clock().now().nanoseconds / 1.e9 - (
                    stamp.sec + stamp.nanosec / 1.e9)
                if not -.15 <= age <= self.timeout:
                    raise ValueError('Trajectory timestamp is stale or future')
            target = checked_target(
                trajectory.joint_names, trajectory.points[0].positions,
                self.current, self.max_step)
            error = float(np.max(np.abs(target - self.current)))
            if error > .001:
                self.robot.send_angles(hardware_degrees(target), self.speed)
                self.was_commanding = True
                self.publish_status('executing_servo_target')
            else:
                if self.was_commanding:
                    self.robot.stop()
                self.was_commanding = False
                self.publish_status('holding')
        except Exception as exc:
            self.halt(f'fault: {exc}')
            self.get_logger().error(str(exc), throttle_duration_sec=2.)


def main(args=None):
    """Start feedback and Servo execution with a disabled motion gate."""
    rclpy.init(args=args)
    node = None
    try:
        node = ServoSerialBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal interrupt and launch may both deliver SIGINT. Finish the
        # stop and serial cleanup even if a second interrupt arrives.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.halt('shutdown')
            node.robot._serial_port.close()
            node.serial_lock.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
