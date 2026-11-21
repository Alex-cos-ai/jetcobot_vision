#!/usr/bin/env python3
"""Independent serial feedback/execution entry; defaults to read-only preview."""

import argparse
from collections import deque
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

# 支持直接运行源码文件；不依赖终端当前目录。
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import rclpy
from visual_servo_6dof.tracking_protocol import init_ros
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from visual_servo_6dof.arm_controller import ArmController
from visual_servo_6dof.kinematics import Kinematics, check_sdk_fk
from visual_servo_6dof.safety import SafetyGuard
from visual_servo_6dof.tracking_config import load_config
from visual_servo_6dof.tracking_protocol import decode, encode, LATEST, stamp_ns
from visual_servo_6dof.trajectory_guard import hardware_degrees, JOINT_NAMES


class SyncExecutor(Node):
    """Own one serial port; never treat actual JointState as a motion command."""

    def __init__(self, config, mode, robot=None):
        """Open serial for reads and wait for a fresh planner session."""
        super().__init__('tracking_sync')
        self.config = config
        c = config['serial']
        calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
        model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'],
                           np.radians(config['model']['margin_deg']), config['model'].get('hardware_limits_deg'))
        self.lock = None
        if robot is None:
            from visual_servo_6dof.serial_sdk import open_robot
            self.lock = open(Path('/tmp') / f'jetcobot-servo-{Path(c["port"]).name}.lock', 'a')
            try:
                fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                owners = subprocess.run(['fuser', c['port']], capture_output=True, text=True, timeout=2)
                if owners.returncode != 1:
                    raise RuntimeError(f'{c["port"]} unavailable/occupied: {owners.stdout} {owners.stderr}')
                robot = open_robot(c['port'], c['baud'], c['read_timeout_s'])
            except Exception:
                self.lock.close()
                raise
        self.arm = ArmController(robot, model, SafetyGuard(model, config), config, mode)
        if mode == 'execute' and self.lock is not None:
            # 每次执行重新核对，不让YAML中的旧标志代替当次反馈检查。
            result = check_sdk_fk(robot, model, config)
            if not result['ok']:
                self.close()
                raise RuntimeError(f'execute_fk_check_failed:{result}')
            actual = [(robot.get_joint_min_angle(i), robot.get_joint_max_angle(i)) for i in range(1, 7)]
            if not np.allclose(np.asarray(actual, dtype=float), config['model']['hardware_limits_deg'], atol=.1):
                self.close()
                raise RuntimeError(f'execute_hardware_limits_mismatch:{actual}')
            if not self.arm.feedback(time.monotonic()):
                self.close()
                raise RuntimeError('execute_feedback_unavailable')
            valid, reason = self.arm.guard.valid(self.arm.current)
            if not valid:
                self.close()
                raise RuntimeError(f'execute_initial_pose_unsafe:{reason}')
        self.latest = None
        self.planner = self.heartbeat = {}
        self.planner_history = deque(maxlen=16)
        self.planner_time = self.heartbeat_time = 0.
        self.session = None
        self.retired_sessions = set()
        self.last_target_ns = 0
        self.created_ns = self.get_clock().now().nanoseconds
        self.previous_reason = ''
        self.joint_pub = self.create_publisher(JointState, '/joint_states', LATEST)
        self.status_pub = self.create_publisher(String, '/tracking/sync_state', LATEST)
        self.create_subscription(JointTrajectory, '/tracking/joint_target', self.target_callback, LATEST)
        self.create_subscription(String, '/tracking/planner_state', self.planner_callback, LATEST)
        self.create_subscription(String, '/tracking/main_heartbeat', self.heartbeat_callback, LATEST)
        # 串口/碰撞检查不阻塞心跳与目标订阅；独立互斥组仍保证串口只由一个tick访问。
        self.io_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1. / c['update_hz'], self.tick, callback_group=self.io_group)
        self.get_logger().info(f'sync mode={mode}; config={config["_path"]}; only this process opens serial')

    def target_callback(self, message):
        """Keep only the latest goal received after the current process began."""
        self.latest = message, time.monotonic()

    def planner_callback(self, message):
        """Save timestamped planner health; malformed state cannot authorize motion."""
        self.planner, self.planner_time = decode(message), time.monotonic()
        self.planner_history.append(self.planner)

    def heartbeat_callback(self, message):
        """Watch the launcher itself, independently of surviving child nodes."""
        self.heartbeat, self.heartbeat_time = decode(message), time.monotonic()

    def check_health(self, now, ros_now):
        """Check session, publishers, metadata and watchdogs before accepting goals."""
        timeout = self.config['serial']['timeout_s']
        for topic, expected in [('/joint_states', 1), ('/tracking/joint_target', 1),
                                ('/tracking/planner_state', 1), ('/tracking/main_heartbeat', 1)]:
            endpoints = self.get_publishers_info_by_topic(topic)
            if len(endpoints) > expected:
                return f'conflicting_publishers:{topic}'
        if now - self.heartbeat_time > timeout or not self.heartbeat.get('alive'):
            return 'waiting_for_main_heartbeat'
        if now - self.planner_time > timeout:
            return 'waiting_for_planner'
        for data in (self.heartbeat, self.planner):
            try:
                age = (ros_now - int(data['stamp_ns'])) / 1.e9
                if not -.02 <= age <= timeout:
                    return 'stale_planner_metadata'
            except (KeyError, TypeError, ValueError):
                return 'invalid_planner_metadata'
        session = self.heartbeat.get('session')
        if not session or session != self.planner.get('session'):
            return 'session_mismatch'
        if self.session is None:
            self.session = session
        if session != self.session or session in self.retired_sessions:
            return 'planner_restarted_restart_sync'
        if self.planner.get('feedback_source') != 'hardware':
            return 'simulation_conflicts_with_hardware'
        if self.planner.get('config_digest') != self.config['_digest']:
            return 'configuration_mismatch'
        if self.planner.get('backend') not in ('moveit_servo', 'weighted_dls'):
            return 'unsupported_planning_backend'
        if not self.planner.get('healthy'):
            reason = self.planner.get('reason', 'planner_not_ready')
            return 'waiting_for_fresh_target' if reason in ('OK', 'projected') else reason
        if self.arm.enabled and not self.planner.get('execution_ready'):
            return 'planning_safety_not_ready'
        return None

    def tick(self):
        """Read actual joints before waiting for any planning information."""
        now = time.monotonic()
        before = self.get_clock().now().nanoseconds
        valid = self.arm.feedback(now)
        after = self.get_clock().now().nanoseconds
        if valid:
            message = JointState()
            sample = (before + after) // 2
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(sample, 1000000000)
            message.name, message.position = JOINT_NAMES, self.arm.current.tolist()
            self.joint_pub.publish(message)
        try:
            error = self.check_health(time.monotonic(), after)
            if error or not valid:
                feedback_recent = (self.arm.feedback_time is not None and
                                   time.monotonic()-self.arm.feedback_time <= self.config['serial']['timeout_s'])
                if (not valid and feedback_recent and not self.arm.faulted and
                        self.arm.consecutive_feedback_failures < 3):
                    self.arm.pause(self.arm.reason, time.monotonic())
                elif valid and error in ('WAIT_FEEDBACK_OR_FRESH_IMAGE', 'waiting_for_timestamped_tool_tf',
                                         'waiting_for_safe_servo_candidate', 'waiting_for_fresh_target',
                                         'waiting_for_planner', 'stale_planner_metadata'):
                    self.arm.pause(error, time.monotonic())
                else:
                    self.arm.stop(error or self.arm.reason, latch=self.arm.enabled)
                self.latest = None
            elif self.latest is not None:
                message, received = self.latest
                generation = stamp_ns(message.header.stamp)
                age = (after - generation) / 1.e9
                timeout = self.config['serial']['timeout_s']
                if not -.02 <= age <= timeout or time.monotonic() - received > timeout:
                    raise ValueError('stale_command')
                if generation < self.created_ns:
                    raise ValueError('command_predates_executor')
                # DDS分别传输两个topic，不能假定“最新状态”一定对应“最新目标”。
                matched = next((p for p in reversed(self.planner_history)
                                if p.get('target_ns') == generation and p.get('healthy') and
                                p.get('session') == self.session and
                                p.get('config_digest') == self.config['_digest']), None)
                if self.planner.get('target_ns') == generation and self.planner.get('healthy'):
                    matched = self.planner
                if matched is not None and generation > self.last_target_ns:
                    if (len(message.points) != 1 or set(message.joint_names) != set(JOINT_NAMES) or
                            len(message.joint_names) != 6):
                        raise ValueError('expected_six_unique_joints_and_one_point')
                    point = message.points[0]
                    if len(point.positions) != 6 or len(point.velocities) != 6:
                        raise ValueError('missing_positions_or_velocities')
                    order = [message.joint_names.index(name) for name in JOINT_NAMES]
                    target, velocity = np.array(point.positions)[order], np.array(point.velocities)[order]
                    horizon = point.time_from_start.sec + point.time_from_start.nanosec / 1.e9
                    self.arm.accept(target, velocity, horizon, generation, received, time.monotonic())
                    self.last_target_ns = generation
                    if self.arm.mode == 'execute' and not self.arm.enabled and not self.arm.faulted:
                        if matched.get('state') != 'TRACKING':
                            self.arm.reason = 'waiting_for_tracking_before_enable'
                        elif not self.arm.guard.execution_ready or not matched.get('execution_ready'):
                            self.arm.reason = 'execution_blocked:model_or_table_unverified'
                        elif self.arm.enable(time.monotonic()):
                            self.latest = None
                # 相同目标不重发；即便节点仍活着，也不能给旧目标重新计时。
                if age > timeout:
                    raise ValueError('command_timeout')
            elif self.arm.enabled and now - self.arm.enabled_at > self.config['serial']['timeout_s']:
                raise ValueError('waiting_new_command_timeout')
        except Exception as exc:
            if str(exc) in ('joint_acceleration_limit', 'stale_command', 'command_timeout'):
                # 暂停后旧规划可能先于软启动状态到达；拒绝该步，等待新的限加速度规划。
                reason = ('waiting_for_joint_acceleration' if str(exc) == 'joint_acceleration_limit'
                          else 'waiting_for_fresh_target')
                self.arm.pause(reason, time.monotonic())
            else:
                self.arm.stop(str(exc), latch=self.arm.enabled)
            self.latest = None
        status = dict(stamp_ns=after, session=self.session, mode=self.arm.mode,
                      enabled=self.arm.enabled, faulted=self.arm.faulted, reason=self.arm.reason,
                      fault_reason=self.arm.fault_reason,
                      accepted_ns=self.arm.accepted_ns, sent_ns=self.arm.sent_ns,
                      feedback_failures=self.arm.feedback_failures,
                      consecutive_feedback_failures=self.arm.consecutive_feedback_failures,
                      feedback_valid=valid, feedback_source='hardware', config_digest=self.config['_digest'])
        status['sent_sdk_deg'] = None if self.arm.last_sent is None else hardware_degrees(self.arm.last_sent)
        status['sent_speed_level'] = self.arm.sent_speed
        self.status_pub.publish(String(data=encode(status)))
        if self.arm.reason != self.previous_reason:
            self.get_logger().info(self.arm.reason)
            self.previous_reason = self.arm.reason

    def close(self):
        """Stop an enabled executor and release only resources this process owns."""
        try:
            self.arm.stop('sync_shutdown')
        finally:
            if hasattr(self.arm.robot, '_serial_port'):
                self.arm.robot._serial_port.close()
            if self.lock is not None:
                self.lock.close()


def main(args=None):
    """Run preview by default; execute requires an explicit command-line choice."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--mode', choices=['preview', 'execute'])
    parser.add_argument('--fk-check', action='store_true', help='read-only FK comparison, then exit')
    options, ros_args = parser.parse_known_args(args)
    config = load_config(options.config)
    init_ros(args=ros_args)
    node = None
    executor = None
    try:
        # 每次执行都必须显式选择 --mode execute，YAML不能使普通启动意外运动。
        node = SyncExecutor(config, 'preview' if options.fk_check else (options.mode or 'preview'))
        if options.fk_check:
            result = check_sdk_fk(node.arm.robot, node.arm.model, config)
            print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
            if not result['ok']:
                node.get_logger().warning('FK自检未通过：检查误差和SDK坐标系对齐配置。')
        else:
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if executor is not None:
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
