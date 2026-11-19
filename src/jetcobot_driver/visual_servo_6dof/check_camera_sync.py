"""Inspect live camera timestamps, joint feedback and TF without commanding motion."""

from collections import Counter, deque
from statistics import median
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def main(args=None):
    """Sample a running system and print measurements and suggested repairs."""
    rclpy.init(args=args)
    node = Node('check_camera_sync')
    node.declare_parameter('sample_seconds', 6.)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    images, joints, image_times = [], [], []
    pending, matches = deque(), []
    statuses, bridge = Counter(), Counter()
    info_count = []

    def age(message):
        return (node.get_clock().now().nanoseconds -
                Time.from_msg(message.header.stamp).nanoseconds) / 1.e9

    def image_callback(message):
        received = time.monotonic()
        images.append(age(message))
        image_times.append(received)
        pending.append((received, Time.from_msg(message.header.stamp)))

    node.create_subscription(
        Image, '/wrist_camera/image_raw', image_callback, qos_profile_sensor_data)
    node.create_subscription(
        CameraInfo, '/wrist_camera/camera_info', lambda m: info_count.append(1),
        qos_profile_sensor_data)
    node.create_subscription(
        JointState, '/joint_states', lambda m: joints.append(age(m)), qos_profile_sensor_data)
    node.create_subscription(
        String, '/visual_servo_6dof/status', lambda m: statuses.update([m.data]), 10)
    node.create_subscription(
        String, '/servo_serial_bridge/status', lambda m: bridge.update([m.data]), 10)
    parameter_client = node.create_client(GetParameters, '/visual_servo_6dof/get_parameters')
    parameters = None
    timeout = .35
    print('只读采样约 6 秒；本命令不启用运动，也不停止已在运行的运动。', flush=True)
    try:
        end = time.monotonic() + float(node.get_parameter('sample_seconds').value)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if parameters is None and parameter_client.service_is_ready():
                parameters = parameter_client.call_async(GetParameters.Request(
                    names=['target_timeout', 'use_sim_time']))
            # Give the 10 Hz joint feedback time to catch up to this image.
            while pending and time.monotonic() - pending[0][0] >= .2:
                _, stamp = pending.popleft()
                matches.append(buffer.can_transform('base_link', '6_Link', stamp))

        if parameters is not None and parameters.done() and parameters.result() is not None:
            values = parameters.result().values
            timeout = values[0].double_value
            if values[1].bool_value != node.get_parameter('use_sim_time').value:
                print('异常：跟踪器与诊断器 use_sim_time 不一致；真机各节点应为 false。')
        print(f'当前允许的图像时间戳差：0..{timeout * 1000:.0f} ms')
        publishers = {}
        for topic in ('/wrist_camera/image_raw', '/joint_states', '/visual_servo_6dof/status'):
            publishers[topic] = [
                f'{p.node_namespace.rstrip("/")}/{p.node_name}'
                for p in node.get_publishers_info_by_topic(topic)]
            print(f'{topic} 发布者：{publishers[topic] or "无"}')
        for label, values in (('图像', images), ('关节反馈', joints)):
            if values:
                measurements = '/'.join(f'{v * 1000:.0f}' for v in (
                    min(values), median(values), max(values)))
                print(f'{label} {len(values)} 帧，时间戳差 最小/中位/最大：{measurements} ms')
            else:
                print(f'{label}：没有收到数据')
        if len(image_times) > 1:
            print(f'观测到的图像到达率：{(len(images) - 1) / (image_times[-1] - image_times[0]):.1f} Hz')
        print(f'相机内参：{len(info_count)} 帧')
        print(f'图像时刻的工具 TF 可匹配：{sum(matches)}/{len(matches)} 帧（等待 200 ms 后检查）')
        print(f'跟踪状态：{dict(statuses)}；桥接状态：{dict(bridge)}')
        service_names = {name for name, _ in node.get_service_names_and_types()}
        service = '/servo_serial_bridge/enable_motion'
        print(f'启用运动服务：{"存在" if service in service_names else "不存在"}')
        if len(publishers['/visual_servo_6dof/status']) > 1:
            print('处理：关闭额外的 visual_servo.launch.py；真机 launch 已包含跟踪器。')
        if any(name.endswith('/usb_cam') for name in publishers['/wrist_camera/image_raw']):
            print('处理：本机旧 usb_cam 有时间戳换算问题。退出它，使用 wrist_camera.launch.py。')
        if images and any(a < 0 or a > timeout for a in images):
            print('处理：有图像时间戳不在允许范围；检查驱动、时钟和负载，不要直接调大超时。')
        if not images:
            print('处理：检查相机进程和 /dev/video0；确认各终端 ROS_DOMAIN_ID 一致。')
        if not joints or service not in service_names:
            print('处理：检查真机 launch 中串口桥接是否退出；用 fuser -v /dev/ttyUSB0 查占用。')
        if matches and not any(matches) and joints:
            print('处理：检查 TF、robot_state_publisher 及相机和关节反馈的时钟是否一致。')
    except KeyboardInterrupt:
        pass
    finally:
        listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
