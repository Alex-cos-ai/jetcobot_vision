"""Launch real joint feedback, MoveIt Servo and calibrated colour tracking."""

from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_devices(context):
    """Reject occupied devices before any tracking or motion nodes start."""
    devices = ['/dev/ttyUSB0']
    if IfCondition(LaunchConfiguration('start_camera')).evaluate(context):
        devices.append('/dev/video0')
    for device in devices:
        if not Path(device).exists():
            raise RuntimeError(f'设备不存在：{device}')
        owners = subprocess.run(
            ['fuser', device], capture_output=True, text=True, timeout=2)
        if owners.returncode == 0:
            raise RuntimeError(
                f'{device} 被进程 {owners.stdout.strip()} 占用。'
                '请先退出旧相机、手动调姿工具或旧真机 launch，再重启。')
        if owners.returncode != 1:
            raise RuntimeError(f'无法检查 {device}：{owners.stderr.strip()}')


def generate_launch_description():
    """Use the serial bridge as the sole actual joint-state publisher."""
    driver = Path(get_package_share_directory('jetcobot_driver'))
    moveit = Path(get_package_share_directory('jetcobot_moveit'))

    def include(path, arguments=None, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments=(arguments or {}).items(), condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('target_colour', default_value='auto'),
        OpaqueFunction(function=check_devices),
        Node(package='jetcobot_driver', executable='servo_serial_bridge',
             name='servo_serial_bridge', output='screen',
             on_exit=[EmitEvent(event=Shutdown(
                 reason='串口桥接已退出，停止真机跟随流程'))]),
        include(driver / 'launch/wrist_camera.launch.py',
                condition=IfCondition(LaunchConfiguration('start_camera'))),
        include(moveit / 'launch/static_virtual_joint_tfs.launch.py'),
        include(moveit / 'launch/rsp.launch.py'),
        include(moveit / 'launch/move_group.launch.py',
                {'allow_trajectory_execution': 'false'}),
        include(moveit / 'launch/moveit_rviz.launch.py',
                condition=IfCondition(LaunchConfiguration('use_rviz'))),
        include(driver / 'launch/moveit_servo.launch.py',
                {'publish_period': '0.1'}),
        include(driver / 'launch/visual_servo.launch.py',
                {'target_colour': LaunchConfiguration('target_colour')}),
    ])
