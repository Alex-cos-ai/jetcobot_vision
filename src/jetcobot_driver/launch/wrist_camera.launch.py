"""Publish wrist USB camera images with the existing camera calibration."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the camera at the resolution used for intrinsic calibration."""
    calibration = Path.home() / '.ros/camera_info/wrist_camera.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'camera_info_url', default_value=calibration.as_uri()),
        Node(
            package='jetcobot_driver',
            executable='wrist_camera',
            namespace='wrist_camera',
            name='wrist_camera',
            output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='夹爪相机已退出'))],
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'frame_id': 'camera_optical_frame',
            }]),
    ])
