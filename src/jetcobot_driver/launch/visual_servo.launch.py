"""Launch the JetCobot six degree of freedom visual servo node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Return the visual servo node and its parameter file."""
    config = os.path.join(
        get_package_share_directory('jetcobot_driver'),
        'visual_servo_6dof', 'config', 'visual_servo.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_colour', default_value='auto',
            description='Track auto (largest), red, green or blue.'),
        Node(
            package='jetcobot_driver',
            executable='visual_servo_6dof',
            name='visual_servo_6dof',
            output='screen',
            parameters=[{'config_file': config,
                'target_colour': LaunchConfiguration('target_colour')}]),
    ])
