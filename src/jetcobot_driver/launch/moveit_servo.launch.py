"""Launch JetCobot MoveIt Servo for the existing mock controller."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
from visual_servo_6dof.tracking_config import load_config


def generate_launch_description():
    """Load the JetCobot model and publish Servo joint trajectories."""
    moveit = (
        MoveItConfigsBuilder('jetcobot', package_name='jetcobot_moveit')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )
    config = (
        Path(get_package_share_directory('jetcobot_driver')) /
        'visual_servo_6dof/config/moveit_servo.yaml'
    )
    with config.open(encoding='utf-8') as stream:
        servo_parameters = yaml.safe_load(stream)

    from launch.actions import OpaqueFunction

    def create_node(context):
        config = load_config(LaunchConfiguration('config_file').perform(context) or None)
        servo_parameters['joint_limit_margin'] = config['model']['margin_deg'] * 3.141592653589793 / 180.
        servo_limits = {'robot_description_planning': {'joint_limits': {
            f'{index}_Joint': {
                'has_velocity_limits': True,
                'max_velocity': config['model']['max_velocity_rad_s'][index - 1],
                'has_acceleration_limits': True,
                'max_acceleration': config['model']['max_acceleration_rad_s2'][index - 1],
            } for index in range(1, 7)
        }}}
        return [Node(
            package='moveit_servo', executable='servo_node_main', name='servo_node', output='screen',
            parameters=[{'moveit_servo': servo_parameters}, moveit.robot_description,
                        moveit.robot_description_semantic, moveit.robot_description_kinematics,
                        servo_limits,
                        {'moveit_servo.publish_period': ParameterValue(
                            LaunchConfiguration('publish_period'), value_type=float),
                         'moveit_servo.joint_topic': LaunchConfiguration('joint_topic'),
                         'moveit_servo.is_primary_planning_scene_monitor': ParameterValue(
                             LaunchConfiguration('primary_scene'), value_type=bool)}])]
    return LaunchDescription([
        DeclareLaunchArgument('publish_period', default_value='0.0333333333'),
        DeclareLaunchArgument('joint_topic', default_value='/joint_states'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('primary_scene', default_value='false'),
        OpaqueFunction(function=create_node),
    ])
