"""Planning-only launch: camera, PID/IK, scene and local window; never serial."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    """Resolve one shared configuration before choosing a mutually exclusive IK backend."""
    from visual_servo_6dof.tracking_config import load_config
    config = load_config(LaunchConfiguration('config_file').perform(context))
    backend = LaunchConfiguration('ik_backend').perform(context)
    feedback = LaunchConfiguration('feedback_source').perform(context)
    driver = Path(get_package_share_directory('jetcobot_driver'))
    moveit_share = Path(get_package_share_directory('jetcobot_moveit'))
    shutdown = [EmitEvent(event=Shutdown(reason='规划/显示节点已退出'))]

    def include(path, arguments=None, condition=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=(arguments or {}).items(), condition=condition)

    actions = [
        include(driver / 'launch/wrist_camera.launch.py',
                {'video_device': config['camera']['video_device']},
                IfCondition(LaunchConfiguration('start_camera'))),
        include(moveit_share / 'launch/static_virtual_joint_tfs.launch.py'),
        include(moveit_share / 'launch/rsp.launch.py'),
        Node(package='jetcobot_driver', executable='visual_servo_6dof', output='screen', on_exit=shutdown,
             parameters=[dict(config_file=config['_path'], session=LaunchConfiguration('session'),
                              target_colour=LaunchConfiguration('target_colour'),
                              feedback_source=feedback, ik_backend=backend)]),
        Node(package='jetcobot_driver', executable='tracking_display', output='screen', on_exit=shutdown,
             condition=IfCondition(LaunchConfiguration('show_window')))]
    if feedback == 'simulation':
        from launch_ros.parameter_descriptions import ParameterValue
        actions.append(Node(package='jetcobot_driver', executable='tracking_mock', output='screen', on_exit=shutdown,
                            parameters=[dict(config_file=config['_path'], synthetic_camera=ParameterValue(
                                LaunchConfiguration('synthetic_camera'), value_type=bool))]))
    if backend == 'moveit_servo':
        actions.extend([
            include(driver / 'launch/moveit_servo.launch.py',
                    {'publish_period': str(1. / config['runtime']['command_hz']), 'config_file': config['_path'],
                     'primary_scene': 'true'}),
            Node(package='jetcobot_driver', executable='tracking_scene', output='screen', on_exit=shutdown,
                 parameters=[{'config_file': config['_path']}])])
    return actions


def generate_launch_description():
    """Declare inputs for the planning launcher; no hardware executor is included."""
    return LaunchDescription([
        DeclareLaunchArgument('config_file'), DeclareLaunchArgument('session', default_value=''),
        DeclareLaunchArgument('target_colour', default_value='auto'),
        DeclareLaunchArgument('ik_backend', default_value='moveit_servo'),
        DeclareLaunchArgument('feedback_source', default_value='hardware'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('synthetic_camera', default_value='false'),
        DeclareLaunchArgument('start_camera', default_value='true'), OpaqueFunction(function=setup)])
