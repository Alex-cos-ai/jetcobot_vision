"""Start the mock MoveIt arm in a bent configuration for Servo testing."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Activate simulation feedback before the arm controller."""
    driver = Path(get_package_share_directory('jetcobot_driver'))
    moveit_share = Path(get_package_share_directory('jetcobot_moveit'))
    initial = driver / 'visual_servo_6dof/config/sim_initial_positions.yaml'
    moveit = (
        MoveItConfigsBuilder('jetcobot', package_name='jetcobot_moveit')
        .robot_description(mappings={'initial_positions_file': str(initial)})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )

    def include(name, condition=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(moveit_share / 'launch' / name)), condition=condition)

    feedback = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60',
                   '--service-call-timeout', '20'])
    arm = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['arm_group_controller',
                   '--controller-manager-timeout', '60',
                   '--service-call-timeout', '20'])
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        include('static_virtual_joint_tfs.launch.py'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[moveit.robot_description]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[moveit.robot_description,
                         str(moveit_share / 'config/ros2_controllers.yaml')],
             output='screen'),
        RegisterEventHandler(OnProcessExit(
            target_action=feedback, on_exit=[arm])),
        feedback,
        include('move_group.launch.py'),
        include('moveit_rviz.launch.py',
                IfCondition(LaunchConfiguration('use_rviz'))),
    ])
