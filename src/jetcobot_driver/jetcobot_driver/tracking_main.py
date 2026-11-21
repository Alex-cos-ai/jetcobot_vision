#!/usr/bin/env python3
"""Launch perception/planning locally; run sync_plan.py separately for feedback."""

import argparse
import os
from pathlib import Path
import sys
import threading
import uuid

# 在导入numpy前限制线程数，降低Jetson多个控制/图像进程之间的抢占。
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from visual_servo_6dof.tracking_config import load_config, package_file


def main(args=None):
    """Keep LaunchService in the main thread and heartbeat in this process."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--target-colour', choices=['auto', 'red', 'green', 'blue'])
    parser.add_argument('--ik-backend', choices=['moveit_servo', 'weighted_dls'])
    parser.add_argument('--feedback-source', choices=['hardware', 'simulation'], default='hardware')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--synthetic-camera', action='store_true', help='requires simulation; opens no devices')
    parser.add_argument('--reuse-camera', action='store_true')
    options = parser.parse_args(args)
    config = load_config(options.config)
    if options.synthetic_camera and options.feedback_source != 'simulation':
        parser.error('--synthetic-camera requires --feedback-source simulation')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription, RegisterEventHandler
    from launch.event_handlers import OnShutdown
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from visual_servo_6dof.tracking_protocol import encode, LATEST

    session = uuid.uuid4().hex
    # LaunchService 独占进程信号处理，避免 rclpy 抢先关闭上下文。
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('tracking_main')
    publisher = node.create_publisher(String, '/tracking/main_heartbeat', LATEST)

    alive_state = [True]

    def heartbeat(alive=None):
        if alive is not None:
            alive_state[0] = alive
        publisher.publish(String(data=encode(dict(
            stamp_ns=node.get_clock().now().nanoseconds, session=session, alive=alive_state[0]))))

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    node.create_timer(.1, heartbeat)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    service = LaunchService(noninteractive=True)
    launch_file = package_file('jetcobot_driver', 'launch/tracking_planner.launch.py')
    arguments = dict(config_file=config['_path'], session=session,
                     target_colour=options.target_colour or config['camera']['target_colour'],
                     ik_backend=options.ik_backend or config['ik']['backend'],
                     feedback_source=options.feedback_source,
                     show_window=str(not options.headless).lower(),
                     synthetic_camera=str(options.synthetic_camera).lower(),
                     start_camera=str(not options.synthetic_camera and not options.reuse_camera).lower())

    def shutdown(event, context):
        heartbeat(False)
        return []

    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnShutdown(on_shutdown=shutdown)), IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file)), launch_arguments=arguments.items())]))
    print(f'规划主文件 session={session} config={config["_path"]}\n'
          '硬件模式请另外运行 sync_plan.py（默认预览）；主文件不打开串口。', flush=True)
    try:
        return service.run()
    finally:
        if rclpy.ok():
            heartbeat(False)
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.)


if __name__ == '__main__':
    sys.exit(main())
