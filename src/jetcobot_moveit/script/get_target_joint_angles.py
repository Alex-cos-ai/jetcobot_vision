#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CurrentJointAngles(Node):
    def __init__(self):
        super().__init__('get_current_joint_angles')
        self.joint_state = None
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def print_current_angles(self):
        self.get_logger().info('等待 /joint_states 当前关节状态...')
        timeout_sec = 10.0
        start_time = time.monotonic()

        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - start_time
            if elapsed >= timeout_sec:
                self.get_logger().error(
                    '等待 /joint_states 超时。请确认机器人或仿真已启动，且正在发布关节状态。'
                )
                return

        msg = self.joint_state
        if not msg.name or not msg.position:
            self.get_logger().error('/joint_states 消息中没有关节名称或角度数据。')
            return

        print('\n当前关节角:')
        for name, angle_rad in zip(msg.name, msg.position):
            print(
                f'  {name}: {angle_rad:.6f} rad  '
                f'({math.degrees(angle_rad):.2f} deg)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CurrentJointAngles()
    try:
        node.print_current_angles()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
