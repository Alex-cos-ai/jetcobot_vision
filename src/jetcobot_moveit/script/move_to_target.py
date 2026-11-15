#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes
)

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class MoveToTarget(Node):

    def __init__(self):
        super().__init__('move_to_target')

        # MoveGroup Action Client
        self.action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # ==============================
        # 目标参数
        # ==============================

        self.group_name = 'arm_group'
        self.end_effector_link = '6_Link'
        self.reference_frame = 'dummy'

        # 目标位置（单位：m）
        self.target_x = 0.15
        self.target_y = -0.063
        self.target_z = 0.30

        # 目标姿态（四元数）
        self.target_qx = 0.479426
        self.target_qy = 0.0
        self.target_qz = 0.0
        self.target_qw = 0.877583

        # ==============================
        # 规划参数
        # ==============================

        self.planner_id = 'RRTConnectkConfigDefault'
        self.pipeline_id = 'ompl'

        self.planning_time = 5.0
        self.planning_attempts = 10

        self.velocity_scaling = 0.2
        self.acceleration_scaling = 0.2

        # 位置容差（m）
        self.position_tolerance = 0.01

        # 姿态容差（rad）
        self.orientation_tolerance = 0.02

    def create_goal_constraints(self):

        constraints = Constraints()

        # ==============================
        # 位置约束
        # ==============================

        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = self.reference_frame
        position_constraint.link_name = self.end_effector_link

        position_constraint.weight = 1.0

        # 使用球形区域定义目标位置容差
        region = BoundingVolume()

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]

        region.primitives.append(sphere)

        target = Pose()
        target.position.x = self.target_x
        target.position.y = self.target_y
        target.position.z = self.target_z

        target.orientation.w = 1.0

        region.primitive_poses.append(target)

        position_constraint.constraint_region = region

        constraints.position_constraints.append(
            position_constraint
        )

        # ==============================
        # 姿态约束
        # ==============================

        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = self.reference_frame
        orientation_constraint.link_name = self.end_effector_link

        orientation_constraint.orientation.x = self.target_qx
        orientation_constraint.orientation.y = self.target_qy
        orientation_constraint.orientation.z = self.target_qz
        orientation_constraint.orientation.w = self.target_qw

        orientation_constraint.absolute_x_axis_tolerance = (
            self.orientation_tolerance
        )

        orientation_constraint.absolute_y_axis_tolerance = (
            self.orientation_tolerance
        )

        orientation_constraint.absolute_z_axis_tolerance = (
            self.orientation_tolerance
        )

        orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(
            orientation_constraint
        )

        return constraints

    def move(self):

        self.get_logger().info(
            'Waiting for MoveGroup action...'
        )

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'MoveGroup action server not available!'
            )
            return

        # ==============================
        # 创建规划请求
        # ==============================

        goal = MoveGroup.Goal()

        request = goal.request

        request.group_name = self.group_name
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnectkConfigDefault'

        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_time

        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = (
            self.acceleration_scaling
        )

        # 使用当前机器人状态作为起始状态
        request.start_state.is_diff = True

        # 设置目标约束
        constraints = self.create_goal_constraints()
        request.goal_constraints.append(constraints)

        # 规划成功后自动执行
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        self.get_logger().info('Sending planning request...')
        self.get_logger().info(
            f'Target position: '
            f'({self.target_x}, {self.target_y}, {self.target_z})'
        )

        # ==============================
        # 发送 Action 请求
        # ==============================

        send_goal_future = self.action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )

        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveGroup!')
            return

        self.get_logger().info(
            'Goal accepted. Waiting for planning and execution...'
        )

        # ==============================
        # 等待规划与执行结果
        # ==============================

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result is None:
            self.get_logger().error('No result received!')
            return

        error_code = result.result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                'Motion planning and execution succeeded!'
            )
        else:
            self.get_logger().error(
                f'MoveIt failed! Error code: {error_code}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = MoveToTarget()

    try:
        node.move()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()