"""Install a table in Servo's own scene and verify the stored geometry."""

import numpy as np
import rclpy
from .tracking_protocol import init_ros
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from .tracking_config import load_config
from .tracking_protocol import encode, LATEST


class TableScene(Node):
    """Never invent a real table height when configuration is incomplete."""

    def __init__(self):
        """Use Servo's primary scene monitor; no move_group executor is needed."""
        super().__init__('tracking_scene')
        self.declare_parameter('config_file', '')
        self.config = load_config(self.get_parameter('config_file').value or None)
        self.client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.objects = self.create_publisher(CollisionObject, '/collision_object', LATEST)
        self.publisher = self.create_publisher(String, '/tracking/scene_state', LATEST)
        self.future = None
        self.ready = False
        self.create_timer(.1, self.tick)

    @staticmethod
    def matches(obj, table):
        """A matching ID alone cannot confirm the configured table geometry."""
        # 当前URDF的dummy与base_link通过单位固定变换连接。
        if obj.id != 'tracking_table' or obj.header.frame_id not in ('base_link', 'dummy'):
            return False
        if len(obj.primitives) != 1 or len(obj.primitive_poses) != 1:
            return False
        shape, local = obj.primitives[0], obj.primitive_poses[0]
        # Humble将发布的primitive位姿归一化到CollisionObject.pose。
        from .kinematics import rotation

        def matrix(pose):
            quaternion = pose.orientation
            xyz = np.array([quaternion.x, quaternion.y, quaternion.z])
            sine = np.linalg.norm(xyz)
            transform = np.eye(4)
            if sine > 1.e-10:
                transform[:3, :3] = rotation(xyz / sine, 2. * np.arctan2(sine, quaternion.w))
            transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
            return transform

        pose = matrix(obj.pose) @ matrix(local)
        return bool(shape.type == SolidPrimitive.BOX and
                    np.allclose(shape.dimensions, [2., 2., .05], atol=1.e-6) and
                    np.allclose(pose[:3, 3], [0., 0., table - .025], atol=1.e-6) and
                    np.allclose(pose[:3, :3], np.eye(3), atol=1.e-6))

    def tick(self):
        """Publish missing geometry and query it back before reporting readiness."""
        table = self.config['safety']['table_z_m']
        if table is not None and self.client.service_is_ready():
            if self.future is not None and self.future.done():
                try:
                    response = self.future.result()
                    self.ready = bool(response and any(self.matches(obj, table)
                                      for obj in response.scene.world.collision_objects))
                except Exception:
                    self.ready = False
                self.future = None
            if self.future is None:
                if not self.ready:
                    obj = CollisionObject()
                    obj.header.frame_id, obj.id = 'base_link', 'tracking_table'
                    obj.operation = CollisionObject.ADD
                    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2., 2., .05])
                    pose = Pose()
                    pose.orientation.w, pose.position.z = 1., table - .025
                    obj.primitives, obj.primitive_poses = [primitive], [pose]
                    self.objects.publish(obj)
                request = GetPlanningScene.Request()
                request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                self.future = self.client.call_async(request)
        else:
            self.ready = False
        self.publisher.publish(String(data=encode(dict(
            stamp_ns=self.get_clock().now().nanoseconds, ready=self.ready,
            reason='table_config_missing' if table is None else ('OK' if self.ready else 'waiting_for_scene')))))


def main(args=None):
    """Run scene setup without trajectory execution permissions."""
    init_ros(args=args)
    node = TableScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
