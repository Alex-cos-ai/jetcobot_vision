#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge
from dt_apriltags import Detector
from rclpy.node import Node
from sensor_msgs.msg import Image


class MarkerDebug(Node):
    def __init__(self):
        super().__init__('detect_marker_debug')
        self.bridge = CvBridge()
        self.last_report = None

        self.apriltag_detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.aruco_parameters = cv2.aruco.DetectorParameters()

        self.subscription = self.create_subscription(
            Image,
            '/wrist_camera/image_raw',
            self.image_callback,
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            '/wrist_camera/tag_debug',
            10,
        )
        self.get_logger().info(
            '检测器已启动：AprilTag tag36h11 + ArUco DICT_4X4_50'
        )

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'图像转换失败: {exc}')
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        debug_image = image.copy()

        april_tags = self.apriltag_detector.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )
        april_ids = sorted(tag.tag_id for tag in april_tags)
        for tag in april_tags:
            corners = tag.corners.astype(int)
            cv2.polylines(
                debug_image,
                [corners.reshape((-1, 1, 2))],
                True,
                (0, 255, 0),
                2,
            )
            center = tuple(tag.center.astype(int))
            cv2.putText(
                debug_image,
                f'AprilTag {tag.tag_id}',
                center,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        aruco_corners, aruco_ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dictionary,
            parameters=self.aruco_parameters,
        )
        aruco_id_list = [] if aruco_ids is None else sorted(
            int(marker_id) for marker_id in aruco_ids.flatten()
        )
        if aruco_ids is not None:
            cv2.aruco.drawDetectedMarkers(debug_image, aruco_corners, aruco_ids)

        report = (tuple(april_ids), tuple(aruco_id_list))
        if report != self.last_report:
            self.last_report = report
            self.get_logger().info(
                f'AprilTag tag36h11 IDs: {april_ids}; '
                f'ArUco 4x4_50 IDs: {aruco_id_list}'
            )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_publisher.publish(debug_msg)
        except Exception as exc:
            self.get_logger().error(f'调试图发布失败: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
