"""Local processed-image window; its GUI runs in this process's main thread."""

import signal
import threading

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from .tracking_protocol import init_ros
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .tracking_protocol import LATEST


def window_is_open(name):
    """Use GTK2's native window list when OpenCV returns unsupported (-1)."""
    visible = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE)
    if visible >= 0:
        return visible >= 1
    # 本机GTK2后端的WND_PROP_VISIBLE未实现；只在GUI主线程查询GTK。
    if 'GUI:                           GTK2' not in cv2.getBuildInformation():
        return True
    import ctypes as ct

    class GList(ct.Structure):
        pass

    GList._fields_ = [('data', ct.c_void_p), ('next', ct.POINTER(GList)),
                      ('previous', ct.POINTER(GList))]
    gtk = ct.CDLL('libgtk-x11-2.0.so.0')
    glib = ct.CDLL('libglib-2.0.so.0')
    gtk.gtk_window_list_toplevels.restype = ct.POINTER(GList)
    gtk.gtk_window_get_title.argtypes = [ct.c_void_p]
    gtk.gtk_window_get_title.restype = ct.c_char_p
    gtk.gtk_widget_get_visible.argtypes = [ct.c_void_p]
    gtk.gtk_widget_get_visible.restype = ct.c_int
    glib.g_list_free.argtypes = [ct.POINTER(GList)]
    head = entry = gtk.gtk_window_list_toplevels()
    try:
        while entry:
            widget = entry.contents.data
            if gtk.gtk_window_get_title(widget) == name.encode():
                return bool(gtk.gtk_widget_get_visible(widget))
            entry = entry.contents.next
        return False
    finally:
        glib.g_list_free(head)


def main(args=None):
    """Show the latest processed image and publish nonblocking colour selection."""
    init_ros(args=args)
    node = Node('tracking_display')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    bridge = CvBridge()
    cv2.setNumThreads(1)
    lock = threading.Lock()
    latest = [None]
    publisher = node.create_publisher(String, '/tracking/select_colour', LATEST)

    def receive(message):
        try:
            frame = bridge.imgmsg_to_cv2(message, 'bgr8')
            with lock:
                latest[0] = frame
        except Exception as exc:
            node.get_logger().warning(str(exc), throttle_duration_sec=2.)

    def click(event, x, y, flags, parameter):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        with lock:
            frame = None if latest[0] is None else latest[0].copy()
        if frame is None:
            return
        # 点击颜色区域选择颜色；空间实例连续性由规划节点负责。
        from .visual_servo_6dof import detect_objects
        for detection in detect_objects(frame):
            bx, by, width, height = detection.bbox
            if bx <= x <= bx + width and by <= y <= by + height:
                publisher.publish(String(data=detection.colour))
                break

    node.create_subscription(Image, '/visual_servo_6dof/debug_image', receive, 1)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    window = 'JetCobot PID tracking'
    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window, click)
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, 'Waiting for camera... q/Esc: exit', (20, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 1)
        # GTK尚未imshow时窗口可见性为0；不能将它误判为用户关闭。
        cv2.imshow(window, placeholder)
        while rclpy.ok():
            with lock:
                frame = latest[0]
            if frame is not None:
                cv2.imshow(window, frame)
            key = cv2.waitKey(10) & 0xff
            if key in (ord('q'), 27) or not window_is_open(window):
                break
            colours = {ord('r'): 'red', ord('g'): 'green', ord('b'): 'blue', ord('a'): 'auto'}
            if key in colours:
                publisher.publish(String(data=colours[key]))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cv2.destroyAllWindows()
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.)
