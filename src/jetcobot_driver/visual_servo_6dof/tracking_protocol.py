"""Small timestamped protocol shared by the two independent processes."""

import json
import signal

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


LATEST = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)


def init_ros(args=None):
    """Keep the context alive until callbacks finish and cleanup runs on the main thread."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # 默认rclpy信号会在回调中途关闭context，导致发布/取消息的退出竞态。
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)


def stamp_ns(stamp):
    """Convert a ROS stamp to integer nanoseconds."""
    return stamp.sec * 1000000000 + stamp.nanosec


def encode(payload):
    """Reject nonfinite JSON rather than forwarding invalid control state."""
    return json.dumps(payload, ensure_ascii=False, allow_nan=False)


def decode(message):
    """Return a protocol object, or an empty object for malformed input."""
    try:
        def invalid_constant(value):
            raise ValueError(f'nonfinite protocol value: {value}')
        result = json.loads(message.data, parse_constant=invalid_constant)
        return result if isinstance(result, dict) else {}
    except (ValueError, TypeError):
        return {}
