"""Pace this application's SDK instance without changing vendor SDK files."""

import threading
import time


def open_robot(port, baud, timeout, minimum_interval=.05):
    """Serialize serial requests and leave a small interval between wire commands."""
    from pymycobot.mycobot import MyCobot
    from pymycobot.common import ProtocolCode

    class PacedMyCobot(MyCobot):
        def __init__(self):
            self.request_lock = threading.Lock()
            self.last_request = 0.
            super().__init__(port, baud, timeout=timeout)

        def _mesg(self, genre, *args, **kwargs):
            with self.request_lock:
                delay = self.last_request + minimum_interval - time.monotonic()
                # 停止命令不等待普通请求的发送间隔。
                if delay > 0 and genre != ProtocolCode.STOP:
                    time.sleep(delay)
                self.last_request = time.monotonic()
                return super()._mesg(genre, *args, **kwargs)

    return PacedMyCobot()
