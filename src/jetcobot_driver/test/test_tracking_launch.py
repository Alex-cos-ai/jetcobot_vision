"""Opt-in no-hardware integration: real launch, synthetic camera, both IK backends."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest
import yaml


@pytest.mark.skipif(os.environ.get('RUN_TRACKING_LAUNCH_TESTS') != '1', reason='opt-in ROS integration')
@pytest.mark.parametrize('backend', ['weighted_dls', 'moveit_servo'])
def test_synthetic_planning_launch(backend, tmp_path):
    """Run from another cwd, receive fresh plans, then cleanly terminate all children."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from trajectory_msgs.msg import JointTrajectory
    from visual_servo_6dof.tracking_protocol import decode, LATEST
    from visual_servo_6dof.tracking_config import default_config
    script = Path(__file__).resolve().parents[1] / 'jetcobot_driver/tracking_main.py'
    log_path = tmp_path / f'{backend}.log'
    log = log_path.open('w')
    config = yaml.safe_load(default_config().read_text())
    # 模拟桌面放在模型下方，验证 Servo 本身确实收到场景，仍然禁止实机执行。
    config['tracking']['safety']['table_z_m'] = -1.
    config_path = tmp_path / 'tracking.yaml'
    config_path.write_text(yaml.safe_dump(config))
    process = subprocess.Popen([sys.executable, str(script), '--feedback-source', 'simulation',
                                '--synthetic-camera', '--headless', '--target-colour', 'blue',
                                '--ik-backend', backend, '--config', str(config_path)],
                               cwd=str(tmp_path), stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=[])
    node = Node('tracking_integration_observer')
    targets, states, frames, scenes = [], [], [], []
    node.create_subscription(JointTrajectory, '/tracking/joint_target', targets.append, LATEST)
    node.create_subscription(String, '/tracking/planner_state', lambda m: states.append(decode(m)), LATEST)
    node.create_subscription(Image, '/visual_servo_6dof/debug_image', lambda m: frames.append(m.header), 1)
    node.create_subscription(String, '/tracking/scene_state', lambda m: scenes.append(decode(m)), LATEST)
    try:
        end = time.monotonic() + 22.
        while time.monotonic() < end and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
            scene_ready = backend != 'moveit_servo' or any(s.get('ready') for s in scenes)
            if len(targets) >= 20 and any(s.get('healthy') for s in states) and len(frames) > 20 and scene_ready:
                break
        assert process.poll() is None, log_path.read_text()[-6000:]
        assert len(targets) >= 10, log_path.read_text()[-6000:]
        assert len(frames) > 20
        assert any(s.get('healthy') and s.get('state') == 'TRACKING' for s in states)
        assert backend != 'moveit_servo' or any(s.get('ready') for s in scenes)
        for message in targets:
            point = message.points[0]
            assert len(point.positions) == len(point.velocities) == 6
            assert np.all(np.isfinite(point.positions))
            assert max(abs(v) for v in point.velocities) <= .100001
        # At least one actual nonzero joint plan, not a stream of holds.
        assert any(max(abs(v) for v in m.points[0].velocities) > 1.e-4 for m in targets)
    finally:
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    assert 'Traceback' not in log_path.read_text(), log_path.read_text()[-6000:]
    assert 'process has died' not in log_path.read_text(), log_path.read_text()[-6000:]
