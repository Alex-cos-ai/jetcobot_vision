"""Offline closed-loop tracking simulation and plots; never connects to serial."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .kinematics import DifferentialIK, Kinematics
from .safety import SafetyGuard
from .tracking_config import load_config
from .tracking_pid import TrackingPID, TrackingStateMachine


def run_simulation(config, duration=30., dt=.05, trajectory='sine'):
    """Project a moving world square through the FK camera and close the feedback loop."""
    calibration = json.loads(Path(config['camera']['calibration_file']).read_text())
    k = np.array(calibration['camera_model']['K']).reshape(3, 3)
    focal = (k[0, 0] + k[1, 1]) / 2.
    principal = k[:2, 2]
    reference = (np.array(config['camera']['reference_pixel']) - principal) / focal
    model = Kinematics(config['model']['urdf'], calibration['T_gripper_camera'],
                       np.radians(config['model']['margin_deg']), config['model'].get('hardware_limits_deg'))
    guard, ik = SafetyGuard(model, config), DifferentialIK(model, config)
    q = np.array(config['simulation']['initial_angles_rad'], dtype=float)
    if trajectory == 'singular':
        # 全零姿态在当前保守工作空间之外；选工作空间内的近奇异伸肘姿态。
        q[:] = [0., 1., 0., .5, 0., 0.]
    full_pose = bool(config['ik']['full_pose'] or trajectory == 'singular')
    valid, reason = guard.valid(q)
    if not valid:
        raise ValueError(f'Initial simulation pose is unsafe: {reason}')
    initial = model.fk(q).camera
    orientation = initial[:3, :3]
    centre = initial[:3, 3] + orientation @ np.array([0., 0., .24])
    half = config['camera']['square_side_mm'] / 2000.
    corners = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]]) @ orientation.T
    pid, machine = TrackingPID(config, focal, reference), TrackingStateMachine()
    result = {name: [] for name in ['time_s', 'q_rad', 'qdot_rad_s', 'qddot_rad_s2', 'camera_m',
                                    'target_world_m', 'sigma_min', 'limit_margin_rad', 'error_m', 'state', 'guard']}
    previous_velocity, previous_camera_velocity = np.zeros(6), np.zeros(3)
    for t in np.arange(0., duration, dt):
        if trajectory == 'step':
            offset = np.array([.035 if t > duration / 3. else -.015, .015, .0])
        elif trajectory == 'boundary':
            offset = np.array([.2 * t / duration, .04, .0])
        else:
            offset = np.array([.025 * np.sin(.4 * t), .018 * np.sin(.3 * t), .025 * np.sin(.2 * t)])
        world = centre + orientation @ offset
        pose = model.fk(q).camera
        camera_corners = (corners + world - pose[:3, 3]) @ pose[:3, :3]
        found = bool(np.all(camera_corners[:, 2] > .03))
        pixels = camera_corners[:, :2] / np.maximum(camera_corners[:, 2:], .001) * focal + principal
        found &= bool(np.all(pixels > 2.) and np.all(pixels < np.array([638., 478.])))
        # 每次测试均包含短暂和长时间遮挡，随后必须能重新进入跟踪。
        found &= not (duration * .35 < t < duration * .37 or duration * .65 < t < duration * .70)
        state, scale, reset = machine.step(found, True, float(t))
        if reset:
            pid.reset()
            guard.previous_velocity[:] = 0.
        if found:
            (cx, cy), radius = cv2.minEnclosingCircle(pixels.astype(np.float32))
            pid.observe(cx, cy, radius, float(t) + 1., principal)
            camera_velocity = pid.tick(dt, scale)
        elif state == 'LOST':
            camera_velocity = previous_camera_velocity * np.exp(-dt / config['runtime']['lost_decay_s'])
        else:
            camera_velocity = np.zeros(3)
        previous_camera_velocity = camera_velocity.copy()
        world_velocity = guard.project_task(q, pose[:3, :3] @ camera_velocity, dt)
        velocity = ik.solve(q, world_velocity, orientation, dt, full_pose)
        if state == 'HOLD':
            velocity[:] = 0.
        velocity = guard.filter_velocity(q, velocity, dt)
        acceleration = (velocity - previous_velocity) / dt
        q_next = q + velocity * dt
        assert np.all(np.isfinite(q_next)), 'nonfinite joint target'
        assert np.max(np.abs(velocity)) <= max(config['model']['max_velocity_rad_s']) + 1.e-6
        assert guard.validate_step(q, q_next)[0], 'unsafe simulation step'
        # 急停优先于普通平滑；正常轨迹逐轴验证加速度限制。
        if guard.reason in ('OK', 'projected') and not reset:
            assert np.all(np.abs(acceleration) <= np.array(config['model']['max_acceleration_rad_s2']) + 1.e-4)
        values = dict(time_s=t, q_rad=q.copy(), qdot_rad_s=velocity, qddot_rad_s2=acceleration,
                      camera_m=pose[:3, 3], target_world_m=world,
                      sigma_min=ik.diagnostics['sigma_min'], limit_margin_rad=ik.diagnostics['limit_margin_rad'],
                      error_m=pid.error.copy(), state=state, guard=guard.reason)
        for key, value in values.items():
            result[key].append(value)
        q, previous_velocity = q_next, velocity.copy()
    return {key: np.asarray(value) for key, value in result.items()}


def save_results(result, output):
    """Write portable CSV, machine-readable arrays and standalone diagnostic plots."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / 'tracking.npz', **result)
    columns = np.column_stack((result['time_s'], result['q_rad'], result['camera_m'],
                               result['sigma_min'], result['limit_margin_rad']))
    np.savetxt(output / 'tracking.csv', columns, delimiter=',',
               header='time_s,q1,q2,q3,q4,q5,q6,camera_x,camera_y,camera_z,sigma_min,limit_margin_rad', comments='')
    fig, axes = plt.subplots(3, 2, figsize=(13, 10))
    t = result['time_s']
    axes[0, 0].plot(t, np.degrees(result['q_rad']))
    axes[0, 0].set_ylabel('joint angle (deg)')
    axes[0, 0].legend([f'J{i+1}' for i in range(6)], ncol=3)
    axes[0, 1].plot(t, result['camera_m'] * 1000)
    axes[0, 1].set_ylabel('camera position (mm)')
    axes[0, 1].legend(['base x', 'base y', 'base z'])
    axes[1, 0].plot(t, result['sigma_min'])
    axes[1, 0].set_ylabel('scaled weighted sigma_min')
    axes[1, 1].plot(t, np.degrees(result['limit_margin_rad']))
    axes[1, 1].set_ylabel('minimum joint margin (deg)')
    axes[2, 0].plot(t, np.degrees(result['qdot_rad_s']))
    axes[2, 0].set_ylabel('joint velocity (deg/s)')
    axes[2, 0].legend([f'J{i+1}' for i in range(6)], ncol=3)
    axes[2, 1].plot(t, result['error_m'] * 1000)
    axes[2, 1].set_ylabel('estimated camera error (mm)')
    axes[2, 1].legend(['camera x', 'camera y', 'camera z'])
    for axis in axes.flat:
        axis.set_xlabel('time (s)')
        axis.grid(True, alpha=.3)
    fig.tight_layout()
    fig.savefig(output / 'tracking.png', dpi=150)
    plt.close(fig)
    fig = plt.figure(figsize=(7, 6))
    axis = fig.add_subplot(111, projection='3d')
    axis.plot(*(result['camera_m'].T * 1000), label='camera')
    axis.plot(*(result['target_world_m'].T * 1000), label='world target')
    axis.set(xlabel='base x (mm)', ylabel='base y (mm)', zlabel='base z (mm)')
    axis.legend()
    fig.savefig(output / 'trajectory_3d.png', dpi=150)
    plt.close(fig)
    summary = dict(samples=len(t), min_joint_margin_deg=float(np.degrees(np.min(result['limit_margin_rad']))),
                   min_sigma=float(np.min(result['sigma_min'])),
                   max_joint_velocity_deg_s=float(np.degrees(np.max(np.abs(result['qdot_rad_s'])))),
                   max_step_deg=float(np.degrees(np.max(np.abs(np.diff(result['q_rad'], axis=0))))),
                   states={state: int(np.sum(result['state'] == state)) for state in np.unique(result['state'])},
                   guard_states={state: int(np.sum(result['guard'] == state)) for state in np.unique(result['guard'])})
    (output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return summary


def main(args=None):
    """Generate a deterministic synthetic trajectory without ROS or hardware."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--trajectory', choices=['sine', 'step', 'boundary', 'singular'], default='sine')
    parser.add_argument('--output', default='tracking_sim_results')
    options = parser.parse_args(args)
    if options.duration < 1.:
        parser.error('--duration must be at least one second')
    result = run_simulation(load_config(options.config), options.duration, trajectory=options.trajectory)
    print(json.dumps(save_results(result, options.output), indent=2))


if __name__ == '__main__':
    main()
