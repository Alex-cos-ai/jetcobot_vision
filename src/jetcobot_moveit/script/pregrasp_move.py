#!/usr/bin/env python3
"""Use calibrated wrist-camera extrinsics to approach a live single AprilTag.

Planning uses real joint feedback and MoveIt collision checks. Execution owns
the serial port exclusively, follows small joint waypoints with feedback, and
does not operate the gripper. The target is a selected URDF link origin;
the finger TCP has not been calibrated.
The default hand-eye file comes from ChArUco calibration; the observed grasp
target remains a separate 20 mm tag36h11 ID=1.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, JointConstraint, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from handeye_solve import as_transform, quaternion_xyzw


JOINTS = [f'{i}_Joint' for i in range(1, 7)]
MODEL_LIMITS = np.array([[-3.05, 3.05], [-1.57, 1.57], [-1.57, 1.57],
                         [-1.57, 1.57], [-3.05, 3.05], [-1.57, 1.57]])


def apriltag_spec(calibration, tag_id=None, marker_length_m=None):
    """The observed target is independent of the board used to calibrate X."""
    single_marker = calibration.get('target_type', 'single_marker') == 'single_marker'
    if tag_id is None:
        tag_id = calibration.get('tag_id', 1) if single_marker else 1
    if marker_length_m is None:
        marker_length_m = calibration['marker_length_m'] if single_marker else .02
    if not isinstance(tag_id, int) or not 0 <= tag_id < 587:
        raise ValueError('tag36h11 tag ID must be an integer in [0, 586]')
    if not math.isfinite(marker_length_m) or marker_length_m <= 0:
        raise ValueError('AprilTag marker length must be positive and finite')
    return {'family': 'tag36h11', 'tag_id': tag_id, 'marker_length_m': marker_length_m}


def camera_model_matches(info, calibration):
    model = calibration.get('camera_model')
    if model is None:
        return True  # Older single-tag results did not store their intrinsics.
    return (info.width == model['width'] and info.height == model['height']
            and info.distortion_model == model['distortion_model']
            and len(info.d) == len(model['D'])
            and np.allclose(info.k, model['K'], atol=1e-8, rtol=0)
            and np.allclose(info.d, model['D'], atol=1e-8, rtol=0))


def base_tag_pose(gripper, handeye, rvec, tvec):
    """Map a freshly observed tag to base; never reuse the calibration board pose."""
    return gripper @ handeye @ as_transform(cv2.Rodrigues(rvec)[0], tvec)


def chain_reach_bound(link):
    from ament_index_python.packages import get_package_share_directory
    urdf = Path(get_package_share_directory('jetcobot_description')) / 'urdf/jetcobot.urdf'
    joints = {j.get('name'): j for j in ET.parse(urdf).getroot().findall('joint')}
    names = JOINTS + (['jiazhua_Joint'] if link == 'jiazhua_Link' else [])
    return float(sum(np.linalg.norm(np.fromstring(joints[name].find('origin').get('xyz'), sep=' '))
                     for name in names))


def hardware_degrees(q):
    values = np.degrees(q).copy()
    values[..., 5] -= 45.0
    return values


def follow_waypoint(robot, read, spin, target, index, speed=5,
                    timeout=12., clock=time.monotonic, pause=time.sleep, diagnostics=None):
    """Wait for stopped, stable feedback; retry only the identical target.

    Fresh-mode controllers can replace unfinished tiny movements. A reading
    inside the joint tolerance alone is not evidence that motion has stopped.
    """
    diagnostics = {} if diagnostics is None else diagnostics
    robot.send_angles(hardware_degrees(target).tolist(), speed)
    sent_at = clock()
    deadline = sent_at + timeout
    retries, consecutive = 0, 0
    previous = None
    pause(.1)
    while clock() < deadline:
        spin()
        current = read()
        errors = np.degrees(current - target)
        error = float(np.max(np.abs(errors)))
        moving = robot.is_moving()
        if moving not in (0, 1):
            raise RuntimeError(f'Waypoint {index}: invalid controller motion state {moving!r}')
        diagnostics.update(waypoint_index=index, command_hardware_degrees=hardware_degrees(target).tolist(),
                           feedback_hardware_degrees=hardware_degrees(current).tolist(),
                           error_per_joint_deg=errors.tolist(), max_error_deg=error,
                           controller_is_moving=int(moving), retry_count=retries, speed=speed)
        if error > 4:
            raise RuntimeError(f'Waypoint {index} tracking error exceeds 4 degrees')
        stable = previous is not None and np.max(np.abs(np.degrees(current-previous))) < .15
        consecutive = consecutive + 1 if moving == 0 and stable and error < 2.0 else 0
        if consecutive >= 2:
            return current, error
        # Give the first command time to settle. A stopped controller with
        # residual error may have ignored a short move; never skip that point.
        if moving == 0 and error >= 2.0 and clock() - sent_at >= 1.0 and retries < 2:
            robot.send_angles(hardware_degrees(target).tolist(), speed)
            retries += 1
            sent_at = clock()
            diagnostics['retry_count'] = retries
            consecutive = 0
        previous = current.copy()
        pause(.1)
    worst = int(np.argmax(np.abs(errors)))
    raise RuntimeError(f'Waypoint {index} feedback timeout: J{worst+1} '
                       f'command={hardware_degrees(target)[worst]:.2f} deg, '
                       f'feedback={hardware_degrees(current)[worst]:.2f} deg, '
                       f'error={error:.2f} deg, moving={moving}, retries={retries}, speed={speed}')


def checked_path(names, points, max_step_deg=1.0):
    if set(names) != set(JOINTS) or len(names) != 6 or len(points) < 2:
        raise ValueError('Invalid trajectory joint names or empty trajectory')
    indices = [names.index(j) for j in JOINTS]
    q = np.asarray(points, dtype=float)[:, indices]
    if not np.isfinite(q).all() or q.shape[1] != 6:
        raise ValueError('Nonfinite or malformed trajectory')
    if np.any(q < MODEL_LIMITS[:, 0]) or np.any(q > MODEL_LIMITS[:, 1]):
        raise ValueError('Trajectory exceeds URDF joint limits')
    raw = hardware_degrees(q)
    raw_limits = np.array([174.7, 90, 90, 90, 174.7, 90])
    if np.any(np.abs(raw) > raw_limits + .01):
        raise ValueError('Trajectory exceeds local controller hardware limits')
    result = [q[0]]
    for a, b in zip(q[:-1], q[1:]):
        count = max(1, math.ceil(np.max(np.abs(np.degrees(b-a))) / max_step_deg))
        result.extend(a + (b-a) * (i / count) for i in range(1, count+1))
    return np.array(result)


def command_waypoint_indices(path, max_step_deg=3.0):
    """Coalesce only exactly straight, forward segments of the validated path.

    The dense path remains the collision-check path. Removing points on one
    straight joint-space segment does not change its geometry. Preserve every
    turn and reversal; do not replace a curved subpath with a new chord.
    """
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 6 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError('Invalid command path')
    if not math.isfinite(max_step_deg) or not 1.0 <= max_step_deg <= 3.0:
        raise ValueError('Command step must be in [1, 3] degrees')
    if np.max(np.abs(np.degrees(np.diff(path, axis=0)))) > max_step_deg + 1e-8:
        raise ValueError('Input path steps exceed command step limit')
    indices = [0]
    for candidate in range(2, len(path)):
        anchor = indices[-1]
        delta = path[candidate] - path[anchor]
        norm_sq = float(delta @ delta)
        intermediates = path[anchor+1:candidate] - path[anchor]
        if norm_sq < 1e-20:
            straight = bool(np.max(np.abs(intermediates)) <= 1e-10)
        else:
            fractions = intermediates @ delta / norm_sq
            straight = (np.max(np.abs(intermediates - fractions[:, None] * delta)) <= 1e-10
                        and np.all(fractions >= -1e-10) and np.all(fractions <= 1+1e-10)
                        and np.all(np.diff(fractions) >= -1e-10))
        if np.max(np.abs(np.degrees(delta))) > max_step_deg + 1e-8 or not straight:
            indices.append(candidate-1)
    if indices[-1] != len(path)-1:
        indices.append(len(path)-1)
    return indices


def next_command_index(path, command_indices, previous_index, current):
    """Split a merged segment if actual feedback leaves too little step margin."""
    endpoint = next(i for i in command_indices if i > previous_index)
    eligible = [i for i in range(previous_index+1, endpoint+1)
                if np.max(np.abs(np.degrees(path[i]-current))) <= 3.75]
    if not eligible:
        raise RuntimeError('No forward path command within 3.75 degrees of actual feedback')
    return eligible[-1]


def nearby_goal(start, position, rotation, link):
    """Find a nearby numerical IK candidate within the declared pose tolerances.

    MoveIt still validates the candidate and the whole planned path.
    """
    from ament_index_python.packages import get_package_share_directory
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    urdf = Path(get_package_share_directory('jetcobot_description')) / 'urdf/jetcobot.urdf'
    joints = {j.get('name'): j for j in ET.parse(urdf).getroot().findall('joint')}
    parts = []
    for name in JOINTS + (['jiazhua_Joint'] if link == 'jiazhua_Link' else []):
        joint = joints[name]; origin = joint.find('origin')
        transform = np.eye(4)
        transform[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        parts.append((transform, axis))
    def fk(q):
        transform = np.eye(4)
        for i, (origin, axis) in enumerate(parts):
            transform = transform @ origin
            if i < 6:
                r = np.eye(4)
                r[:3, :3] = Rotation.from_rotvec(q[i]*axis/np.linalg.norm(axis)).as_matrix()
                transform = transform @ r
        return transform
    lower, upper = MODEL_LIMITS[:, 0].copy(), MODEL_LIMITS[:, 1].copy()
    lower[5] = -math.pi/4  # inverse of the +45-degree feedback mapping
    candidates = []
    rng = np.random.default_rng(17)
    for i in range(6):
        seed = np.asarray(start) if i == 0 else np.clip(np.asarray(start)+rng.normal(0, .6, 6), lower+1e-5, upper-1e-5)
        def residual(q):
            pose = fk(q)
            return np.r_[(pose[:3, 3]-position)*10,
                         Rotation.from_matrix(rotation.T@pose[:3, :3]).as_rotvec()]
        result = least_squares(residual, seed, bounds=(lower, upper), max_nfev=500)
        error = residual(result.x)
        if np.linalg.norm(error[:3])/10 < .004 and np.linalg.norm(error[3:]) < .1:
            candidates.append(result.x)
    if not candidates:
        raise RuntimeError('No nearby IK within 4 mm and 0.1 rad pose tolerance')
    return min(candidates, key=lambda q: np.linalg.norm(q-start))


class Pregrasp(Node):
    def __init__(self):
        super().__init__('pregrasp_move')
        self.latest = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subs = [
            self.create_subscription(Image, '/wrist_camera/image_raw',
                                     lambda m: self.latest.update(image=m), qos_profile_sensor_data),
            self.create_subscription(CameraInfo, '/wrist_camera/camera_info',
                                     lambda m: self.latest.update(info=m), qos_profile_sensor_data),
            self.create_subscription(JointState, '/joint_states',
                                     lambda m: self.latest.update(joints=m), 10),
        ]
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

    def read_robot(self, robot):
        values = None
        for _ in range(4):
            values = robot.get_angles()
            if values and len(values) == 6 and np.isfinite(values).all():
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Lost robot angle feedback')
        q = np.radians(values); q[5] += math.pi/4
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS; msg.position = q.tolist()
        self.latest['joints'] = msg
        self.publisher.publish(msg)
        return q

    def wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('ROS request timed out')
        return future.result()

    def observe(self, calibration, target_spec):
        from dt_apriltags import Detector
        bridge = CvBridge()
        detector = Detector(families=target_spec['family'], nthreads=2)
        matrix = np.asarray(calibration['T_gripper_camera'], dtype=float)
        x = as_transform(matrix[:3, :3], matrix[:3, 3])
        positions, rotations, times, grippers = [], [], [], []
        reprojection_errors = []
        seen = set()
        end = time.monotonic() + 20
        half = target_spec['marker_length_m'] / 2
        object_points = np.array([[-half, half, 0], [half, half, 0],
                                  [half, -half, 0], [-half, -half, 0]])
        while time.monotonic() < end and len(positions) < 10:
            rclpy.spin_once(self, timeout_sec=.03)
            if not all(k in self.latest for k in ('image', 'info', 'joints')):
                continue
            msg = self.latest['image']
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds in seen:
                continue
            if not self.buffer.can_transform(calibration['base_frame'],
                                             calibration['parent_frame'], stamp):
                continue
            seen.add(stamp.nanoseconds)
            age = (self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
            if not 0 <= age < 2 or msg.header.frame_id != calibration['child_frame']:
                continue
            info = self.latest['info']
            if (info.header.frame_id != calibration['child_frame']
                    or (msg.width, msg.height) != (info.width, info.height)
                    or info.distortion_model not in ('plumb_bob', 'rational_polynomial')):
                raise RuntimeError('Image/CameraInfo optical frame, dimensions or distortion model mismatch')
            if not camera_model_matches(info, calibration):
                raise RuntimeError('Camera intrinsics changed since hand-eye calibration')
            gray = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
            for tag in detector.detect(gray):
                if tag.tag_id != target_spec['tag_id']:
                    continue
                ok, rv, tv = cv2.solvePnP(object_points, tag.corners,
                    np.array(info.k).reshape(3, 3), np.array(info.d),
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok or tv[2, 0] <= 0:
                    continue
                projected = cv2.projectPoints(object_points, rv, tv,
                    np.array(info.k).reshape(3, 3), np.array(info.d))[0].reshape(-1, 2)
                reprojection = float(np.sqrt(np.mean(np.sum((projected-tag.corners)**2, axis=1))))
                if not math.isfinite(reprojection) or reprojection > 1.0:
                    continue
                tr = self.buffer.lookup_transform(calibration['base_frame'],
                                                  calibration['parent_frame'], stamp)
                tq, tt = tr.transform.rotation, tr.transform.translation
                r = Rotation.from_quat([tq.x, tq.y, tq.z, tq.w]).as_matrix()
                t = np.array([tt.x, tt.y, tt.z])
                g = as_transform(r, t)
                positions.append(base_tag_pose(g, x, rv, tv)[:3, 3])
                rotations.append(r)
                grippers.append(t.ravel())
                times.append(stamp.nanoseconds/1e9)
                reprojection_errors.append(reprojection)
        if len(positions) < 8:
            raise RuntimeError(f'Not enough fresh AprilTag ID={target_spec["tag_id"]} observations '
                               f'({len(positions)}/8); check tag visibility and timestamped gripper TF')
        center = np.median(positions, axis=0)
        spread = np.max(np.linalg.norm(positions-center, axis=1))
        rotation_spread = max(np.linalg.norm(cv2.Rodrigues(rotations[0].T @ r)[0])
                              for r in rotations)
        if (spread > .01 or np.max(np.linalg.norm(grippers-np.mean(grippers, axis=0), axis=1)) > .003
                or rotation_spread > math.radians(.5)):
            raise RuntimeError('Tag or robot is moving; hold stationary')
        self.observation_report = {
            'target_type': 'apriltag', 'target_spec': target_spec,
            'sample_count': len(positions), 'tag_position_samples_m': np.asarray(positions).tolist(),
            'reprojection_rms_px': reprojection_errors,
            'robot_rotation_spread_deg': math.degrees(rotation_spread),
        }
        return center, rotations[-1], times[-1], float(spread)

    def plan(self, args):
        calibration = json.loads(args.calibration.read_text())
        target_spec = apriltag_spec(calibration, args.tag_id, args.marker_length)
        print('Hand-eye calibration:', args.calibration, flush=True)
        print('Observed target:', target_spec, flush=True)
        # Invalidate any earlier path before observing a new target. A failed
        # detection/planning attempt must never leave an executable old path.
        args.plan.parent.mkdir(parents=True, exist_ok=True)
        args.plan.write_text(json.dumps({
            'status': 'observing', 'path_validated': False, 'motion_sent': False,
            'calibration_file': str(args.calibration.resolve()), 'target_spec': target_spec,
        }, indent=2)+'\n')
        center, rotation, image_stamp, spread = self.observe(calibration, target_spec)
        target = center + [0, 0, args.height]
        if args.pitch is not None:
            yaw = math.atan2(center[1], center[0])
            cz, sz = math.cos(yaw), math.sin(yaw)
            cy, sy = math.cos(args.pitch), math.sin(args.pitch)
            rotation = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array(
                [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        print('Tag base (m):', center, args.link, 'target (m):', target, flush=True)
        upper_bound = chain_reach_bound(args.link)
        observation = {
            **self.observation_report, 'calibration_file': str(args.calibration.resolve()),
            'calibration_sha256': hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
            'T_gripper_camera': calibration['T_gripper_camera'],
            'base_frame': calibration['base_frame'], 'link': args.link,
            'tag_position_m': center.tolist(), 'target_position_m': target.tolist(),
            'target_quaternion_xyzw': quaternion_xyzw(rotation), 'height_m': args.height,
            'tag_spread_m': spread, 'image_stamp_sec': image_stamp,
            'target_distance_m': float(np.linalg.norm(target)),
            'chain_reach_upper_bound_m': upper_bound,
            'within_reach_bound': bool(np.linalg.norm(target) <= upper_bound),
            'motion_sent': False, 'status': 'observed', 'path_validated': False,
        }
        args.plan.write_text(json.dumps(observation, indent=2)+'\n')
        if np.linalg.norm(target) > upper_bound:
            observation.update(status='unreachable', reason='Target exceeds URDF chain-length upper bound')
            args.plan.write_text(json.dumps(observation, indent=2)+'\n')
            raise RuntimeError(f'Target distance {np.linalg.norm(target):.3f} m exceeds '
                               f'{args.link} reach upper bound {upper_bound:.3f} m; report: {args.plan}')
        if args.observe_only:
            print('Observation saved:', args.plan,
                  '(within length bound; IK and collision checks still required)', flush=True)
            return
        joint_msg = self.latest['joints']
        if self.get_clock().now().nanoseconds/1e9 - Time.from_msg(joint_msg.header.stamp).nanoseconds/1e9 > .5:
            raise RuntimeError('Stale joint feedback')
        joint_values = dict(zip(joint_msg.name, joint_msg.position))
        start = [joint_values[j] for j in JOINTS]
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm_group'; req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.allowed_planning_time = 10.; req.num_planning_attempts = 3
        req.max_velocity_scaling_factor = .05; req.max_acceleration_scaling_factor = .05
        req.start_state.joint_state.name = JOINTS
        req.start_state.joint_state.position = start
        req.start_state.is_diff = False
        pc = PositionConstraint(); pc.header.frame_id = calibration['base_frame']
        pc.link_name = args.link; pc.weight = 1.
        shape = SolidPrimitive(); shape.type = SolidPrimitive.SPHERE; shape.dimensions = [.004]
        pose = Pose(); pose.position.x, pose.position.y, pose.position.z = map(float, target)
        pose.orientation.w = 1.
        pc.constraint_region = BoundingVolume(primitives=[shape], primitive_poses=[pose])
        oc = OrientationConstraint(); oc.header.frame_id = calibration['base_frame']
        oc.link_name = args.link; oc.weight = 1.
        q = quaternion_xyzw(rotation)
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = q
        oc.absolute_x_axis_tolerance = .1; oc.absolute_y_axis_tolerance = .1
        oc.absolute_z_axis_tolerance = .1
        req.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        if args.nearby:
            candidate = nearby_goal(np.array(start), target, rotation, args.link)
            print('Nearby hardware goal degrees:', hardware_degrees(candidate), flush=True)
            req.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(joint_name=j, position=float(v), tolerance_above=.002,
                                tolerance_below=.002, weight=1.)
                for j, v in zip(JOINTS, candidate)])]
        goal.planning_options.plan_only = True
        action = ActionClient(self, MoveGroup, '/move_action')
        if not action.wait_for_server(timeout_sec=8):
            raise RuntimeError('MoveIt /move_action is unavailable')
        handle = self.wait(action.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected planning request')
        result = self.wait(handle.get_result_async(), 45).result
        if result.error_code.val != 1:
            raise RuntimeError(f'MoveIt planning failed, error={result.error_code.val}')
        trajectory = result.planned_trajectory.joint_trajectory
        path = checked_path(list(trajectory.joint_names), [list(p.positions) for p in trajectory.points])
        command_indices = command_waypoint_indices(path)
        if np.max(np.abs(path[0]-start)) > .025:
            raise RuntimeError('Planner altered the starting pose')
        fk = self.create_client(GetPositionFK, '/compute_fk')
        validity = self.create_client(GetStateValidity, '/check_state_validity')
        if not fk.wait_for_service(timeout_sec=5) or not validity.wait_for_service(timeout_sec=5):
            raise RuntimeError('MoveIt validation services unavailable')
        minimum_z = float('inf')
        for point in path:
            request = GetPositionFK.Request(); request.header.frame_id = calibration['base_frame']
            request.fk_link_names = ['6_Link', 'jiazhua_Link']
            request.robot_state.joint_state.name = JOINTS
            request.robot_state.joint_state.position = point.tolist()
            out = self.wait(fk.call_async(request), 5)
            if out.error_code.val != 1:
                raise RuntimeError('FK validation failed')
            z = min(p.pose.position.z for p in out.pose_stamped)
            minimum_z = min(z, minimum_z)
            if z < center[2] + .10:
                raise RuntimeError('Path approaches closer than 10 cm to Tag height')
            check = GetStateValidity.Request(); check.group_name = 'arm_group'
            check.robot_state = request.robot_state
            if not self.wait(validity.call_async(check), 5).valid:
                raise RuntimeError('Densified path is in collision')
        final_pose = out.pose_stamped[request.fk_link_names.index(args.link)].pose
        final_position = np.array([final_pose.position.x, final_pose.position.y, final_pose.position.z])
        if np.linalg.norm(final_position-target) > .006:
            raise RuntimeError('Final FK position differs by more than 6 mm from target')
        report = {**observation, 'status': 'planned', 'path_validated': True,
                  'base_frame': calibration['base_frame'], 'link': args.link,
                  'tag_position_m': center.tolist(), 'target_position_m': target.tolist(),
                  'target_quaternion_xyzw': q, 'height_m': args.height,
                  'tag_spread_m': spread, 'image_stamp_sec': image_stamp,
                  'planned_at_sec': self.get_clock().now().nanoseconds/1e9,
                  'joint_names': JOINTS, 'path_radians': path.tolist(),
                  'command_waypoint_indices': command_indices,
                  'command_step_limit_deg': 3.0,
                  'minimum_checked_link_z_m': minimum_z,
                  'motion_sent': False, 'environment_note':
                  'URDF self-collision checks only; no measured table or surrounding objects. '
                  'Target is a named URDF link origin; finger TCP is not calibrated.'}
        args.plan.write_text(json.dumps(report, indent=2)+'\n')
        print('Plan saved:', args.plan, 'waypoints:', len(path), flush=True)
        print('Execution command points:', len(command_indices),
              '(straight segments merged, maximum step 3 deg)', flush=True)
        print('Hardware target degrees:', hardware_degrees(path[-1]), flush=True)

    def execute(self, args):
        from pymycobot.mycobot import MyCobot
        report = json.loads(args.plan.read_text())
        if not report.get('path_validated') or report.get('status') != 'planned':
            raise RuntimeError('Plan has not passed IK, FK and path collision validation')
        if report.get('motion_sent'):
            raise RuntimeError('Plan has already been executed; observe and plan again')
        age = self.get_clock().now().nanoseconds/1e9-report['planned_at_sec']
        if not 0 <= age < 180:
            raise RuntimeError('Plan is older than 180 seconds; observe and plan again')
        path = checked_path(report['joint_names'], report['path_radians'])
        command_indices = command_waypoint_indices(path)
        owners = subprocess.run(['fuser', args.port], capture_output=True, text=True)
        if owners.returncode == 0:
            raise RuntimeError('Serial port occupied. Close local_joint_camera_control first.')
        robot = MyCobot(args.port, 1000000)
        def read():
            return self.read_robot(robot)
        try:
            current = read()
            if np.max(np.abs(np.degrees(current-path[0]))) > 2:
                raise RuntimeError('Robot moved since planning')
            report['motion_sent'] = True; report['execution_status'] = 'running'
            report['execution_speed'] = args.speed
            report['command_waypoint_indices'] = command_indices
            report['command_step_limit_deg'] = 3.0
            report['waypoint_feedback'] = []
            args.plan.write_text(json.dumps(report, indent=2)+'\n')
            previous_index, command_number = 0, 0
            while previous_index < len(path)-1:
                index = next_command_index(path, command_indices, previous_index, current)
                command_number += 1
                target = path[index]
                if np.max(np.abs(np.degrees(current-target))) > 4:
                    raise RuntimeError('Tracking error exceeds 4 degrees')
                feedback = {'command_number': command_number}
                report['waypoint_feedback'].append(feedback)
                print(f'Command {command_number}, path point {index}/{len(path)-1}: '
                      f'hardware degrees {np.round(hardware_degrees(target), 2).tolist()}', flush=True)
                try:
                    current, error = follow_waypoint(
                        robot, read, lambda: rclpy.spin_once(self, timeout_sec=.05),
                        target, index, speed=args.speed, diagnostics=feedback)
                except RuntimeError as exc:
                    raise RuntimeError(f'Command {command_number} failed: {exc}') from exc
                previous_index = index
                if command_number % 10 == 0 or index == len(path)-1:
                    print(f'Command {command_number}, '
                          f'path point {index}/{len(path)-1}, error {error:.2f} deg', flush=True)
            # Check the actual feedback, rather than just the planned endpoint.
            fk = self.create_client(GetPositionFK, '/compute_fk')
            if not fk.wait_for_service(timeout_sec=5):
                raise RuntimeError('Final feedback FK service unavailable; cannot confirm arrival')
            request = GetPositionFK.Request(); request.header.frame_id = report['base_frame']
            request.fk_link_names = [report['link']]
            request.robot_state.joint_state.name = JOINTS
            request.robot_state.joint_state.position = current.tolist()
            out = self.wait(fk.call_async(request), 5)
            if out.error_code.val != 1 or len(out.pose_stamped) != 1:
                raise RuntimeError('Final feedback FK failed; cannot confirm arrival')
            position = out.pose_stamped[0].pose.position
            actual = np.array([position.x, position.y, position.z])
            distance_mm = float(np.linalg.norm(actual-report['target_position_m']) * 1000.)
            report['final_feedback_fk_position_m'] = actual.tolist()
            report['final_feedback_fk_position_error_mm'] = distance_mm
            if distance_mm > 6.0:
                raise RuntimeError(f'Final feedback FK position error {distance_mm:.2f} mm exceeds 6 mm')
            report['execution_status'] = 'reached'
            report['final_model_joint_radians'] = current.tolist()
            report['final_hardware_degrees'] = hardware_degrees(current).tolist()
            report['final_joint_error_max_deg'] = float(np.max(np.abs(np.degrees(current-path[-1]))))
        except BaseException as exc:
            robot.stop()
            report['execution_status'] = 'stopped'
            report['execution_error'] = str(exc)
            if 'current' in locals():
                report['last_feedback_model_radians'] = current.tolist()
                report['last_feedback_hardware_degrees'] = hardware_degrees(current).tolist()
            if report.get('waypoint_feedback'):
                feedback = report['waypoint_feedback'][-1]
                # A failed helper has more recent feedback than the last
                # completed waypoint returned to this function.
                if 'feedback_hardware_degrees' in feedback:
                    report['last_feedback_hardware_degrees'] = feedback['feedback_hardware_degrees']
                    values = np.radians(feedback['feedback_hardware_degrees'])
                    values[5] += math.pi/4
                    report['last_feedback_model_radians'] = values.tolist()
            if 'index' in locals():
                report['last_waypoint_index'] = index
                report['last_command_hardware_degrees'] = hardware_degrees(target).tolist()
            raise
        finally:
            args.plan.write_text(json.dumps(report, indent=2)+'\n')
            robot._serial_port.close()
        print('Reached high pregrasp pose. No descent or gripper command sent.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Execute saved, recent plan with joint feedback')
    parser.add_argument('--speed', type=int, default=5,
                        help='Execution SDK speed, 1..15 (default 5)')
    parser.add_argument('--plan', type=Path, default=Path('/home/jetson/pregrasp_charuco_handeye_plan.json'))
    parser.add_argument('--calibration', type=Path,
                        default=Path('/home/jetson/charuco_handeye_result_filtered.json'))
    parser.add_argument('--tag-id', type=int, help='Observed tag36h11 ID; default 1 for ChArUco hand-eye results')
    parser.add_argument('--marker-length', type=float,
                        help='Observed single AprilTag outer edge length in meters; default 0.020')
    parser.add_argument('--observe-only', action='store_true',
                        help='Measure live AprilTag and check chain length bound, without MoveIt planning')
    parser.add_argument('--height', type=float, default=.20,
                        help='Selected link height above Tag in base Z, meters')
    parser.add_argument('--link', choices=['6_Link', 'jiazhua_Link'], default='6_Link')
    parser.add_argument('--pitch', type=float,
                        help='Optional explicit tool pitch in radians; tool X points toward Tag in base XY')
    parser.add_argument('--nearby', action='store_true', help='Prefer IK close to current joints')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--own-serial', action='store_true',
                        help='Read joint feedback directly while planning; GUI must be closed')
    args = parser.parse_args()
    if args.execute and (args.observe_only or args.own_serial):
        parser.error('--execute cannot be combined with --observe-only or --own-serial')
    if not 1 <= args.speed <= 15:
        parser.error('--speed must be in [1, 15]')
    if not math.isfinite(args.height) or args.height < .15:
        parser.error('First approach height must be at least 0.15 m')
    if args.pitch is not None and not math.isfinite(args.pitch):
        parser.error('--pitch must be finite')
    if args.plan.resolve() == args.calibration.resolve():
        parser.error('--plan must not overwrite the calibration file')
    rclpy.init(); node = Pregrasp()
    robot = None
    try:
        if args.own_serial and not args.execute:
            if subprocess.run(['fuser', args.port], capture_output=True).returncode == 0:
                raise RuntimeError('Close the serial GUI before --own-serial planning')
            from pymycobot.mycobot import MyCobot
            robot = MyCobot(args.port, 1000000)
            node.read_robot(robot)
            node.feedback_timer = node.create_timer(.2, lambda: node.read_robot(robot))
        node.execute(args) if args.execute else node.plan(args)
    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        if robot is not None:
            robot._serial_port.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
