"""URDF kinematics and weighted differential IK; no ROS or serial I/O."""

from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np

from .tracking_pid import norm_limit
from .trajectory_guard import JOINT_NAMES


def skew(v):
    """Return the cross-product matrix."""
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(axis, angle):
    """Apply Rodrigues' formula for a unit axis."""
    s = skew(axis)
    return np.eye(3) + np.sin(angle) * s + (1. - np.cos(angle)) * s @ s


def rotation_error(desired, current):
    """Return a bounded spatial rotation vector without Euler wrapping."""
    r = desired @ current.T
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / 2.
    sine = np.linalg.norm(v)
    cosine = np.clip((np.trace(r) - 1.) / 2., -1., 1.)
    if sine > 1.e-7:
        return v * (np.arctan2(sine, cosine) / sine)
    if cosine < 0:
        values, vectors = np.linalg.eigh((r + np.eye(3)) / 2.)
        return vectors[:, np.argmax(values)] * np.pi
    return v


def sdk_pose_transform(coords, alignment):
    """Convert SDK mm/deg using an explicit, independently verified frame mapping."""
    coords = np.asarray(coords, dtype=float)
    if coords.shape != (6,) or not np.all(np.isfinite(coords)):
        raise ValueError('get_coords must return six finite mm/deg values')
    if alignment['euler_convention'] not in ('body_zyx', 'extrinsic_xyz'):
        raise ValueError('Supported SDK Euler conventions: body_zyx (Rz Ry Rx)')
    rx, ry, rz = np.radians(coords[3:])
    transform = np.eye(4)
    transform[:3, 3] = coords[:3] / 1000.
    transform[:3, :3] = (rotation([0, 0, 1], rz) @ rotation([0, 1, 0], ry) @
                         rotation([1, 0, 0], rx))
    return (np.asarray(alignment['base_from_sdk_base']) @ transform @
            np.asarray(alignment['sdk_tool_from_model_ee']))


def check_sdk_fk(robot, model, config):
    """Read angles/coords/angles while stationary; perform no SDK writes."""
    from .trajectory_guard import model_radians
    c = config['fk_check']
    before = model_radians(robot.get_angles())
    coords = robot.get_coords()
    after = model_radians(robot.get_angles())
    if np.max(np.abs(after - before)) > np.radians(c['stationary_tolerance_deg']):
        raise ValueError('FK check requires a stationary arm; angle samples changed')
    aligned = sdk_pose_transform(coords, c)
    result = model.self_check((before + after) / 2., aligned,
                              c['position_tolerance_mm'] / 1000.,
                              np.radians(c['angle_tolerance_deg']))
    result['alignment_verified'] = c['alignment_verified']
    result['ok'] = bool(result['ok'] and c['alignment_verified'])
    result['sdk_coords_mm_deg'] = np.asarray(coords).tolist()
    return result


@dataclass
class Pose:
    """Transforms, joint axes and collision segments in the base frame."""

    ee: np.ndarray
    camera: np.ndarray
    origins: np.ndarray
    axes: np.ndarray
    segments: np.ndarray
    links: dict


class Kinematics:
    """Use the project's joint origins and axes instead of a copied DH table."""

    def __init__(self, urdf, handeye, margin_rad=0., hardware_limits_deg=None):
        """Extract exactly six named arm joints from base_link to 6_Link."""
        root = ET.parse(urdf).getroot()
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, child = [], '6_Link'
        while child != 'base_link':
            joint = by_child[child]
            chain.append(joint)
            child = joint.find('parent').get('link')
        self.chain = []
        self.raw_limits = []
        names = []
        for joint in reversed(chain):
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = (
                rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @
                rotation([1, 0, 0], rpy[0]))
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axis /= np.linalg.norm(axis)
            limit = joint.find('limit')
            self.raw_limits.append([float(limit.get('lower')), float(limit.get('upper'))])
            names.append(joint.get('name'))
            self.chain.append((transform, axis))
        if names != JOINT_NAMES:
            raise ValueError('URDF arm chain must contain ordered 1_Joint..6_Joint')
        self.raw_limits = np.asarray(self.raw_limits)
        if hardware_limits_deg is not None:
            # 保持官方URDF不变，驱动层取实机范围与模型范围的交集，再留安全余量。
            sdk = np.asarray(hardware_limits_deg, dtype=float)
            if sdk.shape != (6, 2) or not np.all(np.isfinite(sdk)) or np.any(sdk[:, 0] >= sdk[:, 1]):
                raise ValueError('Invalid SDK joint limits')
            sdk = np.radians(sdk)
            sdk[5] += np.pi / 4
            self.raw_limits[:, 0] = np.maximum(self.raw_limits[:, 0], sdk[:, 0])
            self.raw_limits[:, 1] = np.minimum(self.raw_limits[:, 1], sdk[:, 1])
        self.limits = self.raw_limits + np.array([margin_rad, -margin_rad])
        self.handeye = np.asarray(handeye, dtype=float)
        if (self.handeye.shape != (4, 4) or not np.all(np.isfinite(self.handeye)) or
                not np.allclose(self.handeye[3], [0, 0, 0, 1]) or
                not np.allclose(self.handeye[:3, :3].T @ self.handeye[:3, :3], np.eye(3), atol=1.e-5) or
                not np.isclose(np.linalg.det(self.handeye[:3, :3]), 1., atol=1.e-5)):
            raise ValueError('Invalid hand-eye transform')
        gripper = root.find("joint[@name='jiazhua_Joint']/origin")
        self.tool_offset = np.fromstring(gripper.get('xyz'), sep=' ')

    def fk(self, q):
        """Compute transforms and geometry from measured model radians."""
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('FK requires six finite joint angles')
        t, origins, axes = np.eye(4), [], []
        links = {'base_link': t.copy()}
        for index, (value, (origin, axis)) in enumerate(zip(q, self.chain), 1):
            t = t @ origin
            origins.append(t[:3, 3].copy())
            axes.append(t[:3, :3] @ axis)
            turn = np.eye(4)
            turn[:3, :3] = rotation(axis, value)
            t = t @ turn
            links[f'{index}_Link'] = t.copy()
        camera = t @ self.handeye
        tool = t[:3, 3] + t[:3, :3] @ self.tool_offset
        points = np.row_stack((origins, tool))
        segments = [np.array([points[i], points[i + 1]]) for i in range(6)]
        segments.append(np.array([t[:3, 3], camera[:3, 3]]))
        return Pose(t, camera, np.asarray(origins), np.asarray(axes), np.asarray(segments), links)

    def jacobian(self, q, camera=True):
        """Compute the geometric Jacobian at the camera or tool origin."""
        pose = self.fk(q)
        point = (pose.camera if camera else pose.ee)[:3, 3]
        return np.row_stack((np.cross(pose.axes, point - pose.origins).T, pose.axes.T))

    def manipulability(self, q):
        """Use singular-value products for the position task."""
        return float(np.prod(np.linalg.svd(self.jacobian(q)[:3], compute_uv=False)))

    def self_check(self, q, sdk_transform, position_tolerance=.01, angle_tolerance=.1):
        """Compare aligned SDK and URDF tool poses, without accessing serial."""
        actual = self.fk(q).ee
        position = float(np.linalg.norm(actual[:3, 3] - sdk_transform[:3, 3]))
        angle = float(np.linalg.norm(rotation_error(sdk_transform[:3, :3], actual[:3, :3])))
        return {'position_error_m': position, 'rotation_error_rad': angle,
                'ok': position <= position_tolerance and angle <= angle_tolerance}


class DifferentialIK:
    """Solve weighted DLS with a true SVD nullspace and bounded secondary tasks."""

    def __init__(self, model, config):
        """Share model limits with safety and the executor."""
        self.model, self.config = model, config
        self.last_velocity = np.zeros(6)
        self.diagnostics = {}

    def reset(self):
        """Discard velocity history after a hold or feedback interruption."""
        self.last_velocity[:] = 0.

    def solve(self, q, linear, desired_rotation, dt, full_pose=False, aim_angular=None):
        """Return joint velocity; each call starts from measured/bounded state."""
        q = np.asarray(q, dtype=float)
        c = self.config['ik']
        jacobian = self.model.jacobian(q)
        pose = self.model.fk(q)
        orient = norm_limit(c['orientation_gain'] * rotation_error(
            desired_rotation, pose.camera[:3, :3]), c['max_angular_speed_rad_s'])
        scale = 1. / c['reference_length_m']
        task = np.row_stack((jacobian[:3] * scale, jacobian[3:])) if full_pose else jacobian[:3] * scale
        velocity = np.r_[linear * scale, orient] if full_pose else np.asarray(linear) * scale
        if aim_angular is not None:
            # 视野恢复：保持相机位置，主控相机X/Y两个倾转轴，光轴滚转不作主任务。
            axes = pose.camera[:3, :2].T
            task = np.row_stack((jacobian[:3]*scale, axes @ jacobian[3:]))
            velocity = np.r_[np.zeros(3), axes @ aim_angular]
        lo, hi = self.model.limits.T
        middle, half = (lo + hi) / 2., (hi - lo) / 2.
        normalized = (q - middle) / half
        weights = np.minimum(c['weight_max'], 1. + .1 / np.maximum(1. - normalized**2, .01)**2)
        secondary = -c['centering_gain'] * normalized / half
        secondary += jacobian[3:].T @ orient
        if c['manipulability_gain'] > 0 and not full_pose and aim_angular is None:
            h = 1.e-4
            grad = np.array([(np.log(max(self.model.manipulability(q + np.eye(6)[i] * h), 1.e-12)) -
                              np.log(max(self.model.manipulability(q - np.eye(6)[i] * h), 1.e-12))) / (2 * h)
                             for i in range(6)])
            secondary += c['manipulability_gain'] * grad
        secondary = norm_limit(secondary, c['secondary_speed_rad_s'])
        free = np.ones(6, dtype=bool)
        result = np.zeros(6)
        sigma, damping = 0., 0.
        for _ in range(7):
            if not np.any(free):
                break
            j = task[:, free]
            inv_sqrt = 1. / np.sqrt(weights[free])
            a = j * inv_sqrt
            u, singular, vt = np.linalg.svd(a, full_matrices=False)
            sigma = float(singular[-1]) if len(singular) >= task.shape[0] else 0.
            damping = c['lambda_max'] * np.sqrt(max(0., 1. - (sigma / c['epsilon'])**2))
            factors = singular / np.maximum(singular**2 + damping**2, 1.e-14)
            primary = inv_sqrt * (vt.T @ (factors * (u.T @ velocity)))
            _, sj, vj = np.linalg.svd(j, full_matrices=True)
            rank = int(np.sum(sj > c['rank_tolerance']))
            null = vj[rank:].T @ vj[rank:]
            # 阻尼伪逆的 I-J+J 不是真零空间；此处显式用 SVD 零空间。
            fade = min(1., sigma / c['epsilon'])**2
            result[:] = 0.
            result[free] = primary + (null @ secondary[free]) * fade
            vmax = np.asarray(self.config['model']['max_velocity_rad_s'])
            result *= min(1., float(np.min(vmax / np.maximum(np.abs(result), 1.e-12))))
            predicted = q + result * dt
            blocked = free & (((predicted < lo) & (result < 0)) | ((predicted > hi) & (result > 0)))
            if not np.any(blocked):
                break
            free[blocked] = False
        self.diagnostics = {'sigma_min': sigma, 'damping': float(damping),
                            'limit_margin_rad': float(np.min(np.r_[q - lo, hi - q]))}
        return result
