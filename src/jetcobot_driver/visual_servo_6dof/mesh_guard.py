"""Check the original Yahboom collision meshes without modifying model resources."""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from .kinematics import rotation
from .tracking_config import package_file


def origin_transform(element):
    """URDF固定轴RPY：Rz(yaw) Ry(pitch) Rx(roll)。"""
    result = np.eye(4)
    if element is not None:
        r, p, y = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    return result


def mesh_path(uri):
    """Resolve a package mesh URI using the existing package resources."""
    if not uri.startswith('package://'):
        raise ValueError(f'Unsupported collision mesh URI: {uri}')
    package, relative = uri[len('package://'):].split('/', 1)
    return package_file(package, relative)


def read_stl(path):
    """Read binary STL in metres, rejecting missing/invalid geometry."""
    raw = Path(path).read_bytes()
    count = struct.unpack('<I', raw[80:84])[0] if len(raw) >= 84 else 0
    if not count or len(raw) != 84 + 50 * count:
        raise ValueError(f'Expected nonempty binary STL: {path}')
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    vertices = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(np.float64)
    if not np.all(np.isfinite(vertices)):
        raise ValueError('Nonfinite collision mesh')
    return vertices, np.arange(len(vertices), dtype=np.int32).reshape(-1, 3)


class MeshGuard:
    """Use official collision pairs and conservative hulls enclosing each URDF mesh."""

    def __init__(self, model, config):
        """Load meshes once; a missing FCL extension fails closed."""
        from . import _fcl_guard
        self.native, self.model, self.config = _fcl_guard, model, config
        root = ET.parse(config['model']['urdf']).getroot()
        self.names, self.vertices, self.geometry, self.local = [], [], [], []
        for link in root.findall('link'):
            collisions = link.findall('collision')
            if len(collisions) > 1:
                raise ValueError('Multiple collision geometries require explicit handling')
            for collision in collisions:
                mesh = collision.find('geometry/mesh')
                if mesh is None:
                    raise ValueError('Expected official mesh collision geometry')
                v, faces = read_stl(mesh_path(mesh.get('filename')))
                scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                v *= scale
                # 凸包包含原始网格：不会缩小连杆，且比逐三角形距离计算快。
                from scipy.spatial import ConvexHull
                hull = ConvexHull(v)
                indices = np.unique(hull.simplices)
                mapping = np.zeros(len(v), dtype=np.int32)
                mapping[indices] = np.arange(len(indices), dtype=np.int32)
                faces = mapping[hull.simplices].copy()
                v = v[indices]
                normals = np.cross(v[faces[:, 1]] - v[faces[:, 0]], v[faces[:, 2]] - v[faces[:, 0]])
                reverse = np.sum(normals * hull.equations[:, :3], axis=1) < 0
                faces[reverse] = faces[reverse][:, [0, 2, 1]]
                self.geometry.append(self.native.make_mesh(v.tobytes(), faces.tobytes()))
                self.names.append(link.get('name'))
                self.vertices.append(v)
                self.local.append(origin_transform(collision.find('origin')))
        expected = {'base_link', *(f'{i}_Link' for i in range(1, 7)), 'camera_link', 'jiazhua_Link'}
        if set(self.names) != expected:
            raise ValueError('Incomplete official collision geometry')
        self.fixed = []
        for joint in root.findall('joint'):
            if joint.get('type') == 'fixed' and joint.find('parent').get('link') == '6_Link':
                self.fixed.append((joint.find('child').get('link'), origin_transform(joint.find('origin'))))
        srdf = ET.parse(config['model']['srdf']).getroot()
        ignored = {frozenset((p.get('link1'), p.get('link2'))) for p in srdf.findall('disable_collisions')}
        self.pairs = [(i, j) for i, a in enumerate(self.names) for j, b in enumerate(self.names)
                      if j > i and frozenset((a, b)) not in ignored]
        self.labels = [f'mesh_{self.names[i]}_{self.names[j]}' for i, j in self.pairs]
        self.previous_q = self.previous_result = None
        self.cache = {}

    def constraints(self, q):
        """Return exact mesh gaps and full moving-link clearance above the table."""
        parameters = tuple(self.config['safety'][k] for k in
                           ('table_z_m', 'table_clearance_m', 'self_clearance_m'))
        key = (np.asarray(q).tobytes(), parameters)
        if key in self.cache:
            return self.cache[key]
        links = self.model.fk(q).links.copy()
        for name, fixed in self.fixed:
            links[name] = links['6_Link'] @ fixed
        transforms = [links[name] @ local for name, local in zip(self.names, self.local)]
        # 世界轴包围盒间距是实体间距的下界；远处分离时直接采用保守下界。
        world_vertices = [v @ tf[:3, :3].T + tf[:3, 3] for v, tf in zip(self.vertices, transforms)]
        bounds = [(v.min(0), v.max(0)) for v in world_vertices]
        distances, near_pairs, near_indices = [], [], []
        for k, (i, j) in enumerate(self.pairs):
            separation = np.maximum(0., np.maximum(bounds[i][0]-bounds[j][1], bounds[j][0]-bounds[i][1]))
            lower_bound = float(np.linalg.norm(separation))
            distances.append(lower_bound)
            if lower_bound < self.config['safety']['gradient_distance_m'] + .005:
                near_pairs.append((i, j))
                near_indices.append(k)
        distances = np.asarray(distances)
        if near_pairs:
            distances[near_indices] = self.native.distances(
                self.geometry, np.ascontiguousarray(transforms, dtype=np.float64).tobytes(), near_pairs)
        if not np.all(np.isfinite(distances)):
            raise ValueError('Invalid FCL distance')
        values = (distances - self.config['safety']['self_clearance_m']).tolist()
        labels = self.labels.copy()
        table = self.config['safety']['table_z_m']
        if table is not None:
            for name, vertices in zip(self.names, world_vertices):
                # 只有固定基座与安装桌面接触；所有运动连杆均检查完整网格最低点。
                if name == 'base_link':
                    continue
                minimum_z = np.min(vertices[:, 2])
                values.append(float(minimum_z - table - self.config['safety']['table_clearance_m']))
                labels.append(f'mesh_table_{name}')
        self.previous_q = np.array(q).copy()
        self.previous_parameters = parameters
        self.previous_result = np.asarray(values), labels
        if len(self.cache) >= 32:
            self.cache.pop(next(iter(self.cache)))
        self.cache[key] = self.previous_result
        return self.previous_result
