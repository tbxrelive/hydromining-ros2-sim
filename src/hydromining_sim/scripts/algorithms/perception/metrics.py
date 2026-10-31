"""仅验收时使用的网格真值度量，禁止从感知运行链路导入。"""
import numpy as np
import open3d as o3d
import trimesh
from scipy.spatial.transform import Rotation


def mesh_heights(mesh_path, mesh_pose, frame, extent, resolution, ray_start=0.6,
                 return_normals=False):
    """沿固定初始法向光线求交，返回最靠近观测侧的真实表面高度。"""
    pose = np.asarray(mesh_pose, float)
    if pose.shape != (6,) or not np.isfinite(pose).all() or ray_start <= 0:
        raise ValueError('invalid mesh pose or ray start')
    mesh = trimesh.load(mesh_path, process=False, force='mesh')
    vertices = mesh.vertices @ Rotation.from_euler('xyz', pose[3:]).as_matrix().T + pose[:3]
    scene = o3d.t.geometry.RaycastingScene(nthreads=1)
    scene.add_triangles(
        o3d.core.Tensor(np.ascontiguousarray(vertices, np.float32)),
        o3d.core.Tensor(np.ascontiguousarray(mesh.faces, np.uint32)))
    local = frame.grid(extent, resolution)
    local[..., 2] = ray_start
    starts = frame.to_world(local)
    direction = np.broadcast_to(-frame.axes[:, 2], starts.shape)
    rays = np.concatenate((starts, direction), axis=-1).astype(np.float32)
    result = scene.cast_rays(o3d.core.Tensor(rays), nthreads=1)
    hits = result['t_hit'].numpy()
    heights = np.where(np.isfinite(hits), ray_start - hits, np.nan)
    if not return_normals:
        return heights
    normals = result['primitive_normals'].numpy().astype(float)
    flip = normals @ frame.axes[:, 2] < 0
    normals[flip] *= -1
    normals[~np.isfinite(hits)] = np.nan
    return heights, normals


def truth_change(before, after, frame, extent, resolution, radius, threshold, fraction, allow_missing=False):
    """真值类别沿固定法向计算，另外记录无交点，禁止填补无真值区域。"""
    local = frame.grid(extent, resolution)
    region = np.linalg.norm(local[..., :2], axis=-1) <= radius
    valid = np.isfinite(before) & np.isfinite(after)
    delta = np.asarray(before) - np.asarray(after)
    count = int(np.count_nonzero(region))
    if not count:
        raise ValueError('truth does not cover the entire task region')
    if not valid[region].all():
        if not allow_missing:
            raise ValueError('truth does not cover the entire task region')
        return {'status': 'unknown', 'fraction_ge_threshold': None,
                'sampled_area_m2': count * resolution ** 2, 'removed_volume_m3': None,
                'maximum_retreat_m': None, 'resolution_m': resolution,
                'valid_fraction': float(np.count_nonzero(valid & region)/count)}
    passed_fraction = float(np.mean(delta[region] >= threshold))
    area = resolution ** 2
    return {
        'status': 'achieved' if passed_fraction >= fraction else 'not_achieved',
        'fraction_ge_threshold': passed_fraction, 'sampled_area_m2': count * area,
        'removed_volume_m3': float(np.maximum(delta[region], 0).sum() * area),
        'maximum_retreat_m': float(delta[region].max()),
        'resolution_m': resolution,
    }
