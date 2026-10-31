"""可见性预测的共享原语：传感器视场 + 高度场射线遮挡。

此前有四处各自实现（L0 环境、L1 真值掩码、NBV 观测预测、管线探针），
相机投影与 FOV 细节互不一致。这里统一为：把世界点变换到传感器系、
按相机/激光雷达判视场、沿射线在高度场采样判遮挡。射线遮挡与
`heightfield_erosion` 的暴露判定使用同一套格点索引约定。
"""
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.spatial.transform import Rotation


def pose_matrix(pose):
    """接受 4x4 变换或 [x,y,z,qx,qy,qz,qw]，返回 T_world_sensor。"""
    pose = np.asarray(pose, float)
    if pose.shape == (4, 4):
        return pose
    if pose.shape != (7,):
        raise ValueError('pose must be 4x4 or 7-vector')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def sensor_points(points_world, pose):
    """世界点 -> 传感器坐标；points_world 形状为 (..., 3)。"""
    matrix = pose_matrix(pose)
    return (np.asarray(points_world, float) - matrix[:3, 3]) @ matrix[:3, :3]


def camera_inside(local, K, image_shape, min_depth=0.):
    """针孔投影视场判断；local 为传感器坐标。"""
    image_height, image_width = image_shape
    x, y, z = local[..., 0], local[..., 1], local[..., 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        col = K[0, 2] - K[0, 0]*y/x
        row = K[1, 2] - K[1, 1]*z/x
    return (x > min_depth) & (col >= 0) & (col < image_width) & (row >= 0) & (row < image_height)


def lidar_inside(local, elevation_limit_deg=15.):
    """VLP-16 类 360° 方位、给定俯仰带；与 Gazebo render_lidar 约定一致。"""
    elevation = np.arctan2(local[..., 2], np.linalg.norm(local[..., :2], axis=-1))
    return np.abs(elevation) <= np.deg2rad(elevation_limit_deg)


def sample_heights(heights, x, y, spacing, extent, order=1, cval=-np.inf):
    """按本地坐标 (x, y) 在高度场采样；索引约定与高度场蚀除一致。"""
    indices = np.stack(((y+extent/2)/spacing-.5, (x+extent/2)/spacing-.5))
    return map_coordinates(heights, indices, order=order, mode='constant', cval=cval)


def march_blocked(heights, origin_local, targets_local, fractions, spacing, extent,
                  order=1, cval=-np.inf, tolerance=0., finite_only=False):
    """沿 origin->target 的射线在高度场采样，低于表面即判遮挡。

    finite_only=True 时只有采到有效高度才判遮挡（用于实测高度图的空洞）。
    """
    origin_local = np.asarray(origin_local, float)
    targets_local = np.asarray(targets_local, float)
    blocked = np.zeros(targets_local.shape[:-1], bool)
    for fraction in fractions:
        ray = origin_local + fraction*(targets_local-origin_local)
        sampled = sample_heights(heights, ray[..., 0], ray[..., 1], spacing, extent,
                                 order=order, cval=cval)
        below = ray[..., 2] < sampled - tolerance
        if finite_only:
            below &= np.isfinite(sampled)
        blocked |= below
    return blocked
