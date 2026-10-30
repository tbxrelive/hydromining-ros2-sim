"""传感器坐标转换，明确区分 Gazebo 相机坐标与 ROS 光学坐标。"""
import numpy as np
from scipy.spatial.transform import Rotation


def transform_points(points, pose):
    """将 (..., 3) 点数组按 [tx, ty, tz, qx, qy, qz, qw] 变换，保留无效点。"""
    points = np.asarray(points, dtype=float)
    pose = np.asarray(pose, dtype=float)
    if points.ndim < 1 or points.shape[-1] != 3:
        raise ValueError('points must have shape (..., 3)')
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError('pose must contain seven finite values')
    if not np.isclose(np.linalg.norm(pose[3:]), 1.0, atol=1e-5):
        raise ValueError('quaternion must be normalized')
    valid = np.isfinite(points).all(axis=-1)
    result = np.full(points.shape, np.nan)
    result[valid] = points[valid] @ Rotation.from_quat(pose[3:]).as_matrix().T + pose[:3]
    return result


def backproject_depth(depth, intrinsics, convention='gazebo'):
    """反投影轴向深度；Gazebo 为 +X 前/+Y 左/+Z 上，optical 为 +Z 前。

    深度不是斜距，不能先归一化射线再乘深度。必须使用与此图像一致的内参。
    """
    depth = np.asarray(depth, dtype=float)
    k = np.asarray(intrinsics, dtype=float)
    if depth.ndim != 2:
        raise ValueError('depth must be a two-dimensional array')
    if k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('intrinsics must be a finite 3x3 matrix with positive focal lengths')
    if convention not in ('gazebo', 'optical'):
        raise ValueError('unknown camera coordinate convention')
    valid = np.isfinite(depth) & (depth > 0)
    depth = np.where(valid, depth, 0.0)
    rows, cols = np.indices(depth.shape)
    x = (cols - k[0, 2]) * depth / k[0, 0]
    y = (rows - k[1, 2]) * depth / k[1, 1]
    result = np.stack((depth, -x, -y) if convention == 'gazebo' else (x, y, depth), axis=-1)
    result[~valid] = np.nan
    return result


def apply_range_measurements(points, ranges):
    """沿原始点的射线替换斜距；用于 LiDAR 及统一的环境退化层。"""
    points = np.asarray(points, dtype=float)
    ranges = np.asarray(ranges, dtype=float)
    if points.shape[:-1] != ranges.shape or points.shape[-1] != 3:
        raise ValueError('range shape must match point leading dimensions')
    original = np.linalg.norm(points, axis=-1)
    valid = (np.isfinite(points).all(axis=-1) & np.isfinite(ranges) &
             (original > 0) & (ranges > 0))
    result = np.full(points.shape, np.nan)
    result[valid] = points[valid] * (ranges[valid] / original[valid])[..., None]
    return result


def fit_reference_plane(points, center, viewpoint, minimum_points=20):
    """用基线局部观测拟合固定参考平面，法向朝向观测者，不读取煤壁真值。

    通过残差中位数与 MAD 迭代裁剪离群点。调用方必须先用任务区域筛选局部点。
    返回 (投影后的中心, 朝外单位法向, 平面残差尺度)，不是完整曲面重建。
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    center, viewpoint = np.asarray(center, float), np.asarray(viewpoint, float)
    if len(points) < minimum_points or not np.isfinite(np.r_[center, viewpoint]).all():
        raise ValueError('insufficient points or invalid reference geometry')
    selected = points
    for _ in range(5):
        centroid = selected.mean(axis=0)
        _, singular, axes = np.linalg.svd(selected - centroid, full_matrices=False)
        if singular[1] <= 1e-9:
            raise ValueError('reference points must span a surface, not a line')
        normal = axes[-1]
        residual = (points - centroid) @ normal
        median = np.median(residual)
        scale = 1.4826 * np.median(np.abs(residual - median))
        inliers = np.abs(residual - median) <= max(3 * scale, 1e-6)
        if inliers.sum() < minimum_points:
            raise ValueError('insufficient plane inliers')
        selected = points[inliers]
    centroid = selected.mean(axis=0)
    _, _, axes = np.linalg.svd(selected - centroid, full_matrices=False)
    normal = axes[-1]
    if normal @ (viewpoint - center) < 0:
        normal = -normal
    projected = center - normal * ((center - centroid) @ normal)
    return projected, normal, float(scale)
