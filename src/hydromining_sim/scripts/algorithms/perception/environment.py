"""对录制快照施加离线环境退化，不读取真值或依赖 ROS。"""
import hashlib

import numpy as np

from .frames import apply_range_measurements, backproject_depth
from .sensor_model import (
    DROPPED, FALSE_RETURN, INVALID, VALID, degrade_ranges, depth_sigma,
    lidar_sigma)


def sensor_rng(seed, snapshot, sensor):
    """按快照和传感器派生随机流；跨工况共用随机数，不受遍历顺序影响。"""
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError('seed must be a nonnegative integer')
    digest = hashlib.sha256(f'{seed}/{snapshot}/{sensor}'.encode('utf-8')).digest()
    return np.random.default_rng(int.from_bytes(digest[:16], 'little'))


def degrade_snapshot(depth, intrinsics, lidar, visibility, noise_scale,
                     depth_rng, lidar_rng, r_min=0.1):
    """沿各自传感器射线退化，返回观测和独立的评估标签。

    相机深度为 +X 轴向距离；消光按斜距计算。轴向 sigma 必须先转换为
    斜距 sigma，最后再转回轴向深度，避免边缘像素的不确定性单位混用。
    此模型不模拟 RGB 昏暗、原生粒子或传感器系统偏差。
    """
    depth = np.asarray(depth, dtype=float)
    lidar = np.asarray(lidar, dtype=float)
    if lidar.ndim != 2 or lidar.shape[1] != 3:
        raise ValueError('lidar must have shape (N, 3)')
    camera = backproject_depth(depth, intrinsics)
    camera_range = np.linalg.norm(camera, axis=-1)
    valid = np.isfinite(camera_range)
    ratio = np.ones(depth.shape)
    np.divide(camera_range, depth, out=ratio, where=valid)
    sigma = depth_sigma(np.where(valid, depth, 0.0), noise_scale) * ratio
    degraded_range, depth_labels = degrade_ranges(
        camera_range, sigma, visibility, depth_rng, r_min)
    degraded_depth = degraded_range / ratio
    lidar_range = np.linalg.norm(lidar, axis=-1)
    sigma = lidar_sigma(np.where(np.isfinite(lidar_range), lidar_range, 0.0),
                        noise_scale)
    degraded_range, lidar_labels = degrade_ranges(
        lidar_range, sigma, visibility, lidar_rng, r_min)
    degraded_lidar = apply_range_measurements(lidar, degraded_range)
    return {
        'depth': degraded_depth,
        'lidar_local': degraded_lidar,
        'depth_labels': depth_labels,
        'lidar_labels': lidar_labels,
    }


def label_counts(labels):
    """统计全视场射线标签，不将假回波当成可靠表面观测。"""
    return {name: int(np.count_nonzero(labels == value)) for name, value in (
        ('valid', VALID), ('dropped', DROPPED), ('false_return', FALSE_RETURN),
        ('invalid', INVALID))}
