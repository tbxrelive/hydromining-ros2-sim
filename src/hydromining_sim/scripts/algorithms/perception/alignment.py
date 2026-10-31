"""静止视点快照对齐；使用各自时间戳的 TF，不假定两传感器同时曝光。"""
import numpy as np
from scipy.spatial.transform import Rotation

from .frames import backproject_depth, transform_points
from .sensor_model import depth_sigma, lidar_sigma


def stamp_ns(stamp):
    """严格解析 ROS [sec, nanosec]，避免浮点时间戳丢失精度。"""
    stamp = np.asarray(stamp)
    if (stamp.shape != (2,) or stamp.dtype.kind not in 'iu' or
            stamp[0] < 0 or not 0 <= stamp[1] < 1_000_000_000):
        raise ValueError('invalid ROS timestamp')
    return int(stamp[0]) * 1_000_000_000 + int(stamp[1])


def validate_sequence(snapshots, max_skew_s=0.6, max_translation_m=0.002,
                      max_rotation_rad=0.01, camera_frame='camera_link',
                      lidar_frame='lidar_link'):
    """拒绝乱序、重复、错误 frame 和移动视点；当前不支持运动中去畸变。"""
    if not snapshots:
        raise ValueError('empty observation sequence')
    previous = {'depth': -1, 'lidar': -1}
    first = snapshots[0]
    for snapshot in snapshots:
        times = {}
        for sensor, expected in (('depth', camera_frame), ('lidar', lidar_frame)):
            if str(snapshot[f'{sensor}_frame']) != expected:
                raise ValueError(f'unexpected {sensor} coordinate frame')
            times[sensor] = stamp_ns(snapshot[f'{sensor}_stamp'])
            if times[sensor] <= previous[sensor]:
                raise ValueError(f'duplicate or out-of-order {sensor} timestamp')
            previous[sensor] = times[sensor]
            pose = np.asarray(snapshot[f'tf_{sensor}'], float)
            transform_points(np.zeros((1, 3)), pose)
            initial = np.asarray(first[f'tf_{sensor}'], float)
            rotation = Rotation.from_quat(initial[3:]).inv() * Rotation.from_quat(pose[3:])
            if (np.linalg.norm(pose[:3] - initial[:3]) > max_translation_m or
                    rotation.magnitude() > max_rotation_rad):
                raise ValueError('temporal filtering requires a stationary viewpoint')
        if abs(times['depth'] - times['lidar']) > max_skew_s * 1e9:
            raise ValueError('sensor timestamp skew exceeds stationary snapshot budget')
    return {'frames': len(snapshots), 'moving_sensor_deskew_verified': False}


def aligned_sensor(snapshot, sensor, noise_scale=1.0):
    """返回 world 点、射线方向、斜距 sigma；原始无效点保持无效。"""
    if sensor == 'depth':
        points = backproject_depth(snapshot['depth'], snapshot['K'])
        ranges = np.linalg.norm(points, axis=-1)
        depth = np.asarray(snapshot['depth'], float)
        valid = np.isfinite(ranges)
        ratio = np.ones(depth.shape)
        np.divide(ranges, depth, out=ratio, where=valid)
        sigma = depth_sigma(np.where(valid, depth, 0.0), noise_scale) * ratio
    elif sensor == 'lidar':
        points = np.asarray(snapshot['lidar_local'], float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('lidar must have shape (N, 3)')
        ranges = np.linalg.norm(points, axis=-1)
        valid = np.isfinite(ranges) & (ranges > 0)
        sigma = lidar_sigma(np.where(valid, ranges, 0.0), noise_scale)
    else:
        raise ValueError('unknown sensor')
    pose = np.asarray(snapshot[f'tf_{sensor}'], float)
    world = transform_points(points, pose)
    directions = np.full(world.shape, np.nan)
    np.divide(world - pose[:3], ranges[..., None], out=directions,
              where=valid[..., None])
    sigma = np.where(valid, sigma, np.nan)
    return world, directions, sigma
