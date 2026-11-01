"""阶段七替身环境的世界侧：离线蚀除、射线合成观测与真值查询。

与真实链路的边界约定：
- 喷射由 TCP 位姿换算到煤坐标系（同 spray_controller 的 use_tcp 路径）；
- 观测按记录的真实视点外参合成（针孔深度 + LiDAR 扫描），噪声用阶段六
  sensor_model，不读取真值进入感知；
- 真值接口（truth_heights）仅用于回合定义、奖励核对与验收，禁止喂给
  PerceptionSession 或策略状态。
"""
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from ..perception.frames import transform_points
from ..perception.sensor_model import degrade_ranges, depth_sigma, lidar_sigma
from ..spray_simulation import SpraySimulation
from ..erosion_model import SprayAction
from .spray_pose import tcp_target_world

# LiDAR 合成扫描：与记录数据的 VLP-16 类布局一致的角栅。
LIDAR_ELEVATIONS = np.linspace(np.radians(-15), np.radians(15), 16)
LIDAR_AZIMUTHS = np.linspace(0, 2 * np.pi, 1024, endpoint=False)


def pose_to_matrix(pose):
    """[x,y,z,qx,qy,qz,qw] 位姿转 4x4；与 tf 输出的平移+四元数布局一致。"""
    pose = np.asarray(pose, float)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError('pose must be [x,y,z,qx,qy,qz,qw]')
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    T[:3, 3] = pose[:3]
    return T


class SurrogateWorld:
    """单煤壁离线世界；phi 是唯一几何真值，场景缓存随版本失效。"""

    def __init__(self, mesh_path, resolution=(96, 96, 64), dt=0.2,
                 mesh_pose=(1.5, 0., 1.5, 0., -1.9198621771937625, 0.), tcp_erosion_offset=0.,
                 local_task=None, local_output='/tmp/hydromining_local_surrogate', kernel=None):
        # kernel 省略时用 DefaultErosionKernel，与重构前的数值逐位一致。
        if local_task:
            from ..local_spray_simulation import LocalSpraySimulation
            self.simulation_ = LocalSpraySimulation(mesh_path, local_task['source_coal_pose'],
                local_task['region_center_world'], local_task['wall_normal_world'], local_output,
                dt=dt, wall_budget_s=180., spacing=local_task['geometry_spacing_m'],
                patch_extent=local_task['patch_extent_m'], wall_thickness=local_task['wall_thickness_m'],
                kernel=kernel)
            self.rotation_ = self.simulation_.frame_.axes
            self.translation_ = self.simulation_.frame_.origin
        else:
            self.simulation_ = SpraySimulation(mesh_path, tuple(resolution), dt, kernel=kernel)
            pose = np.asarray(mesh_pose, float)
            if pose.shape != (6,) or not np.isfinite(pose).all():
                raise ValueError('invalid mesh pose')
            self.rotation_ = Rotation.from_euler('xyz', pose[3:]).as_matrix()
            self.translation_ = pose[:3]
        self.tcp_erosion_offset_ = tcp_erosion_offset
        self.version_ = 0
        self.scene_cache_ = None

    def reset(self):
        self.simulation_.reset()
        self.version_ += 1
        self.scene_cache_ = None

    def snapshot_phi(self):
        """当前 level-set 几何快照；restore 可回到该状态（确定性蚀除）。"""
        return self.simulation_.grid.phi.copy()

    def restore(self, phi):
        """恢复到 snapshot_phi 捕获的几何；等价于重放全部历史喷射。"""
        phi = np.asarray(phi, float)
        if phi.shape != self.simulation_.grid.phi.shape:
            raise ValueError('phi shape does not match simulation grid')
        self.simulation_.commit(phi)
        self.version_ += 1
        self.scene_cache_ = None

    def scene(self):
        """当前 phi 的射线场景；simulate 内部已使用同一提取路径。"""
        if self.scene_cache_ is None:
            g = self.simulation_.grid
            self.scene_cache_ = self.simulation_.model.surface_scene(
                g.phi, g.origin, g.spacing)
        return self.scene_cache_

    def cast(self, origins, directions):
        """world 系射线转到煤坐标系后求交；刚性变换保持 t_hit 不变。"""
        origins = np.asarray(origins, float)
        directions = np.asarray(directions, float)
        shape = origins.shape[:-1]
        local_o = (origins.reshape(-1, 3) - self.translation_) @ self.rotation_
        local_d = directions.reshape(-1, 3) @ self.rotation_
        rays = np.concatenate((local_o, local_d), axis=-1).astype(np.float32)
        hits = self.scene().cast_rays(o3d.core.Tensor(rays), nthreads=1)['t_hit'].numpy()
        return hits.reshape(shape)

    # pressure/duration 默认值属于旧 20 cm 任务；论文实验由 TaskSpec 显式传入，1.3 改为必填。
    def spray_tcp(self, tcp_world_matrix, pressure=20., duration=12., enabled=True):
        """按 TCP 位姿喷射并提交；返回 simulate 的统计与命中距离。"""
        T = np.asarray(tcp_world_matrix, float)
        if T.shape != (4, 4) or not np.isfinite(T).all():
            raise ValueError('tcp pose must be a finite 4x4 matrix')
        position = self.rotation_.T @ (T[:3, 3] - self.translation_)
        direction = self.rotation_.T @ T[:3, :3] @ np.array([0., 0., 1.])
        position -= self.tcp_erosion_offset_ * direction
        action = SprayAction(position, direction, float(pressure), float(duration),
                             int(enabled))
        phi, stats = self.simulation_.simulate(action)
        self.simulation_.commit(phi)
        self.version_ += 1
        self.scene_cache_ = None
        return stats

    # ---------- 真值侧（禁止进入感知/策略输入） ----------

    def truth_heights(self, frame, extent, resolution, ray_start=0.6):
        """沿固定初始法向的真值高度；与验收 metrics.mesh_heights 同几何。"""
        local = frame.grid(extent, resolution)
        local[..., 2] = ray_start
        starts = frame.to_world(local)
        direction = np.broadcast_to(-frame.axes[:, 2], starts.shape)
        hits = self.cast(starts, direction)
        return np.where(np.isfinite(hits), ray_start - hits, np.nan)

    # ---------- 观测合成 ----------

    def _cast_from(self, pose_matrix, local_directions):
        """把传感器系内的射线转到 world 求交；返回 (t_hit, world_dirs)。"""
        world_dirs = local_directions @ pose_matrix[:3, :3].T
        world_dirs = world_dirs / np.linalg.norm(world_dirs, axis=1, keepdims=True)
        origin = np.broadcast_to(pose_matrix[:3, 3], world_dirs.shape)
        return self.cast(origin, world_dirs), world_dirs

    def render_depth(self, pose, K, image_shape, rng=None, noise_scale=1.0,
                     dropout=0.0, visibility=np.inf):
        """轴向深度图（Gazebo 相机约定 +X 前）；未命中/丢点为 NaN。

        `visibility` 有限时按 `sensor_model.degrade_ranges` 施加环境退化
        （丢点、近距假回波、按透过率放大的噪声）；为无穷大时走原来的
        高斯噪声路径，数值与加入干扰模型之前逐位一致。
        """
        K = np.asarray(K, float)
        height, width = image_shape
        rows, cols = np.indices((height, width))
        directions = np.stack((np.ones_like(cols), -(cols - K[0, 2]) / K[0, 0],
                               -(rows - K[1, 2]) / K[1, 1]), axis=-1).reshape(-1, 3)
        hits, _ = self._cast_from(pose_to_matrix(pose), directions)
        norm = np.linalg.norm(directions, axis=1)
        depth = (hits / norm).reshape(height, width)
        depth[~np.isfinite(depth)] = np.nan
        if rng is not None and np.isfinite(visibility):
            # 退化按斜距进行：sigma 与透过率都定义在斜距上，最后再换回轴向深度。
            axial = np.where(np.isfinite(depth), depth, 0.)
            sigma = depth_sigma(axial, noise_scale) * norm.reshape(depth.shape)
            ranges, _ = degrade_ranges(axial * norm.reshape(depth.shape), sigma,
                                       visibility, rng)
            depth = ranges / norm.reshape(depth.shape)
        elif rng is not None and noise_scale > 0:
            sigma = depth_sigma(np.where(np.isfinite(depth), depth, 0.), noise_scale)
            depth = depth + rng.normal(0, sigma)
            if dropout > 0:
                depth[rng.random(depth.shape) < dropout] = np.nan
        return depth.astype(np.float32)

    def render_lidar(self, pose, rng=None, noise_scale=1.0, dropout=0.0,
                     elevations=LIDAR_ELEVATIONS, azimuths=LIDAR_AZIMUTHS,
                     visibility=np.inf):
        """VLP-16 类角栅点云；点保留在传感器系内，未命中为 NaN。

        `visibility` 有限时按 `degrade_ranges` 施加环境退化，否则保持原来的
        高斯噪声路径（逐位一致）。
        """
        el, az = np.meshgrid(elevations, azimuths, indexing='ij')
        local = np.stack((np.cos(el) * np.cos(az), np.cos(el) * np.sin(az),
                          np.sin(el)), axis=-1).reshape(-1, 3)
        hits, _ = self._cast_from(pose_to_matrix(pose), local)
        ranges = hits.copy()
        if rng is not None and np.isfinite(visibility):
            sigma = lidar_sigma(np.where(np.isfinite(ranges), ranges, 0.), noise_scale)
            ranges, _ = degrade_ranges(ranges, sigma, visibility, rng)
        elif rng is not None and noise_scale > 0:
            sigma = lidar_sigma(np.where(np.isfinite(ranges), ranges, 0.), noise_scale)
            ranges = ranges + rng.normal(0, sigma)
            if dropout > 0:
                ranges[rng.random(ranges.shape) < dropout] = np.nan
        points = local * np.where(np.isfinite(ranges), ranges, np.nan)[:, None]
        return points.astype(np.float32)

    def snapshot(self, pose_depth, pose_lidar, K, image_shape, stamp_index,
                 rng=None, noise_scale=1.0, dropout=0.0, visibility=np.inf):
        """按管线格式输出一帧快照；时间戳为合成递增计数。"""
        return {
            'depth': self.render_depth(pose_depth, K, image_shape, rng, noise_scale,
                                       dropout, visibility),
            'lidar_local': self.render_lidar(pose_lidar, rng, noise_scale, dropout,
                                             visibility=visibility),
            'K': np.asarray(K, float),
            'tf_depth': np.asarray(pose_depth, float),
            'tf_lidar': np.asarray(pose_lidar, float),
            'depth_frame': 'camera_link', 'lidar_frame': 'lidar_link',
            'depth_stamp': np.array([stamp_index, 0], dtype=np.int64),
            'lidar_stamp': np.array([stamp_index, 0], dtype=np.int64),
        }


def load_view_specs(dataset_dir, names):
    """从阶段六录制的快照中提取真实外参/内参，作为合成观测的视点规格。"""
    from pathlib import Path
    specs = {}
    for name in names:
        data = np.load(Path(dataset_dir) / f'baseline_{name}.npz')
        specs[name] = {
            'K': data['K'], 'image_shape': data['depth'].shape,
            'tf_depth': data['tf_depth'], 'tf_lidar': data['tf_lidar'],
            'lidar_shape': data['lidar_shape'],
        }
    return specs


def spray_target_tcp(frame, uv, jet_direction_world, standoff):
    """墙面局部坐标 (u,v) → TCP world 位姿；射流方向为世界向量。"""
    uv = np.asarray(uv, float)
    if uv.shape != (2,) or not np.isfinite(uv).all():
        raise ValueError('uv must be a single finite (u, v) pair')
    target = frame.to_world(np.array([uv[0], uv[1], 0.]))
    return tcp_target_world(target, jet_direction_world, standoff)
