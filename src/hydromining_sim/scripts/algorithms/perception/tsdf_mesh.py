"""有界 Open3D CPU VoxelBlockGrid 辅助重建，不依赖 RGB 亮度或网格真值。"""
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from .frames import backproject_depth, transform_points


class TSDFReconstructor:
    """辅助几何交叉校验；显式限制帧数、图像尺寸和活跃体素块。"""

    def __init__(self, voxel_size=0.005, maximum_blocks=4096, maximum_frames=60):
        if not 0.003 <= voxel_size <= 0.02 or not 1 <= maximum_blocks <= 4096:
            raise ValueError('TSDF resolution/block budget out of bounds')
        if not 1 <= maximum_frames <= 60:
            raise ValueError('TSDF frame budget out of bounds')
        self.maximum_blocks_, self.maximum_frames_ = maximum_blocks, maximum_frames
        self.frames_ = 0
        self.volume_ = o3d.t.geometry.VoxelBlockGrid(
            attr_names=('tsdf', 'weight'),
            attr_dtypes=(o3d.core.float32, o3d.core.float32),
            attr_channels=((1,), (1,)), voxel_size=voxel_size,
            block_resolution=8, block_count=maximum_blocks,
            device=o3d.core.Device('CPU:0'))

    def integrate(self, snapshot, frame, extent=0.5, maximum_height=0.25, depth=None):
        """将 Gazebo 相机 TF 转成 optical 外参，深度值仍是同一轴向距离。"""
        if self.frames_ >= self.maximum_frames_:
            raise ValueError('TSDF frame budget exceeded')
        depth = np.asarray(snapshot['depth'] if depth is None else depth, float)
        if depth.ndim != 2 or depth.size > 320 * 240:
            raise ValueError('TSDF input image exceeds acceptance resolution budget')
        points = backproject_depth(depth, snapshot['K'])
        world = transform_points(points, snapshot['tf_depth'])
        local = frame.to_local(world)
        valid = np.isfinite(local).all(axis=-1)
        valid &= (np.max(np.abs(local[..., :2]), axis=-1) < extent / 2)
        valid &= np.abs(local[..., 2]) <= maximum_height
        clipped = np.where(valid, depth, 0).astype(np.float32)
        pose = np.asarray(snapshot['tf_depth'], float)
        world_camera = np.eye(4)
        world_camera[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
        world_camera[:3, 3] = pose[:3]
        camera_optical = np.eye(4)
        camera_optical[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        extrinsic = o3d.core.Tensor(np.linalg.inv(world_camera @ camera_optical), o3d.core.float64)
        intrinsic = o3d.core.Tensor(np.asarray(snapshot['K'], float), o3d.core.float64)
        image = o3d.t.geometry.Image(o3d.core.Tensor(clipped))
        blocks = self.volume_.compute_unique_block_coordinates(
            image, intrinsic, extrinsic, depth_scale=1., depth_max=2., trunc_voxel_multiplier=4.)
        if blocks.shape[0] == 0:
            self.frames_ += 1
            return
        _, found = self.volume_.hashmap().find(blocks)
        new_blocks = int(np.count_nonzero(~found.numpy()))
        if self.volume_.hashmap().size() + new_blocks > self.maximum_blocks_:
            raise MemoryError('TSDF active block budget exceeded before integration')
        self.volume_.integrate(blocks, image, intrinsic, extrinsic,
                               depth_scale=1., depth_max=2., trunc_voxel_multiplier=4.)
        self.frames_ += 1

    def mesh(self, weight_threshold=1.0):
        """返回仅由有效深度积分得到的网格；不能把洞自动补成观测。"""
        return self.volume_.extract_triangle_mesh(weight_threshold=weight_threshold).to_legacy()
