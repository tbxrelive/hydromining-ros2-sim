"""固定初始法向的煤壁坐标系，不依赖 ROS 或真值网格。"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WallFrame:
    """列向量 axes 分别为墙面 u、v 和朝外法向，长度单位为 m。"""

    origin: np.ndarray
    axes: np.ndarray

    def __post_init__(self):
        origin, axes = np.asarray(self.origin, float), np.asarray(self.axes, float)
        if origin.shape != (3,) or axes.shape != (3, 3):
            raise ValueError('invalid wall frame dimensions')
        if (not np.isfinite(origin).all() or not np.isfinite(axes).all() or
                not np.allclose(axes.T @ axes, np.eye(3), atol=1e-8) or
                np.linalg.det(axes) < 0.999999):
            raise ValueError('wall frame must be finite, right-handed and orthonormal')
        origin, axes = origin.copy(), axes.copy()
        origin.setflags(write=False)
        axes.setflags(write=False)
        object.__setattr__(self, 'origin', origin)
        object.__setattr__(self, 'axes', axes)

    @classmethod
    def from_normal(cls, origin, normal):
        """由任务开始时的中心和朝外法向构建固定坐标系。"""
        normal = np.asarray(normal, float)
        if normal.shape != (3,) or not np.isfinite(normal).all() or np.linalg.norm(normal) < 1e-8:
            raise ValueError('normal must be a finite nonzero vector')
        normal = normal / np.linalg.norm(normal)
        tangent = np.eye(3)[np.argmin(np.abs(normal))]
        tangent = tangent - normal * (normal @ tangent)
        tangent /= np.linalg.norm(tangent)
        return cls(origin, np.column_stack((tangent, np.cross(normal, tangent), normal)))

    def to_local(self, points):
        """world 点转为 (u, v, 沿朝外法向高度)。"""
        return (np.asarray(points, float) - self.origin) @ self.axes

    def to_world(self, points):
        """墙面坐标转为 world。"""
        return np.asarray(points, float) @ self.axes.T + self.origin

    def grid(self, extent, resolution):
        """生成正方形栅格中心；不把栅格外观测夹到边界。"""
        if (not np.isfinite(extent) or not np.isfinite(resolution) or
                min(extent, resolution) <= 0):
            raise ValueError('extent and resolution must be finite and positive')
        size = int(round(extent / resolution))
        if not 1 <= size <= 1000 or not np.isclose(size * resolution, extent):
            raise ValueError('extent must be an integer number of cells, at most 1000 per axis')
        line = (np.arange(size) + 0.5) * resolution - extent / 2
        u, v = np.meshgrid(line, line)
        return np.stack((u, v, np.zeros_like(u)), axis=-1)
