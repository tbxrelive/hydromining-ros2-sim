#!/usr/bin/env python3
"""
冲蚀模型 - Phase 4.1.2

实现用户论文的冲蚀响应公式：
  ⑥ 径向距离 r(x,t)
  ⑦ 有效作用掩码 χ(x,t)
  ⑨ 冲蚀速度 v_e(x,t)

遵循AGENTS.md第2️⃣：纯算法层，无ROS依赖
"""

import numpy as np
from typing import Callable, Tuple, Optional
from dataclasses import dataclass


@dataclass
class SprayAction:
    """喷射动作参数"""
    position: np.ndarray    # 喷嘴位置 (x, y, z) [m]
    direction: np.ndarray   # 射流方向（单位向量）
    pressure: float         # 压力 [MPa]
    duration: float         # 作用时间 [s]
    g: int = 1              # 喷射状态 0=关闭, 1=开启

    def __post_init__(self):
        """Reject invalid inputs before touching the coal state."""
        self.position = np.asarray(self.position, dtype=float)
        self.direction = np.asarray(self.direction, dtype=float)
        if self.position.shape != (3,) or self.direction.shape != (3,):
            raise ValueError('position and direction must be 3-vectors')
        if not np.all(np.isfinite(np.r_[self.position, self.direction, self.pressure, self.duration])):
            raise ValueError('Spray inputs must be finite')
        if abs(np.linalg.norm(self.direction)-1) > 1e-5:
            raise ValueError('direction must be a unit vector')
        if not 0 <= self.pressure <= 60 or not 0 < self.duration <= 60 or self.g not in (0, 1):
            raise ValueError('Require 0<=pressure<=60 MPa, 0<duration<=60 s, g in {0,1}')


class ErosionKernel:
    """
    冲蚀核函数及参数
    
    实现公式⑧和公式⑨中的参数函数
    """
    
    def __init__(
        self,
        R_c_func: Callable[[float, float], float],
        b_func: Callable[[float, float], float]
    ):
        """
        初始化冲蚀核
        
        Args:
            R_c_func: Callable(pressure, distance) -> float
                      中心冲蚀速度 [m/s]
            b_func: Callable(pressure, distance) -> float
                    有效冲蚀半径 [m]
        """
        self.R_c = R_c_func
        self.b = b_func
    
    @staticmethod
    def kernel_F(xi: np.ndarray) -> np.ndarray:
        """
        冲蚀径向核函数
        
        公式⑧: F(ξ) = (1-ξ^(3/2))^6, for 0 ≤ ξ < 1
                      = 0,              for ξ ≥ 1
        
        Args:
            xi: 归一化径向距离 r/b (形状任意)
        
        Returns:
            F(xi) ∈ [0, 1]
        """
        return np.where(xi < 1, (1 - xi**(3/2))**6, 0)


class DefaultErosionKernel(ErosionKernel):
    """
    默认临时冲蚀核参数（待标定）
    
    基于用户论文的参考数据
    """
    
    @staticmethod
    def R_c_default(pressure: float, distance: float) -> float:
        """
        中心冲蚀速度（临时参数）
        
        R_c ∝ P^α / s^β
        
        Args:
            pressure: 压力 [MPa]
            distance: 靶距 [m]
        
        Returns:
            中心冲蚀速度 [m/s]
        """
        # 临时公式（需从论文或实验标定）
        k = 1e-3      # 经验系数（已调整：1e-5 → 1e-3，增大破坏效果）
        alpha = 0.5   # 压力指数
        beta = 2.0    # 距离指数
        
        if distance < 0.01:
            distance = 0.01  # 避免除以零
        
        return k * (pressure ** alpha) / (distance ** beta)
    
    @staticmethod
    def b_default(pressure: float, distance: float) -> float:
        """
        有效冲蚀半径（临时参数）
        
        b ∝ P^γ * s^δ
        
        Args:
            pressure: 压力 [MPa]
            distance: 靶距 [m]
        
        Returns:
            有效冲蚀半径 [m]
        """
        # 临时公式（需标定）
        b_base = 0.05         # 基础半径
        pressure_factor = 0.002   # 压力系数
        distance_factor = 0.05    # 靶距系数
        
        return b_base + pressure_factor * pressure + distance_factor * distance
    
    def __init__(self):
        super().__init__(
            R_c_func=self.R_c_default,
            b_func=self.b_default
        )


class ErosionModel:
    """Surface response (9), recomputed on the current zero set each step.

    Closest-point extension carries the visible surface speed into a 3h narrow
    band. BVH first-hit tests include self occlusion and optional external mesh
    occluders in the same coordinate frame. No angular attenuation is invented.
    """

    def __init__(self, erosion_kernel):
        self.kernel = erosion_kernel
        self.last_hit_distance = None

    @staticmethod
    def surface_scene(phi, origin, spacing):
        """Build the current triangle surface, with outward-facing normals."""
        import open3d as o3d
        from .mesh_converter import MeshConverter
        vertices, faces = MeshConverter.voxel_to_mesh(phi, origin, spacing)
        mesh = o3d.t.geometry.TriangleMesh(
            o3d.core.Tensor(vertices.astype(np.float32)),
            o3d.core.Tensor(faces.astype(np.uint32)))
        scene = o3d.t.geometry.RaycastingScene(nthreads=1)
        scene.add_triangles(mesh)
        return scene

    @staticmethod
    def compute_radial_distance(points, ray_origin, ray_direction):
        """Equation (6), requiring a unit direction."""
        v = np.asarray(points) - ray_origin
        return np.linalg.norm(v - np.sum(v*ray_direction, axis=-1)[..., None]*ray_direction, axis=-1)

    @classmethod
    def find_ray_mesh_intersection(cls, ray_origin, ray_direction, phi, origin, spacing):
        """Current first hit from the positive ray; None means no erosion."""
        import open3d as o3d
        scene = cls.surface_scene(phi, origin, spacing)
        ray = np.r_[ray_origin, ray_direction].astype(np.float32)[None, :]
        hit = float(scene.cast_rays(o3d.core.Tensor(ray), nthreads=1)['t_hit'].numpy()[0])
        return hit if np.isfinite(hit) and hit > 1e-6 else None

    def compute_erosion_velocity(self, points, spray_action, phi, origin, spacing,
                                 normals=None, occluder_meshes=()):
        """Evaluate (7)--(9); geometry is current, duration is handled by integration."""
        import open3d as o3d
        action = spray_action
        velocity = np.zeros_like(phi, dtype=np.float32)
        self.last_hit_distance = None
        if action.g == 0 or action.pressure == 0:
            return velocity
        scene = self.surface_scene(phi, origin, spacing)
        ray = np.r_[action.position, action.direction].astype(np.float32)[None, :]
        hit = float(scene.cast_rays(o3d.core.Tensor(ray), nthreads=1)['t_hit'].numpy()[0])
        if not np.isfinite(hit) or hit <= 1e-6:
            return velocity
        self.last_hit_distance = hit
        rc = float(self.kernel.R_c(action.pressure, hit))
        radius = float(self.kernel.b(action.pressure, hit))
        if not np.isfinite(rc+radius) or rc < 0 or radius <= 0:
            raise ValueError('R_c must be nonnegative and b positive, both finite')
        radial = self.compute_radial_distance(points, action.position, action.direction)
        mask = (np.abs(phi) <= 3*spacing) & (radial < radius+3*spacing)
        selected = points[mask].astype(np.float32)
        if not len(selected):
            return velocity
        closest = scene.compute_closest_points(o3d.core.Tensor(selected), nthreads=1)
        surface = closest['points'].numpy()
        outward = closest['primitive_normals'].numpy()
        delta = surface-action.position
        distances = np.linalg.norm(delta, axis=1)
        directions = delta/np.maximum(distances[:, None], 1e-12)
        for vertices, faces in occluder_meshes:
            scene.add_triangles(o3d.t.geometry.TriangleMesh(
                o3d.core.Tensor(np.asarray(vertices, dtype=np.float32)),
                o3d.core.Tensor(np.asarray(faces, dtype=np.uint32))))
        rays = np.c_[np.broadcast_to(action.position, surface.shape), directions].astype(np.float32)
        first = scene.cast_rays(o3d.core.Tensor(rays), nthreads=1)['t_hit'].numpy()
        exposed = (np.abs(first-distances) < 0.05*spacing) & (np.sum(outward*directions, axis=1) < -1e-4)
        forward = delta@action.direction > 0
        r = self.compute_radial_distance(surface, action.position, action.direction)
        velocity[mask] = exposed*forward*rc*self.kernel.kernel_F(r/radius)
        return velocity
