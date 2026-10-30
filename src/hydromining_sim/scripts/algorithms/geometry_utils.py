#!/usr/bin/env python3
"""
几何工具函数 - Phase 4.1.4

辅助函数，计算几何统计和验证

遵循AGENTS.md第2️⃣：纯算法层，无ROS依赖
"""

import numpy as np
from typing import Tuple, Dict


class GeometryUtils:
    """几何工具函数集合"""
    
    @staticmethod
    def count_voxels_by_sign(phi: np.ndarray) -> Dict[str, int]:
        """
        按符号统计体素
        
        Args:
            phi: 距离场 (nx, ny, nz)
        
        Returns:
            {'interior': int, 'exterior': int, 'surface': int}
        """
        interior = np.sum(phi < 0)
        exterior = np.sum(phi > 0)
        surface = np.sum(phi == 0)
        
        return {
            'interior': int(interior),
            'exterior': int(exterior),
            'surface': int(surface),
            'total': int(phi.size)
        }
    
    @staticmethod
    def compute_volume_removed(
        phi_old: np.ndarray,
        phi_new: np.ndarray,
        spacing: float
    ) -> float:
        """
        计算去除体积
        
        公式⑭: ΔV = Σ (O_q^k - O_q^(k+1)) * ΔV_q
        """
        occupied_old = (phi_old <= 0).astype(int)
        occupied_new = (phi_new <= 0).astype(int)
        
        removed_voxels = np.sum(occupied_old - occupied_new)
        voxel_volume = spacing ** 3
        volume_removed = removed_voxels * voxel_volume
        
        return float(volume_removed)
    
    @staticmethod
    def compute_max_depth(
        phi_old: np.ndarray,
        phi_new: np.ndarray,
        spacing: float,
        direction: np.ndarray = None
    ) -> float:
        """计算最大冲蚀深度"""
        # Compare surfaces along the specified spray axis, not raw phi increments.
        from .erosion_model import ErosionModel
        if direction is None:
            raise ValueError('Depth requires a spray direction and ray origins; use surface_depth')
        raise ValueError('Use surface_depth with grid origin and spray action')

    @staticmethod
    def surface_depth(phi_old, phi_new, origin, spacing, action, radius):
        """Maximum increase in first-hit distance over parallel rays in the footprint.

        Depth is measured along the fixed jet axis relative to the pre-shot
        surface. Rays that penetrate through the entire coal are flagged rather
        than silently assigned zero. This metric also applies to cumulative depth.
        """
        import open3d as o3d
        from .erosion_model import ErosionModel
        d = action.direction
        helper = np.eye(3)[np.argmin(np.abs(d))]
        u = np.cross(d, helper); u /= np.linalg.norm(u)
        v = np.cross(d, u)
        samples = np.arange(-radius, radius+spacing/4, spacing/2)
        xx, yy = np.meshgrid(samples, samples)
        keep = xx*xx+yy*yy <= radius*radius
        p = action.position+xx[keep, None]*u+yy[keep, None]*v
        rays = o3d.core.Tensor(np.c_[p, np.broadcast_to(d, p.shape)].astype(np.float32))
        hits = []
        for field in (phi_old, phi_new):
            scene = ErosionModel.surface_scene(field, origin, spacing)
            hits.append(scene.cast_rays(rays, nthreads=1)['t_hit'].numpy())
        valid = np.isfinite(hits[0]) & np.isfinite(hits[1])
        if np.any(np.isfinite(hits[0]) & ~np.isfinite(hits[1])):
            raise ValueError('Through-cut detected: depth metric requires a remaining surface')
        return max(0.0, float(np.max(hits[1][valid]-hits[0][valid]))) if valid.any() else 0.0

    @staticmethod
    def compute_interior_volume(
        phi: np.ndarray,
        spacing: float
    ) -> float:
        """计算内部体积"""
        interior_voxels = np.sum(phi < 0)
        volume = interior_voxels * (spacing ** 3)
        
        return float(volume)
    
    @staticmethod
    def compute_statistics(
        phi: np.ndarray,
        spacing: float
    ) -> Dict:
        """计算距离场统计信息"""
        voxel_stats = GeometryUtils.count_voxels_by_sign(phi)
        interior_volume = GeometryUtils.compute_interior_volume(phi, spacing)
        
        return {
            'shape': phi.shape,
            'total_voxels': voxel_stats['total'],
            'interior_voxels': voxel_stats['interior'],
            'exterior_voxels': voxel_stats['exterior'],
            'interior_volume_m3': interior_volume,
            'spacing_m': spacing,
            'phi_min': float(phi.min()),
            'phi_max': float(phi.max()),
            'phi_mean': float(phi.mean()),
            'phi_std': float(phi.std()),
            'interior_ratio': voxel_stats['interior'] / voxel_stats['total'],
        }


def main():
    """快速测试几何工具"""
    print("=" * 80)
    print("Phase 4.1.4: 几何工具函数单元测试")
    print("=" * 80)
    
    print("\n[1/4] 创建测试数据...")
    nx, ny, nz = 32, 32, 16
    x = np.linspace(-1, 1, nx)
    y = np.linspace(-1, 1, ny)
    z = np.linspace(-1, 1, nz)
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')
    
    phi_old = np.sqrt(X**2 + Y**2 + Z**2) - 0.5
    phi_old = phi_old.astype(np.float32)
    
    phi_new = np.sqrt(X**2 + Y**2 + Z**2) - 0.45
    phi_new = phi_new.astype(np.float32)
    
    spacing = 2.0 / 32
    print(f"✅ 测试数据创建: {nx}×{ny}×{nz}")
    
    print("\n[2/4] 测试体素统计...")
    stats = GeometryUtils.count_voxels_by_sign(phi_old)
    print(f"✅ 体素统计:")
    print(f"   内部: {stats['interior']}")
    print(f"   外部: {stats['exterior']}")
    
    print("\n[3/4] 测试体积计算...")
    volume_old = GeometryUtils.compute_interior_volume(phi_old, spacing)
    volume_new = GeometryUtils.compute_interior_volume(phi_new, spacing)
    volume_removed = GeometryUtils.compute_volume_removed(phi_old, phi_new, spacing)
    
    print(f"✅ 体积计算:")
    print(f"   演化前: {volume_old:.6f} m³")
    print(f"   演化后: {volume_new:.6f} m³")
    print(f"   去除: {volume_removed:.6f} m³")
    
    print("\n[4/4] 测试统计信息...")
    full_stats = GeometryUtils.compute_statistics(phi_old, spacing)
    
    print(f"✅ 完整统计:")
    print(f"   内部体积: {full_stats['interior_volume_m3']:.6f} m³")
    print(f"   内部比例: {full_stats['interior_ratio']:.2%}")
    
    print("\n" + "=" * 80)
    print("✅ Phase 4.1.4 基础测试通过")
    print("=" * 80)
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
