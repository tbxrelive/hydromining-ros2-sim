#!/usr/bin/env python3
"""
调试冲蚀模型 - 检查中间计算值
"""

import sys
import numpy as np
sys.path.insert(0, 'src/hydromining_sim/scripts')

from algorithms.voxel_grid import VoxelGrid
from algorithms.erosion_model import ErosionModel, SprayAction, DefaultErosionKernel
from algorithms.level_set_evolver import LevelSetEvolver
from algorithms.geometry_utils import GeometryUtils

def main():
    print("=" * 80)
    print("调试冲蚀模型")
    print("=" * 80)

    # 1. 加载 Mesh 和生成 VoxelGrid
    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"
    print(f"\n[1] 加载 Mesh: {mesh_path}")
    grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(32, 32, 16))

    print(f"✅ VoxelGrid:")
    print(f"   分辨率: {grid.resolution}")
    print(f"   原点: {grid.origin}")
    print(f"   体素大小: {grid.spacing:.6f} m")
    print(f"   φ 范围: [{grid.phi.min():.6f}, {grid.phi.max():.6f}]")

    # 2. 定义喷射参数
    print(f"\n[2] 定义喷射参数")
    spray = SprayAction(
        position=np.array([0.0, 0.0, 1.0]),
        direction=np.array([0.0, 0.0, -1.0]),
        pressure=10.0,
        duration=5.0,
        g=1
    )
    print(f"✅ 喷射参数:")
    print(f"   位置: {spray.position}")
    print(f"   方向: {spray.direction}")
    print(f"   压力: {spray.pressure} MPa")
    print(f"   时长: {spray.duration} s")

    # 3. 生成体素中心点坐标
    print(f"\n[3] 生成体素中心点")
    nx, ny, nz = grid.resolution
    x = np.arange(nx) * grid.spacing + grid.origin[0]
    y = np.arange(ny) * grid.spacing + grid.origin[1]
    z = np.arange(nz) * grid.spacing + grid.origin[2]
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')
    points = np.stack([X, Y, Z], axis=-1)
    print(f"✅ 体素中心点:")
    print(f"   形状: {points.shape}")
    print(f"   X 范围: [{points[..., 0].min():.3f}, {points[..., 0].max():.3f}]")
    print(f"   Y 范围: [{points[..., 1].min():.3f}, {points[..., 1].max():.3f}]")
    print(f"   Z 范围: [{points[..., 2].min():.3f}, {points[..., 2].max():.3f}]")

    # 4. 计算冲蚀速度
    print(f"\n[4] 计算冲蚀速度")
    kernel = DefaultErosionKernel()
    model = ErosionModel(erosion_kernel=kernel)
    v_e = model.compute_erosion_velocity(
        points=points,
        spray_action=spray,
        phi=grid.phi,
        origin=grid.origin,
        spacing=grid.spacing
    )
    print(f"✅ 冲蚀速度:")
    print(f"   形状: {v_e.shape}")
    print(f"   v_e 范围: [{v_e.min():.6e}, {v_e.max():.6e}] m/s")
    print(f"   非零体素数: {np.sum(v_e > 1e-10)}")
    print(f"   平均值: {v_e.mean():.6e} m/s")

    if v_e.max() < 1e-10:
        print("⚠️ 警告：冲蚀速度几乎为 0！")

        # 检查中间变量
        print(f"\n[5] 诊断：检查中间变量")

        # 5.1 检查距离计算
        v_rel = points - spray.position
        projections = np.dot(v_rel.reshape(-1, 3), spray.direction).reshape(points.shape[:-1])
        print(f"   投影距离范围: [{projections.min():.3f}, {projections.max():.3f}] m")

        # 5.2 检查冲击角计算（需要法线）
        print(f"\n   提示：需要检查 ErosionModel 内部的中间变量")
        print(f"         建议在 erosion_model.py 中添加调试输出")

    # 5. Level Set 演化
    print(f"\n[6] Level Set 演化（50步，dt=0.5）")
    evolver = LevelSetEvolver(dt=0.5)
    phi_new = grid.phi.copy()
    for step in range(50):
        phi_new = evolver.evolve_step(phi_new, v_e)
        if (step + 1) % 10 == 0:
            change = np.abs(phi_new - grid.phi).max()
            print(f"   步数 {step+1}: φ 变化量 = {change:.6f}")
    print(f"✅ 演化后:")
    print(f"   φ_new 范围: [{phi_new.min():.6f}, {phi_new.max():.6f}]")
    print(f"   φ 总变化量: {np.abs(phi_new - grid.phi).max():.6e}")

    # 6. 体积计算
    print(f"\n[7] 体积计算")
    vol_removed = GeometryUtils.compute_volume_removed(grid.phi, phi_new, grid.spacing)
    print(f"✅ 去除体积: {vol_removed:.9f} m³")

    if vol_removed < 1e-9:
        print("❌ 问题确认：去除体积为 0")
        print("\n可能原因:")
        print("1. 冲蚀速度 v_e 全为 0 或极小")
        print("2. Level Set 演化步长 dt 或步数 n_steps 太小")
        print("3. 喷射位置/方向与煤壁没有交集")
        print("4. 煤体强度参数 K_c 过大")
    else:
        print(f"✅ 模型正常工作")

    print("\n" + "=" * 80)

if __name__ == '__main__':
    main()
