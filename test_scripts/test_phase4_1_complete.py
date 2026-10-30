#!/usr/bin/env python3
"""
Phase 4.1 完整单元测试

测试所有四个子阶段的算法模块
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'hydromining_sim', 'scripts'))

from algorithms.voxel_grid import VoxelGrid
from algorithms.erosion_model import ErosionModel, DefaultErosionKernel, SprayAction
from algorithms.level_set_evolver import LevelSetEvolver
from algorithms.geometry_utils import GeometryUtils


def test_4_1_1_voxel_grid():
    """4.1.1 VoxelGrid 测试"""
    print("\n[TEST 4.1.1] VoxelGrid 类")

    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"
    if not os.path.exists(mesh_path):
        print("  ⚠️  SKIP: Mesh 不存在")
        return True

    grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(32, 32, 16))

    assert grid.phi.shape == (32, 32, 16)
    assert np.sum(grid.phi < 0) > 0
    assert np.sum(grid.phi > 0) > 0

    print("  ✅ PASS")
    return True


def test_4_1_2_erosion_model():
    """4.1.2 ErosionModel 测试"""
    print("\n[TEST 4.1.2] ErosionModel 类")

    kernel = DefaultErosionKernel()
    model = ErosionModel(kernel)

    # 测试核函数
    xi = np.array([0.0, 0.5, 1.0, 1.5])
    F = kernel.kernel_F(xi)

    assert F[0] == 1.0, "F(0) 应为1"
    assert F[2] == 0.0, "F(1) 应为0"
    assert F[3] == 0.0, "F(ξ≥1) 应为0"
    assert np.all(F >= 0) and np.all(F <= 1), "F 值应在[0,1]"

    # 测试参数函数
    R_c = kernel.R_c(10.0, 1.0)
    b = kernel.b(10.0, 1.0)

    assert R_c > 0
    assert b > 0

    print("  ✅ PASS")
    return True


def test_4_1_3_level_set():
    """4.1.3 LevelSetEvolver 测试"""
    print("\n[TEST 4.1.3] LevelSetEvolver 类")

    # 创建球形距离场
    nx, ny, nz = 32, 32, 32
    x = np.linspace(-1, 1, nx)
    y = np.linspace(-1, 1, ny)
    z = np.linspace(-1, 1, nz)
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')
    phi = np.sqrt(X**2 + Y**2 + Z**2) - 0.5
    phi = phi.astype(np.float32)

    evolver = LevelSetEvolver(dt=0.01)

    # 测试梯度计算
    grad_norm = evolver.compute_gradient_norm(phi)

    assert grad_norm.shape == phi.shape
    assert np.all(grad_norm >= 0)
    assert not np.isnan(grad_norm).any()

    # 测试 Level Set 演化
    v_e = np.ones_like(phi) * 0.01
    phi_new = evolver.evolve_step(phi, v_e)

    assert phi_new.shape == phi.shape
    assert not np.isnan(phi_new).any()

    # 测试CFL稳定性
    is_stable = evolver.check_cfl_stability(0.01, 2.0/32)
    assert is_stable

    print("  ✅ PASS")
    return True


def test_4_1_4_geometry_utils():
    """4.1.4 GeometryUtils 测试"""
    print("\n[TEST 4.1.4] GeometryUtils 类")

    # 创建测试距离场
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

    # 测试体素统计
    stats = GeometryUtils.count_voxels_by_sign(phi_old)

    assert stats['interior'] > 0
    assert stats['exterior'] > 0
    assert stats['total'] == nx * ny * nz

    # 测试体积计算
    volume = GeometryUtils.compute_interior_volume(phi_old, spacing)

    assert volume > 0
    assert not np.isnan(volume)

    # 测试体积去除
    volume_removed = GeometryUtils.compute_volume_removed(phi_old, phi_new, spacing)

    assert volume_removed > 0
    assert volume_removed < volume

    # 测试统计信息
    full_stats = GeometryUtils.compute_statistics(phi_old, spacing)

    assert 'interior_volume_m3' in full_stats
    assert 'interior_ratio' in full_stats
    assert 0 <= full_stats['interior_ratio'] <= 1

    print("  ✅ PASS")
    return True


def main():
    print("=" * 80)
    print("Phase 4.1 完整单元测试")
    print("=" * 80)

    try:
        test_4_1_1_voxel_grid()
        test_4_1_2_erosion_model()
        test_4_1_3_level_set()
        test_4_1_4_geometry_utils()

        print("\n" + "=" * 80)
        print("✅ Phase 4.1 所有测试通过！")
        print("核心算法实现完成")
        print("=" * 80)
        return 0

    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
        return 1
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
