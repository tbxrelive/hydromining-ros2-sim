#!/usr/bin/env python3
"""
Phase 4.0 单元测试：距离场初始化验证

测试 VoxelGrid.from_mesh_simple() 的功能
验证符号距离场的正确性
"""

import sys
import os
import numpy as np

# 添加算法模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'hydromining_sim', 'scripts'))

from algorithms.voxel_grid import VoxelGrid


def test_voxel_grid_creation():
    """测试 VoxelGrid 创建"""
    print("\n[TEST 1] VoxelGrid 对象创建...")
    grid = VoxelGrid(64, 64, 32, np.array([0, 0, 0]), 0.1)
    assert grid.nx == 64
    assert grid.ny == 64
    assert grid.nz == 32
    assert grid.phi.shape == (64, 64, 32)
    print("✅ PASS")


def test_mesh_loading():
    """测试 Mesh 加载"""
    print("\n[TEST 2] Mesh 文件加载...")
    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"

    if not os.path.exists(mesh_path):
        print(f"⚠️  SKIP: Mesh 文件不存在")
        return

    import trimesh
    mesh = trimesh.load(mesh_path, process=False)
    assert isinstance(mesh, trimesh.Trimesh)
    assert len(mesh.vertices) > 0
    assert len(mesh.faces) > 0
    print(f"✅ PASS: {len(mesh.vertices)} 顶点, {len(mesh.faces)} 面")


def test_distance_field_generation():
    """测试距离场生成"""
    print("\n[TEST 3] 距离场生成...")
    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"

    if not os.path.exists(mesh_path):
        print(f"⚠️  SKIP: Mesh 文件不存在")
        return

    grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(32, 32, 16))

    # 验证距离场生成
    assert grid.phi.shape == (32, 32, 16)
    assert grid.phi.dtype == np.float32
    assert not np.isnan(grid.phi).any()
    assert not np.isinf(grid.phi).any()
    print(f"✅ PASS: 距离场有效")


def test_distance_field_properties():
    """测试距离场特性"""
    print("\n[TEST 4] 距离场特性...")
    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"

    if not os.path.exists(mesh_path):
        print(f"⚠️  SKIP: Mesh 文件不存在")
        return

    grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(32, 32, 16))

    # 检查是否有内部和外部体素
    interior = np.sum(grid.phi < 0)
    exterior = np.sum(grid.phi > 0)
    total = 32 * 32 * 16

    print(f"  内部体素: {interior} ({interior/total*100:.1f}%)")
    print(f"  外部体素: {exterior} ({exterior/total*100:.1f}%)")
    print(f"  距离范围: [{grid.phi.min():.6f}, {grid.phi.max():.6f}]")

    assert interior > 0, "应有内部体素"
    assert exterior > 0, "应有外部体素"
    assert grid.phi.min() < 0, "应有负距离"
    assert grid.phi.max() > 0, "应有正距离"

    print("✅ PASS")


def test_grid_info():
    """测试网格信息方法"""
    print("\n[TEST 5] 网格信息方法...")
    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"

    if not os.path.exists(mesh_path):
        print(f"⚠️  SKIP: Mesh 文件不存在")
        return

    grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(32, 32, 16))
    info = grid.info()

    assert 'resolution' in info
    assert 'total_voxels' in info
    assert 'interior_voxels' in info
    assert 'interior_volume_m3' in info
    assert 'phi_min' in info
    assert 'phi_max' in info

    print(f"  分辨率: {info['resolution']}")
    print(f"  总体素数: {info['total_voxels']:,}")
    print(f"  内部体积: {info['interior_volume_m3']:.6f} m³")
    print("✅ PASS")


def main():
    print("=" * 80)
    print("Phase 4.0 单元测试：距离场初始化")
    print("=" * 80)

    try:
        test_voxel_grid_creation()
        test_mesh_loading()
        test_distance_field_generation()
        test_distance_field_properties()
        test_grid_info()

        print("\n" + "=" * 80)
        print("✅ 所有测试通过！Phase 4.0 验证成功")
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
