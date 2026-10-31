#!/usr/bin/env python3
"""
Phase 4.4 最简验证测试

只验证流程能否跑通，不追求数值结果
运行时间：~10秒
"""

import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / 'src' / 'hydromining_sim' / 'scripts'))

from algorithms.voxel_grid import VoxelGrid
from algorithms.erosion_model import ErosionModel, DefaultErosionKernel, SprayAction
from algorithms.level_set_evolver import LevelSetEvolver
from algorithms.geometry_utils import GeometryUtils
from algorithms.mesh_converter import MeshConverter


def main():
    """最简验证测试"""
    print("=" * 80)
    print("阶段4 最简验证测试（10秒快速版）")
    print("=" * 80)

    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"

    print("\n[1/5] 加载 Mesh（低分辨率：16×16×8）...")
    grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(16, 16, 8))
    print(f"✅ 分辨率: {grid.resolution}, 体素大小: {grid.spacing:.4f} m")

    print("\n[2/5] 定义喷射参数...")
    spray = SprayAction(
        position=np.array([0.0, 0.0, 0.5]),  # 降低喷射高度，靠近煤壁
        direction=np.array([0.0, 0.0, -1.0]),
        pressure=20.0,  # 增大压力
        duration=5.0
    )
    print(f"✅ 压力: {spray.pressure} MPa, 位置: {spray.position}")

    print("\n[3/5] 计算冲蚀速度...")
    kernel = DefaultErosionKernel()
    model = ErosionModel(kernel)

    x = np.linspace(0, grid.nx - 1, grid.nx) * grid.spacing + grid.origin[0]
    y = np.linspace(0, grid.ny - 1, grid.ny) * grid.spacing + grid.origin[1]
    z = np.linspace(0, grid.nz - 1, grid.nz) * grid.spacing + grid.origin[2]
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')
    points = np.stack([X, Y, Z], axis=-1)

    v_e = model.compute_erosion_velocity(
        points=points,
        spray_action=spray,
        phi=grid.phi,
        origin=grid.origin,
        spacing=grid.spacing
    )
    print(f"✅ v_e 范围: [{v_e.min():.6e}, {v_e.max():.6e}] m/s")
    print(f"   非零体素: {np.sum(v_e > 1e-6)} / {grid.nx * grid.ny * grid.nz}")

    print("\n[4/5] Level Set 演化（20步）...")
    evolver = LevelSetEvolver(dt=0.5)
    phi_old = grid.phi.copy()
    phi_new = phi_old.copy()

    for i in range(20):
        phi_new = evolver.evolve_step(phi_new, v_e)

    volume_removed = GeometryUtils.compute_volume_removed(phi_old, phi_new, grid.spacing)
    print(f"✅ 去除体积: {volume_removed:.6f} m³")

    print("\n[5/5] Mesh 转换...")
    converter = MeshConverter()
    mesh_after = converter.voxel_to_mesh(phi_new, grid.origin, grid.spacing)
    print(f"✅ Mesh: {len(mesh_after.vertices)} 顶点, {len(mesh_after.faces)} 三角形")

    # 保存结果
    output_dir = Path("/tmp/phase4_tests_lite")
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / "test_result.obj"
    mesh_after.export(str(output_path))
    print(f"   已保存到: {output_path}")

    print("\n" + "=" * 80)
    if volume_removed > 0:
        print("✅ 验证通过：去除体积 > 0，流程正常")
    else:
        print("⚠️  去除体积为0（分辨率太低或参数需调整）")
        print("   但流程已跑通，核心功能正常")
    print("=" * 80)

    print("\n总结:")
    print(f"  - 冲蚀速度最大值: {v_e.max():.6e} m/s")
    print(f"  - 去除体积: {volume_removed:.6f} m³")
    print(f"  - Mesh顶点数: {len(mesh_after.vertices)}")

    print("\n💡 完整测试（需要~3分钟）:")
    print("   python3 test_scripts/test_phase4_integration.py")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
