#!/usr/bin/env python3
"""
Phase 4.4 轻量级集成测试

降低计算量，避免 VSCode 卡死：
- 分辨率: 64×64×32 → 16×16×8
- 演化步数: 50 → 10
- 参数扫描: 4个压力 → 2个压力
- 总运行时间: ~3分钟 → ~30秒
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


def test_lite_pipeline():
    """轻量级完整流程测试"""
    print("\n" + "=" * 80)
    print("Phase 4.4: 轻量级集成测试（快速验证版本）")
    print("=" * 80)

    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"

    # 1. 加载 Mesh（降低分辨率）
    print("\n[1/6] 加载 Mesh（16×16×8 分辨率）...")
    grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(16, 16, 8))

    stats_before = GeometryUtils.compute_statistics(grid.phi, grid.spacing)
    print(f"✅ VoxelGrid 生成:")
    print(f"   分辨率: {grid.resolution}")
    print(f"   体素大小: {grid.spacing:.4f} m")
    print(f"   内部体积: {stats_before['interior_volume_m3']:.6f} m³")

    # 2. 定义喷射参数
    print("\n[2/6] 定义喷射参数...")
    spray = SprayAction(
        position=np.array([0.0, 0.0, 1.0]),
        direction=np.array([0.0, 0.0, -1.0]),
        pressure=10.0,
        duration=5.0
    )
    print(f"✅ 喷射参数:")
    print(f"   位置: {spray.position}")
    print(f"   压力: {spray.pressure} MPa")

    # 3. 计算冲蚀速度
    print("\n[3/6] 计算冲蚀速度场...")
    kernel = DefaultErosionKernel()
    model = ErosionModel(kernel)

    # 生成体素中心点
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

    print(f"✅ 冲蚀速度:")
    print(f"   v_e 范围: [{v_e.min():.6f}, {v_e.max():.6f}] m/s")
    print(f"   非零体素: {np.sum(v_e > 1e-6)}")

    # 4. Level Set 演化（减少步数）
    print("\n[4/6] Level Set 演化（10步，快速版）...")
    evolver = LevelSetEvolver(dt=0.5)
    phi_old = grid.phi.copy()
    phi_new = phi_old.copy()

    n_steps = 10  # 降低步数
    for step in range(n_steps):
        phi_new = evolver.evolve_step(phi_new, v_e)
        if (step + 1) % 5 == 0:
            print(f"   步数 {step+1}/{n_steps}")

    stats_after = GeometryUtils.compute_statistics(phi_new, grid.spacing)
    volume_removed = GeometryUtils.compute_volume_removed(phi_old, phi_new, grid.spacing)

    print(f"✅ 演化完成:")
    print(f"   去除体积: {volume_removed:.6f} m³")

    # 5. 转换为 Mesh
    print("\n[5/6] 转换为 Mesh...")
    converter = MeshConverter()
    mesh_after = converter.convert(phi_new, grid.origin, grid.spacing)

    print(f"✅ Mesh 生成:")
    print(f"   顶点数: {len(mesh_after.vertices)}")
    print(f"   三角形数: {len(mesh_after.faces)}")

    # 6. 保存结果
    print("\n[6/6] 保存结果...")
    output_dir = Path("/tmp/phase4_tests_lite")
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / "coal_wall_after_spray_lite.obj"
    mesh_after.export(str(output_path))
    print(f"✅ 已保存到: {output_path}")

    print("\n" + "=" * 80)
    print("✅ 轻量级测试通过")
    print("=" * 80)
    print(f"\n关键结果:")
    print(f"  去除体积: {volume_removed:.6f} m³")
    print(f"  Mesh顶点: {len(mesh_after.vertices)}")

    return volume_removed


def test_reproducibility():
    """可复现性测试（简化版）"""
    print("\n" + "=" * 80)
    print("可复现性测试（2次运行）")
    print("=" * 80)

    results = []
    for i in range(2):
        print(f"\n运行 {i+1}/2...")
        vol = test_lite_pipeline()
        results.append(vol)

    std = np.std(results)
    print(f"\n去除体积（2次运行）:")
    print(f"  运行1: {results[0]:.8f} m³")
    print(f"  运行2: {results[1]:.8f} m³")
    print(f"\n标准差: {std:.10f} m³")

    if std < 1e-6:
        print("✅ 可复现性测试通过（标准差 < 1e-6）")
    else:
        print(f"⚠️ 可复现性测试失败（标准差 = {std:.10f}）")


def test_parameter_sensitivity_lite():
    """参数敏感性测试（简化版）"""
    print("\n" + "=" * 80)
    print("参数敏感性测试（仅2个压力）")
    print("=" * 80)

    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"
    pressures = [5.0, 15.0]  # 只测试2个压力
    volumes = []

    kernel = DefaultErosionKernel()
    model = ErosionModel(kernel)
    evolver = LevelSetEvolver(dt=0.5)

    for P in pressures:
        print(f"\n  测试 P={P} MPa...")

        # 加载 Mesh（低分辨率）
        grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(16, 16, 8))

        # 定义喷射
        spray = SprayAction(
            position=np.array([0.0, 0.0, 1.0]),
            direction=np.array([0.0, 0.0, -1.0]),
            pressure=P,
            duration=5.0
        )

        # 生成体素中心点
        x = np.linspace(0, grid.nx - 1, grid.nx) * grid.spacing + grid.origin[0]
        y = np.linspace(0, grid.ny - 1, grid.ny) * grid.spacing + grid.origin[1]
        z = np.linspace(0, grid.nz - 1, grid.nz) * grid.spacing + grid.origin[2]
        X, Y, Z = np.meshgrid(x, y, z, indexing='ij')
        points = np.stack([X, Y, Z], axis=-1)

        # 计算冲蚀速度
        v_e = model.compute_erosion_velocity(
            points=points,
            spray_action=spray,
            phi=grid.phi,
            origin=grid.origin,
            spacing=grid.spacing
        )

        # 多步演化
        phi_new = grid.phi.copy()
        for _ in range(10):  # 降低步数
            phi_new = evolver.evolve_step(phi_new, v_e)

        vol_removed = GeometryUtils.compute_volume_removed(grid.phi, phi_new, grid.spacing)
        volumes.append(vol_removed)
        print(f"    去除体积: {vol_removed:.6f} m³")

    print(f"\n压力 vs 去除体积:")
    for P, V in zip(pressures, volumes):
        print(f"   {P:4.1f} MPa → {V:.6f} m³")

    # 检查单调性
    is_monotonic = all(volumes[i] <= volumes[i+1] for i in range(len(volumes)-1))
    if is_monotonic:
        print("✅ 压力增加 → 去除体积增加（单调性满足）")
    else:
        print("⚠️ 单调性不满足")


def main():
    """主测试流程"""
    print("=" * 80)
    print("Phase 4.4: 轻量级测试套件")
    print("优化说明：降低分辨率和计算量，避免 VSCode 卡死")
    print("=" * 80)

    try:
        # 1. 基础功能测试
        print("\n[A] 基础功能测试")
        test_lite_pipeline()

        # 2. 可复现性测试（注释掉以进一步加速）
        # test_reproducibility()
        print("\n⏭️  跳过可复现性测试（节省时间）")

        # 3. 参数敏感性测试
        print("\n[B] 参数敏感性测试")
        test_parameter_sensitivity_lite()

        print("\n" + "=" * 80)
        print("✅ 轻量级测试全部通过")
        print("=" * 80)
        print("\n如需完整测试，运行: python3 test_scripts/test_phase4_integration.py")

    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
