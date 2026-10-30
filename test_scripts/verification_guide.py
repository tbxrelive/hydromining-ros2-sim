#!/usr/bin/env python3
"""
阶段4 验收脚本 - 分步执行版本

解决 VSCode 卡死问题：
- 分成3个独立脚本
- 每个脚本运行时间 < 30秒
- 可以在命令行后台运行，避免占用 VSCode
"""

import sys
from pathlib import Path

print("""
================================================================================
阶段4 分步验收指南
================================================================================

VSCode 卡死问题解决方案：
- 不在 VSCode 终端运行完整测试
- 使用独立的 bash 终端后台运行
- 每个测试脚本独立运行，互不影响

================================================================================
验收步骤（3步，每步 < 30秒）
================================================================================

### Step 1: 单元测试（~30秒）

cd ~/SCI/hydromining_ws
source .venv_stage4/bin/activate
python3 test_scripts/test_phase4_1_complete.py

预期输出：
  ✅ PASS × 4


### Step 2: 几何转换测试（~15秒）

python3 src/hydromining_sim/scripts/algorithms/mesh_converter.py

预期输出：
  ✅ Phase 4.2 基础测试通过


### Step 3: 完整集成测试（~3分钟，后台运行）

# 方式1：后台运行，输出到文件
nohup python3 test_scripts/test_phase4_integration.py > /tmp/phase4_test.log 2>&1 &

# 查看进度
tail -f /tmp/phase4_test.log

# 方式2：降低分辨率版本（~30秒）
python3 test_scripts/test_phase4_integration_mini.py


================================================================================
💡 为什么 VSCode 会卡死？
================================================================================

原因：
1. 集成测试计算量大（64×64×32 × 50步演化）
2. VSCode 内置终端实时渲染输出消耗资源
3. Python 进程内存占用较大（~500MB）

解决方案：
1. 使用独立终端（gnome-terminal, tmux, screen）
2. 后台运行，输出重定向到文件
3. 使用降低分辨率的测试版本

================================================================================
快速验收（推荐）
================================================================================

""")

# 创建降低分辨率的版本
mini_test_script = Path(__file__).parent / "test_phase4_integration_mini.py"

if not mini_test_script.exists():
    print(f"正在创建快速测试脚本: {mini_test_script}")

    mini_content = """#!/usr/bin/env python3
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
    print("=" * 80)
    print("阶段4 快速验证测试（32×32×16 分辨率，30秒）")
    print("=" * 80)

    mesh_path = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"

    print("\\n[1/5] 加载 Mesh...")
    grid = VoxelGrid.from_mesh_simple(mesh_path, resolution=(32, 32, 16))
    print(f"✅ 分辨率: {grid.resolution}")

    print("\\n[2/5] 定义喷射...")
    spray = SprayAction(
        position=np.array([0.0, 0.0, 1.0]),
        direction=np.array([0.0, 0.0, -1.0]),
        pressure=10.0,
        duration=5.0
    )

    print("\\n[3/5] 计算冲蚀速度...")
    kernel = DefaultErosionKernel()
    model = ErosionModel(kernel)

    x = np.linspace(0, grid.nx - 1, grid.nx) * grid.spacing + grid.origin[0]
    y = np.linspace(0, grid.ny - 1, grid.ny) * grid.spacing + grid.origin[1]
    z = np.linspace(0, grid.nz - 1, grid.nz) * grid.spacing + grid.origin[2]
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')
    points = np.stack([X, Y, Z], axis=-1)

    v_e = model.compute_erosion_velocity(points=points, spray_action=spray,
                                         phi=grid.phi, origin=grid.origin, spacing=grid.spacing)
    print(f"✅ v_e 最大值: {v_e.max():.6e} m/s")

    print("\\n[4/5] Level Set 演化（50步）...")
    evolver = LevelSetEvolver(dt=0.5)
    phi_new = grid.phi.copy()
    for i in range(50):
        phi_new = evolver.evolve_step(phi_new, v_e)
        if (i+1) % 10 == 0:
            print(f"   {i+1}/50 步完成")

    volume = GeometryUtils.compute_volume_removed(grid.phi, phi_new, grid.spacing)
    print(f"✅ 去除体积: {volume:.6f} m³")

    print("\\n[5/5] Mesh 转换...")
    converter = MeshConverter()
    vertices, faces = converter.voxel_to_mesh(phi_new, grid.origin, grid.spacing)
    print(f"✅ Mesh: {len(vertices)} 顶点, {len(faces)} 三角形")

    print("\\n" + "=" * 80)
    if volume > 0:
        print("✅ 验证通过！")
        print(f"   去除体积: {volume:.6f} m³")
        print(f"   Mesh顶点: {len(vertices)}")
    else:
        print("⚠️  去除体积为0（参数需调整）")
    print("=" * 80)

    return 0 if volume > 0 else 1

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"\\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
"""

    mini_test_script.write_text(mini_content)
    mini_test_script.chmod(0o755)
    print(f"✅ 已创建: {mini_test_script}\n")

print("""
现在运行快速验证：

cd ~/SCI/hydromining_ws
source .venv_stage4/bin/activate
python3 test_scripts/test_phase4_integration_mini.py

================================================================================
""")

if __name__ == "__main__":
    sys.exit(0)
