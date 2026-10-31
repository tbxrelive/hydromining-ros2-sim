#!/usr/bin/env python3
"""
阶段4：喷射效果模拟 - 完整集成测试

按照鱼香ROS规范 + AGENTS.md 3.6️⃣ 规范编写
- 无 GUI 依赖，完全在终端环境运行
- 使用已验证的算法模块（VoxelGrid, ErosionModel, LevelSetEvolver, Marching Cubes）
- 验证完整端到端流程
- Linux ROS2 Jazzy 环境

运行方法：
    cd ~/SCI/hydromining_ws
    source .venv_stage4/bin/activate
    python3 test_scripts/phase4_complete_test.py
"""

import sys
from pathlib import Path
import numpy as np
import json
from datetime import datetime

# 添加算法模块路径
sys.path.insert(0, str(Path(__file__).parent.parent / 'src' / 'hydromining_sim' / 'scripts'))

from algorithms.voxel_grid import VoxelGrid
from algorithms.erosion_model import ErosionModel, DefaultErosionKernel, SprayAction
from algorithms.level_set_evolver import LevelSetEvolver
from algorithms.geometry_utils import GeometryUtils
from algorithms.mesh_converter import MeshConverter


class Phase4CompleteTest:
    """
    阶段4完整集成测试

    验证内容：
    1. 核心算法模块正常工作
    2. 完整端到端流程（Mesh→SDF→冲蚀→演化→重建）
    3. 结果可复现性
    4. 参数敏感性
    """

    def __init__(self):
        self.output_dir_ = Path('/tmp/phase4_linux_test')
        self.output_dir_.mkdir(parents=True, exist_ok=True)
        self.mesh_path_ = Path.home() / 'SCI/hydromining_ws/src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj'
        self.results_ = {}

    def log(self, level, msg):
        """日志输出（鱼香ROS规范）"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        prefix = {
            'INFO': '✅',
            'WARN': '⚠️',
            'ERROR': '❌',
        }.get(level, '📋')
        print(f"[{timestamp}] {prefix} {msg}")

    def test_phase4_1_core_algorithms(self):
        """测试1：核心算法模块"""
        self.log('INFO', '=' * 80)
        self.log('INFO', 'Phase 4.1: 核心算法模块测试')
        self.log('INFO', '=' * 80)

        try:
            # 1. VoxelGrid
            self.log('INFO', '[1/4] VoxelGrid 类...')
            grid = VoxelGrid.from_mesh_simple(str(self.mesh_path_), resolution=(32, 32, 16))
            self.log('INFO', f'      ✓ 分辨率: {grid.resolution}, 体素大小: {grid.spacing:.4f} m')

            # 2. ErosionModel
            self.log('INFO', '[2/4] ErosionModel 类...')
            kernel = DefaultErosionKernel()
            model = ErosionModel(kernel)
            self.log('INFO', '      ✓ 冲蚀核初始化成功')

            # 3. LevelSetEvolver
            self.log('INFO', '[3/4] LevelSetEvolver 类...')
            evolver = LevelSetEvolver(dt=0.5)
            self.log('INFO', '      ✓ Level Set 演化器初始化成功')

            # 4. GeometryUtils + MeshConverter
            self.log('INFO', '[4/4] GeometryUtils + MeshConverter...')
            converter = MeshConverter()
            self.log('INFO', '      ✓ 几何工具初始化成功')

            self.results_['test_1'] = True
            self.log('INFO', '✅ Phase 4.1 通过\n')
            return True

        except Exception as e:
            self.log('ERROR', f'Phase 4.1 失败: {e}')
            self.results_['test_1'] = False
            return False

    def test_phase4_2_end_to_end(self):
        """测试2：完整端到端流程"""
        self.log('INFO', '=' * 80)
        self.log('INFO', 'Phase 4.2: 完整端到端流程测试')
        self.log('INFO', '=' * 80)

        try:
            # 加载模型
            self.log('INFO', '[1/5] 加载 Mesh...')
            grid = VoxelGrid.from_mesh_simple(str(self.mesh_path_), resolution=(32, 32, 16))
            self.log('INFO', f'      ✓ 加载完成: {grid.resolution}')

            # 定义喷射参数
            self.log('INFO', '[2/5] 定义喷射参数...')
            spray = SprayAction(
                position=np.array([0.0, 0.0, 1.0]),
                direction=np.array([0.0, 0.0, -1.0]),
                pressure=10.0,
                duration=5.0
            )
            self.log('INFO', f'      ✓ 压力: {spray.pressure} MPa, 时长: {spray.duration} s')

            # 计算冲蚀速度
            self.log('INFO', '[3/5] 计算冲蚀速度...')
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
            self.log('INFO', f'      ✓ v_e 范围: [{v_e.min():.6e}, {v_e.max():.6e}] m/s')

            # Level Set 演化
            self.log('INFO', '[4/5] Level Set 演化 (50步)...')
            evolver = LevelSetEvolver(dt=0.5)
            phi_new = grid.phi.copy()
            for i in range(50):
                phi_new = evolver.evolve_step(phi_new, v_e)
                if (i + 1) % 10 == 0:
                    self.log('INFO', f'      步数: {i+1}/50')

            # 计算体积变化
            volume_removed = GeometryUtils.compute_volume_removed(grid.phi, phi_new, grid.spacing)
            self.log('INFO', f'      ✓ 去除体积: {volume_removed:.6f} m³')

            # Mesh 转换
            self.log('INFO', '[5/5] Mesh 转换...')
            converter = MeshConverter()
            vertices, faces = converter.voxel_to_mesh(phi_new, grid.origin, grid.spacing)
            self.log('INFO', f'      ✓ Mesh: {len(vertices)} 顶点, {len(faces)} 三角形')

            # 保存 Mesh
            output_path = self.output_dir_ / 'coal_wall_spray_final.obj'
            with open(output_path, 'w') as f:
                for v in vertices:
                    f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
                for face in faces:
                    f.write(f"f {int(face[0])+1} {int(face[1])+1} {int(face[2])+1}\n")
            self.log('INFO', f'      ✓ 已保存: {output_path}')

            self.results_['test_2'] = {
                'volume_removed': float(volume_removed),
                'mesh_vertices': len(vertices),
                'mesh_faces': len(faces),
            }

            self.log('INFO', '✅ Phase 4.2 通过\n')
            return True

        except Exception as e:
            self.log('ERROR', f'Phase 4.2 失败: {e}')
            import traceback
            traceback.print_exc()
            self.results_['test_2'] = False
            return False

    def test_phase4_3_reproducibility(self):
        """测试3：可复现性"""
        self.log('INFO', '=' * 80)
        self.log('INFO', 'Phase 4.3: 可复现性测试')
        self.log('INFO', '=' * 80)

        try:
            volumes = []

            for run in range(2):
                self.log('INFO', f'[运行 {run+1}/2]...')

                grid = VoxelGrid.from_mesh_simple(str(self.mesh_path_), resolution=(32, 32, 16))

                spray = SprayAction(
                    position=np.array([0.0, 0.0, 1.0]),
                    direction=np.array([0.0, 0.0, -1.0]),
                    pressure=10.0,
                    duration=5.0
                )

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

                evolver = LevelSetEvolver(dt=0.5)
                phi_new = grid.phi.copy()
                for _ in range(50):
                    phi_new = evolver.evolve_step(phi_new, v_e)

                volume = GeometryUtils.compute_volume_removed(grid.phi, phi_new, grid.spacing)
                volumes.append(volume)
                self.log('INFO', f'      去除体积: {volume:.6f} m³')

            # 检查可复现性
            std = np.std(volumes)
            self.log('INFO', f'标准差: {std:.10f} m³')

            if std < 1e-6:
                self.log('INFO', '✅ 可复现性测试通过\n')
                self.results_['test_3'] = {
                    'volumes': volumes,
                    'std': float(std),
                    'passed': True,
                }
                return True
            else:
                self.log('WARN', f'⚠️ 标准差过大: {std}\n')
                self.results_['test_3'] = {
                    'volumes': volumes,
                    'std': float(std),
                    'passed': False,
                }
                return False

        except Exception as e:
            self.log('ERROR', f'Phase 4.3 失败: {e}')
            self.results_['test_3'] = False
            return False

    def test_phase4_4_sensitivity(self):
        """测试4：参数敏感性（简化版）"""
        self.log('INFO', '=' * 80)
        self.log('INFO', 'Phase 4.4: 参数敏感性测试')
        self.log('INFO', '=' * 80)

        try:
            pressures = [5.0, 15.0]
            results = []

            for p in pressures:
                self.log('INFO', f'[压力 {p} MPa]...')

                grid = VoxelGrid.from_mesh_simple(str(self.mesh_path_), resolution=(32, 32, 16))

                spray = SprayAction(
                    position=np.array([0.0, 0.0, 1.0]),
                    direction=np.array([0.0, 0.0, -1.0]),
                    pressure=p,
                    duration=5.0
                )

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

                evolver = LevelSetEvolver(dt=0.5)
                phi_new = grid.phi.copy()
                for _ in range(50):
                    phi_new = evolver.evolve_step(phi_new, v_e)

                volume = GeometryUtils.compute_volume_removed(grid.phi, phi_new, grid.spacing)
                results.append((p, volume))
                self.log('INFO', f'      去除体积: {volume:.6f} m³')

            # 检查单调性
            is_monotonic = results[0][1] <= results[1][1]
            if is_monotonic:
                self.log('INFO', '✅ 参数敏感性测试通过（单调性满足）\n')
                self.results_['test_4'] = {
                    'results': results,
                    'monotonic': True,
                }
                return True
            else:
                self.log('WARN', '⚠️ 单调性不满足\n')
                self.results_['test_4'] = {
                    'results': results,
                    'monotonic': False,
                }
                return False

        except Exception as e:
            self.log('ERROR', f'Phase 4.4 失败: {e}')
            self.results_['test_4'] = False
            return False

    def run_all_tests(self):
        """运行所有测试"""
        self.log('INFO', '\n' + '='*80)
        self.log('INFO', '🧪 阶段4完整集成测试（Linux ROS2 环境）')
        self.log('INFO', '='*80 + '\n')

        results = [
            self.test_phase4_1_core_algorithms(),
            self.test_phase4_2_end_to_end(),
            self.test_phase4_3_reproducibility(),
            self.test_phase4_4_sensitivity(),
        ]

        # 总结
        self.log('INFO', '='*80)
        self.log('INFO', '📊 测试总结')
        self.log('INFO', '='*80)

        passed = sum(1 for r in results if r)
        total = len(results)

        self.log('INFO', f'通过: {passed}/{total}')

        if passed == total:
            self.log('INFO', '✅ 所有测试通过')
        else:
            self.log('WARN', f'⚠️ 有 {total - passed} 个测试失败')

        # 保存结果
        result_file = self.output_dir_ / 'test_results.json'
        with open(result_file, 'w') as f:
            json.dump(self.results_, f, indent=2, default=str)

        self.log('INFO', f'结果已保存: {result_file}\n')

        return passed == total


def main():
    """主函数"""
    tester = Phase4CompleteTest()
    success = tester.run_all_tests()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
