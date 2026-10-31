#!/usr/bin/env python3
"""
几何转换器 - Phase 4.2

功能：
1. 将体素网格（SDF）转换为 Mesh（Marching Cubes）
2. 导出为 .obj 文件
3. 可视化对比

遵循AGENTS.md第2️⃣：纯算法层，无ROS依赖
"""

import numpy as np
import mcubes
import trimesh
from typing import Optional, Tuple
from pathlib import Path


class MeshConverter:
    """
    体素网格 → Mesh 转换器

    使用 Marching Cubes 算法
    """

    @staticmethod
    def voxel_to_mesh(
        phi: np.ndarray,
        origin: np.ndarray,
        spacing: float,
        iso_level: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        将体素距离场转换为三角形 Mesh

        使用 Marching Cubes 算法提取等值面

        Args:
            phi: 符号距离场 (nx, ny, nz)
            origin: 网格原点 (3,)
            spacing: 体素大小 [m]
            iso_level: 等值面值（0 = 煤壁表面）

        Returns:
            (vertices, faces)
            vertices: 顶点坐标 (N, 3)
            faces: 三角形索引 (M, 3)
        """
        # 1. 使用 Marching Cubes 提取等值面
        vertices, triangles = mcubes.marching_cubes(phi, iso_level)

        # 2. 将体素坐标转换为世界坐标
        vertices_world = origin + vertices * spacing

        # Exact zeros at grid nodes produce coincident vertices/zero-area faces.
        # Weld only numerical coincidences and drop degeneracies before using BVH.
        mesh = trimesh.Trimesh(vertices_world, triangles[:, ::-1], process=False)
        mesh.merge_vertices(digits_vertex=9)
        mesh.update_faces(mesh.nondegenerate_faces(height=1e-10))
        mesh.update_faces(mesh.unique_faces())
        mesh.remove_unreferenced_vertices()
        return np.asarray(mesh.vertices), np.asarray(mesh.faces)

    @staticmethod
    def save_mesh(
        vertices: np.ndarray,
        faces: np.ndarray,
        output_path: str,
        normals: Optional[np.ndarray] = None
    ) -> bool:
        """
        保存 Mesh 为 .obj 文件

        Args:
            vertices: 顶点坐标 (N, 3)
            faces: 三角形索引 (M, 3)
            output_path: 输出路径
            normals: 顶点法线（可选）

        Returns:
            是否成功
        """
        try:
            # 使用 trimesh 保存
            mesh = trimesh.Trimesh(
                vertices=vertices,
                faces=faces,
                vertex_normals=normals,
                process=False  # 不自动修复
            )

            # Harmonic/DART mesh collision requires normals for every vertex.
            mesh.export(output_path, include_normals=True)

            return True

        except Exception as e:
            print(f"❌ 保存 Mesh 失败: {e}")
            return False

    @staticmethod
    def compute_mesh_info(
        vertices: np.ndarray,
        faces: np.ndarray
    ) -> dict:
        """
        计算 Mesh 统计信息

        Args:
            vertices: 顶点坐标
            faces: 三角形索引

        Returns:
            统计字典
        """
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)

        return {
            'num_vertices': len(vertices),
            'num_faces': len(faces),
            'volume_m3': float(mesh.volume) if mesh.is_watertight else None,
            'is_watertight': mesh.is_watertight,
            'bounds_min': vertices.min(axis=0).tolist(),
            'bounds_max': vertices.max(axis=0).tolist(),
            'centroid': vertices.mean(axis=0).tolist(),
        }


class MeshComparator:
    """
    Mesh 对比工具

    用于对比喷射前后的煤壁几何
    """

    @staticmethod
    def compare_meshes(
        mesh_before: trimesh.Trimesh,
        mesh_after: trimesh.Trimesh
    ) -> dict:
        """
        对比两个 Mesh

        Args:
            mesh_before: 喷射前的 Mesh
            mesh_after: 喷射后的 Mesh

        Returns:
            对比结果字典
        """
        result = {
            'vertices_before': len(mesh_before.vertices),
            'vertices_after': len(mesh_after.vertices),
            'faces_before': len(mesh_before.faces),
            'faces_after': len(mesh_after.faces),
        }

        # 体积对比（如果闭合）
        if mesh_before.is_watertight and mesh_after.is_watertight:
            vol_before = mesh_before.volume
            vol_after = mesh_after.volume
            vol_removed = vol_before - vol_after

            result['volume_before_m3'] = float(vol_before)
            result['volume_after_m3'] = float(vol_after)
            result['volume_removed_m3'] = float(vol_removed)
            result['volume_removed_percent'] = float(vol_removed / vol_before * 100)
        else:
            result['volume_warning'] = 'Mesh 不闭合，无法计算体积'

        return result

    @staticmethod
    def visualize_comparison(
        mesh_before_path: str,
        mesh_after_path: str,
        output_html: Optional[str] = None
    ):
        """
        可视化对比（生成HTML）

        Args:
            mesh_before_path: 喷射前Mesh路径
            mesh_after_path: 喷射后Mesh路径
            output_html: 输出HTML路径（可选）
        """
        # 加载 Mesh
        mesh_before = trimesh.load(mesh_before_path)
        mesh_after = trimesh.load(mesh_after_path)

        # 设置不同颜色
        mesh_before.visual.vertex_colors = [200, 200, 200, 150]  # 灰色半透明
        mesh_after.visual.vertex_colors = [100, 150, 255, 200]   # 蓝色

        # 合并显示
        scene = trimesh.Scene([mesh_before, mesh_after])

        if output_html:
            # 保存为HTML
            html = scene.save_image(resolution=[800, 600], visible=True)
            with open(output_html, 'w') as f:
                f.write(str(html))
            print(f"✅ 可视化保存到: {output_html}")
        else:
            # 直接显示（需要GUI）
            scene.show()


def main():
    """Phase 4.2 单元测试"""
    print("=" * 80)
    print("Phase 4.2: 几何转换单元测试")
    print("=" * 80)

    # 创建简单测试距离场（球形）
    print("\n[1/4] 创建测试距离场...")
    nx, ny, nz = 32, 32, 32
    x = np.linspace(-1, 1, nx)
    y = np.linspace(-1, 1, ny)
    z = np.linspace(-1, 1, nz)
    X, Y, Z = np.meshgrid(x, y, z, indexing='ij')

    phi = np.sqrt(X**2 + Y**2 + Z**2) - 0.5
    phi = phi.astype(np.float32)

    origin = np.array([-1.0, -1.0, -1.0])
    spacing = 2.0 / 32

    print(f"✅ 距离场: {phi.shape}, spacing={spacing:.4f}m")

    # 测试 Marching Cubes
    print("\n[2/4] 测试 Marching Cubes...")
    converter = MeshConverter()
    vertices, faces = converter.voxel_to_mesh(phi, origin, spacing)

    print(f"✅ Mesh 生成:")
    print(f"   顶点数: {len(vertices)}")
    print(f"   三角形数: {len(faces)}")

    assert len(vertices) > 0, "应生成顶点"
    assert len(faces) > 0, "应生成三角形"

    # 测试保存
    print("\n[3/4] 测试保存 Mesh...")
    output_path = "/tmp/test_sphere.obj"
    success = converter.save_mesh(vertices, faces, output_path)

    assert success, "保存应成功"
    assert Path(output_path).exists(), "文件应存在"

    print(f"✅ Mesh 已保存到: {output_path}")

    # 测试统计信息
    print("\n[4/4] 测试 Mesh 统计...")
    info = converter.compute_mesh_info(vertices, faces)

    print(f"✅ Mesh 信息:")
    print(f"   顶点数: {info['num_vertices']}")
    print(f"   三角形数: {info['num_faces']}")
    print(f"   是否闭合: {info['is_watertight']}")
    if info['volume_m3']:
        print(f"   体积: {info['volume_m3']:.6f} m³")

    print("\n" + "=" * 80)
    print("✅ Phase 4.2 基础测试通过")
    print("=" * 80)

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
