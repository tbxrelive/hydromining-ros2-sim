#!/usr/bin/env python3
"""
煤壁几何生成工具
使用 Perlin Noise 生成具有真实凹凸的倾斜煤壁网格
"""

import numpy as np
import trimesh
from perlin_numpy import generate_perlin_noise_3d
import argparse
import os


class CoalWallGenerator:
    """
    煤壁几何生成器
    """

    def __init__(
        self,
        width: float = 3.0,          # 煤壁宽度 (m)
        height: float = 3.0,         # 煤壁高度 (m)
        thickness: float = 0.4,      # 煤壁厚度 (m)
        tilt_angle: float = 20.0,    # 倾斜角度 (度)
        roughness: float = 0.15,     # 表面粗糙度 (m)
        resolution: int = 100,       # 网格分辨率
        seed: int = 42               # 随机种子
    ):
        self.width = width
        self.height = height
        self.thickness = thickness
        self.tilt_angle = np.radians(tilt_angle)
        self.roughness = roughness
        self.resolution = resolution
        self.seed = seed

        np.random.seed(seed)

    def generate_perlin_surface(self, nx: int, ny: int) -> np.ndarray:
        """
        生成 Perlin Noise 高度场

        返回 (ny, nx) 的高度图
        """
        # 使用 3D Perlin Noise 的一个切片来获得更好的连续性
        # 生成多个频率叠加（Fractal Noise）
        height_map = np.zeros((ny, nx))

        # 多个八度（octaves）叠加
        octaves = [
            (1, 1.0),     # 低频大幅度（整体起伏）
            (2, 0.5),     # 中频中等幅度
            (4, 0.25),    # 高频小幅度（细节）
        ]

        for octave, amplitude in octaves:
            # 生成该频率的噪声
            noise_shape = (1, ny * octave, nx * octave)
            noise_3d = generate_perlin_noise_3d(
                noise_shape,
                (1, 4, 4),  # res 参数控制噪声频率
                tileable=(False, False, False)
            )

            # 取一个切片并降采样到目标分辨率
            noise_slice = noise_3d[0, :, :]

            # 双线性插值到目标分辨率
            from scipy.ndimage import zoom
            noise_resized = zoom(noise_slice, (ny / noise_slice.shape[0], nx / noise_slice.shape[1]), order=1)

            height_map += amplitude * noise_resized

        # 归一化到 [-1, 1]
        height_map = (height_map - height_map.mean()) / (height_map.std() + 1e-8)
        height_map = np.clip(height_map, -2, 2) / 2  # 限制到 [-1, 1]

        return height_map

    def create_front_surface_mesh(self) -> trimesh.Trimesh:
        """
        创建前表面（有凹凸的工作面）
        """
        nx = self.resolution
        ny = int(self.resolution * self.height / self.width)

        # 生成 Perlin 高度场
        height_map = self.generate_perlin_surface(nx, ny)

        # 创建网格顶点
        x = np.linspace(0, self.width, nx)
        y = np.linspace(0, self.height, ny)
        X, Y = np.meshgrid(x, y)

        # Z 方向偏移（凹凸）
        Z = height_map * self.roughness

        # 展平为顶点列表
        vertices = np.stack([X.flatten(), Y.flatten(), Z.flatten()], axis=1)

        # 创建三角形面片索引
        faces = []
        for i in range(ny - 1):
            for j in range(nx - 1):
                # 每个格子两个三角形
                v0 = i * nx + j
                v1 = i * nx + (j + 1)
                v2 = (i + 1) * nx + j
                v3 = (i + 1) * nx + (j + 1)

                faces.append([v0, v1, v2])
                faces.append([v1, v3, v2])

        faces = np.array(faces)

        # 创建 Trimesh 对象
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

        return mesh

    def create_back_surface_mesh(self, front_mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        """
        创建后表面（相对平整，但保持厚度）
        """
        # 复制前表面顶点，沿 Z 方向偏移
        back_vertices = front_mesh.vertices.copy()
        back_vertices[:, 2] -= self.thickness

        # 面片法向翻转
        back_faces = front_mesh.faces[:, ::-1]

        back_mesh = trimesh.Trimesh(vertices=back_vertices, faces=back_faces)

        return back_mesh

    def create_side_walls(self, front_mesh: trimesh.Trimesh, back_mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        """
        创建四周的侧壁连接前后表面
        """
        nx = self.resolution
        ny = int(self.resolution * self.height / self.width)

        side_meshes = []

        # 获取前后表面的顶点
        front_verts = front_mesh.vertices
        back_verts = back_mesh.vertices

        # 四条边界
        edges = [
            (0, nx - 1, 1),              # 底边 (y=0)
            (nx - 1, (ny - 1) * nx + nx - 1, nx),  # 右边 (x=max)
            ((ny - 1) * nx, (ny - 1) * nx + nx - 1, 1),  # 顶边 (y=max)
            (0, (ny - 1) * nx, nx),      # 左边 (x=0)
        ]

        for start_idx, end_idx, step in edges:
            indices = list(range(start_idx, end_idx + 1, step))

            # 创建侧壁四边形（两个三角形）
            wall_faces = []
            for i in range(len(indices) - 1):
                v0_front = indices[i]
                v1_front = indices[i + 1]
                v0_back = v0_front
                v1_back = v1_front

                # 第一个三角形
                wall_faces.append([v0_front, v1_front, v0_back + len(front_verts)])
                # 第二个三角形
                wall_faces.append([v1_front, v1_back + len(front_verts), v0_back + len(front_verts)])

            # 创建侧壁网格
            if wall_faces:
                wall_verts = np.vstack([front_verts, back_verts])
                wall_mesh = trimesh.Trimesh(vertices=wall_verts, faces=np.array(wall_faces))
                side_meshes.append(wall_mesh)

        # 合并所有侧壁
        combined_sides = trimesh.util.concatenate(side_meshes)

        return combined_sides

    def generate(self) -> trimesh.Trimesh:
        """
        生成完整的煤壁网格（前表面 + 后表面 + 侧壁）
        """
        print(f"生成煤壁网格...")
        print(f"  尺寸: {self.width}m × {self.height}m × {self.thickness}m")
        print(f"  倾斜角: {np.degrees(self.tilt_angle):.1f}°")
        print(f"  粗糙度: {self.roughness}m")
        print(f"  分辨率: {self.resolution}")
        print(f"  随机种子: {self.seed}")

        # 创建前表面
        front_mesh = self.create_front_surface_mesh()

        # 创建后表面
        back_mesh = self.create_back_surface_mesh(front_mesh)

        # 创建侧壁
        side_mesh = self.create_side_walls(front_mesh, back_mesh)

        # 合并所有部分
        coal_wall = trimesh.util.concatenate([front_mesh, back_mesh, side_mesh])

        # 居中并应用倾斜
        coal_wall.vertices[:, 0] -= self.width / 2
        coal_wall.vertices[:, 1] -= self.height / 2

        # 绕 Y 轴旋转（倾斜）
        rotation_matrix = trimesh.transformations.rotation_matrix(
            self.tilt_angle,
            [0, 1, 0]
        )
        coal_wall.apply_transform(rotation_matrix)

        # 确保水密性（填补可能的孔洞）
        print("  → 检查并填补孔洞...")
        coal_wall.fill_holes()

        # 修复法向（关键：确保一致性）
        print("  → 修复法向向量...")
        coal_wall.fix_normals()

        # 重新合并以确保拓扑一致性
        print("  → 优化网格拓扑...")
        coal_wall = trimesh.util.concatenate([coal_wall])

        print(f"✓ 生成完成")
        print(f"  顶点数: {len(coal_wall.vertices)}")
        print(f"  面片数: {len(coal_wall.faces)}")
        print(f"  法向向量: {len(coal_wall.vertex_normals)}")
        print(f"  是否水密: {coal_wall.is_watertight}")

        return coal_wall


def main():
    parser = argparse.ArgumentParser(description='生成煤壁几何网格')
    parser.add_argument('--width', type=float, default=3.0, help='宽度 (m)')
    parser.add_argument('--height', type=float, default=3.0, help='高度 (m)')
    parser.add_argument('--thickness', type=float, default=0.4, help='厚度 (m)')
    parser.add_argument('--tilt', type=float, default=20.0, help='倾斜角度 (度)')
    parser.add_argument('--roughness', type=float, default=0.15, help='表面粗糙度 (m)')
    parser.add_argument('--resolution', type=int, default=100, help='网格分辨率')
    parser.add_argument('--seed', type=int, default=42, help='随机种子')
    parser.add_argument('--output', type=str, default='coal_wall.obj', help='输出文件路径')
    parser.add_argument('--format', type=str, default='obj', choices=['obj', 'stl', 'dae'], help='输出格式')

    args = parser.parse_args()

    # 创建生成器
    generator = CoalWallGenerator(
        width=args.width,
        height=args.height,
        thickness=args.thickness,
        tilt_angle=args.tilt,
        roughness=args.roughness,
        resolution=args.resolution,
        seed=args.seed
    )

    # 生成网格
    mesh = generator.generate()

    # 导出
    output_path = args.output
    if not output_path.endswith(f'.{args.format}'):
        output_path = os.path.splitext(output_path)[0] + f'.{args.format}'

    # 导出网格（包含法向量）
    print(f"\n导出网格...")
    if args.format.lower() == 'obj':
        # OBJ 格式导出时包含法向量
        mesh.export(output_path, file_type='obj', include_normals=True)

        # 验证法向量导出
        with open(output_path, 'r') as f:
            lines = f.readlines()
            vn_count = sum(1 for line in lines if line.startswith('vn '))
            v_count = sum(1 for line in lines if line.startswith('v '))

        print(f"  OBJ 统计:")
        print(f"    - 顶点 (v): {v_count}")
        print(f"    - 法向 (vn): {vn_count}")

        if vn_count == v_count:
            print(f"    ✓ 法向向量完整")
        else:
            print(f"    ⚠ 警告：法向数量不匹配（{vn_count} vs {v_count}）")
    else:
        mesh.export(output_path)

    print(f"✓ 已保存到: {output_path}")

    # 生成预览图（如果安装了 matplotlib）
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # 绘制网格
        ax.plot_trisurf(
            mesh.vertices[:, 0],
            mesh.vertices[:, 1],
            mesh.vertices[:, 2],
            triangles=mesh.faces,
            cmap='gray',
            alpha=0.8
        )

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Coal Wall Mesh')

        preview_path = os.path.splitext(output_path)[0] + '_preview.png'
        plt.savefig(preview_path, dpi=150, bbox_inches='tight')
        print(f"✓ 预览图保存到: {preview_path}")

    except ImportError:
        print("(跳过预览图生成，需要安装 matplotlib)")


if __name__ == '__main__':
    main()
