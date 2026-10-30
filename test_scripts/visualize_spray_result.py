#!/usr/bin/env python3
"""
可视化喷射前后的煤壁模型对比

使用 Open3D 进行 3D 可视化
需求：pip install open3d
"""

import sys
from pathlib import Path
import numpy as np
import open3d as o3d

def load_mesh(mesh_path: str):
    """加载 Mesh 文件"""
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh


def visualize_comparison():
    """可视化喷射前后对比"""

    # 文件路径
    mesh_before = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"
    mesh_after = "/tmp/phase4_visualization/coal_wall_after_spray.obj"

    print("=" * 80)
    print("煤壁喷射前后对比可视化")
    print("=" * 80)

    # 检查文件是否存在
    if not Path(mesh_before).exists():
        print(f"❌ 找不到喷射前模型: {mesh_before}")
        return 1

    if not Path(mesh_after).exists():
        print(f"⚠️  找不到喷射后模型: {mesh_after}")
        print(f"   请先运行: python3 test_scripts/test_phase4_integration_mini.py")
        return 1

    print(f"\n加载模型...")
    print(f"  喷射前: {mesh_before}")
    print(f"  喷射后: {mesh_after}")

    # 加载模型
    mesh1 = load_mesh(mesh_before)
    mesh2 = load_mesh(mesh_after)

    # 设置颜色
    mesh1.paint_uniform_color([0.7, 0.7, 0.7])  # 灰色 - 喷射前
    mesh2.paint_uniform_color([0.9, 0.3, 0.3])  # 红色 - 喷射后

    # 平移模型以便对比
    mesh2.translate([3.5, 0, 0])  # 向右平移

    print(f"\n模型信息:")
    print(f"  喷射前: {len(mesh1.vertices)} 顶点, {len(mesh1.triangles)} 三角形")
    print(f"  喷射后: {len(mesh2.vertices)} 顶点, {len(mesh2.triangles)} 三角形")

    # 创建坐标系
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    print(f"\n可视化操作:")
    print(f"  - 鼠标左键拖动: 旋转视角")
    print(f"  - 鼠标滚轮: 缩放")
    print(f"  - 鼠标右键拖动: 平移")
    print(f"  - 按 Q 或关闭窗口: 退出")

    print(f"\n颜色说明:")
    print(f"  🟦 灰色 - 喷射前煤壁")
    print(f"  🟥 红色 - 喷射后煤壁（右侧）")
    print(f"\n正在打开 3D 可视化窗口...")

    # 可视化
    o3d.visualization.draw_geometries(
        [mesh1, mesh2, coord_frame],
        window_name="煤壁喷射前后对比",
        width=1280,
        height=720,
        left=50,
        top=50
    )

    print("\n✅ 可视化完成")
    return 0


def visualize_single(mesh_path: str, title: str = "Mesh 可视化"):
    """可视化单个模型"""

    if not Path(mesh_path).exists():
        print(f"❌ 文件不存在: {mesh_path}")
        return 1

    print(f"加载模型: {mesh_path}")
    mesh = load_mesh(mesh_path)
    mesh.paint_uniform_color([0.7, 0.7, 0.9])

    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    print(f"  顶点数: {len(mesh.vertices)}")
    print(f"  三角形数: {len(mesh.triangles)}")
    print(f"\n正在打开 3D 可视化窗口...")

    o3d.visualization.draw_geometries(
        [mesh, coord_frame],
        window_name=title,
        width=1280,
        height=720
    )

    print("\n✅ 可视化完成")
    return 0


def main():
    """主函数"""

    # 检查 Open3D 是否安装
    try:
        import open3d
    except ImportError:
        print("❌ 缺少依赖: Open3D")
        print("\n安装方法:")
        print("  cd ~/SCI/hydromining_ws")
        print("  source .venv_stage4/bin/activate")
        print("  pip install open3d")
        return 1

    # 解析命令行参数
    if len(sys.argv) > 1:
        if sys.argv[1] == "--before":
            return visualize_single("src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj", "喷射前煤壁")
        elif sys.argv[1] == "--after":
            return visualize_single("/tmp/phase4_visualization/coal_wall_after_spray.obj", "喷射后煤壁")
        elif sys.argv[1] == "--help":
            print("用法:")
            print("  python3 test_scripts/visualize_spray_result.py           # 对比视图（默认）")
            print("  python3 test_scripts/visualize_spray_result.py --before  # 仅喷射前")
            print("  python3 test_scripts/visualize_spray_result.py --after   # 仅喷射后")
            return 0
        else:
            print(f"❌ 未知参数: {sys.argv[1]}")
            print("使用 --help 查看帮助")
            return 1

    # 默认：对比视图
    return visualize_comparison()


if __name__ == "__main__":
    sys.exit(main())
