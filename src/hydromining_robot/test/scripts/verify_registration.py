#!/usr/bin/env python3
"""
点云配准验证工具
读取保存的点云，执行 ICP 配准，评估对齐质量
"""

import open3d as o3d
import numpy as np
import glob
import os
import sys


def load_pointclouds(data_dir):
    """加载所有点云文件"""
    pcd_files = sorted(glob.glob(f"{data_dir}/*.pcd"))

    if len(pcd_files) < 2:
        print(f"错误：至少需要 2 个点云文件，但只找到 {len(pcd_files)} 个")
        return [], []

    print(f"找到 {len(pcd_files)} 个点云文件:")
    for f in pcd_files:
        print(f"  - {os.path.basename(f)}")

    pointclouds = [o3d.io.read_point_cloud(f) for f in pcd_files]
    return pointclouds, pcd_files


def downsample_and_filter(pcd, voxel_size=0.05):
    """下采样和离群点移除"""
    # 体素下采样
    pcd_down = pcd.voxel_down_sample(voxel_size)

    # 统计离群点移除
    pcd_filtered, _ = pcd_down.remove_statistical_outlier(
        nb_neighbors=20,
        std_ratio=2.0
    )

    return pcd_filtered


def register_icp(source, target, threshold=0.02):
    """ICP 配准"""
    trans_init = np.identity(4)

    reg_result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
    )

    return reg_result


def evaluate_registration(pcd_a, pcd_b):
    """评估配准质量"""
    print("\n=== 点云配准评估 ===")

    # 预处理
    print("预处理点云...")
    pcd_a_down = downsample_and_filter(pcd_a)
    pcd_b_down = downsample_and_filter(pcd_b)

    print(f"点云 A: {len(pcd_a_down.points)} 点")
    print(f"点云 B: {len(pcd_b_down.points)} 点")

    # 执行 ICP 配准
    print("\n执行 ICP 配准...")
    reg_result = register_icp(pcd_a_down, pcd_b_down, threshold=0.05)

    # 输出结果
    print(f"\n配准适应度 (fitness): {reg_result.fitness:.4f}")
    print(f"配准 RMSE: {reg_result.inlier_rmse:.4f} m")
    print(f"对应点对数量: {len(reg_result.correspondence_set)}")

    print("\n变换矩阵:")
    print(reg_result.transformation)

    # 判断配准质量
    if reg_result.fitness > 0.5 and reg_result.inlier_rmse < 0.05:
        print("\n✓ 配准质量良好（重叠区域对齐成功）")
        verdict = True
    elif reg_result.fitness > 0.3:
        print("\n⚠ 配准质量一般（可能需要调整观测位姿）")
        verdict = False
    else:
        print("\n✗ 配准失败（重叠区域不足或数据质量差）")
        verdict = False

    # 可视化
    print("\n显示配准结果（关闭窗口继续）...")
    pcd_a_down.paint_uniform_color([1, 0, 0])  # 红色
    pcd_b_down.paint_uniform_color([0, 1, 0])  # 绿色
    pcd_b_aligned = pcd_b_down.transform(reg_result.transformation)

    o3d.visualization.draw_geometries(
        [pcd_a_down, pcd_b_aligned],
        window_name="点云配准结果（红色=A，绿色=B对齐后）"
    )

    return verdict


def main():
    if len(sys.argv) < 2:
        print("用法: python3 verify_registration.py <点云数据目录>")
        sys.exit(1)

    data_dir = os.path.expanduser(sys.argv[1])

    if not os.path.exists(data_dir):
        print(f"错误：目录不存在: {data_dir}")
        sys.exit(1)

    # 加载点云
    pointclouds, filenames = load_pointclouds(data_dir)

    if len(pointclouds) < 2:
        sys.exit(1)

    # 配准前两个点云（A 和 B）
    print(f"\n对比点云:")
    print(f"  源: {os.path.basename(filenames[0])}")
    print(f"  目标: {os.path.basename(filenames[1])}")

    success = evaluate_registration(pointclouds[0], pointclouds[1])

    if success:
        print("\n✓✓✓ 验证通过：传感器系统工作正常")
    else:
        print("\n✗✗✗ 验证失败：需要调整配置")


if __name__ == '__main__':
    main()
