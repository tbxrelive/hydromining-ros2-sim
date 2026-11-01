"""E2.3a 干扰参数预实验：退化效果随能见度、距离、视角与传感器噪声如何变化。

为什么先做这一步：同一个能见度参数在不同距离和视角下退化程度不同
（往返透过率 T = exp(-2βr)，且斜视时同一像素对应的斜距更长），所以
"四个干扰等级取什么值"必须先在参数空间上看清结构，再定等级。

度量口径与 Gazebo 的 `jet_effects.cloud_change` **完全同源**（同一个函数、
同样的 1 cm / 2 cm 容差），保证软件层与粒子层可以对齐。

不依赖 ROS，也不需要 Gazebo。命令：
    python experiments/ch2_simulation/e2_3a_interference_pilot.py
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from algorithms.jet_effects import cloud_change  # noqa: E402,F401  (保留同一口径的交叉校验入口)
from algorithms.perception.environment import degrade_snapshot, sensor_rng  # noqa: E402
from algorithms.perception.frames import backproject_depth  # noqa: E402
from experiments.common.interference import disturbed_fraction, sensor_tolerance  # noqa: E402

IMAGE_SHAPE = (240, 320)
FOCAL = 320/(2*np.tan(.75))
K = np.array([[FOCAL, 0., 160.], [0., FOCAL, 120.], [0., 0., 1.]])


def plane_geometry(axial_distance, angle_deg):
    """生成倾斜平面在相机坐标下的轴向深度图与 LiDAR 点。

    相机采用 Gazebo 约定（+X 前 / +Y 左 / +Z 上）：像素射线方向为
    d = [1, -(u-cx)/fx, -(v-cy)/fy]，平面法向 n = [cosθ, sinθ, 0]，
    得到轴向深度 X = r·cosθ / (cosθ - a·sinθ)，其中 a = (u-cx)/fx。
    """
    rows, cols = np.indices(IMAGE_SHAPE)
    a = (cols - K[0, 2])/K[0, 0]
    b = (rows - K[1, 2])/K[1, 1]
    theta = np.deg2rad(angle_deg)
    denominator = np.cos(theta) - a*np.sin(theta)
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = axial_distance*np.cos(theta)/denominator
    depth = np.where((denominator > .2) & (depth > 0), depth, np.nan)
    direction = np.stack((np.ones_like(a), -a, -b), axis=-1)
    points = direction*depth[..., None]
    lidar_direction = direction[::8, ::8].reshape(-1, 3)
    lidar = lidar_direction*depth[::8, ::8].reshape(-1, 1)
    return depth, points.reshape(-1, 3), lidar


def one_trial(depth, clean_points, lidar, visibility, noise_scale, seed, angle_deg, index):
    """对一个条件采样一次，返回深度相机与 LiDAR 的受扰比例等指标。

    容差逐射线取 `max(下限, 3σ(斜距))`，使洁净条件下 D≈0，只有超出传感器
    自身噪声包络的偏差才被计为受扰（见 experiments/common/interference.py）。
    """
    depth_rng = sensor_rng(seed, f'pilot_{index}', 'depth')
    lidar_rng = sensor_rng(seed, f'pilot_{index}', 'lidar')
    degraded = degrade_snapshot(depth, K, lidar, visibility, noise_scale,
                                depth_rng, lidar_rng)
    observed = backproject_depth(degraded['depth'], K).reshape(-1, 3)
    clean_range = np.linalg.norm(clean_points, axis=1)
    lidar_range = np.linalg.norm(lidar, axis=1)
    depth_cloud = disturbed_fraction(clean_points, observed,
                                     sensor_tolerance(clean_range, 'depth'))
    lidar_cloud = disturbed_fraction(lidar, degraded['lidar_local'],
                                     sensor_tolerance(lidar_range, 'lidar'))
    finite = np.isfinite(observed).all(axis=1)
    target = np.isfinite(clean_points).all(axis=1)   # 平面上真正有几何的像素
    usable = target & finite
    error = np.linalg.norm(observed[usable]-clean_points[usable], axis=1)
    normalized = error/np.maximum(sensor_tolerance(clean_range[usable], 'depth'), 1e-9)
    return {'disturbed_depth': depth_cloud['disturbed_fraction'],
            'disturbed_lidar': lidar_cloud['disturbed_fraction'],
            'depth_error_rmse_m': float(np.sqrt(np.mean(error**2))) if len(error) else float('nan'),
            'depth_error_in_sigma': float(np.median(normalized)) if len(normalized) else float('nan'),
            'depth_finite_fraction': float(usable.sum()/max(target.sum(), 1))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E2.3a'))
    parser.add_argument('--repeats', type=int, default=20)
    parser.add_argument('--seed', type=int, default=20261101)
    args = parser.parse_args()

    # 能见度覆盖三个数量级：从几乎洁净（200 m）到近乎全遮挡（1 m）。
    visibility = [np.inf, 200., 120., 80., 50., 30., 20., 12., 8., 5., 3., 2., 1.5, 1.]
    distances = [.5, .8, 1.2, 1.6, 2.0]
    angles = [0., 20., 40., 60.]
    noise_scales = [1., 2.]

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    rows = []
    index = 0
    for visibility_value in visibility:
        for distance in distances:
            for angle in angles:
                depth, clean_points, lidar = plane_geometry(distance, angle)
                for noise in noise_scales:
                    for repeat in range(args.repeats):
                        result = one_trial(depth, clean_points, lidar, visibility_value,
                                           noise, args.seed+repeat, angle, index)
                        rows.append({'visibility_m': visibility_value, 'distance_m': distance,
                                     'angle_deg': angle, 'noise_scale': noise,
                                     'repeat': repeat, **result})
                    index += 1

    def aggregate(records):
        block = {}
        for name in ('disturbed_depth', 'disturbed_lidar', 'depth_error_rmse_m',
                     'depth_error_in_sigma', 'depth_finite_fraction'):
            values = np.array([r[name] for r in records], float)
            finite = values[np.isfinite(values)]
            block[name] = {'mean': float(finite.mean()) if len(finite) else None,
                           'std': float(finite.std(ddof=1)) if len(finite) > 1 else 0.,
                           'n': int(len(finite))}
        return block

    cells = {}
    for row in rows:
        key = f"{row['visibility_m']}|{row['distance_m']}|{row['angle_deg']}|{row['noise_scale']}"
        cells.setdefault(key, []).append(row)
    summary = {'repeats': args.repeats, 'seed': args.seed,
               'image_shape': list(IMAGE_SHAPE),
               'tolerances': {'depth_floor_m': .01, 'lidar_floor_m': .02,
                              'sigma_multiple': 3.0},
               'wall_time_s': time.monotonic()-started,
               'cells': {key: aggregate(value) for key, value in cells.items()}}
    (output/'pilot.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    (output/'pilot_rows.jsonl').write_text(
        '\n'.join(json.dumps(r, ensure_ascii=False) for r in rows) + '\n')

    # 打印"视角 0°、噪声 1×"的 D-距离表，作为定等级的直接依据。
    print(f'受扰比例 D（深度相机，视角 0°，噪声 1×），行=能见度 V，列=距离 r')
    header = '  V\\r ' + ''.join(f'{d:>8.1f}' for d in distances)
    print(header)
    for visibility_value in visibility:
        label = 'inf' if np.isinf(visibility_value) else f'{visibility_value:.1f}'
        line = f'{label:>6s}'
        for distance in distances:
            key = f'{visibility_value}|{distance}|0.0|1.0'
            value = summary['cells'][key]['disturbed_depth']['mean']
            line += f'{value*100:8.2f}'
        print(line)
    plot(summary, output)
    print(f'\n结果写入 {output}')


def plot(summary, output):
    """画 D 随距离变化的曲线（每个能见度一条），以及视角影响。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figures = Path(output)/'figures'
    figures.mkdir(parents=True, exist_ok=True)
    cells = summary['cells']
    visibilities = sorted({float(k.split('|')[0]) for k in cells})
    distances = sorted({float(k.split('|')[1]) for k in cells})
    for angle, tag in ((0., 'angle0'), (60., 'angle60')):
        fig, axis = plt.subplots(figsize=(4.6, 3.4))
        for visibility in visibilities:
            values = [cells[f'{visibility}|{d}|{angle}|1.0']['disturbed_depth']['mean']
                      for d in distances]
            label = 'inf' if np.isinf(visibility) else f'{visibility:g} m'
            axis.plot(distances, np.asarray(values)*100, 'o-', label=label)
        axis.set_xlabel('observation distance [m]')
        axis.set_ylabel('depth disturbed fraction [%]')
        axis.set_title(f'view angle {angle:.0f} deg')
        axis.grid(alpha=.3)
        axis.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(figures/f'{tag}.png', dpi=150)
        plt.close(fig)


if __name__ == '__main__':
    main()
