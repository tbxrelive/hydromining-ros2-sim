#!/usr/bin/env python3
"""离线生成阶段六观测视点（相机看向作业区域中心）并写入 YAML。

用法: make_viewpoints.py --urdf panda.urdf --output viewpoints.yaml
视点以 (距离 m, 方位角 deg, 俯仰角 deg) 相对墙面法向定义；无解或误差超限的视点
会被明确标记为 reachable=false，不会静默丢弃。
"""
import argparse
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from algorithms.perception.kinematics import Chain, look_at

SEED = [-0.0085, -1.4199, 0.0104, -2.8105, 0.0387, 2.9613, 0.785]
CENTER = [1.517, 0.011, 0.694]
COAL_POSE_RPY = [0.0, -1.9198621771937625, 0.0]
VIEWS = {'front_0p6': (0.6, 0, 0), 'front_0p7': (0.7, 0, 0), 'front_0p8': (0.8, 0, 0), 'front_1p0': (1.0, 0, 0), 'front_1p1': (1.1, 0, 0), 'front_1p2': (1.2, 0, 0),
         'left_0p8': (0.8, 20, 0), 'right_0p8': (0.8, -20, 0), 'up_0p8': (0.8, 0, 15), 'down_0p8': (0.8, 0, -15)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--center', type=float, nargs=3, default=CENTER)
    args = ap.parse_args()
    chain = Chain(open(args.urdf).read(), 'panda_link0', 'camera_link')
    normal = Rotation.from_euler('xyz', COAL_POSE_RPY).apply([0, 0, 1])
    center = np.array(args.center)
    out = {'region_center_world': center.tolist(), 'wall_normal_world': normal.tolist(), 'viewpoints': {}}
    for name, (d, az, el) in VIEWS.items():
        base = Rotation.from_euler('z', az, degrees=True) * Rotation.from_euler('y', -el, degrees=True)
        direction = base.apply(normal)
        target = look_at(center + d * direction, center)
        rng = np.random.default_rng(0)
        trials = [chain.ik(target, SEED)] + [chain.ik(target, rng.uniform(chain.lower_, chain.upper_)) for _ in range(30)]
        q, ep, er = min(trials, key=lambda t: t[1] + t[2])
        ok = ep < 1e-3 and er < 0.02
        out['viewpoints'][name] = dict(distance_m=d, azimuth_deg=az, elevation_deg=el, reachable=bool(ok),
                                       position_error_m=ep, orientation_error_rad=er, joints=[float(v) for v in q],
                                       camera_position_world=target[:3, 3].tolist())
        print(f'{name:10s} reachable={ok} pos_err={ep:.2e} ori_err={er:.3f}')
    yaml.safe_dump(out, open(args.output, 'w'), sort_keys=False)


if __name__ == '__main__':
    main()
