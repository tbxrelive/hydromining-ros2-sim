#!/usr/bin/env python3
"""对静止视点的多帧序列测量时间噪声，验证 Gazebo 原生噪声开关。"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np


def summarize(root):
    """只比较同一视点、同一场景的不同时间戳，不把不同表面混为噪声。"""
    paths = sorted((root / 'dataset/baseline').glob('baseline_front_0p8_f*.npz'))
    if len(paths) < 8:
        raise ValueError('At least eight stationary front_0p8 frames are required')
    depths, ranges, depth_stamps, lidar_stamps = [], [], [], []
    for path in paths:
        with np.load(path) as data:
            depths.append(data['depth'])
            ranges.append(np.linalg.norm(data['lidar_local'], axis=-1))
            depth_stamps.append(data['depth_stamp'].tolist())
            lidar_stamps.append(data['lidar_stamp'].tolist())
    if len({tuple(stamp) for stamp in depth_stamps}) != len(paths):
        raise ValueError('Repeated depth frames')
    if len({tuple(stamp) for stamp in lidar_stamps}) != len(paths):
        raise ValueError('Repeated lidar frames')
    depth, distance = np.stack(depths), np.stack(ranges)
    depth_mask = np.isfinite(depth).all(axis=0) & (depth > 0.3).all(axis=0) & (depth < 3).all(axis=0)
    lidar_mask = np.isfinite(distance).all(axis=0) & (distance > 0.3).all(axis=0) & (distance < 8).all(axis=0)
    if depth_mask.sum() < 100 or lidar_mask.sum() < 100:
        raise ValueError('Insufficient stable returns')
    return {'frame_count': len(paths),
            'depth_std_m': float(depth[:, depth_mask].std(axis=0, ddof=1).mean()),
            'lidar_std_m': float(distance[:, lidar_mask].std(axis=0, ddof=1).mean()),
            'depth_pixel_count': int(depth_mask.sum()),
            'lidar_ray_count': int(lidar_mask.sum())}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--clean', type=Path, required=True)
    parser.add_argument('--noisy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    clean, noisy = summarize(args.clean), summarize(args.noisy)
    source = json.loads((args.noisy / 'provenance.json').read_text())['raw_noise_stddev_m']
    checks = {
        'clean_depth_stable': clean['depth_std_m'] < 0.0001,
        'clean_lidar_stable': clean['lidar_std_m'] < 0.0001,
        'native_depth_noise_detected': source['depth'] * 0.65 < noisy['depth_std_m'] < source['depth'] * 1.35,
        'native_lidar_noise_detected': source['lidar'] * 0.65 < noisy['lidar_std_m'] < source['lidar'] * 1.35}
    report = {'passed': all(checks.values()), 'checks': checks, 'clean': clean, 'noisy': noisy,
              'configured_sigma_m': source, 'scope': 'native stationary temporal noise, not calibrated mine dust'}
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
