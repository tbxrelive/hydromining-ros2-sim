#!/usr/bin/env python3
"""独立审计离线退化数据的来源、标签隔离和全视场回波统计。"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    """流式计算哈希；不调用数据生成器的实现。"""
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(block)
    return hasher.hexdigest()


def audit(dataset):
    """验证记录一致性；不将软件模型自洽性解释成物理正确性。"""
    manifest = json.loads((dataset / 'manifest.json').read_text())
    summary = json.loads((dataset / 'summary.json').read_text())
    source = Path(manifest['source_recording'])
    config = manifest['config']
    expected_cases = {(env, index) for env in config['environments']
                      for index in range(len(config['noise_scales']))}
    expected_ids = {f'{env}_noise{index}' for env, index in expected_cases}
    cases = summary['cases']
    if (summary.get('completed') is not True or
            len(cases) != len(expected_ids) or
            {case['id'] for case in cases} != expected_ids or
            summary['case_count'] != len(cases)):
        raise ValueError('missing or duplicate experiment cases')
    expected_frames = {str(Path(p).relative_to('dataset')) for p in manifest['source_hashes']
                       if p.startswith('dataset/') and p.endswith('.npz')}
    for relative, expected in manifest['source_hashes'].items():
        if digest(source / relative) != expected:
            raise ValueError(f'source changed: {relative}')
    rows = []
    for case in cases:
        frames = case['frames']
        if (len(frames) != len(expected_frames) or
                {frame['snapshot'] for frame in frames} != expected_frames):
            raise ValueError('missing or duplicate snapshots')
        env, index = case['id'].rsplit('_noise', 1)
        if (case['environment'] != env or
                case['noise_scale'] != config['noise_scales'][int(index)]):
            raise ValueError('case factors do not match configuration')
        totals = {sensor: np.zeros(4, dtype=np.int64) for sensor in ('depth', 'lidar')}
        for frame in case['frames']:
            relative = frame['snapshot']
            observation = dataset / case['id'] / 'observations' / relative
            evaluation = dataset / case['id'] / 'evaluation_labels' / relative
            if (digest(observation) != frame['observation_sha256'] or
                    digest(evaluation) != frame['labels_sha256']):
                raise ValueError(f'derived data changed: {case["id"]}/{relative}')
            with (np.load(source / 'dataset' / relative) as raw,
                  np.load(observation) as observed, np.load(evaluation) as labels):
                if set(observed.files) != set(raw.files):
                    raise ValueError('observation schema changed or evaluation labels leaked')
                for key in raw.files:
                    if key not in ('depth', 'lidar_local'):
                        np.testing.assert_array_equal(raw[key], observed[key])
                for sensor, key in (('depth', 'depth'), ('lidar', 'lidar_local')):
                    lab = labels[f'{sensor}_labels']
                    points = observed[key]
                    finite = np.isfinite(points)
                    if sensor == 'lidar':
                        finite = finite.all(axis=-1)
                    if lab.shape != finite.shape or not np.isin(lab, [0, 1, 2, 3]).all():
                        raise ValueError('invalid labels or shape')
                    np.testing.assert_array_equal(finite, (lab == 0) | (lab == 2))
                    original = raw[key]
                    if sensor == 'depth':
                        yy, xx = np.indices(original.shape)
                        k = raw['K']
                        ray_factor = np.sqrt(1 + ((xx - k[0, 2]) / k[0, 0]) ** 2 +
                                             ((yy - k[1, 2]) / k[1, 1]) ** 2)
                        raw_valid = np.isfinite(original) & (original > 0)
                        raw_valid &= original * ray_factor >= config['r_min_m']
                    else:
                        raw_valid = np.isfinite(original).all(axis=-1)
                        raw_valid &= np.linalg.norm(original, axis=-1) >= config['r_min_m']
                    np.testing.assert_array_equal(lab != 3, raw_valid)
                    counts = np.bincount(lab.ravel(), minlength=4)
                    recorded = frame[sensor]
                    np.testing.assert_array_equal(
                        counts, [recorded[k] for k in ('valid', 'dropped', 'false_return', 'invalid')])
                    totals[sensor] += counts
                    if case['environment'] == 'clean' and case['noise_scale'] == 0:
                        np.testing.assert_allclose(points[finite], raw[key][finite], atol=1e-7)
                original = raw['lidar_local']
                changed = observed['lidar_local']
                valid = np.isfinite(changed).all(axis=-1)
                if valid.any():
                    unit0 = original[valid] / np.linalg.norm(original[valid], axis=1)[:, None]
                    unit1 = changed[valid] / np.linalg.norm(changed[valid], axis=1)[:, None]
                    np.testing.assert_allclose(unit0, unit1, atol=1e-6)
        row = {key: case[key] for key in ('id', 'environment', 'visibility_m', 'noise_scale')}
        row['frame_count'] = len(case['frames'])
        for sensor, counts in totals.items():
            eligible = int(counts[:3].sum())
            row[sensor] = {
                'input_valid_rays': eligible, 'input_invalid_rays': int(counts[3]),
                'surface_return_fraction': float(counts[0] / eligible) if eligible else None,
                'drop_fraction': float(counts[1] / eligible) if eligible else None,
                'false_return_fraction': float(counts[2] / eligible) if eligible else None,
            }
        rows.append(row)
    generated_count = sum(row['frame_count'] for row in rows)
    if (summary['source_snapshot_count'] != len(expected_frames) or
            summary['generated_snapshot_count'] != generated_count):
        raise ValueError('summary snapshot counts do not match files')
    return {'passed': True, 'calibrated': False,
            'scope': 'file/metadata integrity and full-FOV ray counts; not physical validation',
            'generated_snapshot_count': generated_count, 'rows': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('audit output already exists')
    try:
        report = audit(args.dataset)
    except (ValueError, OSError, KeyError, AssertionError) as error:
        report = {'passed': False, 'error': str(error)}
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
