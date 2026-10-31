#!/usr/bin/env python3
"""阶段 6.0 离线退化数据生成；原始文件只读，标签与观测分开存放。"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
ALGORITHMS = WORKSPACE / 'src/hydromining_sim/scripts/algorithms/perception'
sys.path.insert(0, str(ALGORITHMS.parent.parent))
from algorithms.perception.environment import degrade_snapshot, label_counts, sensor_rng
from algorithms.perception.sensor_model import ENV_PRESETS


def sha256(path):
    """分块计算来源哈希，避免整份网格读入内存。"""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_config(path):
    """校验实验因子，拒绝隐式捆绑噪声与能见度。"""
    config = yaml.safe_load(path.read_text())
    expected = {'schema_version', 'seed', 'environments', 'noise_scales', 'r_min_m'}
    if not isinstance(config, dict) or set(config) != expected:
        raise ValueError('configuration fields do not match schema')
    if config['schema_version'] != 1:
        raise ValueError('unsupported schema version')
    sensor_rng(config['seed'], 'validation', 'depth')
    envs, scales = config['environments'], config['noise_scales']
    if (not isinstance(envs, list) or not envs or
            any(not isinstance(env, str) or env not in ENV_PRESETS for env in envs) or
            len(set(envs)) != len(envs)):
        raise ValueError('environments must be unique known preset names')
    if (not isinstance(scales, list) or not scales or
            any(isinstance(s, bool) or not isinstance(s, (int, float)) or
                not np.isfinite(s) or s < 0 for s in scales) or
            len(set(scales)) != len(scales)):
        raise ValueError('noise scales must be unique finite nonnegative numbers')
    minimum = config['r_min_m']
    if (isinstance(minimum, bool) or not isinstance(minimum, (int, float)) or
            not np.isfinite(minimum) or minimum <= 0):
        raise ValueError('r_min_m must be finite and positive')
    return config


def clean_source(source):
    """仅接受成功的无原生噪声/粒子录制，防止无提示地重复叠加噪声。"""
    summary = json.loads((source / 'summary.json').read_text())
    noise = summary.get('noise')
    noise_off = noise == 'off' or noise == {'depth': 0.0, 'lidar': 0.0}
    if summary.get('passed') is not True or summary.get('effects') != 'off' or not noise_off:
        raise ValueError('source must be a passed recording with native noise/effects off')
    paths = sorted((source / 'dataset').glob('*/*.npz'))
    if not paths:
        raise ValueError('source contains no snapshots')
    return paths


def generate(source, output, config_path):
    """流式处理每份快照，保存完整来源与全因子工况表，不覆盖旧证据。"""
    source, output = source.resolve(), output.resolve()
    config_path = config_path.resolve()
    config = read_config(config_path)
    paths = clean_source(source)
    if output == source or source in output.parents or output in source.parents:
        raise ValueError('output must be disjoint from source')
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError('output must be a new or empty directory')
    output.mkdir(parents=True, exist_ok=True)
    provenance_files = [source / 'summary.json']
    if (source / 'provenance.json').is_file():
        provenance_files.append(source / 'provenance.json')
    truth_files = sorted((source / 'dataset/truth').glob('*'))
    sources = [Path(__file__).resolve(), config_path, *sorted(ALGORITHMS.glob('*.py'))]
    manifest = {
        'schema_version': 1, 'source_recording': str(source),
        'config': config, 'calibrated': False,
        'scope': 'offline synthetic ranges only; no RGB darkness or native particles',
        'counts_scope': 'full sensor field of view, not task region coverage',
        'source_start_provenance_available': (source / 'provenance.json').is_file(),
        'rng': 'PCG64; SHA256(seed/snapshot/sensor) first 128 bits little-endian',
        'common_random_numbers_across_cases': True,
        'software_hashes': {str(p): sha256(p) for p in sources},
        'source_hashes': {
            str(p.relative_to(source)): sha256(p)
            for p in [*provenance_files, *truth_files, *paths] if p.is_file()},
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False))
    cases = []
    for env in config['environments']:
        for index, scale in enumerate(config['noise_scales']):
            case = {'id': f'{env}_noise{index}', 'environment': env,
                    'visibility_m': None if np.isinf(ENV_PRESETS[env]) else ENV_PRESETS[env],
                    'noise_scale': scale, 'frames': []}
            for path in paths:
                relative = path.relative_to(source / 'dataset')
                with np.load(path, allow_pickle=False) as data:
                    values = {key: data[key] for key in data.files}
                result = degrade_snapshot(
                    values['depth'], values['K'], values['lidar_local'],
                    ENV_PRESETS[env], scale,
                    sensor_rng(config['seed'], relative.as_posix(), 'depth'),
                    sensor_rng(config['seed'], relative.as_posix(), 'lidar'),
                    config['r_min_m'])
                labels = {key: result[key] for key in ('depth_labels', 'lidar_labels')}
                for key in ('depth', 'lidar_local'):
                    values[key] = result[key].astype(np.float32)
                observation = output / case['id'] / 'observations' / relative
                evaluation = output / case['id'] / 'evaluation_labels' / relative
                observation.parent.mkdir(parents=True, exist_ok=True)
                evaluation.parent.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(observation, **values)
                np.savez_compressed(evaluation, **labels)
                case['frames'].append({
                    'snapshot': relative.as_posix(),
                    'observation_sha256': sha256(observation),
                    'labels_sha256': sha256(evaluation),
                    'depth': label_counts(labels['depth_labels']),
                    'lidar': label_counts(labels['lidar_labels']),
                })
            cases.append(case)
    report = {'schema_version': 1, 'completed': True, 'calibrated': False,
              'source_snapshot_count': len(paths), 'case_count': len(cases),
              'generated_snapshot_count': len(paths) * len(cases), 'cases': cases}
    (output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='录制根目录，含 summary.json/dataset')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=(
        WORKSPACE / 'src/hydromining_robot/config/phase6_environment.yaml'))
    args = parser.parse_args()
    try:
        report = generate(args.source, args.output, args.config)
    except (ValueError, OSError, KeyError) as error:
        parser.exit(1, f'{error}\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'cases'}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
