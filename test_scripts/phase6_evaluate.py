#!/usr/bin/env python3
"""只读取观测的离线感知入口；不读取 truth 或 evaluation_labels。"""
import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/hydromining_sim/scripts'))
from algorithms.perception.config import PerceptionConfig
from algorithms.perception.pipeline import PerceptionSession


def load_groups(directory, views=None):
    """按视点和帧序号加载；不把不同视点混入时序过滤。"""
    groups = {}
    for path in sorted(directory.glob('*.npz')):
        name = path.stem.removeprefix(directory.name + '_')
        view = re.sub(r'_f\d+$', '', name)
        if views and view not in views:
            continue
        with np.load(path, allow_pickle=False) as data:
            groups.setdefault(view, []).append({key: data[key] for key in data.files})
    if not groups:
        raise ValueError(f'no observation groups in {directory}')
    return list(groups.values())


def run(dataset, output, config, center, views=None, scenarios=None):
    """按同一基线独立评估各喷后场景；地图/判决文件分开保存。"""
    if output.exists() and any(output.iterdir()):
        raise ValueError('output must be a new or empty directory')
    output.mkdir(parents=True, exist_ok=True)
    session = PerceptionSession(center, config)
    session.set_baseline(load_groups(dataset / 'baseline', views))
    rows = {}
    names = scenarios or [
        path.name for path in sorted(dataset.iterdir())
        if path.is_dir() and path.name not in ('baseline', 'truth', 'evaluation_labels')]
    for name in names:
        result = session.evaluate(load_groups(dataset / name, views))
        rows[name] = result['report']
        state = {f'{side}_{key}': value for side in ('baseline', 'current', 'change')
                 for key, value in result[side].items()}
        np.savez_compressed(output / f'{name}_state.npz', **state,
                            origin=session.frame_.origin, axes=session.frame_.axes)
    (output / 'assessment.json').write_text(json.dumps(rows, indent=2, allow_nan=False))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=WORKSPACE / 'src/hydromining_robot/config/perception.yaml')
    parser.add_argument('--center', type=float, nargs=3, default=[1.517, 0.011, 0.694])
    parser.add_argument('--views', nargs='+')
    parser.add_argument('--scenarios', nargs='+')
    args = parser.parse_args()
    config = PerceptionConfig.from_dict(yaml.safe_load(args.config.read_text()))
    report = run(args.dataset, args.output, config, args.center, args.views, args.scenarios)
    print(json.dumps({name: row['assessment'] for name, row in report.items()}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
