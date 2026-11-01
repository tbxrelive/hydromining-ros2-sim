"""E2.1c 在 sim-v1 上重建规则基线，并与 E0.3 逐回合比对。

E2.1a 把压力指数从 0.5 改为 1.0，但按"标称工况 R_c 不变"重新锚定了 k，
因此任务压力（30 MPa）下的全部计算应当与此前完全一致。本实验负责证明这一点，
并检查任务可行性：单次喷射在 2–8 s 时长范围内既能推进 5 cm 目标，又不会打穿。

命令：
    python experiments/ch2_simulation/e2_1c_baseline.py
"""
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from experiments.common.crater import flat_plate, measure, through_check  # noqa: E402
from experiments.common.regression import compare  # noqa: E402
from experiments.common.task import load_task  # noqa: E402

REFERENCE = ROOT/'results/thesis/E0.3'
OUTPUT = ROOT/'results/thesis/E2.1c'


def feasibility_single_shot():
    """任务可行性：标称工况下单次喷射的孔深随持续时间的变化。"""
    from algorithms import heightfield_erosion
    from experiments.common.crater import flat_plate as plate
    task = load_task()
    kernel = task.erosion_kernel()
    extent, spacing = task.patch_extent_m, task.geometry_spacing_m
    heights, grid, _ = plate(extent, spacing)
    before = heights.copy()
    rows = []
    for duration in (2., 4., 6., 8.):
        working = before.copy()
        heightfield_erosion.spray_heightfield(working, grid, 0., 0., duration,
                                              task.pressure_mpa, task.standoff_m,
                                              kernel, spacing, extent)
        result = measure(before, working, spacing, center=(0., 0.))
        rows.append({'duration_s': duration,
                     'depth_center_m': result['depth_center_m'],
                     'diameter_half_m': result['diameter_half_m'],
                     'volume_m3': result['volume_m3'],
                     'reaches_required_depth': bool(result['depth_center_m'] >= task.required_depth),
                     'through': through_check(before, working, task.wall_thickness_m)})
    return rows


def main():
    started = time.monotonic()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    run_dir = OUTPUT/'baseline'
    subprocess.run([sys.executable, str(ROOT/'experiments/ch0_baseline/run_baseline.py'),
                    '--output', str(run_dir)], check=True, cwd=ROOT,
                   stdout=subprocess.DEVNULL)

    reference = REFERENCE/'episodes.jsonl'
    candidate = run_dir/'episodes.jsonl'
    if reference.exists():
        differences, count = compare(reference, candidate)
    else:
        differences, count = [('参考缺失', '', '', '')], 0

    shots = feasibility_single_shot()
    summary = json.loads((run_dir/'summary.json').read_text())
    report = {
        'baseline_success_rate': summary['success_rate'],
        'baseline_episodes': summary['episodes'],
        'identical_to_E0.3': not differences,
        'difference_count': len(differences),
        'differences': [str(d) for d in differences[:20]],
        'feasibility': {
            'single_shot': shots,
            'min_duration_reaching_required_depth': min(
                [row['duration_s'] for row in shots if row['reaches_required_depth']], default=None),
            'any_through_cut': any(row['through'] for row in shots),
        },
        'wall_time_s': time.monotonic()-started,
    }
    (OUTPUT/'e2_1c.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps({k: v for k, v in report.items() if k != 'feasibility'},
                     indent=2, ensure_ascii=False))
    print('单次喷射可行性：')
    for row in shots:
        print(f"  {row['duration_s']:.0f} s  深 {row['depth_center_m']*1000:5.1f} mm  "
              f"孔径 {row['diameter_half_m']*1000:5.1f} mm  "
              f"达标 {row['reaches_required_depth']}  打穿 {row['through']}")


if __name__ == '__main__':
    main()
