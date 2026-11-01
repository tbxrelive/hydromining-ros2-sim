"""画三轮核（v0/v1/v1b）的响应曲线对比图，供论文第 2 章使用。"""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RUNS = (('v0', 'v0 (alpha=0.5, no threshold)', 'v0'),
        ('v1', 'v1 (alpha=1, no threshold)', 'v1'),
        ('v1b', 'v1b (threshold + 0.22x + decay 2)', 'v1b'))
COLORS = {'v0': '#999999', 'v1': '#4477aa', 'v1b': '#cc3311'}
LITERATURE_PC = 5.0


def load_slices(tag, directory):
    summary = json.loads((Path(directory)/'summary.json').read_text())
    return summary['fits']['slices'], summary['fits']


def main():
    output = ROOT/'results/thesis/E2.1a/v1b/figures'
    output.mkdir(parents=True, exist_ok=True)
    data = {tag: load_slices(tag, ROOT/f'results/thesis/E2.1a/{directory}')
            for tag, _, directory in RUNS}

    # 图 1：孔深 vs 压力
    fig, axis = plt.subplots(figsize=(4.6, 3.4))
    for tag, label, _ in RUNS:
        slices, _ = data[tag]
        axis.plot(slices['pressure'], np.asarray(slices['depth_at_pressure'])*1000,
                  'o-', color=COLORS[tag], label=label)
    axis.set_xlabel('jet pressure [MPa]')
    axis.set_ylabel('hole depth [mm]')
    axis.set_title('depth vs pressure (s=0.5 m, t=4 s)')
    axis.grid(alpha=.3)
    axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output/'depth_vs_pressure_three_kernels.png', dpi=150)
    plt.close(fig)

    # 图 2：log-log 指数对比（压力）
    fig, axis = plt.subplots(figsize=(4.6, 3.4))
    for tag, label, _ in RUNS:
        _, fits = data[tag]
        exponent = fits['depth_vs_pressure']['exponent']
        axis.bar(tag, exponent, color=COLORS[tag])
    axis.axhline(1.0005, color='k', linestyle='--', linewidth=1,
                 label='Zhang 2019 fit: 1.0005')
    axis.set_ylabel('log-log exponent of depth vs pressure')
    axis.set_title('pressure response exponent')
    axis.grid(alpha=.3, axis='y')
    axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output/'exponent_comparison.png', dpi=150)
    plt.close(fig)

    # 图 3：孔径 vs 靶距（半宽 0.22x 的验证）
    fig, axis = plt.subplots(figsize=(4.6, 3.4))
    slices, _ = data['v1b']
    standoffs = np.asarray(slices['standoff'])
    axis.plot(standoffs, np.asarray(slices['depth_at_standoff'])*1000, 'o-',
              color=COLORS['v1b'], label='hole depth')
    # 0.22x 半宽的参考线（按孔径 1 cm 判据的 2 倍近似对比）
    axis2 = axis.twinx()
    axis2.plot(standoffs, 220*standoffs, '--', color='#228833', label='jet width 0.22x')
    axis2.set_ylabel('jet half-width reference [mm]')
    axis.set_xlabel('standoff [m]')
    axis.set_ylabel('hole depth [mm]')
    axis.set_title('v1b: depth & jet width vs standoff (P=30, t=4 s)')
    axis.grid(alpha=.3)
    lines1, labels1 = axis.get_legend_handles_labels()
    lines2, labels2 = axis2.get_legend_handles_labels()
    axis.legend(lines1+lines2, labels1+labels2, fontsize=8)
    fig.tight_layout()
    fig.savefig(output/'v1b_depth_width_vs_standoff.png', dpi=150)
    plt.close(fig)
    print(f'图表写入 {output}')


if __name__ == '__main__':
    main()
