"""E1.4 示例实验：用统一接口跑 2 回合，写 manifest/episodes/summary 并复算校验。

这是阶段 1.4 的框架自检，不是论文实验：只证明"配置 → 运行 → 记录 → 统计"
链路可用。命令：
    python experiments/ch0_baseline/e1_4_example.py --output results/thesis/E1.4
"""
import argparse
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from experiments.common.large_env import make_fast_env, prepare_meshes  # noqa: E402
from experiments.common.records import RunWriter, read_episodes  # noqa: E402
from experiments.common.stats import summarize  # noqa: E402

METRICS = ('steps', 'sprays', 'spray_duration_s', 'removed_volume_m3', 'truth_fraction')
CASES = (('test_empty', 0), ('test_offset', 1))


def run_case(env, scenario, mesh_index, seed):
    obs, info = env.reset(seed=seed, options={'scenario': scenario, 'mesh_index': mesh_index})
    while True:
        obs, _, terminated, truncated, info = env.step(env.rule_action())
        if terminated or truncated:
            break
    retreat = env._map_heights(env.initial_-env.heights_)
    return {'scenario': scenario, 'mesh_index': mesh_index,
            'verdict': info['verdict'], 'truth': info['truth'],
            'truth_fraction': info['truth_fraction'],
            'steps': env.steps_used_, 'sprays': env.sprays_used_,
            'spray_duration_s': env.total_spray_duration_,
            'removed_volume_m3': float(np.sum(np.maximum(retreat, 0)*env.area_)),
            'truncated': bool(truncated)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E1.4'))
    parser.add_argument('--seed', type=int, default=10000)
    args = parser.parse_args()
    output = Path(args.output)
    prepare_meshes(output/'assets')
    env = make_fast_env(output/'assets', args.seed, 'test')
    try:
        with RunWriter(output, 'E1.4', {'policy': 'rule', 'fidelity': 'L0', 'seed': args.seed,
                                        'episodes': len(CASES)}) as writer:
            for scenario, mesh_index in CASES:
                writer.write_episode(run_case(env, scenario, mesh_index, args.seed))
            summary = writer.finish(METRICS)
    finally:
        env.close()
    # 从落盘记录复算，必须与 summary.json 完全一致。
    recomputed = summarize(read_episodes(output/'episodes.jsonl'), METRICS)
    for key, value in recomputed.items():
        assert summary[key] == value, f'summary 与复算不一致: {key}'
    print(f'E1.4 通过：{output}  回合 {summary["episodes"]}  '
          f'平均步数 {summary["steps_mean"]:.1f}  平均喷射 {summary["sprays_mean"]:.1f}')


if __name__ == '__main__':
    main()
