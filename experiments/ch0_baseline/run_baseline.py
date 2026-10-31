"""阶段 0.3：在 L0 测试集上记录规则策略基准，供重构前后对比。

复用 phase7_evaluate_large 的 evaluate/rule_action，保证与 2026-10-31 的
评估口径一致（4 个测试工况 × 3 面测试墙，seed=10000）。输出统一三件套：
manifest.json / episodes.jsonl / summary.json，写入 results/thesis/E0.3/。
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'test_scripts'))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from phase7_large_env import make_fast_env, prepare_meshes  # noqa: E402
from phase7_evaluate_large import evaluate, rule_action  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E0.3'))
    parser.add_argument('--episodes', type=int, default=12)
    parser.add_argument('--seed', type=int, default=10000)
    args = parser.parse_args()

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    assets = output/'assets'
    prepare_meshes(assets)
    started = time.monotonic()
    env = make_fast_env(assets, args.seed, 'test')
    try:
        result = evaluate(lambda _: env, rule_action, args.episodes, args.seed, 'L0')
    finally:
        env.close()
    episodes = result['episodes']
    summary = {'policy': 'rule', 'fidelity': 'L0', 'episodes': len(episodes),
               'seed': args.seed, **result['summary']['all']}
    with (output/'episodes.jsonl').open('w') as handle:
        for record in episodes:
            handle.write(json.dumps(record) + '\n')
    (output/'summary.json').write_text(json.dumps(summary, indent=2))
    sources = [ROOT/'test_scripts/phase7_evaluate_large.py', ROOT/'test_scripts/phase7_large_env.py',
               ROOT/'src/hydromining_sim/scripts/algorithms/planning/fast_heightfield.py']
    (output/'manifest.json').write_text(json.dumps({
        'experiment': 'E0.3', 'purpose': '重构前规则基线', 'wall_time_s': time.monotonic()-started,
        'code_sha256': hashlib.sha256(b''.join(p.read_bytes() for p in sources)).hexdigest(),
        'sources': [str(p.relative_to(ROOT)) for p in sources]}, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
