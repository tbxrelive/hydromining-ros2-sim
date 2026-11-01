"""重构回归比对：逐回合逐字段比较两份 episodes.jsonl。

用途：任何改动 L0 蚀除、可见性或规则决策之后，都要与基准运行比对，
不能只看测试是否通过。默认忽略墙钟计时字段（每次运行必然不同）。
"""
import json
from pathlib import Path

# 每次运行必然不同的计时字段；比对时一律跳过。
TIMING_FIELDS = ('wall_time_s', 'reset_wall_s', 'step_wall_s', 'elapsed_s')


def load(path):
    """读取 episodes.jsonl 为字典列表。"""
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def compare(baseline, candidate, skip=TIMING_FIELDS):
    """返回 (差异列表, 回合数)；差异为空表示逐字段一致。

    每个字段比较用 `!=`，因此 NaN 出现在两侧时也会被记为差异（NaN != NaN）。
    需要容忍 NaN 的实验请自行在上游把 NaN 换成哨兵值。
    """
    a, b = (load(baseline), load(candidate)) if not isinstance(baseline, list) else (baseline, candidate)
    if len(a) != len(b):
        raise ValueError(f'回合数不一致: {len(a)} != {len(b)}')
    skip = set(skip)
    differences = []
    for index, (left, right) in enumerate(zip(a, b)):
        for key in sorted(set(left) | set(right)):
            if key in skip:
                continue
            if key not in left or key not in right:
                differences.append((index, key, left.get(key), right.get(key)))
            elif left[key] != right[key]:
                differences.append((index, key, left[key], right[key]))
    return differences, len(a)


def assert_identical(baseline, candidate, skip=TIMING_FIELDS):
    """比对失败时抛错并列出前若干条差异。"""
    differences, count = compare(baseline, candidate, skip)
    if differences:
        preview = '\n'.join(str(d) for d in differences[:10])
        raise AssertionError(f'{count} 个回合中有 {len(differences)} 处字段差异：\n{preview}')
    return count
