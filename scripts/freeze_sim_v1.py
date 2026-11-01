"""冻结仿真基准：生成 configs/sim_v1.lock.json。

锁定的是"改了就作废一切下游数字"的文件：破煤核、高度场蚀除、可见性、
传感器退化模型、替身环境渲染、任务配置、干扰等级、场景库清单。
任何一项变化都必须升版本号（sim-v1 → sim-v2）并重新记录规则基线。

命令：
    python scripts/freeze_sim_v1.py            # 写入/更新锁文件
    python scripts/freeze_sim_v1.py --check    # 只检查，不写
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
LOCK = ROOT/'configs/sim_v1.lock.json'

# 冻结集合：算法实现 + 任务配置 + 干扰等级 + 场景库
LOCKED_FILES = (
    'src/hydromining_sim/scripts/algorithms/erosion_model.py',
    'src/hydromining_sim/scripts/algorithms/heightfield_erosion.py',
    'src/hydromining_sim/scripts/algorithms/level_set_evolver.py',
    'src/hydromining_sim/scripts/algorithms/spray_simulation.py',
    'src/hydromining_sim/scripts/algorithms/local_spray_simulation.py',
    'src/hydromining_sim/scripts/algorithms/voxel_grid.py',
    'src/hydromining_sim/scripts/algorithms/perception/visibility.py',
    'src/hydromining_sim/scripts/algorithms/perception/sensor_model.py',
    'src/hydromining_sim/scripts/algorithms/perception/environment.py',
    'src/hydromining_sim/scripts/algorithms/planning/fast_heightfield.py',
    'src/hydromining_sim/scripts/algorithms/planning/surrogate.py',
    'src/hydromining_robot/config/phase7_large_task.yaml',
    'configs/interference_levels.yaml',
    'results/thesis/E2.5/library.json',
)


def digest(path):
    """文件内容的 sha256。"""
    return hashlib.sha256((ROOT/path).read_bytes()).hexdigest()


def build():
    """构造锁文件内容。"""
    return {'version': 'sim-v1b',
            'frozen_at': '2026-11-01',
            'note': 'v1b：按文献全文加阈值压力与 0.22x 半宽；场景库内容与 sim-v1 逐字节一致',
            'files': {path: digest(path) for path in LOCKED_FILES}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    current = build()
    missing = [path for path in LOCKED_FILES if not (ROOT/path).exists()]
    if missing:
        raise SystemExit(f'缺少待冻结的文件：{missing}')
    if args.check:
        if not LOCK.exists():
            raise SystemExit('锁文件不存在，先运行 scripts/freeze_sim_v1.py')
        expected = json.loads(LOCK.read_text())
        changed = {path: (expected['files'].get(path), value)
                   for path, value in current['files'].items()
                   if expected['files'].get(path) != value}
        if changed:
            for path, (old, new) in changed.items():
                print(f'已变化: {path}\n  旧 {old}\n  新 {new}')
            raise SystemExit('仿真基准已被修改；请升版本号并重新记录规则基线')
        print(f"{current['version']} 未变化（{len(current['files'])} 个文件）")
        return
    LOCK.write_text(json.dumps(current, indent=2, ensure_ascii=False))
    print(f"写入 {LOCK}（{len(current['files'])} 个文件）")


if __name__ == '__main__':
    sys.exit(main())
