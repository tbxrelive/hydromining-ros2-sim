"""sim-v1 冻结校验：仿真基准被改动时必须失败，除非升版本号。

锁文件由 `scripts/freeze_sim_v1.py` 生成。改动破煤核、蚀除数值、可见性、
传感器退化、替身渲染、任务配置、干扰等级或场景库之后，这个测试会失败——
这是有意的：这些改动会作废全部下游数字，必须先升版本（sim-v2）并重记基线。
"""
import hashlib
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

LOCK = ROOT/'configs/sim_v1.lock.json'


def _digest(path):
    return hashlib.sha256((ROOT/path).read_bytes()).hexdigest()


def test_lock_file_exists():
    assert LOCK.exists(), '缺少 configs/sim_v1.lock.json，先运行 scripts/freeze_sim_v1.py'


def test_locked_files_unchanged():
    """冻结集合里每个文件的内容必须与锁文件一致。"""
    expected = json.loads(LOCK.read_text())
    assert expected['version'] == 'sim-v1'
    missing = [path for path in expected['files'] if not (ROOT/path).exists()]
    assert not missing, f'冻结文件缺失: {missing}'
    changed = [path for path, digest in expected['files'].items() if _digest(path) != digest]
    assert not changed, ('以下文件在冻结后被修改，会作废全部下游数字；'
                         f'请升版本号并重新记录规则基线: {changed}')


def test_scenario_library_matches_lock():
    """场景库清单也在冻结集合内，保证实验数据可追溯。"""
    expected = json.loads(LOCK.read_text())
    library = 'results/thesis/E2.5/library.json'
    if library in expected['files']:
        assert _digest(library) == expected['files'][library]
