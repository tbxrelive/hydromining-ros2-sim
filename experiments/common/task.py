"""实验脚本读取任务参数的统一入口；路径与算法实现解耦。"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))

from algorithms.task_spec import TaskSpec  # noqa: E402

DEFAULT_TASK_PATH = ROOT/'src/hydromining_robot/config/phase7_large_task.yaml'


def load_task(path=DEFAULT_TASK_PATH):
    """返回不可变 TaskSpec；所有实验与算法都从这里取任务参数。"""
    return TaskSpec.from_yaml(path)
