"""实验运行记录：manifest + episodes.jsonl + summary.json 三件套。

manifest 记录配置、代码版本与耗时；episodes 每行一个回合；summary 由
`stats.summarize` 从已写入的回合记录算出，可离线复算比对。
"""
import json
from pathlib import Path
import subprocess
import time


class RunWriter:
    """在 output 目录下写一次实验的完整记录。"""

    def __init__(self, output, experiment, config):
        self.output = Path(output)
        self.experiment = experiment
        self.config = dict(config)
        self.episodes = []
        self.started = time.monotonic()

    def __enter__(self):
        self.output.mkdir(parents=True, exist_ok=True)
        self.manifest = {
            'experiment': self.experiment,
            'config': self.config,
            'code_revision': _git_revision(),
            'started_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
        (self.output/'manifest.json').write_text(json.dumps(self.manifest, indent=2, ensure_ascii=False))
        self._stream = (self.output/'episodes.jsonl').open('w')
        return self

    def write_episode(self, record):
        self.episodes.append(record)
        self._stream.write(json.dumps(record, ensure_ascii=False)+'\n')

    def finish(self, metrics, extra=None):
        """按 metrics 字段汇总本回合记录并落盘，返回 summary。"""
        from .stats import summarize
        summary = {'experiment': self.experiment, **summarize(self.episodes, metrics),
                   **{k: v for k, v in self.config.items() if k in ('fidelity', 'seed', 'policy')}}
        summary.update(extra or {})
        self.manifest['wall_time_s'] = time.monotonic()-self.started
        (self.output/'manifest.json').write_text(json.dumps(self.manifest, indent=2, ensure_ascii=False))
        (self.output/'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
        return summary

    def __exit__(self, *_):
        self._stream.close()


def read_episodes(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def _git_revision():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'
