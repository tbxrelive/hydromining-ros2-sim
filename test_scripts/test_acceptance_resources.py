"""Configurable resource protection must identify offending workers."""
import json
from types import SimpleNamespace

import pytest
import test_phase4_acceptance as acceptance


@pytest.mark.parametrize('worker_mib,available_mib,blocked', [
    (2900, 2048, False), (3100, 2048, True), (1000, 900, True),
])
def test_guard_records_processes_before_stopping(tmp_path, monkeypatch, worker_mib, available_mib, blocked):
    """Exercise both real protection conditions and a valid run without ROS startup."""
    class Worker:
        def __init__(self, pid, mib):
            self.pid, self.mib = pid, mib

        def children(self, recursive):
            return []

        def memory_info(self):
            return SimpleNamespace(rss=self.mib*1024**2)

        def name(self):
            return 'test_worker'

    monkeypatch.setenv("HYDROMINING_RSS_GIB", "3")
    monkeypatch.setenv("HYDROMINING_MIN_AVAILABLE_GIB", "1")
    root, client = Worker(1, worker_mib), Worker(2, 10)
    monkeypatch.setattr(acceptance.psutil, 'Process', lambda pid=None: client if pid is None else root)
    monkeypatch.setattr(acceptance.psutil, 'virtual_memory',
                        lambda: SimpleNamespace(available=available_mib*1024**2))
    node = SimpleNamespace(process_=SimpleNamespace(pid=1, poll=lambda:None),
                           output_=tmp_path, peak_rss_=0, owned_processes_={}, max_joint_drift_=0)
    if blocked:
        with pytest.raises(RuntimeError, match='Memory guard:'):
            acceptance.AcceptanceTest.monitor(node)
    else:
        acceptance.AcceptanceTest.monitor(node)
    snapshot = json.loads((tmp_path/'resources.jsonl').read_text())
    assert snapshot['rss_mib'] == worker_mib+10
    assert snapshot['available_mib'] == available_mib
    assert snapshot['processes'][0]['pid'] == root.pid


def test_new_machine_budgets_and_invalid_values(monkeypatch):
    from runtime_resources import memory_limits
    monkeypatch.delenv('HYDROMINING_RSS_GIB', raising=False)
    monkeypatch.delenv('HYDROMINING_MIN_AVAILABLE_GIB', raising=False)
    assert memory_limits() == (24 * 1024**3, 8 * 1024**3)
    monkeypatch.setenv('HYDROMINING_RSS_GIB', 'nan')
    with pytest.raises(ValueError):
        memory_limits()
