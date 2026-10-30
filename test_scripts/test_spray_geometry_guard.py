"""An uncertain simulator mutation must not permit another numerical erosion."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parents[1]/'src/hydromining_robot/scripts'))
from spray_controller_node import SprayControllerNode


def test_unconfirmed_scene_rejects_spray_before_accessing_solver():
    """No request geometry or solver is provided: either access would fail."""
    phases = []
    node = SimpleNamespace(active_=True, cancel_=threading.Event(), phase_='IDLE',
        operation_lock_=threading.Lock(), gazebo_=SimpleNamespace(synchronized_=False),
        get_logger=lambda:SimpleNamespace(error=lambda message:None),
        set_phase=lambda *args:phases.append(args))
    response = SprayControllerNode.spray_callback(node, SimpleNamespace(), SimpleNamespace())
    assert not response.success and 'reset' in response.message
    assert phases[0][0]=='ERROR' and not node.operation_lock_.locked()
