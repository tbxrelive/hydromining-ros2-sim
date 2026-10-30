"""Lost ACKs must not desynchronize numerical coal or cause duplicate writes."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import pytest
sys.path.insert(0, str(Path(__file__).parents[1]/'src/hydromining_robot/scripts'))
import gazebo_coal_wall as wall


def model(path):
    return {'name':'coal_wall', 'link':[{'visual':[{'name':'wall_visual',
            'geometry':{'mesh':{'filename':str(path)}}}]}]}


class FakeWall(wall.GazeboCoalWall):
    def __init__(self, fault):
        super().__init__('world', [0]*6)
        self.model = model('/old.obj')
        self.fault = fault
        self.creates = 0
        self.unknown = False
        self.reads = 0

    def scene_wall(self):
        self.reads += 1
        if self.fault == 'initial_read' and self.reads <= 2:
            raise RuntimeError('Initial scene discovery delayed')
        if self.unknown:
            raise RuntimeError('Scene unavailable')
        return self.model

    def request(self, endpoint, request_type, payload, **kwargs):
        if endpoint == 'remove':
            self.model = None
            if self.fault == 'remove_ack':
                raise RuntimeError('Remove reply lost')
        else:
            self.creates += 1
            assert payload.endswith('allow_renaming: false')
            sdf, _ = json.JSONDecoder().raw_decode(payload[5:])
            path = unquote(urlparse(ET.fromstring(sdf).find('.//visual/geometry/mesh/uri').text).path)
            if self.fault == 'rejected':
                raise wall.RequestRejected('Explicit rejection')
            if self.fault == 'missing' and self.creates == 1:
                raise RuntimeError('Create reply lost and no effect')
            self.model = model('/wrong.obj' if self.fault == 'wrong' else path)
            if self.fault == 'unknown':
                self.unknown = True
                raise RuntimeError('Unknown outcome')
            if self.fault == 'create_ack':
                raise RuntimeError('Create applied but reply lost')
        return 'data: true'


@pytest.fixture(autouse=True)
def fast_deadlines(monkeypatch):
    ticks = iter(i*.5 for i in range(10000))
    monkeypatch.setattr(wall.time, 'monotonic', lambda:next(ticks))
    monkeypatch.setattr(wall.time, 'sleep', lambda _:None)


@pytest.mark.parametrize('fault,creates', [('none',1),('remove_ack',1),('create_ack',1),('missing',2),('initial_read',1)])
def test_reconcile_lost_reply_without_duplicate_creation(tmp_path, fault, creates):
    adapter = FakeWall(fault)
    adapter.replace(tmp_path/'new.obj')
    assert adapter.synchronized_ and adapter.creates == creates
    assert adapter.matches_mesh(adapter.model, tmp_path/'new.obj')


@pytest.mark.parametrize('fault', ['wrong','unknown','rejected'])
def test_ambiguous_or_wrong_geometry_requires_reset(tmp_path, fault):
    adapter = FakeWall(fault)
    with pytest.raises(RuntimeError, match='reset required'):
        adapter.replace(tmp_path/'new.obj')
    assert not adapter.synchronized_ and adapter.creates == 1
    assert adapter.current_mesh_ is None
    adapter.fault, adapter.unknown = 'none', False
    adapter.replace(tmp_path/'reset.obj')
    assert adapter.synchronized_ and adapter.matches_mesh(adapter.model, tmp_path/'reset.obj')


def test_scene_identity_uses_exact_model_and_decoded_mesh_path(monkeypatch, tmp_path):
    adapter = wall.GazeboCoalWall('world', [0]*6)
    scene = {'model':[{'name':'coal_wall_other','link':[]},model(tmp_path/'煤 wall.obj')]}
    scene['model'][1]['link'][0]['visual'][0]['geometry']['mesh']['filename'] = (tmp_path/'煤 wall.obj').as_uri()
    monkeypatch.setattr(wall.subprocess, 'run', lambda *a,**kw:SimpleNamespace(stdout=json.dumps(scene)))
    assert adapter.matches_mesh(adapter.scene_wall(), tmp_path/'煤 wall.obj')
    assert not adapter.matches_mesh(adapter.scene_wall(), tmp_path/'other.obj')
    assert adapter.matches_mesh(model(tmp_path/'煤#?%20.obj'), tmp_path/'煤#?%20.obj')


def test_target_appearing_at_confirmation_deadline_is_accepted(tmp_path):
    adapter = FakeWall('none')
    wait = adapter.wait_exists

    def late(wanted, *args, **kwargs):
        if wanted:
            raise RuntimeError('Confirmation deadline reached just before scene update')
        return wait(wanted, *args, **kwargs)

    adapter.wait_exists = late
    adapter.replace(tmp_path/'late.obj')
    assert adapter.synchronized_ and adapter.creates == 1
