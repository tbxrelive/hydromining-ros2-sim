#!/usr/bin/env python3
"""Harmonic UserCommands adapter; no numerical or ROS dependencies."""
import json
import subprocess
import time
from pathlib import Path
from xml.sax.saxutils import escape
from urllib.parse import unquote, urlparse


class RequestRejected(RuntimeError):
    """The server explicitly rejected a mutation, rather than losing its reply."""


class GazeboCoalWall:
    """Replace both visual and collision meshes, retaining the world pose."""

    def __init__(self, world, pose, scene_probe=None, warning=None):
        self.world_ = world
        self.pose_ = tuple(pose)
        self.current_sdf_ = None
        self.current_mesh_ = None
        self.synchronized_ = True
        self.scene_probe_ = str(scene_probe or Path(__file__).with_name('coal_wall_scene'))
        self.warning_ = warning or (lambda message: None)

    def request(self, endpoint, request_type, payload, response_type='gz.msgs.Boolean', timeout_ms=5000):
        """A missing reply is ambiguous: never equate it with a rejected mutation."""
        try:
            result = subprocess.run(
                ['gz', 'service', '-s', f'/world/{self.world_}/{endpoint}',
                 '--reqtype', request_type, '--reptype', response_type,
                 '--timeout', str(timeout_ms), '--req', payload],
                capture_output=True, text=True, timeout=timeout_ms/1000+2, check=True)
        except (subprocess.SubprocessError, OSError) as error:
            raise RuntimeError(f'{endpoint}: response unavailable: {error}') from error
        if response_type == 'gz.msgs.Boolean':
            if 'data: false' in result.stdout:
                raise RequestRejected(f'{endpoint}: server rejected request')
            if 'data: true' not in result.stdout:
                raise RuntimeError(f'{endpoint}: response unavailable: {result.stdout} {result.stderr}')
        return result.stdout

    def scene_wall(self):
        """Typed native protobuf inspection, including exact model and mesh identity.

        SceneBroadcaster does not include collision geometry in scene/info.
        This confirms the displayed model URI; both geometries are created from
        the same submitted SDF, without claiming an independent contact test.
        """
        try:
            result = subprocess.run([self.scene_probe_, self.world_], capture_output=True,
                                    text=True, timeout=4, check=True)
            models = [m for m in json.loads(result.stdout).get('model', []) if m.get('name') == 'coal_wall']
        except (subprocess.SubprocessError, OSError, ValueError) as error:
            raise RuntimeError(f'Cannot confirm coal_wall scene state: {error}') from error
        if len(models) > 1:
            raise RuntimeError('Multiple coal_wall models; reset required')
        return models[0] if models else None

    @staticmethod
    def matches_mesh(model, mesh_path):
        """Reject stale or wrong mesh instances, even when their name matches."""
        expected = Path(mesh_path).resolve()
        visuals = [v for link in model.get('link', []) for v in link.get('visual', [])
                   if v.get('name') == 'wall_visual']
        if len(visuals) != 1:
            return False
        uri = visuals[0].get('geometry', {}).get('mesh', {}).get('filename', '')
        parsed = urlparse(uri)
        actual = unquote(parsed.path) if parsed.scheme == 'file' else uri
        return bool(uri) and parsed.scheme in ('', 'file') and Path(actual).resolve() == expected

    def exists(self):
        return self.scene_wall() is not None

    def known_wall(self, timeout=6):
        """A transient read failure must not be mistaken for confirmed absence."""
        deadline = time.monotonic()+timeout
        detail = ''
        while time.monotonic() < deadline:
            try:
                return self.scene_wall()
            except RuntimeError as error:
                detail = str(error)
            time.sleep(.1)
        raise RuntimeError(f'Cannot establish coal_wall state: {detail}')

    def wait_exists(self, wanted, mesh_path=None, timeout=8):
        """Poll bounded, read-only observations after either success or lost ACK."""
        deadline = time.monotonic()+timeout
        detail = ''
        while time.monotonic() < deadline:
            try:
                model = self.scene_wall()
                if (model is not None) == wanted and (not wanted or mesh_path is None or self.matches_mesh(model, mesh_path)):
                    return
                detail = 'absent' if model is None else 'observed meshes: '+str([
                    v.get('geometry', {}).get('mesh', {}).get('filename')
                    for link in model.get('link', []) for v in link.get('visual', [])])
            except RuntimeError as error:
                detail = str(error)
            time.sleep(0.1)
        raise RuntimeError(f'coal_wall state unconfirmed (present={wanted}, mesh={mesh_path}): {detail}')

    def replace(self, mesh_path):
        """Confirm scene convergence before committing numerical state.

        Lost mutation replies are reconciled by inspection. At most one create
        retry is allowed, only after confirmed absence, with renaming disabled.
        An unresolved outcome blocks further sprays until a successful reset;
        blindly restoring an old model could race a late create of the new one.
        """
        mesh_path = Path(mesh_path).resolve()
        uri = escape(mesh_path.as_uri())
        pose = ' '.join(str(v) for v in self.pose_)
        sdf = f'''<sdf version="1.9"><model name="coal_wall"><static>true</static>
<pose>{pose}</pose><link name="wall_link">
<collision name="wall_collision"><geometry><mesh><uri>{uri}</uri></mesh></geometry></collision>
<visual name="wall_visual"><geometry><mesh><uri>{uri}</uri></mesh></geometry>
<material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material>
</visual></link></model></sdf>'''
        self.synchronized_ = False
        try:
            if self.known_wall() is not None:
                try:
                    self.request('remove', 'gz.msgs.Entity', 'name: "coal_wall", type: MODEL')
                except RuntimeError as error:
                    self.warning_(f'{error}; checking actual removal before proceeding')
                self.wait_exists(False)
            payload = 'sdf: '+json.dumps(sdf)+' allow_renaming: false'
            for attempt in range(2):
                rejected = False
                try:
                    self.request('create', 'gz.msgs.EntityFactory', payload, timeout_ms=10000)
                except RuntimeError as error:
                    rejected = isinstance(error, RequestRejected)
                    self.warning_(f'{error}; checking actual target mesh before deciding failure')
                try:
                    self.wait_exists(True, mesh_path, timeout=6)
                    break
                except RuntimeError:
                    # An unknown scene or wrong existing mesh must never trigger
                    # a second write. A late same-name create cannot be renamed.
                    observed = self.known_wall()
                    if observed is not None and self.matches_mesh(observed, mesh_path):
                        break
                    if attempt or rejected or observed is not None:
                        raise
                    self.warning_('Target confirmed absent; resubmitting once with renaming disabled')
            self.current_sdf_ = sdf
            self.current_mesh_ = str(mesh_path)
            self.synchronized_ = True
        except Exception as error:
            raise RuntimeError(f'Coal geometry update unconfirmed; reset required: {error}') from error
