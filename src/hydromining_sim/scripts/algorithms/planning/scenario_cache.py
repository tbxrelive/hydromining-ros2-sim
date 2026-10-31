"""Atomic deterministic initialization cache; no random observations are cached."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np


def cache_path(env, name):
    """Invalidate on algorithm, mesh, calibration, resolution and task changes."""
    root = Path(__file__).resolve().parents[1]
    digest = hashlib.sha256()
    for path in sorted(root.rglob('*.py')):
        digest.update(path.read_bytes())
    # Source mesh is already loaded by the simulation; include its geometry.
    mesh = env.world_.simulation_.grid.source_mesh
    for array in (mesh.vertices, mesh.faces):
        digest.update(np.asarray(array).tobytes())
    digest.update(json.dumps(env.cfg_, sort_keys=True).encode())
    digest.update(str((env.world_.simulation_.grid.resolution, env.world_.simulation_.evolver.dt)).encode())
    for view, spec in sorted(env.specs_.items()):
        digest.update(view.encode())
        for key in ('tf_depth', 'tf_lidar', 'K', 'image_shape'):
            digest.update(np.asarray(spec[key]).tobytes())
    return Path(env.cache_dir_) / (name + '_' + digest.hexdigest()[:24] + '.npz')


def load_entry(env, name):
    path = cache_path(env, name)
    if not path.exists():
        return None
    with np.load(path, allow_pickle=False) as data:
        masks = {}
        for key in data.files:
            if key.startswith('mask__'):
                _, view, sensor = key.split('__')
                masks.setdefault(view, {})[sensor] = data[key]
        return {'phi': data['phi'], 'pristine': data['pristine'],
                'surface': (data['local'], data['world']), 'masks': masks}


def save_entry(env, name, entry):
    path = cache_path(env, name)
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = dict(phi=entry['phi'], pristine=entry['pristine'],
                  local=entry['surface'][0], world=entry['surface'][1])
    for view, sensors in entry['masks'].items():
        for sensor, mask in sensors.items():
            arrays[f'mask__{view}__{sensor}'] = mask
    temporary = path.with_name(path.stem + f'.{os.getpid()}.npz')
    np.savez_compressed(temporary, **arrays)
    os.replace(temporary, path)
