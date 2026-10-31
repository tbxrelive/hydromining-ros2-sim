"""Shared task construction and disjoint wall sets for user-run experiments."""
from pathlib import Path
import hashlib
import os
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))
from algorithms.perception.kinematics import look_at
from algorithms.perception.metrics import mesh_heights
from algorithms.perception.wall_frame import WallFrame
from algorithms.planning.fast_heightfield import FastHeightfieldEnv, TRAIN_SCENARIOS, TEST_SCENARIOS
from algorithms.perception.kinematics import Chain
from algorithms.planning.rl_env import HydrominingSurrogateEnv
from algorithms.planning.surrogate import SurrogateWorld
from algorithms.task_spec import TaskSpec
from scipy.spatial.transform import Rotation

TASK = Path('src/hydromining_robot/config/phase7_large_task.yaml')
TRAIN_SEEDS = (10, 11, 12)
TEST_SEEDS = (20, 21, 22)


def fidelity_signature():
    """Bind the L0 gate to the task, original wall and numerical implementations."""
    root = Path('src/hydromining_sim/scripts/algorithms')
    paths = [TASK, Path('src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj'),
             root/'planning/fast_heightfield.py', root/'heightfield_erosion.py',
             root/'perception/visibility.py', root/'erosion_model.py', root/'level_set_evolver.py',
             root/'local_spray_simulation.py', root/'spray_simulation.py', root/'voxel_grid.py']
    return hashlib.sha256(b''.join(path.read_bytes() for path in paths)).hexdigest()


def prepare_meshes(directory):
    """Generate separate deterministic train/test walls using the project generator."""
    from generate_coal_wall import CoalWallGenerator
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for seed in (*TRAIN_SEEDS, *TEST_SEEDS):
        path = directory/f'coal_{seed}.obj'
        if not path.exists():
            mesh = CoalWallGenerator(roughness=.12, seed=seed).generate()
            mesh.merge_vertices(digits_vertex=7, merge_norm=True, merge_tex=True)
            if not mesh.is_watertight:
                raise ValueError(f'Generated wall is not watertight: {path}')
            mesh.export(path, file_type='obj', include_normals=True)


def make_fast_env(directory, seed=0, split='train', noise_scale=0.):
    """L0 samples each separate mesh and produces the same v3 observation schema."""
    task = TaskSpec.from_yaml(TASK).as_dict()
    frame = WallFrame.from_normal(task['region_center_world'], task['wall_normal_world'])
    mesh_seeds = TRAIN_SEEDS if split == 'train' else TEST_SEEDS
    heights = [mesh_heights(str(Path(directory)/f'coal_{s}.obj'), task['source_coal_pose'],
                            frame, task['patch_extent_m'], .005) for s in mesh_seeds]
    cameras = [look_at(np.array(task['viewpoints'][name]['camera_position_world']), frame.origin)
               for name in ('front_0p8', 'left_0p8', 'right_0p8')]
    focal = 320/(2*np.tan(.75))
    K = np.array([[focal, 0., 160.], [0., focal, 120.], [0., 0., 1.]])
    scenarios = TRAIN_SCENARIOS if split == 'train' else TEST_SCENARIOS
    return FastHeightfieldEnv(heights, frame, cameras, K, (240, 320),
                              pressure=task['pressure_mpa'], standoff=task['standoff_m'],
                              max_steps=task['max_steps'], noise_scale=noise_scale,
                              seed=seed, scenarios=scenarios, patch_extent=task['patch_extent_m'],
                              diameter=task['diameter'], required_depth=task['required_depth'],
                              required_fraction=task['required_fraction'], wall_thickness=task['wall_thickness_m'])


def make_reference_env(directory, mesh_index, fidelity, seed=0, noise_scale=0.):
    """L1/L2 use the same fine geometry and task but retain their sensor pipelines."""
    task = TaskSpec.from_yaml(TASK).as_dict()
    mesh = Path(directory)/f'coal_{TEST_SEEDS[mesh_index]}.obj'
    world = SurrogateWorld(str(mesh), local_task=task, tcp_erosion_offset=.05,
                           local_output=f'/tmp/hydromining_ref_{os.getpid()}_{mesh_index}')
    robot = Path('src/hydromining_robot/urdf/panda_with_water_gun.urdf.xacro')
    urdf = subprocess.run(['xacro', str(robot)], capture_output=True, text=True, check=True).stdout
    camera = Chain(urdf, 'panda_link0', 'camera_link')
    lidar = Chain(urdf, 'panda_link0', 'lidar_link')
    lidar_pose = lidar.fk([])
    focal = 320/(2*np.tan(.75))
    K = np.array([[focal, 0., 160.], [0., focal, 120.], [0., 0., 1.]])
    specs = {}
    for name, view in task['viewpoints'].items():
        depth_pose = camera.fk(view['joints'])
        specs[name] = {'K': K, 'image_shape': (240, 320),
            'tf_depth': np.r_[depth_pose[:3, 3], Rotation.from_matrix(depth_pose[:3, :3]).as_quat()],
            'tf_lidar': np.r_[lidar_pose[:3, 3], Rotation.from_matrix(lidar_pose[:3, :3]).as_quat()],
            'joints': view['joints']}
    return HydrominingSurrogateEnv(world, specs, task, fidelity=fidelity, noise_scale=noise_scale,
        seed=seed, max_steps=task['max_steps'], scenarios=TEST_SCENARIOS,
        cache_dir=str(Path(directory)/'.reference_cache') if fidelity == 'L1' else None)
