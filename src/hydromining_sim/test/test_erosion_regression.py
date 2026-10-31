"""Physical invariants and transactional regressions; CPU-only, bounded grids."""
import sys
from pathlib import Path
import numpy as np
import pytest
import trimesh
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from algorithms.voxel_grid import VoxelGrid
from algorithms.level_set_evolver import LevelSetEvolver
from algorithms.erosion_model import SprayAction, ErosionKernel, ErosionModel
from algorithms.spray_simulation import SpraySimulation
from algorithms.geometry_utils import GeometryUtils
from algorithms.mesh_converter import MeshConverter


@pytest.fixture
def box_file(tmp_path):
    path = tmp_path/'box.obj'
    trimesh.creation.box(extents=[1, 1, 0.4]).export(path)
    return path


def action(**kwargs):
    return SprayAction(**dict(position=np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]),
                              pressure=10., duration=4., **kwargs))


def test_sdf_sign_distance_and_bounds(box_file):
    g = VoxelGrid.from_mesh_simple(box_file, (40, 40, 24), chunk_size=128)
    assert g.sample(np.array([[0., 0., 0.]]))[0] == pytest.approx(-0.2, abs=g.spacing)
    assert g.sample(np.array([[0., 0., 0.3]]))[0] == pytest.approx(0.1, abs=g.spacing)
    assert g.phi[0].min() > 0 and g.phi[-1].min() > 0
    v, f = MeshConverter.voxel_to_mesh(g.phi, g.origin, g.spacing)
    mesh = trimesh.Trimesh(v, f)
    assert mesh.is_watertight and mesh.volume > 0
    assert mesh.volume == pytest.approx(0.4, rel=0.08)
    exported = box_file.parent/'exported.obj'
    assert MeshConverter.save_mesh(v, f, str(exported))
    lines = exported.read_text().splitlines()
    assert sum(line.startswith('vn ') for line in lines) == len(v)


def test_upwind_planar_retreat_independent_of_spacing():
    with pytest.raises(ValueError):
        VoxelGrid(10000,10000,10000,[0,0,0],.01)
    for h in [0.05, 0.1]:
        phi = np.broadcast_to(np.arange(12)[:, None, None]*h, (12, 8, 8)).copy()
        e = LevelSetEvolver(0.1)
        result = e.evolve_step(phi, np.full_like(phi, 0.01), h)
        assert result[4, 4, 4]-phi[4, 4, 4] == pytest.approx(0.001, abs=1e-7)
        assert e.compute_gradient_norm(phi, h)[4, 4, 4] == pytest.approx(1.)
    with pytest.raises(ValueError):
        e.evolve_step(phi, np.ones_like(phi)*100, h)


def test_shots_reset_time_and_depth(box_file):
    kernel = ErosionKernel(lambda p, s: .015, lambda p, s: .18)
    s = SpraySimulation(box_file, (40, 40, 24), dt=.2, kernel=kernel)
    initial = s.grid.phi.copy()
    first, a = s.simulate(action())
    assert a['elapsed_s'] == pytest.approx(4.)
    assert a['volume_removed_m3'] > 0
    assert a['max_depth_m'] == pytest.approx(.06, abs=.02)
    assert np.array_equal(initial, s.grid.phi), 'simulate must not commit'
    assert np.all(first >= initial)
    s.commit(first)
    second, b = s.simulate(action())
    assert b['cumulative_volume_m3'] > a['cumulative_volume_m3']
    assert b['cumulative_depth_m'] > a['cumulative_depth_m']
    s.reset()
    replay, c = s.simulate(action())
    assert np.array_equal(first, replay)
    assert a == c
    assert np.array_equal(initial, s.grid.phi)


def test_no_hit_off_invalid_and_occlusion(box_file):
    g = VoxelGrid.from_mesh_simple(box_file, (32, 32, 20))
    model = ErosionModel(ErosionKernel(lambda p,s: .01, lambda p,s: .2))
    points = g.points
    for a in [SprayAction([0,0,1],[0,0,1],10,1), action(g=0)]:
        speed = model.compute_erosion_velocity(points,a,g.phi,g.origin,g.spacing)
        assert not speed.any()
    with pytest.raises(ValueError):
        SprayAction([0,0,1],[0,0,-2],10,1)
    with pytest.raises(ValueError):
        SprayAction([0,0,1],[0,0,-1],-1,1)
    blocker = trimesh.creation.box(extents=[1,1,.1]);blocker.apply_translation([0,0,.6])
    speed = model.compute_erosion_velocity(points,action(),g.phi,g.origin,g.spacing,
                                          occluder_meshes=[(blocker.vertices,blocker.faces)])
    assert not speed.any(), 'a forward-facing surface behind an obstacle is not exposed'


def test_original_coal_zero_shot_fidelity():
    import open3d as o3d
    path = Path(__file__).resolve().parents[1]/'models/coal_wall/meshes/coal_wall.obj'
    g = VoxelGrid.from_mesh_simple(path, (80,80,56))
    scene = ErosionModel.surface_scene(g.phi,g.origin,g.spacing)
    query = o3d.core.Tensor(g.source_mesh.vertices[::8].astype(np.float32))
    error = scene.compute_distance(query,nthreads=1).numpy()
    assert float(error.max()) < 1.5*g.spacing
    assert np.percentile(error,95) < g.spacing


def test_reinitialized_fine_solver_keeps_advancing_at_jet_axis(tmp_path):
    """A 5 mm Level Set must not freeze after the initial 1–2 cm retreat."""
    from algorithms.local_spray_simulation import surface_mesh, surface_rays
    coordinates = np.linspace(-.15, .15, 61)
    mesh_path = tmp_path/'flat.obj'
    surface_mesh(coordinates, coordinates, np.zeros((61, 61)), .2).export(mesh_path)
    sim = SpraySimulation(mesh_path, (66, 66, 46), wall_budget_s=60., reinitialize_interval_s=.5)
    jet = SprayAction([0, 0, .6], [0, 0, -1], 30., 2.)
    heights = []
    for _ in range(2):
        phi, _ = sim.simulate(jet)
        sim.commit(phi)
        scene = sim.model.surface_scene(phi, sim.grid.origin, sim.grid.spacing)
        heights.append(float(surface_rays(scene, np.array([0.]), np.array([0.]))[0, 0]))
    assert heights[0] < -.015
    assert heights[1] < heights[0] - .01


def test_surface_update_keeps_wall_back_and_removes_volume():
    from algorithms.local_spray_simulation import surface_mesh
    coordinates = np.linspace(-.1, .1, 21)
    U, V = np.meshgrid(coordinates, coordinates)
    initial = np.zeros_like(U)
    eroded = -.02*np.exp(-(U**2+V**2)/.002)
    before = surface_mesh(coordinates, coordinates, initial, .4)
    after = surface_mesh(coordinates, coordinates, eroded, .4, initial-.4)
    assert before.is_watertight and after.is_watertight
    np.testing.assert_allclose(after.vertices[initial.size:, 2], -.4)
    assert 0 < before.volume-after.volume < .02*.2*.2


def test_missing_boundary_rays_do_not_create_invalid_mesh(monkeypatch):
    """A measured boundary-ray miss must not turn the closed wall into NaNs."""
    from types import SimpleNamespace
    import trimesh
    from algorithms import local_spray_simulation as local
    coords = np.linspace(-.03, .03, 7)
    sim = local.LocalSpraySimulation.__new__(local.LocalSpraySimulation)
    sim.patch_ = sim.u_ = sim.v_ = coords
    sim.front_ = sim.initial_patch_ = np.zeros((7, 7))
    sim.patch_u_indices_ = sim.patch_v_indices_ = np.arange(7)
    sim.wall_thickness_ = .4
    sim.initial_phi = np.array([0.])
    sim.grid = SimpleNamespace(origin=np.zeros(3), spacing=.01)
    sim.model = SimpleNamespace(surface_scene=lambda *args: None)
    current = np.full((7, 7), -.01)
    current[0, 0] = np.nan
    monkeypatch.setattr(local, 'surface_rays', lambda *args: current)
    vertices, faces = sim.mesh(np.array([1.]))
    assert np.isfinite(vertices).all()
    assert trimesh.Trimesh(vertices=vertices, faces=faces).is_watertight
