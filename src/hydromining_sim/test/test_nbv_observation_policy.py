"""Analytical FOV/unknown rejection and versioned observation contracts."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.planning.nbv import observed_visibility, choose_view
from algorithms.planning.observations import encode_observation, potential_reward
from algorithms.perception.wall_frame import WallFrame
from algorithms.perception.config import PerceptionConfig


def test_measured_visibility_rejects_unknown_and_outside_camera():
    cfg = PerceptionConfig()
    frame = WallFrame.from_normal([0, 0, 0], [0, 0, 1])
    heights = np.zeros((50, 50)); heights[25, 25] = np.nan
    camera = np.eye(4); camera[:3, :3] = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]
    camera[:3, 3] = [0, 0, .8]
    # Lidar at +Z sees no plane points in its horizontal elevation band.
    spec = dict(tf_depth=camera, tf_lidar=np.diag([1,1,1,1.]),
                K=np.array([[200.,0,160],[0,200.,120],[0,0,1]]), image_shape=(240,320))
    spec['tf_lidar'][:3,3] = [0,0,.8]
    state = {'height': heights, 'coverage': np.ones((50,50)), 'sigma': np.full((50,50),.02)}
    mask = observed_visibility(state, frame, {'front':spec}, cfg)['front']
    assert mask.sum() > 2000 and not mask[25,25]
    reverse = {**spec, 'tf_depth':camera.copy()}
    reverse['tf_depth'][:3,:3] = camera[:3,:3] @ np.diag([-1,-1,1])
    assert not observed_visibility(state, frame, {'back':reverse}, cfg)['back'].any()
    best, scores = choose_view(state, frame, {'front':camera[:3,3]}, {'front':mask}, cfg)
    assert best == 'front' and np.isfinite(scores['front']['gain'])


def test_policy_versions_finite_missing_and_terminal_potential():
    cfg = PerceptionConfig(); frame = WallFrame.from_normal([0,0,0],[0,0,1])
    state = {'coverage':np.zeros((50,50))}
    change = {'delta':np.full((50,50),np.nan),'sigma':np.full((50,50),np.nan),'significant':np.zeros((50,50))}
    assessment = dict(fraction_lower=0.,fraction_upper=1.,coverage=0.)
    obs = encode_observation(state,change,assessment,frame.grid(.5,.01),cfg.required_depth,
                             diameter=cfg.diameter, pressure=30., standoff=.5)
    assert obs.shape == (3756,) and np.isfinite(obs).all()
    # True terminal potential is zero; truncation retains the continuing value.
    assert potential_reward(.2,.9,terminal=True) == -1.6
    assert potential_reward(.2,.9,terminal=False) > 0


def test_observed_ridge_occludes_distant_surface():
    cfg=PerceptionConfig();frame=WallFrame.from_normal([0,0,0],[0,0,1])
    heights=np.zeros((50,50));heights[:,20:23]=.12
    camera=np.eye(4);camera[:3,3]=[-.35,0,.08]
    spec=dict(tf_depth=camera,tf_lidar=camera.copy(),
              K=np.array([[180.,0,160],[0,180.,120],[0,0,1]]),image_shape=(240,320))
    blocked=observed_visibility({'height':heights},frame,{'view':spec},cfg)['view']
    clear=observed_visibility({'height':np.zeros_like(heights)},frame,{'view':spec},cfg)['view']
    assert clear[25,40] and not blocked[25,40]
    assert blocked[25,10]


def test_nbv_does_not_predict_removal_of_systematic_floor():
    cfg=PerceptionConfig();frame=WallFrame.from_normal([0,0,0],[0,0,1])
    state={'coverage':np.ones((50,50)),'sigma':np.full((50,50),cfg.systematic_sigma)}
    best,scores=choose_view(state,frame,{'front':[0,0,.6]},{'front':np.ones((50,50),bool)},cfg)
    assert best is None and scores['front']['gain']==0.
