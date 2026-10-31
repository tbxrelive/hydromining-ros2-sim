"""Tiny bounded learning tests: replay mixing, Q filter, rates and serialization."""
import sys
from pathlib import Path
import gymnasium as gym
import numpy as np
import pytest
import torch
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from algorithms.planning.ddpg_optimized import DemonstrationReplayBuffer, DemonstrationDDPG


class TinyTask(gym.Env):
    """Analytic continuous task used only for learner mechanics, not mining results."""
    observation_space=gym.spaces.Box(-1.,1.,(4,),dtype=np.float32)
    action_space=gym.spaces.Box(-1.,1.,(3,),dtype=np.float32)

    def reset(self,seed=None,options=None):
        super().reset(seed=seed)
        return np.zeros(4,np.float32),{}

    def step(self,action):
        return np.zeros(4,np.float32),-float(np.sum(action**2)),True,False,{}


def dataset(path):
    np.savez(path,observations=np.zeros((16,4),np.float32),next_observations=np.zeros((16,4),np.float32),
             actions=np.full((16,3),.25,np.float32),rewards=np.ones(16,np.float32),dones=np.ones(16,np.float32))


def test_demonstrations_training_and_resume(tmp_path):
    torch.set_num_threads(1)
    demo=tmp_path/'demo.npz';dataset(demo)
    env=TinyTask()
    model=DemonstrationDDPG('MlpPolicy',env,seed=0,learning_starts=1,buffer_size=32,batch_size=8,
              replay_buffer_class=DemonstrationReplayBuffer,replay_buffer_kwargs={'demo_path':str(demo)},
              policy_kwargs={'net_arch':[8,8]},device='cpu',gradient_steps=-1)
    before=[p.detach().clone() for p in model.actor.parameters()]
    model.pretrain_actor(2,8)
    assert any(not torch.equal(a,b) for a,b in zip(before,model.actor.parameters()))
    model.learn(6)
    assert model._n_updates > 0
    assert model.actor.optimizer.param_groups[0]['lr'] == 1e-4
    assert model.critic.optimizer.param_groups[0]['lr'] == 1e-3
    sampled=model.replay_buffer.sample(8)
    assert sampled.observations.shape==(8,4)
    assert (sampled.rewards==1).sum()>=2
    model.save(str(tmp_path/'model'));model.save_replay_buffer(str(tmp_path/'replay'))
    restored=DemonstrationDDPG.load(str(tmp_path/'model'),env=env,device='cpu')
    restored.load_replay_buffer(str(tmp_path/'replay'))
    np.testing.assert_allclose(model.predict(np.zeros(4,np.float32))[0],restored.predict(np.zeros(4,np.float32))[0])
    restored.learn(2,reset_num_timesteps=False)
    assert restored.num_timesteps==8
    env.close()


def test_demo_shape_and_action_validation(tmp_path):
    path=tmp_path/'bad.npz';dataset(path)
    with pytest.raises(ValueError,match='dimensions'):
        DemonstrationReplayBuffer(32,gym.spaces.Box(-1.,1.,(5,)),TinyTask.action_space,demo_path=str(path),device='cpu')


def test_atomic_cache_roundtrip_and_configuration_invalidation(tmp_path):
    from types import SimpleNamespace
    import trimesh
    from algorithms.planning.scenario_cache import save_entry, load_entry, cache_path
    world=SimpleNamespace(simulation_=SimpleNamespace(grid=SimpleNamespace(source_mesh=trimesh.creation.box(),resolution=(8,8,8)),evolver=SimpleNamespace(dt=.2)))
    env=SimpleNamespace(world_=world,cache_dir_=str(tmp_path),cfg_={'diameter':.2},specs_={})
    entry={'phi':np.ones((8,8,8)), 'pristine':np.zeros((2,2)),
           'surface':(np.zeros((2,2,3)),np.ones((2,2,3))),
           'masks':{'front':{'depth':np.ones((2,2),bool),'lidar':np.zeros((2,2),bool)}}}
    save_entry(env,'empty',entry)
    restored=load_entry(env,'empty')
    np.testing.assert_array_equal(restored['phi'],entry['phi'])
    np.testing.assert_array_equal(restored['masks']['front']['depth'],entry['masks']['front']['depth'])
    old=cache_path(env,'empty');env.cfg_['diameter']=.3
    assert cache_path(env,'empty')!=old and load_entry(env,'empty') is None


@pytest.mark.parametrize('status,expected_truncated',[('achieved',False),('not_achieved',True)])
def test_success_at_budget_is_terminal_not_timeout(status,expected_truncated):
    from types import SimpleNamespace
    from algorithms.planning.rl_env import HydrominingSurrogateEnv
    from algorithms.perception.wall_frame import WallFrame
    cfg={'region_center_world':[0,0,0],'wall_normal_world':[0,0,1]}
    world=SimpleNamespace(spray_tcp=lambda *a:{'volume_removed_m3':.001})
    task=dict(cfg,pressure_mpa=30.,standoff_m=.5,diameter=.4,required_depth=.05,required_fraction=.9)
    env=HydrominingSurrogateEnv(world,{},task,max_steps=1,scenarios={'empty':{'shots':[]}})
    env.frame_=WallFrame.from_normal([0,0,0],[0,0,1])
    before={'status':'not_achieved','fraction_lower':0.,'coverage':1.}
    after={'status':status,'fraction_lower':1. if status=='achieved' else 0.,'coverage':1.}
    surface=SimpleNamespace(centers_=np.zeros((2,2,3)))
    change={'delta':np.zeros((2,2)),'sigma':np.zeros((2,2)),
            'valid':np.ones((2,2),bool),'significant':np.zeros((2,2),bool)}
    env.last_progress_=0.
    env.last_evaluated_={'report':{'assessment':before},'surface':surface,'change':change}
    env._do_evaluate=lambda **kw:{'report':{'assessment':after},'surface':surface,'change':change}
    env._finish=lambda a,r,truncated=False:truncated
    assert env.step(np.zeros(3)) is expected_truncated
