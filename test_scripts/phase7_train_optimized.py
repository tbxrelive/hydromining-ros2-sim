#!/usr/bin/env python3
"""User-run DDPG experiments. --smoke is hard bounded; no automatic long training."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
import numpy as np
import yaml
import torch
from stable_baselines3 import DDPG
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.vec_env import SubprocVecEnv
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/hydromining_sim/scripts'))
from phase7_surrogate_loop import load_specs
from phase7_evaluate import RuleAdapter
from algorithms.planning.surrogate import SurrogateWorld
from algorithms.planning.rl_env import HydrominingSurrogateEnv, VIEW_NAMES
from algorithms.planning.ddpg_optimized import DemonstrationDDPG, DemonstrationReplayBuffer


def make_env(variant, seed, fidelity='L1', cache_dir='.cache/phase7', smoke=False, noise_scale=0.):
    cfg = yaml.safe_load(Path('src/hydromining_robot/config/viewpoints.yaml').read_text())
    resolution = (64,64,40) if fidelity == 'L1' else (96,96,64)
    return HydrominingSurrogateEnv(
        SurrogateWorld('src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj', resolution),
        load_specs(VIEW_NAMES), cfg, fidelity=fidelity, noise_scale=noise_scale, seed=seed,
        action_mode='spray', observation_version='v1' if variant == 'baseline' else 'v2',
        reward_mode='legacy' if variant == 'baseline' else 'potential', cache_dir=cache_dir,
        scenario_names=['empty'] if smoke else None)


def collect_demonstrations(path, episodes=24, seed=100):
    """Training-only rule rollouts; exclude initially completed scenarios."""
    env = make_env('demo', seed)
    rule = RuleAdapter(env)
    arrays = {k:[] for k in ('observations','next_observations','actions','rewards','dones')}
    scenarios = ['empty','single','pair','tri_r5']
    try:
        for episode in range(episodes):
            obs, _ = env.reset(seed=seed+episode, options={'scenario':scenarios[episode%4]})
            while True:
                action = rule.act(obs)
                next_obs, reward, terminated, truncated, _ = env.step(action)
                for key, value in zip(arrays, (obs,next_obs,action,reward,float(terminated))):
                    arrays[key].append(value)
                obs = next_obs
                if terminated or truncated:
                    break
        path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, **{k:np.asarray(v,dtype=np.float32) for k,v in arrays.items()})
        path.with_suffix('.json').write_text(json.dumps({'split':'train','seed':seed,'episodes':episodes,'observation_version':'v2','samples':len(arrays['actions'])},indent=2))
    finally:
        env.close()


class CheckpointCallback(BaseCallback):
    """Persist learner/replay/RNG; interrupted rollouts restart at episode boundaries."""

    def __init__(self, output, config, initial_noise=.2, final_noise=.05):
        super().__init__()
        self.output, self.config = Path(output), config
        self.initial_noise, self.final_noise = initial_noise, final_noise
        self.last_saved = 0
        self.last_logged = 0
        self.started = time.monotonic()
        self.prior_wall_s = config.get('prior_wall_s', 0.)

    def save(self):
        self.output.mkdir(parents=True, exist_ok=True)
        snapshots = self.output/'snapshots'; snapshots.mkdir(exist_ok=True)
        generation = f'step_{self.model.num_timesteps}_{time.time_ns()}'
        temporary = snapshots/('.'+generation); temporary.mkdir()
        self.model.save(str(temporary/'checkpoint'))
        self.model.save_replay_buffer(str(temporary/'replay.pkl'))
        torch.save({'numpy':np.random.get_state(),'torch':torch.get_rng_state(),
                    'cuda':torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None}, temporary/'rng.pt')
        metadata = {**self.config,'num_timesteps':self.model.num_timesteps,
                    'wall_s':self.prior_wall_s+time.monotonic()-self.started,
                    'resume_semantics':'learner/replay/RNG restored; environments restart episodes'}
        (temporary/'metadata.json').write_text(json.dumps(metadata,indent=2))
        destination = snapshots/generation
        os.replace(temporary,destination)
        pending = self.output/'.latest_pending'
        pending.unlink(missing_ok=True)
        pending.symlink_to(Path('snapshots')/generation)
        os.replace(pending,self.output/'latest')
        for name in ('checkpoint.zip','replay.pkl','rng.pt','metadata.json'):
            alias = self.output/name
            if not alias.is_symlink():
                alias.unlink(missing_ok=True)
                alias.symlink_to(Path('latest')/name)
        # Retain two complete generations instead of duplicating GB replay arrays.
        import shutil
        generations = sorted((p for p in snapshots.glob('step_*') if p.is_dir()), key=lambda p:p.stat().st_mtime)
        retained = set(generations[-2:])
        for step in self.config.get('retain_steps', []):
            milestone = [p for p in generations if p.name.startswith(f'step_{step}_')]
            if milestone:
                retained.add(milestone[-1])
        for old in generations:
            if old not in retained:
                shutil.rmtree(old)

    def _on_step(self):
        progress = min(self.model.num_timesteps/max(self.config['timesteps']*.7,1),1.)
        sigma = self.initial_noise + progress*(self.final_noise-self.initial_noise)
        noise = self.model.action_noise
        for obj in getattr(noise,'noises',[noise]):
            if obj is not None and hasattr(obj,'_sigma'):
                obj._sigma[:] = sigma
        if self.model.num_timesteps-self.last_logged >= 100 or self.config['smoke']:
            record = {'timesteps':self.model.num_timesteps,'updates':self.model._n_updates,
                      'wall_s':self.prior_wall_s+time.monotonic()-self.started}
            for key,value in self.model.logger.name_to_value.items():
                if key.startswith('train/') and np.isscalar(value):
                    if not np.isfinite(value):
                        raise FloatingPointError(f'Nonfinite learner metric {key}; resume from last valid checkpoint')
                    record[key] = float(value)
            with (self.output/'training_metrics.jsonl').open('a') as stream:
                stream.write(json.dumps(record)+'\n')
            self.last_logged = self.model.num_timesteps
        if self.model.num_timesteps-self.last_saved >= 1000:
            self.save(); self.last_saved = self.model.num_timesteps
        if self.config['smoke'] and (self.model.num_timesteps >= min(self.config['timesteps'],100) or time.monotonic()-self.started > 110):
            return False
        return True


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--variant',choices=['baseline','optimized','demo'],default='demo')
    parser.add_argument('--output',required=True)
    parser.add_argument('--timesteps',type=int,default=15000)
    parser.add_argument('--num-envs',type=int,default=2)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--demo-path',default=None)
    parser.add_argument('--collect-demo',action='store_true')
    parser.add_argument('--demo-episodes',type=int,default=24)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--bc-updates',type=int,default=1000)
    parser.add_argument('--demo-fraction',type=float,default=.25)
    parser.add_argument('--bc-weight',type=float,default=.1)
    parser.add_argument('--q-filter',action=argparse.BooleanOptionalAction,default=True)
    args=parser.parse_args()
    if args.collect_demo:
        collect_demonstrations(args.output,args.demo_episodes,args.seed); return
    if args.smoke:
        if args.timesteps > 100 or args.bc_updates > 100:
            parser.error('smoke requires timesteps and BC updates <=100')
        signal.signal(signal.SIGALRM,lambda *_: (_ for _ in ()).throw(TimeoutError('120s smoke limit')))
        signal.alarm(120)
    if args.variant == 'demo' and not args.demo_path:
        parser.error('demo variant requires --demo-path from training split')
    torch.set_num_threads(1)
    if args.device.startswith('cuda'):
        if not torch.cuda.is_available(): raise RuntimeError('CUDA unavailable; use .venv_rl')
        torch.cuda.set_per_process_memory_fraction(.6,torch.device(args.device))
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    config=vars(args).copy()
    config['observation_version']='v1' if args.variant=='baseline' else 'v2'
    config['fidelity']='L1'
    import hashlib, stable_baselines3
    config['software']={'torch':torch.__version__,'sb3':stable_baselines3.__version__}
    config['source_sha256']=hashlib.sha256(b''.join(p.read_bytes() for p in sorted(Path('src/hydromining_sim/scripts/algorithms').rglob('*.py')))).hexdigest()
    config['physical_gpu_mask']=os.environ.get('CUDA_VISIBLE_DEVICES','unset')
    if args.demo_path:
        config['demo_sha256']=hashlib.sha256(Path(args.demo_path).read_bytes()).hexdigest()
    if args.demo_path:
        sidecar=Path(args.demo_path).with_suffix('.json')
        if json.loads(sidecar.read_text()).get('split')!='train': raise ValueError('demonstrations must use training split')
    env=None; model=None
    try:
        factories=[lambda i=i: make_env(args.variant,args.seed+i,smoke=args.smoke) for i in range(args.num_envs)]
        env=SubprocVecEnv(factories,start_method='spawn') if args.num_envs>1 else factories[0]()
        cls=DDPG if args.variant=='baseline' else DemonstrationDDPG
        noise=NormalActionNoise(np.zeros(3),.2*np.ones(3))
        if args.resume:
            old=json.loads((output/'metadata.json').read_text())
            for key in ('variant','observation_version','num_envs','seed','source_sha256','demo_sha256','smoke','bc_weight','demo_fraction','q_filter'):
                if key == 'demo_sha256' and key not in config and key not in old:
                    continue
                if old[key]!=config[key]: raise ValueError(f'resume {key} mismatch')
            model=cls.load(str(output/'checkpoint'),env=env,device=args.device)
            model.load_replay_buffer(str(output/'replay.pkl'))
            rng=torch.load(output/'rng.pt',weights_only=False)
            np.random.set_state(rng['numpy']);torch.set_rng_state(rng['torch'])
            if rng['cuda'] is not None and torch.cuda.is_available():torch.cuda.set_rng_state_all(rng['cuda'])
        else:
            kwargs=dict(seed=args.seed,device=args.device,buffer_size=128 if args.smoke else 50000,
                        learning_starts=2 if args.smoke else 500,batch_size=16 if args.smoke else (128 if args.variant=='baseline' else 256),
                        tau=.005,gamma=.95 if args.variant=='baseline' else .99,
                        train_freq=(1,'step'),gradient_steps=1 if args.variant=='baseline' else -1,
                        action_noise=noise,policy_kwargs={'net_arch':[256,256]},verbose=1)
            if cls is DemonstrationDDPG:
                kwargs.update(actor_lr=1e-4,critic_lr=1e-3,bc_weight=args.bc_weight if args.variant=='demo' else 0.,q_filter=args.q_filter,
                              replay_buffer_class=DemonstrationReplayBuffer,
                              replay_buffer_kwargs={'demo_path':args.demo_path if args.variant=='demo' else None,'demo_fraction':args.demo_fraction if args.variant=='demo' else 0.})
            model=cls('MlpPolicy',env,**kwargs)
            if args.variant=='demo':model.pretrain_actor(args.bc_updates,16 if args.smoke else 256)
        callback=CheckpointCallback(output,config)
        remaining=max(args.timesteps-model.num_timesteps,0)
        if remaining:model.learn(remaining,callback=callback,reset_num_timesteps=not args.resume)
        callback.model=model;callback.save()
        (output/'result.json').write_text(json.dumps({'passed':True,'timesteps':model.num_timesteps,'updates':model._n_updates,'smoke':args.smoke,'performance_evaluated':False},indent=2))
    finally:
        if env is not None:env.close()
        if args.smoke:signal.alarm(0)


if __name__=='__main__':
    main()
