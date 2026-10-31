#!/usr/bin/env python3
"""User-run three-GPU scheduler with a shared RAM budget and owned-process cleanup."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import psutil
from runtime_resources import memory_limits


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='results/phase7_training')
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--task',choices=('legacy','large'),default='large')
    parser.add_argument('--timesteps',type=int,default=50000)
    parser.add_argument('--initial-steps',type=int,default=15000)
    parser.add_argument('--seeds',type=int,default=3)
    parser.add_argument('--num-envs',type=int,default=2)
    parser.add_argument('--eval-episodes',type=int,default=12)
    parser.add_argument('--reference-episodes',type=int,default=None,
                        help='L1/L2 cases per run; default matches --eval-episodes')
    parser.add_argument('--fidelity-report',default='results/phase7_large_task/fidelity_release_v2/fidelity.json')
    args=parser.parse_args()
    out=Path(args.output).resolve()
    # Do not resolve the interpreter symlink: Python identifies its venv from this path.
    python=str(Path('.venv_rl/bin/python').absolute())
    jobs={gpu:[] for gpu in (1,2,3)}
    if args.task == 'large':
        def evaluation_done(destination, variant, seed, fidelity, episodes):
            path = destination/f'evaluation_{fidelity}.json'
            if not path.exists() or not (destination/'metadata.json').exists():
                return False
            result = json.loads(path.read_text())
            metadata = json.loads((destination/'metadata.json').read_text())
            return (result['fidelity'] == fidelity and result['algorithm'] == variant and
                    result['training_seed'] == seed and result['trained_steps'] == metadata['num_timesteps'] and
                    metadata['num_timesteps'] >= args.timesteps and
                    len(result[variant]['episodes']) == episodes and
                    (variant != 'ddpg' or seed != 0 or all(k in result for k in ('random','rule'))))
        for gpu, variants in ((1, ('ddpg','ddpg_demo')), (2, ('td3',)), (3, ('sac',))):
            for variant in variants:
                for seed in range(args.seeds):
                    destination=out/variant/f'seed_{seed}'
                    common=[python,'test_scripts/phase7_train_large.py','--algorithm',variant,'--seed',str(seed),
                            '--assets',str(out/'assets'),'--output',str(destination),
                            '--num-envs',str(args.num_envs),'--device','cuda:0',
                            '--schedule-steps',str(args.timesteps),'--fidelity-report',str(Path(args.fidelity_report).resolve()),
                            *(['--demo-path',str(out/'demonstrations.npz')] if variant=='ddpg_demo' else [])]
                    existing = (json.loads((destination/'metadata.json').read_text())
                                if (destination/'metadata.json').exists() else None)
                    steps = existing['num_timesteps'] if existing else 0
                    if steps < min(args.initial_steps, args.timesteps):
                        first=common+['--timesteps',str(min(args.initial_steps,args.timesteps))]
                        if existing:first.append('--resume')
                        jobs[gpu].append(first)
                    if args.timesteps > args.initial_steps and steps < args.timesteps:
                        jobs[gpu].append(common+['--timesteps',str(args.timesteps),'--noise-scale','1','--resume'])
                    evaluation=[python,'test_scripts/phase7_evaluate_large.py','--checkpoint',str(destination),
                                '--assets',str(out/'assets'),'--output',str(destination/'evaluation_L0.json'),
                                '--episodes',str(args.eval_episodes)]
                    if variant=='ddpg' and seed==0:evaluation.append('--include-reference')
                    if not evaluation_done(destination, variant, seed, 'L0', args.eval_episodes):
                        jobs[gpu].append(evaluation)
        # Reference solvers run after this GPU's learners, keeping training busy first.
        for gpu, variants in ((1, ('ddpg','ddpg_demo')), (2, ('td3',)), (3, ('sac',))):
            for variant in variants:
                for seed in range(args.seeds):
                    destination=out/variant/f'seed_{seed}'
                    for fidelity in ('L1','L2'):
                        evaluation=[python,'test_scripts/phase7_evaluate_large.py','--checkpoint',str(destination),
                                    '--assets',str(out/'assets'),'--output',str(destination/f'evaluation_{fidelity}.json'),
                                    '--episodes',str(args.eval_episodes if args.reference_episodes is None else args.reference_episodes),
                                    '--fidelity',fidelity]
                        if variant=='ddpg' and seed==0:evaluation.append('--include-reference')
                        episodes = args.eval_episodes if args.reference_episodes is None else args.reference_episodes
                        if not evaluation_done(destination, variant, seed, fidelity, episodes):
                            jobs[gpu].append(evaluation)
    else:
        for gpu,variant in zip(jobs,('baseline','optimized','demo')):
            for seed in range(3):
                destination=out/variant/f'seed_{seed}'
                command=[python,'test_scripts/phase7_train_optimized.py','--variant',variant,'--seed',str(seed),
                         '--output',str(destination),'--timesteps','15000','--num-envs','2','--device','cuda:0',
                         *(['--demo-path',str(out/'demonstrations.npz')] if variant=='demo' else [])]
                if (destination/'metadata.json').exists():command.append('--resume')
                jobs[gpu].append(command)
                for fidelity in ('L1','L2'):
                    jobs[gpu].append([python,'test_scripts/phase7_evaluate.py','--checkpoint',str(destination),
                                      '--seed',str(10000+seed),'--fidelity',fidelity,
                                      '--output',str(destination/f'evaluation_{fidelity}')])
    if args.dry_run:
        print(json.dumps({'display_gpu_reserved':0,'task':args.task,'jobs':jobs},indent=2));return
    if args.task == 'large':
        from phase7_large_env import fidelity_signature, prepare_meshes
        report=json.loads(Path(args.fidelity_report).read_text())
        if (not report['passed'] or report.get('scope')!='full case set' or
                report.get('signature')!=fidelity_signature()):
            raise RuntimeError('L0 fidelity gate not passed; run the full fidelity benchmark first')
        prepare_meshes(out/'assets')
    display = subprocess.run(['nvidia-smi','--query-gpu=index,display_active','--format=csv,noheader'],capture_output=True,text=True,check=True).stdout
    for line in display.splitlines():
        index, active_display = [value.strip() for value in line.split(',')]
        if int(index) in jobs and active_display == 'Enabled':
            raise RuntimeError(f'GPU{index} drives a display; refusing to train on it')
    out.mkdir(parents=True,exist_ok=True)
    os.sched_setaffinity(0,os.sched_getaffinity(0)-{0,6})
    active={};logs={};owned={};interrupted=False
    def stop(*_):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        if not (out/'demonstrations.npz').exists():
            # Same ownership/monitoring loop also covers demonstration collection.
            collect=[python,'test_scripts/phase7_train_large.py' if args.task == 'large' else
                     'test_scripts/phase7_train_optimized.py','--collect-demo',
                     '--output',str(out/'demonstrations.npz'),'--seed','100']
            if args.task == 'large':collect.extend(['--assets',str(out/'assets')])
            jobs[1].insert(0,collect)
            collecting=True
        else:collecting=False
        while any(jobs.values()) or active:
            if interrupted:raise KeyboardInterrupt
            for gpu in jobs:
                if gpu not in active and jobs[gpu] and (not collecting or gpu==1):
                    command=jobs[gpu].pop(0)
                    env={**os.environ,'CUDA_VISIBLE_DEVICES':str(gpu)}
                    logs[gpu]=(out/f'gpu{gpu}.log').open('a')
                    process=subprocess.Popen(command,env=env,stdout=logs[gpu],stderr=subprocess.STDOUT,start_new_session=True)
                    active[gpu]=process
                    owned[process.pid]=psutil.Process(process.pid)
            rss=0
            for root in list(owned.values()):
                try:
                    for child in root.children(recursive=True):owned[child.pid]=child
                except psutil.NoSuchProcess:pass
            for process in owned.values():
                try:rss+=process.memory_info().rss
                except psutil.NoSuchProcess:pass
            available=psutil.virtual_memory().available
            gpu_usage=subprocess.run(['nvidia-smi','--query-gpu=index,memory.used,utilization.gpu','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=5).stdout
            with (out/'resources.jsonl').open('a') as stream:
                stream.write(json.dumps({'time':time.time(),'aggregate_rss_mib':rss/1024**2,'available_mib':available/1024**2,'gpu_usage':gpu_usage})+'\n')
            limit,minimum=memory_limits()
            if rss>limit or available<minimum:raise RuntimeError('Shared training RAM guard reached; see resources.jsonl')
            for gpu,process in list(active.items()):
                code=process.poll()
                if code is not None:
                    logs[gpu].close();del active[gpu]
                    if code:raise RuntimeError(f'GPU{gpu} job exited {code}; inspect gpu{gpu}.log')
                    if collecting:collecting=False
            time.sleep(2)
        aggregate = {}
        if args.task == 'large':
            import numpy as np
            for variant in ('ddpg','ddpg_demo','td3','sac'):
                aggregate[variant]={}
                for fidelity in ('L0','L1','L2'):
                    evaluations=[json.loads((out/variant/f'seed_{seed}'/f'evaluation_{fidelity}.json').read_text())
                                 for seed in range(args.seeds)]
                    runs=[result[variant] for result in evaluations]
                    rates=[r['summary']['needs_work']['success_rate'] for r in runs
                           if r['summary']['needs_work']['success_rate'] is not None]
                    aggregate[variant][fidelity]={'runs':runs, 'success_rate_mean':float(np.mean(rates)) if rates else None,
                        'success_rate_std':float(np.std(rates)) if rates else None,
                        'false_achieved':sum(r['summary']['all']['false_achieved'] for r in runs)}
                    groups = {}
                    for group in ('all', 'needs_work', 'initially_achieved'):
                        groups[group] = {}
                        for metric in ('success_rate', 'false_achieved', 'truth_unknown', 'mean_steps', 'mean_sprays',
                                       'mean_spray_duration_s', 'mean_removed_volume_m3', 'mean_water_m3',
                                       'mean_energy_j', 'mean_wall_time_s'):
                            values = [run['summary'][group].get(metric, 0 if metric == 'truth_unknown' else None)
                                      for run in runs if run['summary'][group].get(
                                          metric, 0 if metric == 'truth_unknown' else None) is not None]
                            groups[group][metric] = {'mean': float(np.mean(values)) if values else None,
                                                     'std': float(np.std(values)) if values else None}
                    training_times = [result['training_wall_s'] for result in evaluations]
                    aggregate[variant][fidelity].update(groups=groups,
                        training_wall_s={'mean':float(np.mean(training_times)), 'std':float(np.std(training_times))})
                aggregate[variant]['fidelity_gap'] = {
                    level: {group: {
                        metric: (aggregate[variant]['L0']['groups'][group][metric]['mean'] -
                                 aggregate[variant][level]['groups'][group][metric]['mean'])
                        if (aggregate[variant]['L0']['groups'][group][metric]['mean'] is not None and
                            aggregate[variant][level]['groups'][group][metric]['mean'] is not None) else None
                        for metric in ('success_rate', 'mean_sprays', 'mean_spray_duration_s')}
                        for group in ('needs_work', 'initially_achieved')}
                    for level in ('L1', 'L2')}
            for fidelity in ('L0','L1','L2'):
                baseline=json.loads((out/f'ddpg/seed_0/evaluation_{fidelity}.json').read_text())
                for key in ('random','rule'):aggregate.setdefault(key,{})[fidelity]=baseline[key]
        else:
            for variant in ('baseline','optimized','demo'):
                aggregate[variant] = {}
                for fidelity in ('L1','L2'):
                    runs = [json.loads((out/variant/f'seed_{seed}'/f'evaluation_{fidelity}'/'evaluation.json').read_text())['ddpg'] for seed in range(3)]
                    aggregate[variant][fidelity] = {'runs':runs,'seeds':[0,1,2],
                        'success_rate':sum(r['verdicts'].get('achieved',0) for r in runs)/sum(len(r['steps']) for r in runs),
                        'false_achieved':sum(r['false_achieved'] for r in runs)}
        (out/'comparison.json').write_text(json.dumps(aggregate,indent=2))
    finally:
        for process in active.values():
            try:os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and any(p.poll() is None for p in active.values()):time.sleep(.1)
        for process in active.values():
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
            process.wait()
        for log in logs.values():log.close()


if __name__=='__main__':main()
