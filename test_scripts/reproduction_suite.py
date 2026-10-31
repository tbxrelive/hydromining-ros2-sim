#!/usr/bin/env python3
"""Sequential source-backed stage matrix with unique evidence directories."""
import argparse
import datetime
import json
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='results/full_reproduction_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--gui',action='store_true',help='Finish with the interactive rule GUI demo')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args();out=Path(args.output).resolve();python=sys.executable
    commands=[
        ('build',['colcon','build','--base-paths','src','--symlink-install','--packages-select','hydromining_robot','hydromining_sim','--executor','sequential']),
        ('unit',[python,'-m','pytest','src/hydromining_sim/test','-q','-p','no:cacheprovider']),
        ('geometry_smooth',[python,'src/hydromining_sim/scripts/generate_coal_wall.py','--resolution','64','--roughness','.05','--seed','42','--output',str(out/'phase1/smooth.obj')]),
        ('geometry_rough',[python,'src/hydromining_sim/scripts/generate_coal_wall.py','--resolution','64','--roughness','.2','--seed','43','--output',str(out/'phase1/rough.obj')]),
        ('phase1',[python,'test_scripts/reproduce_geometry.py','--output',str(out/'phase1')]),
        ('phase3',['scripts/run_phase6_record.sh','--scenarios','none','--viewpoints','front_0p8','left_0p8','right_0p8','--frames-per-view','3','--output',str(out/'phase3')]),
        ('phase3_audit',[python,'test_scripts/audit_phase6_dataset.py','--dataset',str(out/'phase3/dataset'),'--output',str(out/'phase3_audit.json')]),
        ('multiview',[python,'test_scripts/audit_multiview_registration.py','--dataset',str(out/'phase3/dataset'),'--output',str(out/'multiview_registration.json')]),
        ('phase4',['scripts/run_phase4_acceptance.sh','--output',str(out/'phase4')]),
        ('phase5',['scripts/run_phase5_acceptance.sh','--output',str(out/'phase5')]),
        ('phase6_clean',['scripts/run_phase6_acceptance.sh','--output',str(out/'phase6_clean')]),
        ('phase6_noise',['scripts/run_phase6_acceptance.sh','--depth-noise-stddev','.01','--lidar-noise-stddev','.02','--noise-mode','--output',str(out/'phase6_noise')]),
        ('tcp',[python,'test_scripts/test_phase7_spray_tcp.py','--output',str(out/'tcp')]),
        ('phase7',['scripts/run_phase7_batch.sh',str(out/'phase7')])]
    if args.gui:
        commands.append(('gui',['scripts/run_gui_demo.sh','--output',str(out/'gui')]))
    if args.dry_run:
        print(json.dumps({'commands':commands,'long_training':False},indent=2));return
    if out.exists() and any(out.iterdir()):parser.error('output must be new or empty')
    (out/'phase1').mkdir(parents=True,exist_ok=True)
    progress={}
    for name,command in commands:
        print('START',name,flush=True);started=time.monotonic()
        with (out/f'{name}.log').open('w') as stream:
            code=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT).returncode
        progress[name]={'returncode':code,'elapsed_s':time.monotonic()-started}
        (out/'progress.json').write_text(json.dumps(progress,indent=2))
        if code:raise SystemExit(f'{name} failed ({code}); see {out/name}.log')
        print('PASSED',name,flush=True)
    (out/'summary.json').write_text(json.dumps({'passed':True,'stages':progress,'long_training':False},indent=2))


if __name__=='__main__':main()
