#!/usr/bin/env python3
"""Summarize actual artifacts; incomplete evidence never becomes a passed stage."""
import argparse
from collections import Counter
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def read(path):
    return json.loads(path.read_text()) if path.is_file() else None


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,default=Path('results/reproduction_20261030'))
    args=parser.parse_args();root=args.root
    files={'geometry':'phase1/summary.json','observations':'phase3/summary.json',
           'observation_audit':'phase3_audit.json','multiview':'multiview_registration.json',
           'spray':'phase4/summary.json','effects':'phase5/summary.json',
           'perception_clean':'phase6_clean/summary.json','perception_noise':'phase6_noise/summary.json',
           'tcp':'tcp/summary.json','cuda':'cuda_three_gpu.json','learner_smoke':'learner_release_smoke/result.json',
           'deployment_inference':'model_deployment_smoke.json','gui':'gui_rule/summary.json'}
    evidence={key:read(root/path) for key,path in files.items()}
    statuses={key:bool(data and data.get('passed')) for key,data in evidence.items()}
    cases={};effects=[]
    for name in ['empty','single','pair','tri_r5','ring_r6','dense']:
        data=read(root/'phase7'/name/'summary.json')
        if data is None:cases[name]={'passed':False,'missing':True};continue
        cases[name]={'passed':data.get('passed',False),'steps':data['steps'],
                     'verdict':data['verdict'],'truth':data.get('truth'),
                     'actions':dict(Counter(e['type'] for e in data['log'])),
                     'final_assessment':data['log'][-1]['assessment'],
                     'peak_rss_mib':data.get('aggregate_peak_rss_mib')}
        effects.extend({'scenario':name,'step':e['step'],**e['observation_effect']} for e in data['log'] if 'observation_effect' in e)
    tests=root/'final_unit_tests.xml'
    unit=None
    if tests.is_file():
        suites=ET.parse(tests).getroot();s=suites if suites.tag=='testsuite' else suites.find('testsuite')
        unit={k:int(s.attrib.get(k,0)) for k in ['tests','failures','errors','skipped']}
    complete=all(statuses.values()) and all(d['passed'] for d in cases.values()) and unit is not None and unit['failures']==unit['errors']==0
    result={'passed':complete,'evidence_status':statuses,'phase7':cases,'unit_tests':unit,
            'nbv_observation_effects':effects,'long_training_performed':False,
            'rl_performance_improvement_verified':False,
            'research_limits':['uncalibrated erosion parameters','uncertainty calibration / dark and heavy interference matrix not fully met','no independent MoveIt collision-planning validation']}
    (root/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    (root/'nbv_effects.json').write_text(json.dumps(effects,ensure_ascii=False,indent=2))
    print(json.dumps({'passed':complete,'evidence_status':statuses,'cases':{k:v['passed'] for k,v in cases.items()}},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
