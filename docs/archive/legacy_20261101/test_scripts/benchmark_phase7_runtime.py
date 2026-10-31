#!/usr/bin/env python3
"""One non-training step to separate cold/hot reset, erosion and assessment costs."""
import json
from pathlib import Path
import time
from phase7_train_optimized import make_env
from phase7_evaluate import RuleAdapter


def main():
    env=make_env('optimized',123,smoke=True)
    try:
        started=time.monotonic();obs,_=env.reset(seed=123,options={'scenario':'empty'})
        cold=time.monotonic()-started
        started=time.monotonic();obs,_=env.reset(seed=123,options={'scenario':'empty'})
        hot=time.monotonic()-started
        action=RuleAdapter(env).act(obs)
        started=time.monotonic();env.step(action);step=time.monotonic()-started
        result={'cold_reset_s':cold,'hot_reset_s':hot,'step_s':step,'profile':env.profile_,
                'training_performed':False,'scope':'single L1 worker; full multiworker throughput not measured'}
        Path('results/reproduction_20261030/runtime_profile.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
    finally:env.close()


if __name__=='__main__':main()
