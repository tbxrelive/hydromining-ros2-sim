#!/usr/bin/env python3
"""Regression for Ctrl+C after six steps, using the real headless keep-open path."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import psutil


def main():
    root=Path(__file__).resolve().parents[1]
    output=root/'results/phase4_sigint_regression'
    output.mkdir(parents=True,exist_ok=True)
    log_path=output/'console.log'
    command=[str(root/'scripts/run_phase4_acceptance.sh'),'--keep-open','--output',str(output)]
    with log_path.open('w') as log:
        process=subprocess.Popen(command,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        owned=[]
        try:
            deadline=time.monotonic()+240
            while time.monotonic()<deadline:
                if process.poll() is not None:
                    raise RuntimeError('Acceptance exited before the keep-open state; see '+str(log_path))
                if 'Headless simulation remains open for inspection' in log_path.read_text():
                    break
                time.sleep(.2)
            else:
                raise TimeoutError('Six-step acceptance did not reach keep-open within 240 seconds')
            owned=psutil.Process(process.pid).children(recursive=True)
            process.send_signal(signal.SIGINT)
            returncode=process.wait(timeout=35)
            text=log_path.read_text()
            assert returncode==0,(returncode,text[-2000:])
            assert 'rcl_shutdown already called' not in text
            assert 'Traceback' not in text
            result=json.loads((output/'summary.json').read_text())
            assert result['passed'] and result['mode']=='headless'
            assert result['owned_processes_remaining']==[] and result['cleanup_errors']==[]
            remaining=[]
            for child in owned:
                try:
                    if child.is_running() and child.status()!=psutil.STATUS_ZOMBIE:
                        remaining.append(child.pid)
                except psutil.NoSuchProcess:
                    pass
            assert not remaining,remaining
            evidence={'passed':True,'GUI_started':False,'interrupted_after_six_steps':True,
                      'exit_code':returncode,'double_shutdown_error':False,
                      'remaining_owned_processes':remaining,'aggregate_peak_rss_mib':result['aggregate_peak_rss_mib']}
            (output/'sigint_regression.json').write_text(json.dumps(evidence,indent=2))
            print(json.dumps(evidence,indent=2))
            return 0
        finally:
            if process.poll() is None:
                owned.extend(psutil.Process(process.pid).children(recursive=True))
                process.terminate()
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
                for child in owned:
                    try:child.terminate()
                    except psutil.NoSuchProcess:pass


if __name__=='__main__':
    sys.exit(main())
