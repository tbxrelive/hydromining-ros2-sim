#!/usr/bin/env python3
"""Interrupt real native particles mid-spray and audit only owned descendants."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import psutil


def alive(process):
    """psutil identities protect against PID reuse."""
    try:
        return process.is_running() and process.status()!=psutil.STATUS_ZOMBIE
    except psutil.NoSuchProcess:
        return False


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='/tmp/hydromining_phase5_sigint')
    parser.add_argument('--phase',choices=('PREPARING','SPRAYING','UPDATING','COMPLETE'),default='SPRAYING')
    parser.add_argument('--overview',action=argparse.BooleanOptionalAction,default=None)
    args=parser.parse_args()
    output=Path(args.output).resolve()
    output.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    command=[str(root/'scripts/run_phase5_acceptance.sh'),'--output',str(output/'session')]
    if args.overview is not None:
        command.append('--overview' if args.overview else '--no-overview')
    if args.phase=='COMPLETE':
        command.append('--keep-open')
    log_path=output/'console.log'
    owned=[]
    with log_path.open('w') as log:
        process=subprocess.Popen(command,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+(480 if args.phase=='COMPLETE' else 130)
            while time.monotonic()<deadline:
                if process.poll() is not None:
                    raise RuntimeError('Acceptance exited before spray; see '+str(log_path))
                native=output/'session/simulation.log'
                reached=('Acceptance finished; Ctrl+C' in log_path.read_text()) if args.phase=='COMPLETE' else (
                    native.exists() and ('Spray state: '+args.phase) in native.read_text())
                if reached:
                    break
                time.sleep(.1)
            else:
                raise TimeoutError('Never entered '+args.phase)
            # Wait for particle creation to have reached the renderer.
            time.sleep(.5)
            owned=psutil.Process(process.pid).children(recursive=True)
            process.send_signal(signal.SIGINT)
            code=process.wait(timeout=35)
            text=log_path.read_text()
            native=(output/'session/simulation.log').read_text()
            remaining=[child.pid for child in owned if alive(child)]
            expected=0 if args.phase=='COMPLETE' else 130
            assert code==expected,(code,text[-2000:])
            assert not remaining,remaining
            assert 'Traceback' not in text and 'rcl_shutdown already called' not in text,text[-2000:]
            assert 'Traceback' not in native and 'exit code -11' not in native and 'exit code -6' not in native,native[-3000:]
            assert 'exception was never retrieved' not in native,native[-3000:]
            assert not any('process has died' in line and 'spray_controller_node.py' in line
                           for line in native.splitlines()),native[-3000:]
            result={'passed':True,'GUI_started':False,'interrupted_phase':args.phase,
                    'interrupted_during_spray':args.phase=='SPRAYING',
                    'expected_exit_code':expected,'actual_exit_code':code,
                    'remaining_owned_processes':remaining,'double_shutdown_error':False,
                    'spray_node_crashed':False}
            if args.phase=='COMPLETE':
                acceptance=json.loads((output/'session/summary.json').read_text())
                assert acceptance['passed'],acceptance
                result['full_acceptance_passed']=True
            (output/'summary.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(result,indent=2))
            return 0
        finally:
            if process.poll() is None:
                owned.extend(psutil.Process(process.pid).children(recursive=True))
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait(timeout=5)
                for child in owned:
                    try:
                        child.terminate()
                    except psutil.NoSuchProcess:
                        pass
                _,left=psutil.wait_procs(owned,timeout=5)
                for child in left:
                    try:
                        child.kill()
                    except psutil.NoSuchProcess:
                        pass


if __name__=='__main__':
    sys.exit(main())
