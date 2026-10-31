#!/usr/bin/env python3
"""Real Harmonic server with deterministic post-mutation ACK loss injection."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory

sys.path.insert(0, str(Path(__file__).parents[1]/'src/hydromining_robot/scripts'))
from gazebo_coal_wall import GazeboCoalWall


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/hydromining_coal_transport')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    sim = Path(get_package_share_directory('hydromining_sim'))
    prefix = Path(get_package_prefix('hydromining_robot'))
    os.environ['GZ_PARTITION'] = 'coal_transport_'+str(os.getpid())
    os.environ['GZ_SIM_RESOURCE_PATH'] = str(sim/'models')
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(Path(get_package_prefix('hydromining_sim'))/'lib')
    os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:2])
    os.nice(10)
    log = (output/'server.log').open('w')
    process = subprocess.Popen(['gz','sim','-r','-s','--headless-rendering',
        str(sim/'worlds/coal_wall_acceptance.sdf')], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    adapter = GazeboCoalWall('hydromining_world', [1.5,0,1.5,0,-1.9198621771937625,0],
        scene_probe=prefix/'lib/hydromining_robot/coal_wall_scene', warning=print)
    scenarios = []
    try:
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            try:
                if adapter.exists():
                    break
            except RuntimeError:
                pass
            time.sleep(.2)
        else:
            raise RuntimeError('Real server never became ready')
        real_request, real_scene, real_wait = adapter.request, adapter.scene_wall, adapter.wait_exists
        for fault in ('none','create_ack','remove_ack','scene_reply','not_delivered','late_confirmation','initial_read'):
            path = output/(fault+'.obj')
            shutil.copyfile(sim/'models/coal_wall/meshes/coal_wall.obj',path)
            calls = []
            pending_scene_loss = [False]
            initial_losses = [2 if fault=='initial_read' else 0]

            def request(endpoint, *args, **kwargs):
                calls.append(endpoint)
                if endpoint=='create' and fault=='not_delivered' and calls.count('create')==1:
                    raise RuntimeError('Injected request loss BEFORE reaching the real server')
                result = real_request(endpoint, *args, **kwargs)
                if endpoint=='create' and fault=='scene_reply':
                    pending_scene_loss[0] = True
                if (endpoint=='create' and fault=='create_ack') or (endpoint=='remove' and fault=='remove_ack'):
                    raise RuntimeError('Injected lost ACK AFTER real server applied '+endpoint)
                return result

            def scene():
                if initial_losses[0]:
                    initial_losses[0] -= 1
                    raise RuntimeError('Injected initial read discovery delay')
                if pending_scene_loss[0]:
                    pending_scene_loss[0] = False
                    raise RuntimeError('Injected transient scene query failure')
                return real_scene()

            def wait(wanted, *args, **kwargs):
                if wanted and fault=='late_confirmation':
                    raise RuntimeError('Injected expiration immediately before final scene read')
                return real_wait(wanted, *args, **kwargs)

            adapter.request, adapter.scene_wall, adapter.wait_exists = request, scene, wait
            adapter.replace(path)
            actual = real_scene()
            assert adapter.synchronized_ and adapter.matches_mesh(actual,path)
            expected = 2 if fault=='not_delivered' else 1
            assert calls.count('create')==expected,calls
            scenarios.append({'fault':fault,'confirmed_mesh':str(path),'create_calls':expected})
        (output/'summary.json').write_text(json.dumps({'passed':True,'GUI_started':False,
            'faults_are_injected':True,'real_server':True,'scenarios':scenarios},indent=2))
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:
            process.wait(5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            process.wait(5)
        log.close()
    print(json.dumps(scenarios,indent=2))


if __name__=='__main__':
    main()
