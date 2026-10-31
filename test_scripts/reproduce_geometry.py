#!/usr/bin/env python3
"""Validate generated meshes and load both in a separate real Harmonic server."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import trimesh


def main():
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='results/reproduction_20261030/phase1')
    out=Path(parser.parse_args().output).resolve()
    report={};models=[]
    for name in ('smooth','rough'):
        path=out/f'{name}.obj'
        mesh=trimesh.load(path,force='mesh',process=True)
        mesh.merge_vertices(digits_vertex=7,merge_norm=True,merge_tex=True)
        assert mesh.is_watertight and mesh.volume>0
        report[name]={'vertices':len(mesh.vertices),'faces':len(mesh.faces),'watertight':True,'volume_m3':mesh.volume,'bounds':mesh.bounds.tolist()}
        models.append(f'<model name="{name}"><static>true</static><pose>0 {4*len(models)} 0 0 0 0</pose><link name="wall"><visual name="visual"><geometry><mesh><uri>{path}</uri></mesh></geometry></visual><collision name="collision"><geometry><mesh><uri>{path}</uri></mesh></geometry></collision></link></model>')
    world=out/'generated_world.sdf'
    world.write_text('<sdf version="1.9"><world name="geometry_reproduction"><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>'+''.join(models)+'</world></sdf>')
    env={**os.environ,'GZ_PARTITION':f'geometry_reproduction_{os.getpid()}'}
    with (out/'gazebo.log').open('w') as log:
        process=subprocess.Popen(['gz','sim','-r','-s',str(world)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                if process.poll() is not None:raise RuntimeError('geometry server exited')
                reply=subprocess.run(['gz','service','-s','/world/geometry_reproduction/scene/info','--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','2000','--req',''],env=env,capture_output=True,text=True,timeout=5)
                if 'smooth' in reply.stdout and 'rough' in reply.stdout:
                    (out/'scene.txt').write_text(reply.stdout);break
            else:raise TimeoutError('models missing from real scene service')
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
    report['gazebo_models_loaded']=True
    report['passed']=True
    (out/'summary.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
