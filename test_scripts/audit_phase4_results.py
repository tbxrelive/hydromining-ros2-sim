#!/usr/bin/env python3
"""Independently audit the six acceptance artifacts; no ROS or Gazebo startup."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
import trimesh
import yaml


def audit(directory):
    """Recompute from saved measurements instead of trusting the passed flag."""
    directory=Path(directory)
    summary=json.loads((directory/'summary.json').read_text())
    assert summary['mode']=='headless', 'This audit covers the developer headless run'
    assert summary['owned_processes_remaining']==[]
    assert summary['aggregate_peak_rss_mib'] < 3072
    assert len(summary['cpu_affinity']) <= 2
    labels=['01_before','02_after_first','03_after_second','04_reset','05_replay']
    clouds=[];stamps=[];image_digests=[];counts=[]
    for label in labels:
        with np.load(directory/(label+'.npz')) as data:
            world=data['world'];local=data['local']
            valid=np.isfinite(world).all(axis=1)
            assert str(data['frame'])=='world' and str(data['source_frame'])=='camera_link'
            transformed=local[valid]@Rotation.from_quat(data['tf_quaternion']).as_matrix().T+data['tf_translation']
            assert np.max(np.abs(transformed-world[valid])) < 1e-10
            pcd=directory/(label+'.pcd')
            with pcd.open() as f:
                while True:
                    line=f.readline()
                    assert line, 'PCD header ended prematurely'
                    if line.strip()=='DATA ascii':
                        break
                saved=np.loadtxt(f)
            assert saved.shape==world[valid].shape
            assert np.max(np.abs(saved-world[valid])) < 1e-7
            image=np.asarray(Image.open(directory/(label+'_rgb.png')))
            assert image.shape==(int(data['height']),int(data['width']),3)
            assert image.std()>1
            image_digests.append(hashlib.sha256(image.tobytes()).hexdigest())
            stamps.append(int(data['stamp'][0])*1_000_000_000+int(data['stamp'][1]))
            counts.append(int(valid.sum()))
            clouds.append(world.copy())
    assert all(a<b for a,b in zip(stamps,stamps[1:])), 'Frames must follow the six-step sequence'
    assert image_digests[0]!=image_digests[1] and image_digests[1]!=image_digests[2]
    with np.load(directory/'joint_hold.npz') as joints:
        positions=joints['samples'][:,1:]
        assert len(positions)>100 and np.isfinite(positions).all()
        assert np.all(np.diff(joints['samples'][:,0])>=0)
        drift=float(np.max(np.abs(positions-joints['reference'])))
        assert drift<float(joints['limit_rad'])
        assert abs(drift-summary['joint_drift_rad']) < 1e-12
        assert len(positions)==summary['joint_hold_sample_count']
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'src/hydromining_robot/config/spray_acceptance.yaml').read_text())
    pose=config['spray_controller_node']['ros__parameters']['coal_pose']
    rotation=Rotation.from_euler('xyz',pose[3:]).as_matrix()
    position=rotation@summary['first']['position_mesh']+pose[:3]
    direction=rotation@summary['first']['direction_mesh']
    relative=clouds[0]-position
    axial=relative@direction
    radial=np.linalg.norm(relative-np.outer(axial,direction),axis=1)
    metrics=[]
    for cloud in clouds[1:3]:
        valid=np.isfinite(clouds[0]).all(axis=1)&np.isfinite(cloud).all(axis=1)
        roi=valid&(radial<.16)&(axial>.3)&(axial<1.5)
        annulus=valid&(radial>.23)&(radial<.35)&(axial>.3)&(axial<1.5)
        change=(cloud-clouds[0])@direction
        changed=int(np.count_nonzero(change[roi]>.005))
        p95=float(np.percentile(change[roi],95))
        reference=float(np.percentile(np.abs(change[annulus]),95))
        assert changed>=20 and p95>.01 and reference<.005
        metrics.append({'changed_points':changed,'p95_retreat_m':p95,'reference_p95_abs_m':reference})
    assert metrics[1]['p95_retreat_m']>metrics[0]['p95_retreat_m']
    first=summary['first'];second=summary['second'];replay=summary['replay']
    assert first['volume_removed_m3']>0 and first['max_depth_m']>0
    assert second['cumulative_volume_m3']>first['cumulative_volume_m3']
    assert second['cumulative_depth_m']>first['cumulative_depth_m']
    assert abs(second['cumulative_volume_m3']-first['volume_removed_m3']-second['volume_removed_m3'])<1e-12
    assert first['volume_removed_m3']==replay['volume_removed_m3']
    digests=[]
    for i in range(1,6):
        path=directory/f'meshes/coal_wall_{i:04d}.obj'
        mesh=trimesh.load(path,force='mesh',process=True)
        assert np.isfinite(mesh.vertices).all() and mesh.is_watertight and mesh.volume>0
        digests.append(hashlib.sha256(path.read_bytes()).hexdigest())
    assert digests[0]==digests[3] and digests[1]==digests[4]
    assert digests[0]!=digests[1] and digests[1]!=digests[2]
    errors=[]
    for before,after in [(clouds[0],clouds[3]),(clouds[1],clouds[4])]:
        valid=np.isfinite(before).all(1)&np.isfinite(after).all(1)
        error=float(np.max(np.linalg.norm(after[valid]-before[valid],axis=1)))
        assert error<.005
        errors.append(error)
    result={'automated_scope_complete':True,
            'requirements':{str(i):True for i in range(1,7)},
            'finite_cloud_point_counts':counts,'joint_hold_sample_count':len(positions),
            'max_observed_joint_drift_rad':drift,'independent_sensor_metrics':metrics,
            'reset_max_point_error_m':errors[0],'replay_max_point_error_m':errors[1],
            'mesh_sha256':dict(zip(['initial','first','cumulative','reset','replay'],digests)),
            'GUI_final_user_validation':'Not executed: user reserved GUI inspection for themselves',
            'calibrated':False}
    (directory/'completion_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    audit(args.directory)
