#!/usr/bin/env python3
"""Baseline-only multi-view check: report TF-only alignment before optional ICP."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import open3d as o3d
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/hydromining_sim/scripts'))
from algorithms.perception.frames import backproject_depth,transform_points
from scipy.spatial.transform import Rotation


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--dataset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();clouds={};positions={}
    for name in ['front_0p8','left_0p8','right_0p8']:
        path=sorted((args.dataset/'baseline').glob(f'baseline_{name}_f*.npz'))[-1]
        with np.load(path) as data:
            points=transform_points(backproject_depth(data['depth'],data['K']),data['tf_depth']).reshape(-1,3)
            positions[name]=data['tf_depth'][:3].tolist()
        keep=np.isfinite(points).all(axis=1)&(np.linalg.norm(points-[1.517,.011,.694],axis=1)<.3)
        clouds[name]=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[keep])).voxel_down_sample(.003)
    rows=[]
    for name in ['left_0p8','right_0p8']:
        source,target=clouds['front_0p8'],clouds[name]
        before=o3d.pipelines.registration.evaluate_registration(source,target,.02,np.eye(4))
        after=o3d.pipelines.registration.registration_icp(source,target,.02,np.eye(4),o3d.pipelines.registration.TransformationEstimationPointToPoint())
        rows.append({'pair':['front_0p8',name],'tf_only_rmse_m':before.inlier_rmse,'tf_only_fitness':before.fitness,'icp_rmse_m':after.inlier_rmse,'icp_translation_m':float(np.linalg.norm(after.transformation[:3,3])),'icp_rotation_rad':float(Rotation.from_matrix(after.transformation[:3,:3].copy()).magnitude())})
    passed=all(r['tf_only_rmse_m']<.01 and r['tf_only_fitness']>.9 for r in rows)
    report={'passed':passed,'rows':rows,'camera_origins_world':positions,'scope':'baseline depth clouds; ICP is an audit only and is not used to align changed surfaces'}
    args.output.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    if not passed:raise SystemExit(1)


if __name__=='__main__':main()
