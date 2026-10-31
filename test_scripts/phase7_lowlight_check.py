#!/usr/bin/env python3
"""Capture a fixed arm RGB-D view in the normal or low-light Gazebo world."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image
import rclpy
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml
from ament_index_python.packages import get_package_share_directory

from test_phase4_acceptance import AcceptanceTest, cleanup_acceptance

sys.path.insert(0, get_package_share_directory('hydromining_sim'))
from gui_view import write_scene_world


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--low-light', action='store_true')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_LOG_DIR=str(output/'ros_logs'), ROS_DOMAIN_ID='56',
                      GZ_PARTITION='hydromining_light_'+str(os.getpid()))
    robot = Path(get_package_share_directory('hydromining_robot'))
    sim = Path(get_package_share_directory('hydromining_sim'))
    task = yaml.safe_load((robot/'config/phase7_large_task.yaml').read_text())
    world = output/'world.sdf'
    write_scene_world(sim/'worlds/coal_wall_acceptance.sdf', world, task['source_coal_pose'], args.low_light)
    command = ['ros2', 'launch', 'hydromining_robot', 'robot_simulation.launch.py',
               'gui:=false', 'start_spray:=false', f'world:={world}', 'camera_rate:=5',
               'camera_width:=320', 'camera_height:=240', 'lidar_rate:=2']
    log = (output/'simulation.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AcceptanceTest(output, process)
    try:
        node.wait(node.motion_.server_is_ready, 90, 'trajectory action missing')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['panda_joint'+str(i) for i in range(1, 8)]
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=task['viewpoints']['front_0p8']['joints'], time_from_start=Duration(sec=6))]
        handle = node.future(node.motion_.send_goal_async(goal), 20)
        result = node.future(handle.get_result_async(), 90)
        if result.result.error_code:
            raise RuntimeError(result.result.error_string)
        node.capture('fixed_view')
        rgb = np.asarray(Image.open(output/'fixed_view_rgb.png'))
        with np.load(output/'fixed_view.npz') as data:
            valid = float(np.isfinite(data['world']).all(axis=-1).mean())
        report = {'low_light': args.low_light, 'rgb_mean': float(rgb.mean()),
                  'rgb_median': float(np.median(rgb)), 'finite_depth_fraction': valid}
    finally:
        cleanup = cleanup_acceptance(node, process)
        log.close()
    report.update(cleanup)
    (output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
