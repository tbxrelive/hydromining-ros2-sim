#!/usr/bin/env python3
"""Measure native mist and dust separately with fixed robot and clean baselines."""
import json
import os
from pathlib import Path
import subprocess

import rclpy
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

from test_phase4_acceptance import cleanup_acceptance
from test_phase5_acceptance import Phase5Acceptance


def main():
    output = Path('results/phase7_large_task/isolated_particles').resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_LOG_DIR=str(output/'ros_logs'), ROS_DOMAIN_ID='47',
                      GZ_PARTITION='hydromining_isolated_'+str(os.getpid()))
    robot = Path(get_package_share_directory('hydromining_robot'))
    sim = Path(get_package_share_directory('hydromining_sim'))
    command = ['ros2', 'launch', 'hydromining_robot', 'robot_simulation.launch.py',
               'gui:=false', f'world:={sim/"worlds/coal_wall_phase5.sdf"}',
               'start_spray:=true', 'enable_effects:=true',
               f'spray_config:={robot/"config/spray_acceptance.yaml"}',
               f'spray_output_dir:={output/"meshes"}', 'camera_rate:=5',
               'camera_width:=320', 'camera_height:=240', 'lidar_rate:=2']
    with (output/'simulation.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Phase5Acceptance(output, process)
        try:
            node.ready()
            node.wait(lambda: node.diag_ is not None and node.lidar_ is not None and node.depth_ is not None,
                      30, 'Effects bridge or sensor missing')
            report = {}
            for name in ('mist', 'dust'):
                node.configure(jet_core=False, jet_mist=name == 'mist', jet_splash=False,
                               jet_dust=name == 'dust')
                for preset in ('low', 'high'):
                    assert node.future(node.reset_.call_async(Trigger.Request()), 40).success
                    _, report[name+'_'+preset] = node.paired_shot(name+'_'+preset, preset, duration=8.)
        finally:
            cleanup = cleanup_acceptance(node, process)
    report['cleanup'] = cleanup
    (output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
