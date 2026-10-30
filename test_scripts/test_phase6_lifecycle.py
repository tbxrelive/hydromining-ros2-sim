#!/usr/bin/env python3
"""独立真实 launch 生命周期验证，不启动 Gazebo，不使用节点自转换。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


def main():
    output = Path('/tmp') / f'phase6_lifecycle_{os.getpid()}'
    output.mkdir()
    os.environ['ROS_DOMAIN_ID'] = '54'
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    log = (output / 'launch.log').open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'hydromining_robot', 'wall_perception.launch.py',
         'use_sim_time:=false', f'output_directory:={output / "perception"}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('phase6_lifecycle_probe')
    client = node.create_client(GetState, '/wall_perception_node/get_state')
    report = {'passed': False, 'output': str(output)}
    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('perception launch exited early')
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                if future.done() and future.result().current_state.id == 3:
                    report.update(passed=True, state='active', externally_managed=True)
                    break
            rclpy.spin_once(node, timeout_sec=0.05)
        if not report['passed']:
            raise TimeoutError('external configure/activate failed')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        (output / 'summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
