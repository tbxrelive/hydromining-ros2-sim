"""E2.3c Gazebo 粒子验证：把 off/low/medium/high 映射到等效能见度。

定位：干扰等级**由软件层（能见度）定义**，本实验只做两件事——
1. 证明粒子档位与软件层方向一致（档位越高，点云受扰比例越高）；
2. 给出"粒子档位 ↔ 等效能见度 V"的映射，供阶段 4 的 E4.4 使用。

已知风险（阶段 5 旧结论）：低/高档点云受扰比例只差约 0.1 个百分点
（1.42% 对 1.52%），与单帧波动相当，因此"相邻档可分"列为争取目标，
"单调"才是必须达成的目标。每个重复都先 reset 煤壁再重新喷射，
不用同一次喷射里的多帧（那样不独立，会低估标准差）。

命令：
    python experiments/ch2_simulation/e2_3c_particle_validation.py --repeats 3
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src/hydromining_sim/scripts'))
sys.path.insert(0, str(ROOT/'test_scripts'))

PRESETS = ('off', 'low', 'medium', 'high')


def level_of(disturbed_fraction):
    """按 E2.3b 的等级区间把受扰比例落到最近的软件层等级。"""
    from experiments.common.interference import load_levels
    regions = {'none': 0., 'low': .104, 'medium': .336, 'high': .730}
    values = load_levels()
    best = min(regions, key=lambda name: abs(regions[name]-disturbed_fraction))
    return best, values[best].visibility_m


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E2.3c'))
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--duration', type=float, default=8.)
    args = parser.parse_args()

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from ament_index_python.packages import get_package_share_directory
    from test_phase4_acceptance import cleanup_acceptance
    from test_phase5_acceptance import Phase5Acceptance
    from std_srvs.srv import Trigger

    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_LOG_DIR=str(output/'ros_logs'), ROS_DOMAIN_ID='48',
                      GZ_PARTITION='hydromining_e23c_'+str(os.getpid()))
    robot = Path(get_package_share_directory('hydromining_robot'))
    sim = Path(get_package_share_directory('hydromining_sim'))
    command = ['ros2', 'launch', 'hydromining_robot', 'robot_simulation.launch.py',
               'gui:=false', f'world:={sim/"worlds/coal_wall_phase5.sdf"}',
               'start_spray:=true', 'enable_effects:=true',
               f'spray_config:={robot/"config/spray_acceptance.yaml"}',
               f'spray_output_dir:={output/"meshes"}', 'camera_rate:=5',
               'camera_width:=320', 'camera_height:=240', 'lidar_rate:=2']
    records = []
    with (output/'simulation.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Phase5Acceptance(output, process)
        try:
            node.ready()
            node.wait(lambda: node.diag_ is not None and node.lidar_ is not None
                      and node.depth_ is not None, 30, 'Effects bridge or sensor missing')
            for repeat in range(args.repeats):
                for preset in PRESETS:
                    if not node.future(node.reset_.call_async(Trigger.Request()), 40).success:
                        raise RuntimeError('煤壁复位失败')
                    _, report = node.paired_shot(f'{preset}_{repeat}', preset=preset,
                                                 duration=args.duration)
                    record = {'preset': preset, 'repeat': repeat,
                              'cloud_disturbed_fraction': report['cloud_disturbed_fraction'],
                              'depth_disturbed_fraction': report['depth_disturbed_fraction'],
                              'lidar_disturbed_fraction': report['lidar_disturbed_fraction'],
                              'cloud_visible_fraction': report['cloud_visible_fraction']}
                    records.append(record)
                    print(f"[{preset} #{repeat}] 点云受扰 "
                          f"{record['cloud_disturbed_fraction']*100:.2f}%", flush=True)
        finally:
            cleanup = cleanup_acceptance(node, process)

    summary = {}
    for preset in PRESETS:
        values = [r['cloud_disturbed_fraction'] for r in records if r['preset'] == preset]
        mean = sum(values)/len(values) if values else None
        variance = (sum((v-mean)**2 for v in values)/(len(values)-1)) if len(values) > 1 else 0.
        summary[preset] = {'mean': mean, 'std': variance**.5, 'n': len(values),
                           'values': values}
    means = [summary[p]['mean'] for p in PRESETS]
    monotone = all(a < b for a, b in zip(means[:-1], means[1:]))
    mapping = {}
    for preset in PRESETS:
        name, visibility = level_of(summary[preset]['mean'])
        mapping[preset] = {'equivalent_level': name, 'equivalent_visibility_m': visibility}
    report = {'repeats': args.repeats, 'duration_s': args.duration,
              'presets': list(PRESETS), 'summary': summary, 'monotone': monotone,
              'mapping': mapping,
              'records': records}
    (output/'particles.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps({'summary': summary, 'monotone': monotone, 'mapping': mapping},
                     indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
