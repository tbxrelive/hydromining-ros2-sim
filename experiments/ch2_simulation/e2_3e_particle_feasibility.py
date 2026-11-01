"""E2.3e Gazebo 粒子层可行性验证：把发射器调到最强，看受扰比例的上限。

为什么需要这个实验：阶段 4 的 E4.4 要求"干扰模型验证 r ≥ 0.8"，前提是粒子层
能产生**可测且可分**的点云退化。E2.3c 已经证明预设 off/low/medium/high 三档
都停在 1.3–1.4%，但那只覆盖了默认设置。本实验把可控参数推到允许的最大值，
给出**上限**：如果上限仍远低于软件层"低"档（10.4%），E4.4 就必须改验证对象。

扫的参数（`command_packet` 与 `spray_acceptance.yaml` 允许的范围）：
- 水雾发射率比例 jet_mist_rate_scale ∈ {1.0}（已到上限）
- 水雾扩散半宽 jet_mist_spread ∈ {0.24 默认, 0.5 上限}
- 同时开启水雾+飞溅+粉尘
- 参考压力 effect_reference_pressure_mpa ∈ {20 默认, 10}（把强度推到 2.0 上限）

每个配置重复 --repeats 次（每次 reset 煤壁后重新喷射），报告点云受扰比例的上限。

命令：
    python experiments/ch2_simulation/e2_3e_particle_feasibility.py --repeats 2
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

# 低档门限：软件层"低"档的包络平均受扰比例（E2.3b）
LOW_LEVEL_THRESHOLD = .104

CONFIGS = (
    {'name': 'default_high', 'spread': .24, 'reference': 20., 'splash': False, 'dust': False},
    {'name': 'max_spread', 'spread': .50, 'reference': 20., 'splash': False, 'dust': False},
    {'name': 'max_spread_all_effects', 'spread': .50, 'reference': 10., 'splash': True, 'dust': True},
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default=str(ROOT/'results/thesis/E2.3e'))
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--duration', type=float, default=8.)
    args = parser.parse_args()

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from ament_index_python.packages import get_package_share_directory
    from std_srvs.srv import Trigger
    from test_phase4_acceptance import cleanup_acceptance
    from test_phase5_acceptance import Phase5Acceptance

    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_LOG_DIR=str(output/'ros_logs'), ROS_DOMAIN_ID='49',
                      GZ_PARTITION='hydromining_e23e_'+str(os.getpid()))
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
            for config in CONFIGS:
                node.configure(jet_core=True, jet_mist=True,
                               jet_splash=config['splash'], jet_dust=config['dust'],
                               effect_preset='high',
                               effect_reference_pressure_mpa=config['reference'],
                               jet_mist_rate_scale=1.,
                               jet_mist_spread=config['spread'],
                               jet_splash_spread=.5 if config['splash'] else .12,
                               jet_dust_spread=.5 if config['dust'] else .18)
                for repeat in range(args.repeats):
                    if not node.future(node.reset_.call_async(Trigger.Request()), 40).success:
                        raise RuntimeError('煤壁复位失败')
                    _, report = node.paired_shot(f"{config['name']}_{repeat}",
                                                 preset='high', duration=args.duration)
                    record = {'config': config['name'], 'spread': config['spread'],
                              'reference_mpa': config['reference'],
                              'splash': config['splash'], 'dust': config['dust'],
                              'repeat': repeat,
                              'cloud_disturbed_fraction': report['cloud_disturbed_fraction'],
                              'depth_disturbed_fraction': report['depth_disturbed_fraction'],
                              'lidar_disturbed_fraction': report['lidar_disturbed_fraction']}
                    records.append(record)
                    print(f"[{config['name']} #{repeat}] 点云受扰 "
                          f"{record['cloud_disturbed_fraction']*100:.2f}%", flush=True)
        finally:
            cleanup_acceptance(node, process)

    summary = {}
    for config in CONFIGS:
        values = [r['cloud_disturbed_fraction'] for r in records if r['config'] == config['name']]
        summary[config['name']] = {'mean': sum(values)/len(values) if values else None,
                                   'n': len(values), 'values': values}
    upper = max((block['mean'] for block in summary.values() if block['mean'] is not None),
                default=None)
    report = {'repeats': args.repeats, 'duration_s': args.duration,
              'low_level_threshold': LOW_LEVEL_THRESHOLD,
              'upper_bound': upper,
              'reaches_low_level': bool(upper is not None and upper >= LOW_LEVEL_THRESHOLD),
              'summary': summary, 'records': records,
              'conclusion': ('粒子层最强配置仍低于软件层低档，E4.4 需改验证对象'
                             if (upper is not None and upper < LOW_LEVEL_THRESHOLD)
                             else '粒子层可达到低档量级，E4.4 可按原计划验证')}
    (output/'feasibility.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps({k: v for k, v in report.items()
                      if k not in ('records', 'summary')}, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
