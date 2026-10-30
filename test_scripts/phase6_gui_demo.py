#!/usr/bin/env python3
"""阶段六 GUI 演示驱动：真实 Gazebo 界面下完成 观测→喷射→再观测→三分类判决。

复用阶段六验收脚本中的客户端、射流参数与独立真值核对；真值网格只用于
终端展示，不进入感知算法输入。演示结束后保留仿真窗口，关闭窗口自动清理。
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_phase4_acceptance import cleanup_acceptance
from test_phase6_acceptance import Phase6Acceptance


def settle(node, seconds):
    """等待喷射粒子消散，同时保持事件循环与内存监控。"""
    end = time.monotonic() + seconds
    node.wait(lambda: time.monotonic() >= end, seconds + 10, 'settle timed out')


def show(stage, result, truth=None):
    print(f'\n===== {stage} =====', flush=True)
    print(f'  判决 assessment      : {result.assessment}')
    print(f'  消息 message         : {result.message}')
    print(f'  覆盖率 coverage      : {result.coverage:.3f}')
    print(f'  达标比例区间         : [{result.fraction_lower:.3f}, {result.fraction_upper:.3f}]')
    print(f'  移除体积/质量        : {result.removed_volume_m3:.6f} m3 / {result.removed_mass_kg:.3f} kg')
    print(f'  区域外异常变化告警   : {result.external_alarm}')
    print(f'  报告                 : {result.report_path}')
    if truth:
        print(f'  独立真值(仅展示)     : {truth["status"]}, 达标面积比例 {truth["fraction_ge_threshold"]:.3f}')
    print('', flush=True)


def main():
    """启动带 GUI 的完整仿真，按步骤驱动验收流程并打印中间结果。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/p6_gui_demo')
    parser.add_argument('--settle', type=float, default=15.,
                        help='喷射结束到喷后观测之间的消散等待秒数')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    venv_python = Path(__file__).resolve().parents[1] / '.venv_stage4/bin/python'
    if venv_python.is_file():
        os.environ.setdefault('HYDROMINING_PYTHON', str(venv_python))
    os.environ.update({'ROS_LOG_DIR': str(output / 'ros_logs'), 'ROS_DOMAIN_ID': '53',
                       'GZ_PARTITION': 'hydromining_phase6_gui_' + str(os.getpid())})
    robot = Path(get_package_share_directory('hydromining_robot'))
    config = yaml.safe_load((robot / 'config/viewpoints.yaml').read_text())
    views = {name: config['viewpoints'][name] for name in ('front_0p8', 'left_0p8', 'right_0p8')}
    command = ['ros2', 'launch', 'hydromining_robot', 'phase6_simulation.launch.py',
               'gui:=true', 'enable_effects:=true',
               f'spray_output_dir:={output / "meshes"}',
               f'perception_output_dir:={output / "perception"}']
    log = (output / 'simulation.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Phase6Acceptance(output, process, views, config['region_center_world'])
    node.noise_mode_ = False
    code, summary = 1, {}
    try:
        print('等待 Gazebo 与两个生命周期节点进入 active ...', flush=True)
        node.wait(lambda: node.active(node.state_) and node.active(node.perception_state_), 180,
                  'lifecycle nodes did not become active')
        node.wait(node.motion_.server_is_ready, 90, 'trajectory action missing')
        # GUI + 粒子特效下内存紧贴 3 GiB 保护线；喷射前降档以减少粒子开销。
        spray_params = node.create_client(SetParameters, '/spray_controller_node/set_parameters')
        node.wait(spray_params.service_is_ready, 30, 'spray set_parameters missing')
        reply = node.future(spray_params.call_async(SetParameters.Request(
            parameters=[Parameter('effect_preset', value='low').to_parameter_msg(),
                        Parameter('jet_mist', value=False).to_parameter_msg(),
                        Parameter('jet_splash', value=False).to_parameter_msg(),
                        Parameter('jet_dust', value=False).to_parameter_msg()])), 20)
        assert all(r.successful for r in reply.results), [r.reason for r in reply.results]
        print('就绪（保留水柱显示，已关闭水雾/飞溅/粉尘粒子以控制内存）。请观察 Gazebo 窗口。\n', flush=True)

        print('===== 第 1 步：重置煤壁并建立喷前基线 =====', flush=True)
        print('机械臂将依次移动到 front/left/right 三个观测位姿采集数据。', flush=True)
        node.baseline_mesh_ = node.reset()
        show('基线评估', node.assess(node.goal(True)))

        print('===== 第 2 步：未喷射状态核对（应判 not_achieved） =====', flush=True)
        show('未变化区域评估', node.assess(node.goal(False)))

        print('===== 第 3 步：单点喷射 20 MPa x 12 s =====', flush=True)
        print('请在窗口中观察水柱、水雾与煤壁凹坑形成。', flush=True)
        mesh = node.spray('single')
        print(f'喷射完成，等待 {args.settle:.0f} s 让粒子消散后重新观测 ...', flush=True)
        settle(node, args.settle)
        result = node.assess(node.goal(False))
        show('单点喷射后评估', result, node.truth_summary(result, mesh))

        print('===== 第 4 步：重置后七点喷射（中心 + 半径 6 cm 环） =====', flush=True)
        node.baseline_mesh_ = node.reset()
        mesh = node.spray('hex_r6')
        print(f'喷射完成，等待 {args.settle:.0f} s 让粒子消散后重新观测 ...', flush=True)
        settle(node, args.settle)
        result = node.assess(node.goal(False))
        show('七点喷射后评估', result, node.truth_summary(result, mesh))

        summary = {'passed': True, 'feedback_messages': len(node.feedback_),
                   'map_messages': len(node.map_messages_),
                   'peak_rss_mib': node.peak_rss_ / 1024 ** 2}
        (output / 'demo_summary.json').write_text(json.dumps(summary, indent=2))
        print('===== 演示流程结束 =====', flush=True)
        print(f'峰值内存 {summary["peak_rss_mib"]:.0f} MiB；报告目录 {output}', flush=True)
        print('Gazebo 窗口保持打开，可自由旋转视角查看凹坑；', flush=True)
        print('关闭窗口或对启动终端 Ctrl+C 后自动清理其余进程。', flush=True)
        while process.poll() is None and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
        code = 0
    except KeyboardInterrupt:
        code = 130
    except Exception as error:
        import traceback
        traceback.print_exc()
        (output / 'failure.json').write_text(json.dumps({'error': str(error)}))
    finally:
        print('清理本次启动的仿真进程 ...', flush=True)
        cleanup = cleanup_acceptance(node, process)
        log.close()
        print(json.dumps({'summary': summary, 'cleanup': cleanup}, indent=2), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
