#!/usr/bin/env python3
"""
使用预设配置快速生成煤壁
"""

import yaml
import subprocess
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
PRESETS_FILE = os.path.join(WORKSPACE_ROOT, "assets", "walls", "presets.yaml")

def load_presets():
    with open(PRESETS_FILE, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def main():
    if len(sys.argv) < 2:
        print("用法: python generate_from_preset.py <预设名称>")
        print("\n可用预设:")
        presets = load_presets()
        for name, config in presets.items():
            print(f"  - {name}: {config}")
        sys.exit(1)

    preset_name = sys.argv[1]
    presets = load_presets()

    if preset_name not in presets:
        print(f"错误: 未找到预设 '{preset_name}'")
        print(f"可用预设: {', '.join(presets.keys())}")
        sys.exit(1)

    config = presets[preset_name]

    # 构建命令
    cmd = [
        sys.executable,
        os.path.join(SCRIPT_DIR, "generate_coal_wall.py"),
        "--width", str(config["width"]),
        "--height", str(config["height"]),
        "--thickness", str(config["thickness"]),
        "--tilt", str(config["tilt"]),
        "--roughness", str(config["roughness"]),
        "--resolution", str(config["resolution"]),
        "--seed", str(config["seed"]),
        "--output", os.path.join(WORKSPACE_ROOT, "src", "hydromining_sim", "models", "coal_wall", "meshes", "coal_wall.obj")
    ]

    print(f"使用预设 '{preset_name}' 生成煤壁...")
    subprocess.run(cmd)

if __name__ == '__main__':
    main()
