#!/usr/bin/env bash
# Read-only dependency check; returns nonzero on any missing prerequisite.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
set +u
source "$workspace_dir/setup_env.sh"
set -u
python - <<'PYTHON'
import importlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_prefix
assert sys.version_info[:2] == (3, 12), sys.version
for module in ('numpy', 'scipy', 'open3d', 'trimesh', 'mcubes', 'perlin_numpy',
               'yaml', 'psutil', 'matplotlib', 'mpl_toolkits.mplot3d', 'py4dgeo',
               'torch', 'gymnasium', 'stable_baselines3', 'rclpy', 'graphviz',
               'hydromining_robot.action', 'hydromining_robot.srv'):
    importlib.import_module(module)
    print(f'导入通过：{module}')
for manifest in Path('src').glob('*/package.xml'):
    for element in ET.parse(manifest).getroot():
        if element.tag.endswith('depend') and not element.text.startswith(('python3-', 'lib')):
            get_package_prefix(element.text)
for resource in ('src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj',
                 'src/hydromining_robot/config/viewpoints.yaml'):
    assert Path(resource).is_file(), resource
print('ROS 包依赖与模型资源检查通过')
PYTHON
python -m pip check
ros2 control --help >/dev/null
gz sim --versions
