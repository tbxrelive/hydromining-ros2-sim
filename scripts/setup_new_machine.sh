#!/usr/bin/env bash
# Restore dependencies in this checkout; do not clone or overwrite another workspace.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo '请先安装 Ubuntu 24.04 对应的 ROS 2 Jazzy / ros-jazzy-ros-gz。' >&2
    exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ "${ROS_DISTRO:-}" != jazzy ]]; then
    echo '必须使用 ROS 2 Jazzy。' >&2
    exit 1
fi
if [[ ! -x .venv_stage4/bin/python ]]; then
    /usr/bin/python3 -m venv --system-site-packages .venv_stage4
fi
touch .venv_stage4/COLCON_IGNORE
.venv_stage4/bin/python -c 'import sys; assert sys.version_info[:2] == (3, 12)'
.venv_stage4/bin/python -m pip install 'torch==2.5.1+cpu' --index-url https://download.pytorch.org/whl/cpu
.venv_stage4/bin/python -m pip install -r requirements.txt
if [[ -f requirements-local.lock.txt ]]; then
    .venv_stage4/bin/python -m pip install -r requirements-local.lock.txt
fi
# Run ROS CLI plugins with the same interpreter as the project dependencies.
cat > .venv_stage4/bin/ros2 <<'PYTHON'
#!/usr/bin/env python3
import sys
from ros2cli.cli import main
if __name__ == '__main__':
    sys.exit(main())
PYTHON
chmod +x .venv_stage4/bin/ros2
# Pure Python ROS package overlay, usable without sudo. ROS binaries stay in /opt.
for ros_package in ros2controlcli joint_state_publisher; do
    if ! ros2 pkg prefix "$ros_package" >/dev/null 2>&1 && \
            [[ ! -f ".local_ros_deps/opt/ros/jazzy/share/$ros_package/package.xml" ]]; then
        mkdir -p .local_ros_deps/debs
        (
            cd .local_ros_deps/debs
            deb_name="ros-jazzy-${ros_package//_/-}"
            apt download "$deb_name"
            for package_file in "${deb_name}"_*.deb; do
                dpkg-deb -x "$package_file" ..
            done
        )
    fi
done
if [[ -d .local_ros_deps/opt/ros/jazzy ]]; then
    cat > .local_ros_deps/opt/ros/jazzy/local_setup.bash <<'BASH'
# Project-local official ROS deb overlay (pure Python CLI).
_hydromining_local_prefix="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="$_hydromining_local_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PYTHONPATH="$_hydromining_local_prefix/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
unset _hydromining_local_prefix
BASH
fi
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS='-j2 -l2'
colcon build --base-paths src --symlink-install --packages-select hydromining_robot hydromining_sim --executor sequential
set +u
source "$workspace_dir/setup_env.sh"
set -u
python -c 'import rclpy, open3d, mcubes, trimesh, perlin_numpy, py4dgeo, torch, gymnasium, stable_baselines3; from hydromining_robot.action import AssessRegion; print("核心依赖与 ROS 接口导入通过")'
python -m pip check
ros2 control --help >/dev/null
PYTHONPATH="$workspace_dir/src/hydromining_sim/scripts${PYTHONPATH:+:$PYTHONPATH}" python -m pytest src/hydromining_sim/test -q -p no:cacheprovider
echo '环境搭建与算法测试通过。新终端执行：source setup_env.sh'
