#!/usr/bin/env bash
# Source this file to load Jazzy, project Python packages and the built workspace.
_hydromining_load_environment() {
    local workspace_dir local_ros_prefix
    workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
    if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$workspace_dir/.venv_stage4/bin/activate" ]]; then
        echo '缺少 Jazzy 或项目虚拟环境，请先运行 scripts/setup_new_machine.sh' >&2
        return 1
    fi
    source /opt/ros/jazzy/setup.bash || return
    source "$workspace_dir/.venv_stage4/bin/activate" || return
    if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
        echo '工作区尚未构建，请先运行 scripts/setup_new_machine.sh' >&2
        return 1
    fi
    source "$workspace_dir/install/setup.bash" || return
    local_ros_prefix="$workspace_dir/.local_ros_deps/opt/ros/jazzy"
    if [[ -d "$local_ros_prefix" ]]; then
        export AMENT_PREFIX_PATH="$local_ros_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
        export PYTHONPATH="$local_ros_prefix/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
    fi
    export HYDROMINING_PYTHON="$workspace_dir/.venv_stage4/bin/python"
    export GZ_SIM_RESOURCE_PATH="$workspace_dir/src/hydromining_sim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
    export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
    export PYTHONDONTWRITEBYTECODE=1
    echo "水力采煤环境已加载：ROS $ROS_DISTRO，Python $HYDROMINING_PYTHON"
}
_hydromining_load_environment
_hydromining_setup_status=$?
unset -f _hydromining_load_environment
return "$_hydromining_setup_status" 2>/dev/null || exit "$_hydromining_setup_status"
