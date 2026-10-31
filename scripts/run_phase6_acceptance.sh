#!/usr/bin/env bash
# Phase 6 real headless ROS/Gazebo acceptance. GUI is intentionally not started.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export MALLOC_ARENA_MAX=2
export MALLOC_MMAP_THRESHOLD_=131072 MALLOC_TRIM_THRESHOLD_=131072
export MAKEFLAGS='-j2 -l2' CMAKE_BUILD_PARALLEL_LEVEL=2
export HYDROMINING_PYTHON="$workspace_dir/.venv_stage4/bin/python"
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon build --base-paths src --symlink-install --packages-select hydromining_sim hydromining_robot --executor sequential
set +u
source install/setup.bash
set -u
exec "$HYDROMINING_PYTHON" test_scripts/test_phase6_acceptance.py "$@"
