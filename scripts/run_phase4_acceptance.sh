#!/usr/bin/env bash
# One command: bounded build, then six-step acceptance. GUI only via --gui.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export MAKEFLAGS='-j2 -l2'
export CMAKE_BUILD_PARALLEL_LEVEL=2
export HYDROMINING_PYTHON="$workspace_dir/.venv_stage4/bin/python"
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon build --base-paths src --symlink-install --packages-select hydromining_sim hydromining_robot --executor sequential --parallel-workers 1
set +u
source install/setup.bash
set -u
exec "$HYDROMINING_PYTHON" test_scripts/test_phase4_acceptance.py "$@"
