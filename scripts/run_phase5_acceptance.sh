#!/usr/bin/env bash
# One bounded launch; GUI is exclusively opt-in.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
# Solver callbacks run on several executor threads. Bound glibc's per-thread
# allocation arenas so released BVH/NumPy workspaces do not multiply with them.
# This affects only this session and its descendants; the configured RSS/free-RAM guards stay on.
export MALLOC_ARENA_MAX=2
# Keep large temporary arrays separately mapped, so free returns their pages
# instead of raising glibc's adaptive threshold and retaining them in arenas.
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
exec "$HYDROMINING_PYTHON" test_scripts/test_phase5_acceptance.py "$@"
