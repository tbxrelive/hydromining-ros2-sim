#!/usr/bin/env bash
# Complete sequential reproduction. Does not invoke the long-training scheduler.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
set +u
source setup_env.sh
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS="-j2 -l2"
exec "$HYDROMINING_PYTHON" test_scripts/reproduction_suite.py "$@"
