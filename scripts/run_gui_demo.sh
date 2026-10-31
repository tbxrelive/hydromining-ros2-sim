#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
set +u
source setup_env.sh
set -u
export CUDA_VISIBLE_DEVICES=0
exec "$HYDROMINING_PYTHON" test_scripts/phase7_real_loop.py --scenario tri_r5 --gui --keep-open \
    --task large --max-steps 128 --output "results/gui_demo_$(date +%Y%m%d_%H%M%S)" "$@"
