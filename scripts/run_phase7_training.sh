#!/usr/bin/env bash
# Explicit USER-RUN long experiments. This script is never launched by acceptance.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
set +u
source setup_env.sh
set -u
[[ -x .venv_rl/bin/python ]] || { echo 'Run scripts/setup_cuda_training.sh first' >&2; exit 1; }
exec .venv_rl/bin/python test_scripts/phase7_training_jobs.py "$@"
