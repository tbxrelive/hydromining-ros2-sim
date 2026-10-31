#!/usr/bin/env bash
# Sequential real Gazebo regression, no RL training.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
set +u
source setup_env.sh
set -u
output_dir="${1:-results/phase7_batch_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$output_dir"
for scenario in empty single pair tri_r5 ring_r6 dense; do
    "$HYDROMINING_PYTHON" test_scripts/phase7_real_loop.py --scenario "$scenario" \
        --max-steps 24 --output "$output_dir/$scenario" > "$output_dir/$scenario.log" 2>&1
done
python - "$output_dir" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]);report={}
for name in ['empty','single','pair','tri_r5','ring_r6','dense']:
    data=json.loads((root/name/'summary.json').read_text())
    assert data['passed'] and data['truth']['status']=='achieved',name
    report[name]={k:data[k] for k in ['passed','steps','verdict','truth','aggregate_peak_rss_mib']}
(root/'summary.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
PY
