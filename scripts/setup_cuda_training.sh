#!/usr/bin/env bash
# Separate CUDA environment. Does not replace simulation's CPU PyTorch.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
[[ -x .venv_rl/bin/python ]] || /usr/bin/python3 -m venv --system-site-packages .venv_rl
touch .venv_rl/COLCON_IGNORE
.venv_rl/bin/python -m pip install 'torch==2.5.1+cu118' --index-url https://download.pytorch.org/whl/cu118
.venv_rl/bin/python -m pip install -r requirements-rl.txt
if [[ -f requirements-rl.lock.txt ]]; then
    .venv_rl/bin/python -m pip install -r requirements-rl.lock.txt
fi
CUDA_VISIBLE_DEVICES=1,2,3 .venv_rl/bin/python - <<'PY'
import torch
assert torch.cuda.is_available()
assert 'sm_70' in torch.cuda.get_arch_list(), torch.cuda.get_arch_list()
assert torch.cuda.device_count() == 3
for index in range(3):
    with torch.cuda.device(index):
        tensor = torch.ones((32,32), device=f'cuda:{index}', requires_grad=True)
        (tensor @ tensor).sum().backward()
        assert torch.isfinite(tensor.grad).all()
        print(index, torch.cuda.get_device_name(index), 'CUDA forward/backward passed')
PY
