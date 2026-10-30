#!/bin/bash
cd /home/star/SCI/hydromining_ws
source venv/bin/activate
python src/hydromining_sim/scripts/generate_coal_wall.py \
  --output src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj \
  --roughness 0.12 \
  --resolution 80 \
  --seed 42
