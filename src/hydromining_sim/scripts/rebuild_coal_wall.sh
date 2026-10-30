#!/bin/bash
# 快速生成并加载新的煤壁配置

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

# 激活虚拟环境
source "$WORKSPACE_ROOT/venv/bin/activate"

# 默认参数
WIDTH=3.0
HEIGHT=3.0
THICKNESS=0.4
TILT=20.0
ROUGHNESS=0.12
RESOLUTION=80
SEED=42

# 解析命令行参数
while [[ $# -gt 0 ]]; do
  case $1 in
    --width) WIDTH="$2"; shift 2 ;;
    --height) HEIGHT="$2"; shift 2 ;;
    --thickness) THICKNESS="$2"; shift 2 ;;
    --tilt) TILT="$2"; shift 2 ;;
    --roughness) ROUGHNESS="$2"; shift 2 ;;
    --resolution) RESOLUTION="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --help)
      echo "用法: $0 [选项]"
      echo "选项:"
      echo "  --width <值>        煤壁宽度 (m), 默认: 3.0"
      echo "  --height <值>       煤壁高度 (m), 默认: 3.0"
      echo "  --thickness <值>    煤壁厚度 (m), 默认: 0.4"
      echo "  --tilt <值>         倾斜角度 (度), 默认: 20.0"
      echo "  --roughness <值>    表面粗糙度 (m), 默认: 0.12"
      echo "  --resolution <值>   网格分辨率, 默认: 80"
      echo "  --seed <值>         随机种子, 默认: 42"
      exit 0
      ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

# 生成网格
echo "正在生成煤壁网格..."
python "$SCRIPT_DIR/generate_coal_wall.py" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --thickness "$THICKNESS" \
  --tilt "$TILT" \
  --roughness "$ROUGHNESS" \
  --resolution "$RESOLUTION" \
  --seed "$SEED" \
  --output "$WORKSPACE_ROOT/src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"

if [ $? -eq 0 ]; then
  echo ""
  echo "✓ 煤壁生成完成！"
  echo ""
  echo "启动 Gazebo 预览："
  echo "  cd $WORKSPACE_ROOT"
  echo "  source setup_env.sh"
  echo "  gz sim src/hydromining_sim/worlds/coal_wall_v02.sdf"
else
  echo "✗ 煤壁生成失败"
  exit 1
fi
