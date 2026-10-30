#!/bin/bash
# 环境检查脚本

echo "================================"
echo "  Hydromining Workspace 检查"
echo "================================"
echo ""

# 检查虚拟环境
echo "1. Python 虚拟环境"
if [ -d "venv" ]; then
  echo "   ✓ venv 目录存在"
  source venv/bin/activate
  echo "   ✓ 虚拟环境已激活"

  # 检查依赖
  for pkg in trimesh perlin_numpy numpy scipy networkx pyyaml; do
    if python -c "import ${pkg}" 2>/dev/null; then
      echo "   ✓ $pkg 已安装"
    else
      echo "   ✗ $pkg 未安装"
    fi
  done
else
  echo "   ✗ venv 目录不存在"
fi
echo ""

# 检查项目结构
echo "2. 项目文件结构"
files=(
  "src/hydromining_sim/scripts/generate_coal_wall.py"
  "src/hydromining_sim/models/coal_wall/model.config"
  "src/hydromining_sim/models/coal_wall/model.sdf"
  "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"
  "src/hydromining_sim/worlds/coal_wall_v02.sdf"
  "assets/walls/presets.yaml"
  "setup_env.sh"
)

for file in "${files[@]}"; do
  if [ -f "$file" ]; then
    echo "   ✓ $file"
  else
    echo "   ✗ $file (缺失)"
  fi
done
echo ""

# 检查 ROS 2 和 Gazebo
echo "3. ROS 2 & Gazebo"
if command -v gz &> /dev/null; then
  echo "   ✓ Gazebo 可用"
  gz --version | head -1
else
  echo "   ✗ Gazebo 不可用"
fi

if [ -d "/opt/ros/jazzy" ]; then
  echo "   ✓ ROS 2 Jazzy 已安装"
else
  echo "   ✗ ROS 2 Jazzy 未安装"
fi
echo ""

# 检查煤壁网格
echo "4. 煤壁网格"
if [ -f "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj" ]; then
  mesh_size=$(wc -l < src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj)
  echo "   ✓ coal_wall.obj 存在 ($mesh_size 行)"

  # 统计顶点和面片
  vertices=$(grep -c "^v " src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj || echo 0)
  faces=$(grep -c "^f " src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj || echo 0)
  echo "     - 顶点: $vertices"
  echo "     - 面片: $faces"
else
  echo "   ✗ coal_wall.obj 不存在"
fi
echo ""

# 检查环境变量
echo "5. 环境变量"
echo "   GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
if [[ "$GZ_SIM_RESOURCE_PATH" == *"hydromining_sim/models"* ]]; then
  echo "   ✓ 包含 hydromining_sim/models"
else
  echo "   ⚠ 未包含 hydromining_sim/models (需要运行 source setup_env.sh)"
fi
echo ""

# 快速命令提示
echo "================================"
echo "  快速开始命令"
echo "================================"
echo ""
echo "# 激活环境"
echo "source venv/bin/activate"
echo "source setup_env.sh"
echo ""
echo "# 生成煤壁（选一种）"
echo "cd src/hydromining_sim/scripts"
echo "python generate_from_preset.py rough"
echo "# 或"
echo "./rebuild_coal_wall.sh --roughness 0.2 --seed 99"
echo ""
echo "# 启动 Gazebo"
echo "cd /home/star/SCI/hydromining_ws"
echo "gz sim src/hydromining_sim/worlds/coal_wall_v02.sdf"
echo ""
