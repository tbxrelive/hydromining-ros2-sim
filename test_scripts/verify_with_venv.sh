#!/bin/bash
# 自动在虚拟环境中验证点云配准
# 如果虚拟环境不存在会自动创建

WS_DIR=~/SCI/hydromining_ws
VENV_DIR=$WS_DIR/venv_test

cd $WS_DIR

# 自动查找点云数据目录（优先使用最新的）
if [ -d "$WS_DIR/src/hydromining_robot/test/data" ] && [ $(ls $WS_DIR/src/hydromining_robot/test/data/*.pcd 2>/dev/null | wc -l) -ge 2 ]; then
  DATA_DIR="$WS_DIR/src/hydromining_robot/test/data"
elif [ -d ~/pointcloud_data ] && [ $(ls ~/pointcloud_data/*.pcd 2>/dev/null | wc -l) -ge 2 ]; then
  DATA_DIR=~/pointcloud_data
else
  echo "❌ 错误：找不到足够的点云文件"
  echo ""
  echo "请先运行观测测试："
  echo "  ./test_stage3_complete.sh"
  exit 1
fi

echo "使用点云数据目录: $DATA_DIR"
PCD_COUNT=$(ls $DATA_DIR/*.pcd 2>/dev/null | wc -l)
echo "找到 $PCD_COUNT 个点云文件"
echo ""

# 如果虚拟环境不存在，创建并安装依赖
if [ ! -d "$VENV_DIR" ]; then
  echo "=========================================="
  echo "  首次运行：创建虚拟环境"
  echo "=========================================="
  python3 -m venv $VENV_DIR
  source $VENV_DIR/bin/activate
  echo "安装 Open3D（约 400 MB，需要几分钟）..."
  pip install open3d numpy --quiet
  echo "✅ 虚拟环境已准备就绪"
  echo ""
else
  source $VENV_DIR/bin/activate
fi

# 运行验证
echo "=========================================="
echo "  验证点云配准质量"
echo "=========================================="
python3 src/hydromining_robot/test/scripts/verify_registration.py $DATA_DIR

deactivate
