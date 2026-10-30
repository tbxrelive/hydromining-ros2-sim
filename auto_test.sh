#!/bin/bash
# 自动化测试脚本 - 阶段 3 传感器集成测试
# 在一个终端中顺序执行所有测试步骤

set -e  # 遇到错误立即退出

WS_DIR=~/SCI/hydromining_ws
DATA_DIR=$WS_DIR/src/hydromining_robot/test/data

cd $WS_DIR
source install/setup.bash

echo "=========================================="
echo "  阶段 3 自动化测试"
echo "=========================================="
echo ""

# 清理旧数据
echo "1. 清理旧的测试数据..."
rm -f $DATA_DIR/*.pcd
echo "✓ 已清理"
echo ""

# 启动仿真（后台）
echo "2. 启动仿真环境（后台）..."
ros2 launch hydromining_robot robot_simulation.launch.py > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
echo "   进程 PID: $GAZEBO_PID"
echo "   等待 Gazebo 启动..."
sleep 15
echo "✓ 仿真已启动"
echo ""

# 启动点云采集节点（后台）
echo "3. 启动点云采集节点（后台）..."
ros2 run hydromining_robot pointcloud_capture.py \
  --ros-args -p output_dir:=$DATA_DIR > /tmp/capture.log 2>&1 &
CAPTURE_PID=$!
echo "   进程 PID: $CAPTURE_PID"
sleep 3
echo "✓ 采集节点已启动"
echo ""

# 运行观测测试
echo "4. 运行多视角观测测试..."
ros2 run hydromining_robot test_observation.py
TEST_RESULT=$?

echo ""
echo "=========================================="
echo "  测试结果"
echo "=========================================="

if [ $TEST_RESULT -eq 0 ]; then
    echo "✅ 观测测试通过"

    # 统计点云文件
    PCD_COUNT=$(find $DATA_DIR -name "*.pcd" 2>/dev/null | wc -l)
    echo "✅ 采集了 $PCD_COUNT 个点云文件"

    if [ $PCD_COUNT -ge 2 ]; then
        echo ""
        echo "=========================================="
        echo "  验证点云配准（可选）"
        echo "=========================================="
        echo ""
        echo "点云文件列表："
        ls -lh $DATA_DIR/*.pcd
        echo ""
        echo "运行验证脚本："
        echo "  ./verify_with_venv.sh"
    fi
else
    echo "❌ 观测测试失败"
    echo ""
    echo "查看日志："
    echo "  cat /tmp/capture.log  # 采集节点日志"
    echo "  cat /tmp/gazebo.log   # 仿真日志"
fi

# 清理后台进程
echo ""
echo "=========================================="
echo "  清理进程"
echo "=========================================="
echo "停止采集节点（PID: $CAPTURE_PID）..."
kill $CAPTURE_PID 2>/dev/null || true
sleep 1

echo "停止仿真环境（PID: $GAZEBO_PID）..."
kill $GAZEBO_PID 2>/dev/null || true
sleep 2

# 强制清理所有相关进程
pkill -f "gz sim" 2>/dev/null || true
pkill -f "pointcloud_capture" 2>/dev/null || true

echo "✓ 已清理"
echo ""
echo "=========================================="
echo "  测试完成"
echo "=========================================="
