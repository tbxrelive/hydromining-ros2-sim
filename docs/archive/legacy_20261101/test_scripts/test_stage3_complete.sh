#!/bin/bash
# 阶段 3 完整测试脚本（经过验证）
#
# 使用方法：
#   终端 1: 启动 Gazebo
#     source ~/SCI/hydromining_ws/install/setup.bash
#     ros2 launch hydromining_robot robot_simulation.launch.py
#
#   终端 2: 运行本脚本
#     ./test_stage3_complete.sh

set -e

WS_DIR=~/SCI/hydromining_ws
cd $WS_DIR

echo "=========================================="
echo "  阶段 3 完整测试（已验证版本）"
echo "=========================================="

# 1. 检查 Gazebo 是否运行
echo "
1. 检查仿真环境..."
source install/setup.bash
if ! ros2 topic list | grep -q "/joint_states"; then
    echo "❌ 错误：Gazebo 未运行！"
    echo "   请在另一个终端先启动："
    echo "   ros2 launch hydromining_robot robot_simulation.launch.py"
    exit 1
fi
echo "✓ Gazebo 正在运行"

# 2. 清理旧数据
echo "
2. 清理旧的测试数据..."
rm -rf ~/pointcloud_data/*.pcd 2>/dev/null || true
mkdir -p ~/pointcloud_data
echo "✓ 已清理"

# 3. 启动点云采集节点（后台）
echo "
3. 启动点云采集节点..."
timeout 120 python3 src/hydromining_robot/scripts/pointcloud_capture.py > /tmp/capture.log 2>&1 &
CAPTURE_PID=$!
echo "   PID: $CAPTURE_PID"
sleep 3

# 验证节点启动
if ! ros2 node list | grep -q "pointcloud_capture"; then
    echo "❌ 错误：采集节点启动失败"
    cat /tmp/capture.log
    exit 1
fi
echo "✓ 采集节点已启动"

# 4. 运行观测测试
echo "
4. 运行多视角观测测试..."
timeout 60 python3 src/hydromining_robot/test/scripts/test_observation.py
TEST_EXIT=$?

# 停止采集节点
kill $CAPTURE_PID 2>/dev/null || true
wait $CAPTURE_PID 2>/dev/null || true

if [ $TEST_EXIT -ne 0 ]; then
    echo "❌ 测试失败"
    exit 1
fi

# 5. 检查结果
echo "
=========================================="
echo "  测试结果"
echo "=========================================="

NUM_PCD=$(ls ~/pointcloud_data/*.pcd 2>/dev/null | wc -l)
if [ $NUM_PCD -ge 2 ]; then
    echo "✅ 采集了 $NUM_PCD 个点云文件"
    ls -lh ~/pointcloud_data/*.pcd | tail -3
else
    echo "❌ 点云文件不足（需要 ≥2，实际 $NUM_PCD）"
    exit 1
fi

echo "
=========================================="
echo "  测试成功！"
echo "=========================================="
echo ""
echo "下一步：验证点云配准"
echo "  cd ~/SCI/hydromining_ws"
echo "  ./verify_with_venv.sh"
