#!/bin/bash
# 测试机器人运动（新终端执行）

echo "=========================================="
echo "  测试机器人运动"
echo "=========================================="
echo ""

cd /home/star/SCI/hydromining_ws

# Source 环境
echo "【1/2】设置环境..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "✓ 环境已设置"
echo ""

# 测试运动
echo "【2/2】执行运动测试..."
echo ""
echo "  机器人将移动到三个位置："
echo "  1. 观测位置（煤壁前方）"
echo "  2. 喷射位置 1（上部区域）"
echo "  3. 喷射位置 2（中部区域）"
echo ""
echo "=========================================="
echo ""

ros2 run hydromining_robot test_motion.py
