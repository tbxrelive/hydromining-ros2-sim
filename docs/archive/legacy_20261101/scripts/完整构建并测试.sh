#!/bin/bash
# 完整构建并测试脚本 - 一次性解决所有问题

set -e  # 遇到错误立即停止

echo "=========================================="
echo "  完整构建与测试流程"
echo "=========================================="
echo ""

cd /home/star/SCI/hydromining_ws

# 步骤 1：完全清理
echo "【1/5】完全清理旧构建..."
rm -rf build/ install/ log/
echo "✓ 清理完成"
echo ""

# 步骤 2：构建两个包
echo "【2/5】构建 ROS 2 包..."
echo "  → 构建 hydromining_sim（仿真环境）"
echo "  → 构建 hydromining_robot（机器人系统）"
echo ""

colcon build --packages-select hydromining_sim hydromining_robot 2>&1 | tee build.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo ""
    echo "✗ 构建失败，请查看 build.log"
    exit 1
fi

echo ""
echo "✓ 构建完成"
echo ""

# 步骤 3：验证安装
echo "【3/5】验证包安装..."
if [ ! -d "install/hydromining_sim" ]; then
    echo "✗ hydromining_sim 未安装"
    exit 1
fi
if [ ! -d "install/hydromining_robot" ]; then
    echo "✗ hydromining_robot 未安装"
    exit 1
fi
echo "✓ 两个包都已安装"
echo ""

# 步骤 4：验证环境
echo "【4/5】验证 ROS 2 环境..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

if ! ros2 pkg list | grep -q hydromining_sim; then
    echo "✗ hydromining_sim 包不可见"
    exit 1
fi

if ! ros2 pkg list | grep -q hydromining_robot; then
    echo "✗ hydromining_robot 包不可见"
    exit 1
fi

echo "✓ 两个包都可见"
echo ""

# 步骤 5：验证 Launch 文件
echo "【5/5】验证 Launch 文件..."
if ros2 launch hydromining_robot robot_simulation.launch.py --show-args 2>&1 | grep -q "Error"; then
    echo "✗ Launch 文件有错误"
    exit 1
fi

echo "✓ Launch 文件正常"
echo ""

echo "=========================================="
echo "  ✓ 所有检查通过！"
echo "=========================================="
echo ""
echo "现在可以启动仿真了："
echo ""
echo "  1. 确保在工作空间根目录："
echo "     cd /home/star/SCI/hydromining_ws"
echo ""
echo "  2. Source 环境："
echo "     source /opt/ros/jazzy/setup.bash"
echo "     source install/setup.bash"
echo ""
echo "  3. 启动仿真："
echo "     ros2 launch hydromining_robot robot_simulation.launch.py"
echo ""
echo "  4. 新终端测试运动（等仿真启动后）："
echo "     cd /home/star/SCI/hydromining_ws"
echo "     source /opt/ros/jazzy/setup.bash"
echo "     source install/setup.bash"
echo "     ros2 run hydromining_robot test_motion.py"
echo ""
echo "=========================================="
