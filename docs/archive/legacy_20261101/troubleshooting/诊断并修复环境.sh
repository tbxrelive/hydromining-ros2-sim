#!/bin/bash
# 诊断并修复 ROS 2 环境问题

echo "=========================================="
echo "  ROS 2 环境诊断工具"
echo "=========================================="
echo ""

cd /home/star/SCI/hydromining_ws

# 1. 检查构建结果
echo "【1/5】检查构建结果..."
if [ ! -d "install/hydromining_robot" ]; then
    echo "✗ install/hydromining_robot 不存在"
    echo "  需要重新构建"
    exit 1
fi
echo "✓ install 目录存在"
echo ""

# 2. 检查当前环境
echo "【2/5】检查当前环境变量..."
echo "AMENT_PREFIX_PATH="
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | sed 's/^/  /'
echo ""

# 3. 清空环境重新 source
echo "【3/5】清空并重新设置环境..."
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset ROS_DISTRO

# Source 基础环境
echo "  → source /opt/ros/jazzy/setup.bash"
source /opt/ros/jazzy/setup.bash

echo "  → 基础环境设置完成"
echo "    ROS_DISTRO=$ROS_DISTRO"
echo ""

# Source 工作空间
echo "  → source install/setup.bash"
source install/setup.bash

echo "  → 工作空间环境设置完成"
echo ""

# 4. 验证环境
echo "【4/5】验证环境变量..."
echo "AMENT_PREFIX_PATH="
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | sed 's/^/  /'
echo ""

if echo "$AMENT_PREFIX_PATH" | grep -q "hydromining_ws"; then
    echo "✓ 工作空间已加入搜索路径"
else
    echo "✗ 工作空间未加入搜索路径"
    echo "  这是一个严重问题，可能需要重新构建"
fi
echo ""

# 5. 测试包可见性
echo "【5/5】测试包可见性..."
if ros2 pkg list | grep -q hydromining_robot; then
    echo "✓ hydromining_robot 包可见"
    echo ""
    echo "=========================================="
    echo "  ✓ 环境配置正确！"
    echo "=========================================="
    echo ""
    echo "现在可以启动仿真："
    echo "  ros2 launch hydromining_robot robot_simulation.launch.py"
    echo ""
else
    echo "✗ hydromining_robot 包不可见"
    echo ""
    echo "=========================================="
    echo "  问题诊断"
    echo "=========================================="
    echo ""
    echo "可能原因："
    echo "1. 构建未完成或失败"
    echo "2. package.xml 配置错误"
    echo "3. CMakeLists.txt 配置错误"
    echo ""
    echo "建议操作："
    echo "  cd /home/star/SCI/hydromining_ws"
    echo "  rm -rf build/ install/ log/"
    echo "  colcon build --packages-select hydromining_robot"
    echo "  source install/setup.bash"
    echo ""
fi
