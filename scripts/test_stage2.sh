#!/bin/bash
# 阶段 2 测试脚本

echo "=========================================="
echo "  阶段 2 机器人仿真测试"
echo "=========================================="
echo ""

# 1. 检查环境
echo "1. 检查环境..."
cd /home/star/SCI/hydromining_ws

if [ ! -d "install" ]; then
    echo "✗ install 目录不存在，请先构建："
    echo "  colcon build --packages-select hydromining_robot"
    exit 1
fi

# 2. Source 环境
echo "2. 设置 ROS 2 环境..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 验证包是否可见
if ! ros2 pkg list | grep -q hydromining_robot; then
    echo "✗ 找不到 hydromining_robot 包"
    echo "  当前搜索路径："
    echo $AMENT_PREFIX_PATH | tr ':' '\n' | sed 's/^/    /'
    exit 1
fi

echo "✓ hydromining_robot 包已找到"
echo ""

# 3. 检查世界文件
echo "3. 检查煤壁世界文件..."
WORLD_FILE="src/hydromining_sim/worlds/coal_wall_v02.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "✗ 煤壁世界文件不存在: $WORLD_FILE"
    exit 1
fi
echo "✓ 世界文件存在"
echo ""

# 4. 启动仿真
echo "4. 启动 Gazebo 仿真..."
echo "   提示：这将启动 Gazebo，可能需要 10-30 秒"
echo "   关闭 Gazebo 窗口可停止仿真"
echo ""

ros2 launch hydromining_robot robot_simulation.launch.py
