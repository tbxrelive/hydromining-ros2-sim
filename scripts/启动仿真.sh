#!/bin/bash
# 一键启动仿真（最终版本）

echo "=========================================="
echo "  启动水力采煤仿真系统"
echo "=========================================="
echo ""

cd /home/star/SCI/hydromining_ws

# 检查环境
if [ ! -d "install/hydromining_robot" ] || [ ! -d "install/hydromining_sim" ]; then
    echo "✗ 包未安装，请先运行："
    echo "  bash 完整构建并测试.sh"
    exit 1
fi

# Source 环境
echo "【1/2】设置环境..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "✓ 环境已设置"
echo ""

# 启动仿真
echo "【2/2】启动 Gazebo 仿真..."
echo ""
echo "  → 煤壁模型加载中..."
echo "  → 机器人系统初始化中..."
echo "  → 控制器启动中..."
echo ""
echo "=========================================="
echo "  仿真窗口即将打开"
echo "=========================================="
echo ""
echo "测试运动（在新终端执行）："
echo "  cd /home/star/SCI/hydromining_ws"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 run hydromining_robot test_motion.py"
echo ""
echo "按 Ctrl+C 停止仿真"
echo "=========================================="
echo ""

ros2 launch hydromining_robot robot_simulation.launch.py
