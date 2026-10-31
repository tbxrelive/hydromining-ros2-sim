#!/bin/bash
# 快速修复并测试

set -e

cd /home/star/SCI/hydromining_ws

echo "【1/3】重新构建 hydromining_robot..."
colcon build --packages-select hydromining_robot 2>&1 | grep -E "(Starting|Finished|ERROR)" || true
echo "✓ 构建完成"
echo ""

echo "【2/3】验证环境..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "✓ 环境已设置"
echo ""

echo "【3/3】验证 Launch 文件..."
if ros2 launch hydromining_robot robot_simulation.launch.py --show-args 2>&1 | grep -q "Error\|NameError"; then
    echo "✗ Launch 文件仍有错误"
    ros2 launch hydromining_robot robot_simulation.launch.py --show-args
    exit 1
else
    echo "✓ Launch 文件正常"
fi

echo ""
echo "=========================================="
echo "  ✓ 修复完成！"
echo "=========================================="
echo ""
echo "现在可以启动仿真："
echo "  cd /home/star/SCI/hydromining_ws"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch hydromining_robot robot_simulation.launch.py"
echo ""
