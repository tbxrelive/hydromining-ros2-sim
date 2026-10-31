#!/bin/bash
# 完全重建 hydromining_robot 包

echo "=========================================="
echo "  完全重建 hydromining_robot"
echo "=========================================="
echo ""

cd /home/star/SCI/hydromining_ws

# 1. 清理旧构建
echo "【1/4】清理旧构建..."
rm -rf build/hydromining_robot install/hydromining_robot log/
echo "✓ 清理完成"
echo ""

# 2. 检查源代码
echo "【2/4】检查源代码..."
if [ ! -f "src/hydromining_robot/package.xml" ]; then
    echo "✗ package.xml 不存在"
    exit 1
fi
if [ ! -f "src/hydromining_robot/CMakeLists.txt" ]; then
    echo "✗ CMakeLists.txt 不存在"
    exit 1
fi
echo "✓ 源代码文件存在"
echo ""

# 3. 构建
echo "【3/4】开始构建..."
echo "  这可能需要 1-2 分钟..."
colcon build --packages-select hydromining_robot --event-handlers console_direct+

if [ $? -ne 0 ]; then
    echo ""
    echo "✗ 构建失败"
    echo "  请查看上面的错误信息"
    exit 1
fi

echo ""
echo "✓ 构建完成"
echo ""

# 4. 验证
echo "【4/4】验证构建结果..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

if ros2 pkg list | grep -q hydromining_robot; then
    echo "✓ hydromining_robot 包可见"
    echo ""
    echo "=========================================="
    echo "  ✓ 重建成功！"
    echo "=========================================="
    echo ""
    echo "现在可以启动仿真："
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source install/setup.bash"
    echo "  ros2 launch hydromining_robot robot_simulation.launch.py"
    echo ""
else
    echo "✗ hydromining_robot 包仍然不可见"
    echo "  这是一个严重问题，需要进一步调查"
    echo ""
    echo "请提供以下信息："
    echo "  1. cat src/hydromining_robot/package.xml"
    echo "  2. cat src/hydromining_robot/CMakeLists.txt"
    echo "  3. ls -la install/hydromining_robot/"
    echo ""
fi
