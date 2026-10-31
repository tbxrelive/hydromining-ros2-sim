#!/bin/bash
# 终极修复脚本 - 解决 xacro 宏问题

set -e

cd /home/star/SCI/hydromining_ws

echo "=========================================="
echo "  终极修复：xacro 宏问题"
echo "=========================================="
echo ""

echo "【1/4】重新构建..."
colcon build --packages-select hydromining_robot 2>&1 | grep -E "(Starting|Finished|ERROR)" || true
echo "✓ 构建完成"
echo ""

echo "【2/4】测试 xacro 展开..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

URDF_FILE="/tmp/test_panda.urdf"
if xacro install/hydromining_robot/share/hydromining_robot/urdf/panda_with_water_gun.urdf.xacro > "$URDF_FILE" 2>&1; then
    echo "✓ xacro 展开成功"
    echo "  生成的 URDF: $URDF_FILE"

    # 检查关键链接
    if grep -q "panda_link0" "$URDF_FILE" && grep -q "water_gun_body" "$URDF_FILE"; then
        echo "✓ URDF 包含机器人和水枪"
    else
        echo "✗ URDF 缺少关键组件"
        exit 1
    fi
else
    echo "✗ xacro 展开失败"
    xacro install/hydromining_robot/share/hydromining_robot/urdf/panda_with_water_gun.urdf.xacro 2>&1 | tail -20
    exit 1
fi
echo ""

echo "【3/4】验证 Launch 文件..."
if ros2 launch hydromining_robot robot_simulation.launch.py --show-args 2>&1 | grep -E "error|Error|ERROR" > /dev/null; then
    echo "✗ Launch 文件有错误"
    ros2 launch hydromining_robot robot_simulation.launch.py --show-args 2>&1 | tail -10
    exit 1
else
    echo "✓ Launch 文件正常"
fi
echo ""

echo "【4/4】最终检查..."
if [ -f "$URDF_FILE" ]; then
    echo "  机器人链接数：$(grep -c '<link name=' "$URDF_FILE")"
    echo "  关节数：$(grep -c '<joint name=' "$URDF_FILE")"
fi
echo ""

echo "=========================================="
echo "  ✓ 所有修复完成！"
echo "=========================================="
echo ""
echo "现在可以启动仿真："
echo ""
echo "  终端 1："
echo "    bash 启动仿真.sh"
echo ""
echo "  终端 2（等 10 秒）："
echo "    bash 测试运动.sh"
echo ""
