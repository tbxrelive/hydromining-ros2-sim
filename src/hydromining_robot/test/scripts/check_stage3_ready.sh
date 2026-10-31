#!/bin/bash
# 阶段 3 环境检查脚本

echo "==================================="
echo "阶段 3 环境检查"
echo "==================================="
echo ""

cd ~/SCI/hydromining_ws

# 检查 1: 项目构建
echo "[1/6] 检查项目构建..."
if [ -d "install/hydromining_robot" ]; then
    echo "✓ hydromining_robot 已构建"
else
    echo "✗ hydromining_robot 未构建"
    echo "   运行: colcon build --packages-select hydromining_robot --symlink-install"
    exit 1
fi

# 检查 2: Python 依赖
echo ""
echo "[2/6] 检查 Python 依赖..."
python3 -c "import sensor_msgs_py" 2>/dev/null && echo "✓ sensor_msgs_py" || echo "✗ sensor_msgs_py 缺失"
python3 -c "import open3d" 2>/dev/null && echo "✓ open3d" || echo "⚠ open3d 缺失（配准验证需要）"

# 检查 3: 脚本文件
echo ""
echo "[3/6] 检查脚本文件..."
scripts=(
    "scripts/pointcloud_capture.py"
    "test/scripts/test_observation.py"
    "test/scripts/verify_registration.py"
)
for script in "${scripts[@]}"; do
    if [ -x "src/hydromining_robot/$script" ]; then
        echo "✓ $script"
    else
        echo "✗ $script 不存在或不可执行"
    fi
done

# 检查 4: 测试数据目录
echo ""
echo "[4/6] 检查测试数据目录..."
if [ -d "src/hydromining_robot/test/data" ]; then
    echo "✓ test/data 目录存在"
    count=$(ls -1 src/hydromining_robot/test/data/*.pcd 2>/dev/null | wc -l)
    if [ $count -gt 0 ]; then
        echo "   已有 $count 个 PCD 文件"
    fi
else
    echo "⚠ test/data 目录不存在，将自动创建"
fi

# 检查 5: ROS 2 依赖
echo ""
echo "[5/6] 检查 ROS 2 包..."
source install/setup.bash 2>/dev/null
ros2 pkg list 2>/dev/null | grep -q "tf2_sensor_msgs" && echo "✓ tf2_sensor_msgs" || echo "✗ tf2_sensor_msgs 缺失"
ros2 pkg list 2>/dev/null | grep -q "sensor_msgs_py" && echo "✓ sensor_msgs_py" || echo "✗ sensor_msgs_py 缺失"

# 检查 6: Launch 文件
echo ""
echo "[6/6] 检查 Launch 文件..."
if [ -f "src/hydromining_robot/launch/robot_simulation.launch.py" ]; then
    echo "✓ robot_simulation.launch.py"
else
    echo "✗ robot_simulation.launch.py 不存在"
fi

echo ""
echo "==================================="
echo "环境检查完成"
echo "==================================="
echo ""
echo "如果所有检查通过，执行:"
echo "  ros2 launch hydromining_robot robot_simulation.launch.py"
echo "  ros2 run hydromining_robot pointcloud_capture.py"
echo "  ros2 run hydromining_robot test_observation.py"
