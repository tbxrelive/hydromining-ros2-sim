#!/bin/bash
# 传感器数据验证脚本

cd ~/SCI/hydromining_ws
source install/setup.bash

echo "========================================"
echo "   传感器数据频率测试"
echo "========================================"
echo ""
echo "⏳ 等待 5 秒让仿真稳定..."
sleep 5

echo ""
echo "=== 📷 相机 RGB 图像 ==="
timeout 10 ros2 topic hz /hydromining/camera/color/image_raw 2>&1 | grep "average rate" | tail -1

echo ""
echo "=== 📷 相机深度图像 ==="
timeout 10 ros2 topic hz /hydromining/camera/depth/image_rect_raw 2>&1 | grep "average rate" | tail -1

echo ""
echo "=== 📡 LiDAR 点云 ==="
timeout 10 ros2 topic hz /hydromining/lidar/points 2>&1 | grep "average rate" | tail -1

echo ""
echo "========================================"
echo "   传感器话题内容测试（获取一帧数据）"
echo "========================================"

echo ""
echo "=== 📷 相机 RGB 图像信息 ==="
timeout 5 ros2 topic echo /hydromining/camera/color/image_raw --once 2>&1 | head -10

echo ""
echo "✅ 测试完成！"
