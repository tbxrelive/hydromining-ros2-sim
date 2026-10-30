#!/bin/bash
# 设置 Gazebo 环境变量

# ROS 2 环境
source /opt/ros/jazzy/setup.bash

# 设置 Gazebo 模型路径
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/star/SCI/hydromining_ws/src/hydromining_sim/models

echo "环境已设置："
echo "  GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
