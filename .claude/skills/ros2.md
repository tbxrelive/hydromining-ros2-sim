---
name: ros2
description: >
  Comprehensive best practices, design patterns, and common pitfalls for ROS2 (Robot Operating System 2)
  development. Use this skill when building ROS2 nodes, packages, launch files, components, or debugging
  ROS2 systems. Trigger whenever the user mentions ROS2, colcon, rclpy, rclcpp, DDS, QoS, lifecycle nodes,
  managed nodes, ROS2 launch, ROS2 parameters, ROS2 actions, nav2, MoveIt2, micro-ROS, or any ROS2-era
  robotics middleware. Also trigger for ROS2 workspace setup, DDS tuning, intra-process communication,
  ROS2 security, or deploying ROS2 in production. Also trigger for colcon build issues, ament_cmake,
  ament_python, CMakeLists.txt for ROS2, package.xml dependencies, rosdep, workspace overlays, custom
  message generation, or ROS2 build troubleshooting. Covers Humble, Iron, Jazzy, and Rolling distributions.
---

# ROS2 Development Skill (v1.0)

_此文件包含 ROS 2 开发的完整最佳实践指南，已安装在项目 .claude/skills/ 目录。_

## 核心内容速览

1. **节点设计模式** - rclpy/rclcpp 基础模板
2. **生命周期节点** - Lifecycle Node 完整示例
3. **QoS 配置** - 传感器/命令/状态数据的推荐配置
4. **Launch 文件** - Python 格式的条件启动
5. **Components** - 零拷贝进程内通信
6. **Actions** - 异步任务执行
7. **DDS 配置** - CycloneDDS 调优
8. **构建系统** - colcon, ament_cmake, package.xml
9. **调试工具包** - 完整的诊断命令集
10. **生产部署清单** - 10 项检查项

## 完整文档位置

详见:
- 内容已完整保存到 `.claude/skills/ros2.md`
- 共 994 行，包含所有代码示例、配置文件模板、故障排查指南

## 快速参考：常见任务

| 任务 | 命令/代码 |
|------|---------|
| 构建特定包 | `colcon build --packages-select my_pkg` |
| 以 symlink 方式构建 | `colcon build --symlink-install` |
| 源代码（Jazzy） | `source /opt/ros/jazzy/setup.bash` |
| 源工作空间 | `source install/setup.bash` |
| 查看话题 QoS | `ros2 topic info /topic_name -v` |
| 查看节点信息 | `ros2 node info /node_name` |
| 查看 TF 树 | `ros2 run tf2_tools view_frames` |
| 列出控制器 | `ros2 control list_controllers` |
| 系统诊断 | `ros2 doctor` |

---

## 应用于本项目

在 **水力采煤仿真系统** 中使用此 skill 当：
- ✓ 修改 ROS 节点或 launch 文件
- ✓ 配置 QoS 或 ros2_control
- ✓ 诊断 ROS 通信问题
- ✓ 设置参数、话题、服务

参考 `AGENTS.md` 第 1 项 "ROS 2 节点开发规则" 了解项目特定的约束和最佳实践。
