# Hydromining Workspace 快速开始

## 阶段 5 当前入口（2026-10-26）

实时水柱、飞溅、水雾和粉尘已接入；原生粒子对 RGB、深度、RGB-D 点云与 LiDAR 的无 GUI 对照验证通过。物理参数未标定，GUI 最终效果待用户验收。

```bash
./scripts/run_phase5_acceptance.sh --output /tmp/hydromining_phase5_acceptance

# 显式 GUI 验收；Ctrl+C 清理本次启动的仿真
./scripts/run_phase5_acceptance.sh --gui --keep-open --output /tmp/hydromining_phase5_gui
```

[阶段五设计与使用](docs/阶段文档/阶段5-观测干扰模拟/01-设计与使用.md) · [进度交接](docs/阶段文档/阶段5-观测干扰模拟/00-进度交接.md)。结果在 `results/phase5_acceptance/2026-10-26/`。

## 阶段 4 当前入口（2026-10-26）

用户决定阶段四先通过；冲蚀参数标定、独立碰撞接触等遗留项继续保留。阶段五接入后六步兼容回归通过。

```bash
# 工作区根目录；自动构建、启动、验证并清理自己的测试进程
./scripts/run_phase4_acceptance.sh

# 用户最终可视化验收；本轮开发未启动 GUI
./scripts/run_phase4_acceptance.sh --gui --keep-open --output /tmp/hydromining_gui_acceptance
```

当前结果：`results/phase4_acceptance/2026-10-26/comparison.png`、`summary.json`；完整证据和适用边界见 `docs/阶段文档/阶段4-喷射效果模拟/08-六步无GUI集成验证报告.md`。

以下为早期快速开始资料，阶段 4 请优先使用上述统一入口。


---
## 项目概述

面向急倾斜煤层水力采煤的机器人自主作业仿真系统。

## 已完成阶段

### ✓ 阶段 1：煤壁几何生成
- Perlin Noise 生成真实凹凸煤壁
- 6 个预设配置（smooth, rough, steep, large, thin, default）
- Gazebo 集成，深灰色材质，真实碰撞

### ✅ 阶段 2：机器人运动控制（已完成）
- Panda 7-DOF 机械臂（moveit_resources）
- 自建水枪末端执行器（TCP 定义明确）
- RGB-D 相机 + 16 线激光雷达
- ros2_control + gz_ros2_control 插件
- ✅ **运动控制系统验证成功**（2026-10-22）

## 快速启动

### 环境设置

```bash
cd /home/star/SCI/hydromining_ws
source install/setup.bash
```

### 阶段 1：查看煤壁

```bash
# 生成煤壁（如需重新生成）
source venv/bin/activate
cd src/hydromining_sim/scripts
python generate_from_preset.py default

# 启动 Gazebo
source /home/star/SCI/hydromining_ws/setup_env.sh
gz sim src/hydromining_sim/worlds/coal_wall_v02.sdf
```

### 阶段 2：运行机器人仿真 ✅

```bash
# 终端 1：启动完整仿真（Gazebo + 机器人 + 控制器）
cd /home/star/SCI/hydromining_ws
source install/setup.bash
source setup_env.sh
ros2 launch hydromining_robot robot_simulation.launch.py

# 终端 2：测试机器人运动（等待 10-15 秒后启动）
cd /home/star/SCI/hydromining_ws
source install/setup.bash
ros2 run hydromining_robot test_motion.py
# 预期：机器人执行 5 段平滑运动，总时长 18 秒

# 终端 3：查看传感器数据
source install/setup.bash
ros2 topic hz /hydromining/camera/color/image_raw
ros2 topic hz /hydromining/lidar/points
ros2 topic echo /joint_states --once
```

**验证要点**：
- ✅ Gazebo 中机器人清晰可见
- ✅ 运动平滑，无碰撞
- ✅ 控制器状态为 active

## 项目结构

```
hydromining_ws/
├── src/
│   ├── hydromining_sim/          # 阶段 1：仿真环境
│   │   ├── models/coal_wall/
│   │   ├── worlds/
│   │   └── scripts/              # 煤壁生成工具
│   └── hydromining_robot/        # 阶段 2：机器人系统
│       ├── models/               # panda_robot + water_gun
│       ├── urdf/                 # 组合模型 + 传感器
│       ├── config/               # ros2_control
│       ├── launch/               # 启动文件
│       └── scripts/              # test_motion.py
├── docs/                         # 文档中心
│   ├── 阶段文档/                # 按阶段组织的设计与评估
│   ├── 技术笔记/                # 调试记录、经验总结
│   │   └── 运动控制调试记录.md  # ⭐ QoS+时间戳修复
│   ├── 参考资料/                # 外部文档、API 引用
│   └── archive/                 # 历史版本归档
├── troubleshooting/              # 故障排查手册
├── 阶段2最终测试指南.md          # 当前阶段验收标准
└── venv/                         # Python 虚拟环境
```

## 验收测试

### 阶段 1 测试

启动 Gazebo 后应看到：
- ✓ 灰色地面上的深灰色倾斜煤壁
- ✓ 表面有明显凹凸起伏
- ✓ 从侧面可见厚度
- ✓ 无白色或散状显示

### 阶段 2 测试 ✅

启动仿真后应看到：
- ✅ Panda 机器人 + 黄色喷嘴水枪
- ✅ 机器人前方 3m 处的煤壁
- ✅ 传感器话题正常发布（30Hz 相机，10Hz 雷达）
- ✅ test_motion.py 控制机器人执行 5 段平滑运动（18 秒）

**验证运动成功**：
```bash
# 监控关节状态变化
ros2 topic echo /joint_states --field position[0]

# 检查控制器
ros2 control list_controllers
# 预期：panda_arm_controller [active]
```

**关键技术点**（详见 `docs/技术笔记/运动控制调试记录.md`）：
- QoS 配置必须为 BEST_EFFORT 匹配控制器
- Duration 时间戳需正确处理纳秒字段
- 发布前需等待 DDS 连接建立（sleep 0.5s）

检查命令：
```bash
# TF 树
ros2 run tf2_tools view_frames

# 话题列表
ros2 topic list | grep -E "joint_states|camera|lidar"

# 控制器状态
ros2 control list_controllers
```

## 下一步

**✅ 阶段 2 已完成**（2026-10-22）：
- 运动控制系统验证成功
- 所有关键技术问题已解决
- 完整文档已记录

**⏳ 阶段 3：感知与喷射集成**
- 传感器数据处理节点
- 煤壁表面识别算法
- 喷射效果模拟
- 动态场景更新

## 文档索引

- **阶段 1**：
  - 使用指南：`docs/coal_wall_generation.md`
  - 完成总结：`docs/stage1_summary.md`
  
- **阶段 2**：
  - 设计文档：`docs/stage2_design.md`
  - 评估报告：`docs/stage2_review.md` ⭐

- **故障排查**：
  - 阶段 1 修复：`troubleshooting/STAGE1_FIX_SUMMARY.md`

## 常见问题

**Q: Gazebo 找不到煤壁模型？**
```bash
source setup_env.sh  # 设置 GZ_SIM_RESOURCE_PATH
```

**Q: 机器人启动失败？**
```bash
# 检查控制器
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

**Q: 传感器话题没有数据？**
```bash
# 检查 ros_gz_bridge
ros2 node info /ros_gz_bridge
```

---

**当前版本**: 阶段 2 完成  
**最后更新**: 2026-10-22  
**状态**: ✓ 运行正常
