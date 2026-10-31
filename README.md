# 水力采煤机器人自主作业仿真系统

> **2026-10-30 本机环境已恢复**：当前工作目录为 `/home/star/SCI/hydromining_ws`。
> 新终端运行 `source setup_env.sh`；只读检查运行 `./scripts/check_environment.sh`。
> 依赖恢复与构建入口为 `./scripts/setup_new_machine.sh`（要求已安装 Jazzy/Harmonic）。
> Python 直接依赖见 `requirements.txt`，本机 pip 包快照见 `requirements-local.lock.txt`。
> 完整说明及验证边界见 [新机器环境搭建记录](docs/新机器环境搭建记录-2026-10-30.md)。

> **2026-10-30 全阶段复现完成**：144项回归、六工况真实闭环与GUI截图检查通过。
> 演示：`./scripts/run_gui_demo.sh`；用户长训：`./scripts/run_phase7_training.sh --output results/phase7_training`。
> 本次未长训；DDPG优劣、物理/噪声校准仍待验证。详见 [本机交付报告](docs/全阶段复现与DDPG优化-2026-10-30.md)。

[![ROS 2 Version](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo Version](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/docs/harmonic/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.12%2B-blue)](https://www.python.org/)

**Hydromining ROS 2 Gazebo Simulation System**  
面向急倾斜煤层水力采煤的机器人自主作业仿真平台

---

## 📋 项目概述

本项目是一个**完整的机器人自主作业仿真系统**，用于水力采煤场景的研究与验证。系统实现了：

- ✅ **真实煤壁几何**：基于 Perlin Noise 的粗糙表面生成
- ✅ **7-DOF 机械臂**：Franka Panda + 水枪末端执行器
- ✅ **多传感器融合**：RGB-D 相机 + VLP-16 LiDAR
- ✅ **物理喷射模拟**：Level Set 方法 + Marching Cubes 重建
- ✅ **观测干扰**：水射流显示 + 水雾/粉尘粒子效果
- ✅ **感知算法**：不确定性量化 + 表面重建 + 任务评估
- ✅ **规划决策**：NBV 主动观测 + 规则/强化学习决策

---

## 🎥 演示视频

> 📹 **待补充**：演示视频正在录制中

---

## 🚀 快速开始

### 系统要求

| 组件 | 版本 | 必需 |
|-----|------|------|
| **Ubuntu** | 24.04 LTS | ✅ |
| **ROS 2** | Jazzy | ✅ |
| **Gazebo** | Harmonic (gz-sim-8) | ✅ |
| **Python** | 3.12+ | ✅ |
| **内存** | 16 GB+（推荐 32 GB） | ✅ |
| **GPU** | 可选（用于 Gazebo GUI） | ⭕ |

### 一键部署（推荐）

```bash
# 1. 下载部署脚本
wget https://raw.githubusercontent.com/tbxrelive/hydromining-ros2-sim/main/scripts/setup_new_machine.sh

# 2. 运行（约 15-30 分钟）
chmod +x setup_new_machine.sh
./setup_new_machine.sh
```

### 手动部署

<details>
<summary>点击展开手动部署步骤</summary>

```bash
# 1. 克隆仓库
mkdir -p ~/SCI && cd ~/SCI
git clone https://github.com/tbxrelive/hydromining-ros2-sim.git hydromining_ws
cd hydromining_ws

# 2. 安装 ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz

# 3. 安装依赖
sudo apt install -y python3-pip python3-venv python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. 创建虚拟环境并安装 Python 依赖
python3 -m venv --system-site-packages .venv_stage4
source .venv_stage4/bin/activate
python -m pip install torch==2.5.1+cpu --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements.txt
python -m pip install -r requirements-local.lock.txt

# 5. 构建
colcon build --symlink-install

# 6. 激活环境
source .venv_stage4/bin/activate
source install/setup.bash
```

</details>

### 快速测试

```bash
# 激活环境
cd ~/SCI/hydromining_ws
source setup_env.sh

# 运行测试（tri_r5 工况，约 5-10 分钟）
python3 test_scripts/phase7_real_loop.py --scenario tri_r5 --max-steps 24 --output results/phase7_try_$(date +%Y%m%d_%H%M%S)
```

**预期输出**：
```
Step 0: spray 缺口最大 24.8 mm，位于 [-0.095 0.005]，喷射 7.4 s
  → status=not_achieved, lower=0.325
...
Step 6: finish 置信下界 0.921 >= 要求 0.9，收尾
  → status=achieved

✅ 工况 tri_r5：achieved，用时 7 步
```

---

## 📚 项目结构

```
hydromining_ws/
├── src/
│   ├── hydromining_robot/         # 机器人控制包
│   │   ├── urdf/                  # URDF/xacro 机器人模型
│   │   ├── config/                # 配置文件
│   │   └── launch/                # Launch 文件
│   └── hydromining_sim/           # 仿真环境包
│       ├── models/                # Gazebo 模型（煤壁等）
│       └── worlds/                # Gazebo 世界定义
├── algorithms/                    # 核心算法（无 ROS 依赖）
│   ├── perception/                # 感知算法
│   ├── erosion/                   # 喷射蚀除模拟
│   └── planning/                  # 规划决策算法
├── test_scripts/                  # 测试脚本
│   ├── phase7_real_loop.py        # 真实 Gazebo 闭环
│   ├── phase7_surrogate_loop.py   # 替身环境测试
│   └── phase7_evaluate.py         # 策略对比评估
├── docs/                          # 文档
│   ├── 阶段文档/                   # 各阶段详细文档
│   └── GitHub部署指南.md           # 部署指南
├── config/                        # 全局配置
│   └── perception/                # 感知模块配置
├── scripts/                       # 工具脚本
│   ├── setup_new_machine.sh       # 新机器部署
│   └── run_phase7_batch.sh        # 批量测试
└── results/                       # 测试结果
    └── phase7/                    # 阶段 7 验收结果
```

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                  阶段 7：规划与决策（当前）                    │
│     NBV 主动观测 + 规则/RL 决策 + 完整任务闭环                │
└─────────────────────────────────────────────────────────────┘
                              ▲
                              │
┌─────────────────────────────────────────────────────────────┐
│                  阶段 6：感知算法集成                          │
│     多传感器融合 + 表面重建 + 不确定性量化 + 任务评估           │
└─────────────────────────────────────────────────────────────┘
                              ▲
                              │
        ┌─────────────────────┴─────────────────────┐
        │                                           │
┌───────────────────┐                     ┌───────────────────┐
│  阶段 5：观测干扰  │                     │  阶段 4：喷射模拟  │
│  水射流 + 粒子    │                     │  Level Set 蚀除   │
└───────────────────┘                     └───────────────────┘
        │                                           │
        └─────────────────────┬─────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  阶段 3：传感器集成                            │
│     RGB-D + LiDAR + 点云配准（3.1 mm 精度）                   │
└─────────────────────────────────────────────────────────────┘
                              ▲
                              │
┌─────────────────────────────────────────────────────────────┐
│                  阶段 2：机器人集成                            │
│     Franka Panda 7-DOF + 水枪 + ros2_control                 │
└─────────────────────────────────────────────────────────────┘
                              ▲
                              │
┌─────────────────────────────────────────────────────────────┐
│                  阶段 1：煤壁几何生成                          │
│     Perlin Noise + 倾斜角 + 可调粗糙度                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 🎯 核心功能

### 1. 煤壁几何生成（阶段 1）

- **技术**：Perlin Noise + Fractal Noise
- **参数**：粗糙度、倾斜角、尺寸可调
- **导出**：OBJ/DAE 格式，支持 Gazebo 导入

### 2. 机器人系统（阶段 2）

- **机械臂**：Franka Panda 7-DOF（官方 URDF）
- **末端执行器**：水枪模型（visual + collision）
- **控制**：ros2_control + gz_ros2_control
- **传感器**：RGB-D 相机 + VLP-16 LiDAR

**评分**: 94/100（MoveIt 2 集成、运动验证通过）

### 3. 传感器集成（阶段 3）

- **多视角采集**：支持预定义观测位姿
- **坐标变换**：sensor_frame → world_frame
- **点云配准**：ICP 精度 **3.1 mm**（远超 50mm 目标）

**评分**: 100/100

### 4. 喷射效果模拟（阶段 4）

- **算法**：Level Set 方法 + Marching Cubes
- **物理模型**：压力相关、距离衰减、径向核
- **集成**：ROS 服务接口 + Gazebo 模型替换
- **验证**：真实 Gazebo 六步流程通过

**峰值内存**: 1940 MB  
**验收状态**: ✅ 无 GUI 通过

### 5. 观测干扰模拟（阶段 5）

- **水射流**：连续渐扩水柱 + Gazebo 粒子系统
- **干扰**：RGB/深度/点云降级（可调 4 级）
- **状态机**：准备 → 喷射 → 煤壁更新 → 消散
- **验证**：无 GUI + GUI 验收通过

**峰值内存**: 2070 MB  
**验收状态**: ✅ GUI 通过

### 6. 感知算法集成（阶段 6）

- **多传感器融合**：深度相机 + LiDAR（可见性并集）
- **不确定性量化**：σ 估计 + 置信区间
- **任务评估**：三分类判决（achieved / insufficient_evidence / not_achieved）
- **验证**：真实无 GUI 全链路通过

**峰值内存**: 2166 MB  
**评分**: 96/100

### 7. 规划与决策（阶段 7，当前）✨

#### 7.1 NBV（下一最佳视点）

- **算法**：信息增益驱动（可见性 × σ 改善预测）
- **可见性**：相机 ∪ LiDAR（解决坑底遮挡）
- **验证**：覆盖率提升 0.342 → 0.443

#### 7.2 规则决策基线

- **策略**：补看 / 喷射 / 收尾（专家知识）
- **喷射参数**：缺口驱动位置 + 深度驱动时长（5-8s）
- **判决准则**：置信下界 ≥ 0.9
- **可解释性**：每步决策附清晰理由

**真实 Gazebo 验证**：

| 工况 | 步数 | 判决 | 置信下界 |
|-----|------|------|---------|
| empty | 14 | achieved | 0.946 |
| single | 10 | achieved | 0.922 |
| pair | 15 | achieved | 0.910 |
| tri_r5 | 7 | achieved | 0.921 |
| ring_r6 | 1 | achieved | 0.926 |
| dense | 1 | achieved | 0.972 |
| **总计** | **6/6** | **100%** | **0.933** |

**性能**：
- 平均步数：8 步
- 峰值内存：2.1 GB
- 平均耗时：~16 分钟/工况

#### 7.3 强化学习（DDPG）

- **框架**：Stable-Baselines3 + Gymnasium
- **动作空间**：连续（u, v, duration）
- **状态空间**：高度图 + 不确定性 + 缺口图 + 标量特征
- **替身环境**：L1（快速训练）+ L2（高保真验证）

**当前状态**: ⚠️ 性能待优化（4/12 达成 vs 规则 12/12）

---

## 📊 性能指标

### 系统资源

| 阶段 | 峰值内存 | CPU 占用 | 单工况耗时 |
|-----|---------|---------|----------|
| 阶段 4（喷射） | 1.9 GB | 65% | ~5 分钟 |
| 阶段 5（干扰） | 2.1 GB | 70% | ~8 分钟 |
| 阶段 6（感知） | 2.2 GB | 75% | ~12 分钟 |
| 阶段 7（决策） | 2.1 GB | 67% | ~16 分钟 |

**内存保护限制**: 3072 MB（所有阶段满足）

### 算法精度

| 指标 | 目标 | 实际 |
|-----|------|------|
| **点云配准精度** | ≤ 50 mm | **3.1 mm** ✅ |
| **误达检查** | 已测场景不误报达标 | 已测场景零误达；不是普适准确率 |
| **喷射体积一致性** | 偏差 ≤ 10% | **逐位一致** ✅ |
| **NBV 覆盖率提升** | 按原始记录核查 | 历史文字 +10.1 个百分点；本轮真实增益另归档 |

---

## 📖 文档

### 快速入门

- [GitHub 部署指南](docs/GitHub部署指南.md) - 新机器部署完整流程
- [AGENTS.md](AGENTS.md) - 开发规则与约定
- [QUICKSTART.md](QUICKSTART.md) - 快速开始教程（待创建）

### 阶段文档

每个阶段包含三份文档：**设计方案** / **执行指南** / **评估报告**

- [阶段 1：煤壁生成](docs/阶段文档/阶段1-煤壁生成/)
- [阶段 2：机器人集成](docs/阶段文档/阶段2-机器人集成/)
- [阶段 3：传感器集成](docs/阶段文档/阶段3-传感器集成/)
- [阶段 4：喷射模拟](docs/阶段文档/阶段4-喷射效果模拟/)
- [阶段 5：观测干扰](docs/阶段文档/阶段5-观测干扰模拟/)
- [阶段 6：感知算法](docs/阶段文档/阶段6-感知算法集成/)
- [阶段 7：规划决策](docs/阶段文档/阶段7-规划决策/) ✨ **最新**

### 技术规范

- [鱼香ROS代码规范](docs/鱼香ROS代码规范总结.md)
- [ROS 2 Jazzy 开发规范](.claude/skills/ros2.md)
- [Humble vs Jazzy 差异](AGENTS.md#📚-版本差异速查表)

---

## 🧪 测试

### 单元测试

```bash
# 所有单元测试
python -m pytest src/hydromining_sim/test -v

# 特定模块
python -m pytest src/hydromining_sim/test/test_planning_decision.py src/hydromining_sim/test/test_nbv_observation_policy.py -v
```

### 集成测试

```bash
# 阶段 4-7 完整验收
./scripts/run_phase4_acceptance.sh   # 喷射模拟
./scripts/run_phase5_acceptance.sh   # 观测干扰
./scripts/run_phase6_acceptance.sh   # 感知算法
./scripts/run_phase7_batch.sh        # 规划决策（所有工况）
```

### 策略对比

```bash
# 三策略评估（random / rule / ddpg）
python3 test_scripts/phase7_evaluate.py --policies random rule ddpg
```

---

## 当前复现与训练入口

- 全阶段顺序复现（不长训）：`./scripts/run_full_reproduction.sh`
- Gazebo GUI规则闭环：`./scripts/run_gui_demo.sh`
- CUDA训练环境：`./scripts/setup_cuda_training.sh`
- 仅用户执行长训：`./scripts/run_phase7_training.sh --output results/phase7_training`
- 方法、证据与限制：[全阶段复现与DDPG优化](docs/全阶段复现与DDPG优化-2026-10-30.md)。

## 🛠️ 开发

### 添加新工况

1. 在 `algorithms/planning/episodes.py` 定义工况
2. 在 `test_scripts/phase7_real_loop.py` 添加测试
3. 运行验证：`python3 test_scripts/phase7_real_loop.py <new_scenario>`

### 训练自定义 RL 策略

```bash
# 修改 test_scripts/phase7_ddpg_train.py 中的奖励函数
./scripts/run_phase7_training.sh --output results/phase7_training
```

### 贡献指南

1. Fork 本仓库
2. 创建特性分支：`git checkout -b feature/new-feature`
3. 提交更改：`git commit -m "Add new feature"`
4. 推送分支：`git push origin feature/new-feature`
5. 提交 Pull Request

**开发规则**：参见 [AGENTS.md](AGENTS.md)

---

## 🤝 致谢

- **ROS 2 Jazzy**: https://docs.ros.org/en/jazzy/
- **Gazebo Harmonic**: https://gazebosim.org/docs/harmonic/
- **Franka Panda**: https://github.com/frankaemika/franka_ros2
- **Stable-Baselines3**: https://stable-baselines3.readthedocs.io/
- **鱼香ROS**: https://github.com/fishros（代码规范参考）

---

## 📄 许可证

本项目采用 [Apache License 2.0](LICENSE) 开源。

```
Copyright 2026 Star (@tbxrelive)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
```

---

## 📧 联系方式

- **维护者**: Star
- **GitHub**: [@tbxrelive](https://github.com/tbxrelive)
- **Issue**: https://github.com/tbxrelive/hydromining-ros2-sim/issues

---

## 🗺️ 路线图

### 已完成 ✅

- [x] 阶段 1：煤壁几何生成
- [x] 阶段 2：机器人集成（Panda + 水枪）
- [x] 阶段 3：传感器集成（RGB-D + LiDAR）
- [x] 阶段 4：喷射效果模拟（Level Set）
- [x] 阶段 5：观测干扰（水雾/粉尘）
- [x] 阶段 6：感知算法（融合 + 评估）
- [x] 阶段 7：规划决策（NBV + 规则基线）

### 进行中 🚧

- [ ] DDPG 性能优化（超参数调优）
- [ ] GUI 演示视频录制
- [ ] 英文文档翻译

### 未来计划 🔮

- [ ] 混合动作空间（P-DQN / TD3 / SAC）
- [ ] 多机械臂协同决策
- [ ] 真实矿井验证（缩小尺寸实验）
- [ ] 在线学习与元学习
- [ ] Docker 镜像支持

---

**最后更新**: 2026-10-29  
**版本**: v0.7.0（阶段 7 验收版）
