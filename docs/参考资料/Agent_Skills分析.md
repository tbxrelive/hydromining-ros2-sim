# Agent Skills 分析与选择

## 我们的 SCI 研究方向

**核心问题**：机器人在干扰观测、动态环境、不确定反馈条件下的自主作业能力

**三个研究环节**：
1. **感知**：干扰条件下的煤壁状态与作业效果可信估计
2. **规划**：面向作业确认需求的主动观测与运动规划
3. **决策与执行**：基于效果反馈的自适应喷射动作选择

## 参考项目分析

### 1. obra/superpowers

**项目定位**：通用 AI Agent 能力增强工具集

**主要功能**：
- 文件操作、代码生成、测试执行
- 适用于软件开发 Agent

**适用性评估**：❌ 不适用
- 面向软件开发，不涉及机器人或仿真
- 我们不需要代码生成 skills
- 与我们的研究方向无关

---

### 2. arpitg1304/robotics-agent-skills

**项目定位**：机器人领域的 Agent Skills

**假设包含内容**（需验证）：
- 运动规划 skills
- 感知处理 skills
- 机器人控制 skills

**适用性评估**：⭐⭐⭐ 可能高度相关

**需要确认**：
1. 是否包含**主动感知/NBV（Next Best View）规划** skills？
   - 对应我们的规划环节

2. 是否包含**不确定性处理** skills？
   - 对应我们的感知环节（置信度估计）

3. 是否包含**强化学习/自适应决策** skills？
   - 对应我们的决策环节

4. 是否支持 **ROS 2 / Gazebo** 集成？
   - 我们的仿真环境

**建议**：
- 需要详细查看该仓库的具体 skills 清单
- 如果有相关 skills，可以作为参考或直接集成

---

### 3. K-Dense-AI/scientific-agent-skills

**项目定位**：科学研究领域的 Agent Skills

**假设包含内容**（需验证）：
- 数据分析 skills
- 实验设计 skills
- 论文写作 skills
- 结果可视化 skills

**适用性评估**：⭐⭐ 部分相关

**可能有用的部分**：
1. **数据分析 skills**
   - 实验结果统计分析
   - 对比实验数据处理
   - 性能指标计算

2. **可视化 skills**
   - 轨迹可视化
   - 不确定性可视化
   - 实验结果图表生成

3. **实验设计 skills**
   - 参数扫描实验设计
   - 对照组设置
   - 消融实验设计

**不相关的部分**：
- 文献检索（我们已有方向）
- 论文写作（你自己完成）

---

## 我们需要的 Skills 分类

基于我们的研究内容，我建议分为以下几类：

### A. 感知处理 Skills（优先级：⭐⭐⭐）

```
perception_skills/
├── point_cloud_fusion/           # 多传感器点云融合
│   ├── registration.py           # 配准算法（ICP, NDT）
│   ├── filtering.py              # 滤波去噪
│   └── outlier_removal.py        # 离群点去除
│
├── surface_reconstruction/       # 表面重建
│   ├── tsdf_fusion.py            # TSDF 融合
│   ├── poisson_reconstruction.py # Poisson 表面重建
│   └── occupancy_mapping.py      # 占据栅格建图
│
├── uncertainty_estimation/       # 不确定性估计
│   ├── confidence_map.py         # 置信度地图
│   ├── covariance_estimation.py  # 协方差估计
│   └── bayesian_fusion.py        # 贝叶斯融合
│
└── change_detection/             # 变化检测（作业效果评估）
    ├── pre_post_comparison.py    # 前后对比
    ├── volume_estimation.py      # 体积估计
    └── effect_quantification.py  # 效果量化
```

**来源建议**：
- 查看 `robotics-agent-skills` 是否有相关内容
- 否则参考 Open3D、PCL 库封装

---

### B. 规划 Skills（优先级：⭐⭐⭐）

```
planning_skills/
├── nbv_planning/                 # 主动观测规划（核心）
│   ├── information_gain.py       # 信息增益计算
│   ├── viewpoint_sampling.py     # 视点采样
│   ├── frontier_detection.py     # 边界检测
│   └── coverage_planning.py      # 覆盖规划
│
├── motion_planning/              # 运动规划
│   ├── moveit_interface.py       # MoveIt 2 接口
│   ├── collision_checking.py     # 碰撞检测
│   ├── trajectory_optimization.py # 轨迹优化
│   └── cartesian_path.py         # 笛卡尔路径
│
└── task_planning/                # 任务规划
    ├── task_sequencing.py        # 任务排序
    ├── priority_assignment.py    # 优先级分配
    └── replanning.py             # 重规划
```

**来源建议**：
- NBV 规划：可能需要自己实现或查找专门的研究代码
- 运动规划：MoveIt 2 已有，封装接口即可

---

### C. 决策 Skills（优先级：⭐⭐⭐）

```
decision_skills/
├── action_selection/             # 动作选择
│   ├── mpc_controller.py         # 模型预测控制
│   ├── rl_policy.py              # 强化学习策略
│   └── rule_based.py             # 规则基决策
│
├── parameter_adaptation/         # 参数自适应
│   ├── online_learning.py        # 在线学习
│   ├── bandit_optimization.py    # Bandit 优化
│   └── bayesian_optimization.py  # 贝叶斯优化
│
└── effect_prediction/            # 效果预测
    ├── simple_model.py           # 简化几何模型
    ├── data_driven_model.py      # 数据驱动模型
    └── neural_predictor.py       # 神经网络预测
```

**来源建议**：
- RL 相关：Stable-Baselines3、RLlib
- 优化相关：Optuna、BoTorch

---

### D. 实验与分析 Skills（优先级：⭐⭐）

```
experiment_skills/
├── scenario_generation/          # 场景生成
│   ├── coal_wall_generator.py    # 煤壁实例生成（已有）
│   ├── parameter_sampling.py     # 参数采样
│   └── difficulty_grading.py     # 难度分级
│
├── data_collection/              # 数据采集
│   ├── trajectory_logger.py      # 轨迹记录
│   ├── sensor_logger.py          # 传感器数据记录
│   └── performance_metrics.py    # 性能指标记录
│
├── analysis/                     # 分析工具
│   ├── statistical_test.py       # 统计检验
│   ├── comparison_plot.py        # 对比图表
│   └── ablation_study.py         # 消融实验
│
└── visualization/                # 可视化
    ├── trajectory_viz.py         # 轨迹可视化
    ├── uncertainty_viz.py        # 不确定性可视化
    └── result_dashboard.py       # 结果仪表盘
```

**来源建议**：
- 查看 `scientific-agent-skills` 的分析和可视化部分
- 使用 Matplotlib、Plotly、Weights & Biases

---

### E. 仿真交互 Skills（优先级：⭐⭐⭐）

```
simulation_skills/
├── gazebo_interface/             # Gazebo 接口
│   ├── model_spawner.py          # 模型生成
│   ├── physics_control.py        # 物理控制
│   └── sensor_manager.py         # 传感器管理
│
├── ros2_interface/               # ROS 2 接口
│   ├── topic_handler.py          # 话题处理
│   ├── service_client.py         # 服务客户端
│   └── action_client.py          # Action 客户端
│
└── state_management/             # 状态管理
    ├── world_state.py            # 世界状态
    ├── robot_state.py            # 机器人状态
    └── checkpoint.py             # 检查点保存/恢复
```

**来源建议**：
- 基于我们已有的代码封装
- 参考 `robotics-agent-skills` 的接口设计

---

## 推荐的 Skills 优先级

### 立即需要（阶段 3）

1. **仿真交互 Skills** - 用于破坏响应模型开发
2. **感知处理 Skills** - 基础的点云处理和表面重建

### 近期需要（阶段 4-5）

3. **规划 Skills** - NBV 规划和运动规划
4. **决策 Skills** - 效果预测和动作选择

### 后期需要（阶段 6-7）

5. **实验与分析 Skills** - 大规模实验和论文数据

---

## 行动建议

### 第一步：调研 robotics-agent-skills

```bash
cd /home/star/SCI/hydromining_ws/assets
git clone https://github.com/arpitg1304/robotics-agent-skills.git
cd robotics-agent-skills
ls -la  # 查看结构
cat README.md  # 查看说明
```

**重点查找**：
- `perception/` 或 `sensing/` 目录
- `planning/` 目录，特别是 NBV 相关
- `control/` 或 `decision/` 目录
- ROS 2 集成示例

### 第二步：调研 scientific-agent-skills

```bash
cd /home/star/SCI/hydromining_ws/assets
git clone https://github.com/K-Dense-AI/scientific-agent-skills.git
cd scientific-agent-skills
ls -la
cat README.md
```

**重点查找**：
- `analysis/` 目录（统计分析）
- `visualization/` 目录（结果可视化）
- `experiment/` 目录（实验设计）

### 第三步：创建我们自己的 Skills 库

基于调研结果，决定：
- 哪些 skills 可以直接使用
- 哪些需要改造
- 哪些需要自己实现

创建：
```
src/hydromining_skills/
├── perception_skills/
├── planning_skills/
├── decision_skills/
├── experiment_skills/
└── simulation_skills/
```

---

## 总结

| 项目 | 相关性 | 优先级 | 行动 |
|------|--------|--------|------|
| **superpowers** | ❌ 不相关 | 无 | 不使用 |
| **robotics-agent-skills** | ⭐⭐⭐ 高度相关 | 高 | 立即调研 |
| **scientific-agent-skills** | ⭐⭐ 部分相关 | 中 | 后期参考 |

**下一步**：
1. 我帮你调研 `robotics-agent-skills` 的具体内容
2. 根据调研结果，决定集成策略
3. 开始实现阶段 3 需要的核心 skills

你希望我现在开始调研这两个项目吗？还是你先手动测试第二阶段的仿真系统？
