# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 先读什么

- `AGENTS.md` — 项目规则（ROS 2 Jazzy/Gazebo Harmonic 开发规范、强制测试与诊断流程、git 与交接约定）。**规则以它为准**。
- `docs/论文总体计划.md` — 当前工作主线：大论文章节、实验路线（阶段 0–7，实验号 E2–E6，⛔ 关卡与完成标准）。
- `docs/进度总表.md` — 做到哪个实验、结果在哪。

本项目不是"做完就完"的工程：所有工作服务于大论文和两篇小论文（A：感知+NBV；B：非均质煤体规则+残差 RL）。动手前先确认当前处于哪个实验、完成标准是什么。

## 常用命令

```bash
# 环境（每个新终端）
source setup_env.sh            # Jazzy + .venv_stage4 + install/setup.bash + GZ_SIM_RESOURCE_PATH

# 构建（必须带 --base-paths src，否则会扫到外部目录）
colcon build --symlink-install --base-paths src --packages-select hydromining_robot hydromining_sim

# 单元测试（当前 117 项，约 30 s）
python3 -m pytest src/hydromining_sim/test -q
python3 -m pytest src/hydromining_sim/test/test_planning_decision.py -q          # 单文件
python3 -m pytest src/hydromining_sim/test/test_nbv_observation_policy.py::test_name -q  # 单用例

# 规则基线回归（重构 L0/规则后必须与 results/thesis/E0.3 逐回合比对，成功率应为 0.917）
.venv_rl/bin/python experiments/ch0_baseline/run_baseline.py --output results/thesis/<新目录名>

# 旧阶段验收（回归用，无 GUI）
./scripts/run_phase4_acceptance.sh | run_phase5_acceptance.sh | run_phase6_acceptance.sh
./scripts/run_phase7_batch.sh        # 阶段七六工况真实闭环
./scripts/run_gui_demo.sh            # Gazebo GUI 演示（规则决策；--model 可加载训练模型）

# 长期训练/评估由用户启动；Claude 只跑 ≤100 步冒烟
./scripts/run_phase7_training.sh --output results/phase7_training
```

两个虚拟环境：`.venv_stage4`（仿真/感知，CPU，`setup_env.sh` 自动激活）与 `.venv_rl`（stable-baselines3 + CUDA）。训练与 `phase7_*` 评估脚本用 `.venv_rl/bin/python`。

## 架构大图

两个 ROS 包 + 一层无 ROS 的算法库：

```
src/hydromining_sim/scripts/algorithms/   纯 NumPy/SciPy/Open3D，无 ROS 依赖，可直接跑
    ├── task_spec.py        TaskSpec：任务参数唯一来源（读 config/phase7_large_task.yaml）
    ├── erosion_model.py    R_c/b 径向核 + 暴露/遮挡；level_set_evolver.py 做时间推进
    ├── spray_simulation.py / local_spray_simulation.py   整块体素 / 3 m 墙内 5 mm 局部补丁
    ├── perception/         PerceptionSession 管线、高度图融合、σ 模型、判决、干扰滤波
    └── planning/           规则决策、NBV、L0/L1/L2 强化学习环境、替代世界
src/hydromining_robot/scripts/            ROS 适配层（生命周期节点），通过
                                          sys.path.insert(share_dir) 导入 algorithms
```

**核心数据流（同一套接口贯穿离线与真实）**

1. 观测：`PerceptionSession.set_baseline/evaluate/observe(groups)`，输入各视点的 depth/lidar 快照，输出 `report.assessment`（`achieved / not_achieved / insufficient_evidence`，基于退缩量置信下界 δ − kσ）、`change.delta/sigma/valid`、`surface`。
2. 规划：`planning/nbv.py` 选视点（`observed_visibility` 只用观测高度预测遮挡，不读真值）；`planning/decision.py` 的 `RulePolicy` 按置信下界缺口选喷射点与时长。
3. 执行：`SprayAction` 服务（`spray_controller_node.py`）→ `SpraySimulation`/`LocalSpraySimulation` → Marching Cubes 换网格；`AssessRegion` 动作（`wall_perception_node.py`）包装 `PerceptionSession`。
4. 学习：`planning/fast_heightfield.py`（L0，高度场快速）、`planning/rl_env.py`（L1 真值+可见性、L2 完整管线）。三者必须实现同一 Gym 接口；**真值只进奖励与评估，不进观测**。

**保真度约定**：L0 用于批量训练，L1/L2 用于验证。L0 与 Level Set 的精度由 `test_scripts/phase7_fidelity_benchmark.py` 把关，改动物理后必须重跑。

**仿真与 ROS 要点**（Jazzy + Harmonic，不是 Humble）

- world 里必须加载一次 `gz-sim-sensors-system`，否则 `<sensor>` 不工作；传感器话题后缀为 `rgbd_camera/image|depth_image|points`、`gpu_lidar/points`。
- 控制插件 `gz_ros2_control-system`，bridge 是 `ros_gz_bridge`（不是 `ros_ign_*`）。
- 煤壁替换走 `gazebo_coal_wall.py` 的 UserCommands 事务；状态机见 `troubleshooting/KNOWN_ISSUES.md`。

## 实验与结果约定

- 每个小实验一个文件：`experiments/<章节>/e<实验号>_<名称>.py`；共用逻辑进 `experiments/common/`。
- 结果写 `results/thesis/<实验号>/<run_id>/`：`manifest.json`（配置、代码哈希、种子）、`episodes.jsonl`、`summary.json`、`figures/`。只有 manifest/summary/figures 进 git。
- `results/legacy/` 是 2026-11-01 前的旧结果，只读；不要往里面写。
- 每完成一个小实验或一次独立验证就 git 提交（格式见 AGENTS.md），阶段完成打标签；**推送前征得用户同意**。

## 代码风格

- **注释与文档字符串一律用中文**（模块说明、函数说明、行内注释）；变量名、函数名和技术术语保持原文。修改旧文件时顺手把触到的英文注释改成中文，不做无关的全量翻译。

## 容易踩的坑

- 改 L0 蚀除、可见性或规则后，跑规则基线逐回合比对，不要只看测试通过。
- 旧文档（`docs/阶段文档/`、`README.md` 顶部状态、评测自评分）可能过时；旧 RL 模型成功率全 0 是奖励缺陷，不是方法结论。
- `test_scripts/` 里多数不是 pytest（是验收/实验入口），`pytest` 只收集 `src/hydromining_sim/test/`。
- 重建全部（`rm -rf build install log`）很少需要；先试增量构建。
