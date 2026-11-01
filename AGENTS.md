# AGENTS.md — 水力采煤仿真系统项目规则

## 用户指定的长期沟通与写作规则

- 自 2026-11-01 起，本项目的后续回答、解释和文稿默认采用 ASD-STE100 的写作原则。此偏好持续有效，直到用户明确更改。
- 中文回答采用其适用原则：使用短句、常用词和明确表达；一句只表达一个主要意思；同一概念使用同一术语；直接说明动作、条件和结果；避免歧义、冗词和复杂嵌套。
- 英文技术写作按 ASD-STE100 规则组织语言。未经规则及词汇检查，不声称文本已严格符合该标准。
- 保留必要的技术术语、数值、单位和证据。默认沿用用户提问的语言。

**项目**: Hydromining ROS 2 Gazebo 仿真系统  
**环境**: 
- **ROS 2 Jazzy** (NOT Humble)
- **Gazebo Harmonic** (gz-sim-8, NOT gz-sim-7 or Ignition)
- **构建工具**: ament_cmake, colcon

**维护者**: Star (@tbxrelive)

---

## 🧭 新对话必读：当前工作重点（2026-11-01 起）

> 本节是给下一次对话 / 其他 AI 的入口。**动手前先读完本节和下面两份文档。**

1. [`docs/论文总体计划.md`](docs/论文总体计划.md)：做什么、按什么顺序做、做到什么程度算完成。用户已批准，**严格按其第七节的阶段顺序执行**。
2. [`docs/进度总表.md`](docs/进度总表.md)：当前做到哪个实验、结果在哪、对应论文哪张图。**这是"做到哪一步"的唯一依据。**

### 目标

本项目服务于用户的**大论文**，并从中产出两篇小论文：

| 论文 | 方向 | 内容 | 大论文章节 |
|---|---|---|---|
| 论文 A（先做） | 机器人方向英文 SCI | 干扰下的感知与效果评估 + 判决导向主动观测规划（NBV） | 第 3、4 章 |
| 论文 B（后做） | 《煤炭学报》 | 非均质煤体下规则先验 + 残差深度强化学习的自适应喷射 | 第 2、5 章 |
| 大论文 | — | 以上 + 仿真系统 + 闭环集成实验 | 第 2–6 章 |

**主线**：任务达标置信度判决。退缩量置信下界 δ − kσ ≥ 5 cm 的面积 ≥ 90%，输出 achieved / not_achieved / insufficient_evidence 三分类。感知给出 δ、σ；NBV 降低影响判决的不确定性；喷射决策补齐置信下界缺口。

当前任务：60° 煤壁、30 MPa、直径 40 cm 区域、90% 面积退缩 5 cm，参数唯一来源为 `src/hydromining_robot/config/phase7_large_task.yaml`，代码中通过 `algorithms/task_spec.py` 的 `TaskSpec` 读取。

### 执行纪律（每个小实验都必须遵守）

1. **开始前**：列出要改的文件和要跑的实验，**等用户确认**再动手。
2. **固定流程**：写配置 → 2 回合冒烟 → 正式运行（长训/长评估由用户启动，我只跑 ≤100 步冒烟）→ 统计 → 对照完成标准判定 → 更新 `docs/进度总表.md`。
3. **⛔ 关卡实验**不达标不得进入下一步；**不得放宽完成标准去凑结果**，未通过要如实写进进度表和提交信息。
4. **每个阶段完成后 git 提交一次并打标签**（如 `step0-baseline`、`step1-refactor`）；阶段内的小实验和修复不逐个提交。提交信息格式 `<类型>(<阶段或模块>): <中文摘要>`，正文写验证结果。**推送远程前必须征得用户同意**。
5. **重构必须保持结果不变**：改动 L0 环境或规则后，重跑 `experiments/ch0_baseline/run_baseline.py`，与 `results/thesis/E0.3/episodes.jsonl` 逐回合比对，除耗时字段外必须零差异（成功率 0.917）。

### 目录约定

```
experiments/<章节>/e<实验号>_<名称>.py   每个小实验一个文件，文件名以实验号开头
experiments/common/                      共用：task.py(TaskSpec)、划分、接口、记录、统计
configs/experiments/<实验号>.yaml         每个实验一份配置
results/thesis/<实验号>/<run_id>/         manifest.json、summary.json、figures/ 进 git；episodes.jsonl、模型不进 git
results/legacy/                          2026-11-01 前的全部旧结果（只读，不进 git）
docs/archive/legacy_20261101/            已归档的旧脚本与旧文档
docs/阶段文档/                            旧"阶段 1–7"开发记录，仅作历史参考
```

### 已确认的关键事实（不要重新推导，也不要被旧文档误导）

- **旧 RL 结论作废**：2026-10-31 的 12 个模型（DDPG/示范 DDPG/TD3/SAC × 3 种子）L0 测试成功率全为 0，规则基线 0.92。根因是**奖励缺陷**：打穿煤壁的回合回报（约 −17）高于达标回合（约 −33），策略学会了"边缘反复短喷直到打穿"。这不是调参问题，修奖励前不得重训，也不得写成"DRL 不如规则"。
- **RL 仍然要做**（研究内容 4），但必须在规则处理不好的条件下：非均质、强度未知的煤体，结构为"规则先验 + 残差 RL + 防贯穿护盾"。
- **旧 NBV 增益很小**：3 个基线视点已近全覆盖（99.68%→100%），旧文档中的"0.342→0.443"等数字不能用于论文。新 NBV 是判决导向 + 动态水雾干扰 + 运动代价 + 序贯停止，且必须先过可行性关卡 E4.0。
- **破煤模型是"文献标定 v1b"**：核为 `R_c = k·(P − P_th)^α/s^β`、`b = b0 + 0.22·s`、径向剖面 `(1−ξ^1.5)²`。
  阈值压力 P_th=5 MPa、α=1、b_distance=0.22、radial_decay=2 都有全文出处
  （Zhang 2019 / Zhang & Zou 2018，PDF 在 `docs/文献/`）；只有 P_th 的具体数值和 k 是工程取值。
  依据与置信度见 `docs/文献/E2.1_标定依据.md`。论文只能比较相对趋势。
- **改核会作废下游数字**：`configs/sim_v1.lock.json`（版本 `sim-v1b`）锁住破煤核、蚀除数值、可见性、
  传感器退化、替身渲染、任务配置、干扰等级与场景库；改动任一文件都会让 `test_sim_v1_frozen.py` 失败。
  场景库内容与核无关（v1b 与 v1 逐字节一致），核改动只需重记基线。
- **场景库**：`experiments/common/splits.py` 是唯一入口。训练墙 10–19、验证 30–32、测试 20–25，互不相交；工况由 `(划分, 族, 序号, 墙面)` 唯一确定，回合种子由 SHA256 派生。旧基线仍用 `LEGACY_WALL_SEEDS`（10–12 / 20–22）。
- **干扰等级**：由能见度定义（无/inf、低/120 m、中/30 m、高/8 m），传感器噪声档 1×/2× 是与之正交的第二因素。见 `configs/interference_levels.yaml`。
- **Gazebo 粒子层不能当干扰等级用，也达不到可用量级**：E2.3c 实测 off/low/medium/high 的点云受扰比例是 0 / 1.27% / 1.43% / 1.40%（不单调）；E2.3e 把发射器推到允许的最大值（扩散 0.5、参考压力 10 MPa、全效果齐开）也只有 **2.52%**，不到软件层"低"档（10.4%）的四分之一。**结论：干扰一律走软件层；阶段 4 的 E4.4（干扰模型验证 r ≥ 0.8）不能按原计划做**，验证对象必须改成定性一致性 + 敏感性讨论，或先把粒子发射器调到 ≥10% 的量级（属阶段 5）。
- **规则策略在凸起墙面会打穿**：E2.5 全库基线（v1b 核）train/test 成功率 1.000、验证 0.667，失败全部是打穿——墙面局部凸起使有效靶距变小，规则按固定时长连喷会穿透 400 mm 壁厚。这不是 bug，是阶段 6「防贯穿安全护盾」（E5.3）的动机。
- **文档里的自评分**（如"93/100""96/100"）和"✅ 完成"标记不能作为证据；证据只认 `results/` 下的 json。
- 查重：与本课题最接近的已有工作及区分点见总体计划第一节。中文文献由用户自行检索。

---

**⚠️ 版本警告**：
- 本项目使用 **ROS 2 Jazzy + Gazebo Harmonic**
- 鱼香ROS代码示例基于 **ROS 2 Humble + Gazebo Fortress/Garden**
- **禁止直接复制鱼香ROS的 Humble 代码**，插件名称、API、launch 格式可能不兼容
- 参考鱼香ROS时，仅学习设计思路，代码必须适配 Jazzy + Harmonic

**官方文档参考**：
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/
- gz_ros2_control: https://github.com/ros-controls/gz_ros2_control (jazzy 分支)

---

## 🎯 项目概述（历史开发阶段，仅供参考）

面向急倾斜煤层水力采煤的机器人自主作业**仿真系统**。

> 以下"阶段 1–7"是 2026-10-22 至 10-31 的开发过程记录，状态描述可能过时。2026-11-01 起的工作按论文总体计划执行，实验编号为 E2–E6，与这里的"阶段"不是一回事。

### 核心组件（7 阶段开发计划，已结束）

| 阶段 | 名称 | 技术栈 | 状态 |
|------|------|--------|------|
| **阶段 1** | **煤壁几何升级** | Python + Perlin Noise + Gazebo SDF | ✅ 完成 (2026-10-22) |
| **阶段 2** | **机械臂与水枪集成** | URDF/xacro + ros2_control + gz_ros2_control | ✅ 完成 (2026-10-22) |
| **阶段 3** | **传感器集成与感知验证** | 点云处理 + TF变换 + 多视角融合 | ✅ 完成 (2026-10-24) |
| **阶段 4** | **喷射效果模拟** | Level Set 方法 + SDF + Marching Cubes | 用户决定先通过；参数未标定，碰撞独立接触等遗留项保留（2026-10-26） |
| **阶段 5** | **观测干扰模拟** | 水射流显示 + Gazebo 原生粒子 | ✅ 无 GUI + GUI 验收通过；参数未标定（2026-10-26） |
| **阶段 6** | **感知算法集成** | 多传感器融合 + 表面重建 + 不确定性量化 | 6.0–6.6 已实现，真实无 GUI 全链路验收通过；噪声校准/昏暗/粒子数据等边界未达标，用户验收待进行（2026-10-28） |
| **阶段 7** | **规划与决策算法** | NBV + 规则决策 + DDPG(自适应喷射) | � 实现中：替身/真实闭环已过，RL 评估与 GUI 验收待完成 |

### 各阶段详细说明

<details>
<summary><b>阶段 1：煤壁几何升级</b> ✅ 完成</summary>

**目标**：从平整煤壁升级为凹凸不平的真实几何

**技术路线**：
1. 使用 Perlin Noise / Fractal Noise 生成表面高度场
2. 转换为 Mesh（.obj 或 .dae）导入 Gazebo
3. 支持参数化生成（粗糙度、倾斜角、尺寸）

**验收标准**：
- ✅ 煤壁表面呈现明显凹凸
- ✅ 可以调整参数生成不同实例
- ✅ Gazebo 中渲染和碰撞检测正常

**文档**：`docs/阶段文档/阶段1-煤壁生成/`
</details>

<details>
<summary><b>阶段 2：机械臂与水枪集成</b> ✅ 完成</summary>

**目标**：加载机械臂 URDF，末端附加水枪模型

**技术路线**：
1. 使用 Franka Panda 7-DOF 机械臂
2. 定义水枪末端执行器（visual + collision + TCP）
3. 配置 ros2_control + gz_ros2_control 接口
4. 实现基础运动控制（关节空间轨迹）

**验收标准**：
- ✅ 机械臂在 Gazebo 中正常显示和运动
- ✅ 可以通过 ROS 2 Action 控制
- ✅ 传感器（RGB-D + LiDAR）数据正常发布

**评估**：94/100 分  
**文档**：`docs/阶段文档/阶段2-机器人集成/`
</details>

<details>
<summary><b>阶段 3：传感器集成与感知验证</b> ✅ 完成</summary>

**目标**：验证传感器数据采集、多视角观测、点云配准

**技术路线**：
1. 实现观测位姿规划（定义观测点 A、B）
2. 采集多视角点云数据
3. 点云坐标变换（sensor_frame → world_frame）
4. 多视角点云配准验证
5. 保存基准观测数据

**验收标准**：
- ✅ 机械臂能移动到预定义的观测位姿
- ✅ 点云数据正常采集并保存（~8000 点/帧）
- ✅ 两次观测的点云能在同一坐标系对齐
- ✅ 重叠区域配准误差 < 5cm（实际达到 **3.1 mm**）

**评估**：100/100 分（配准精度远超预期）  
**文档**：`docs/阶段文档/阶段3-传感器集成/`
</details>

<details>
<summary><b>阶段 4：喷射效果模拟</b> 六步无 GUI 验证通过，GUI 待验收</summary>

**2026-10-26 更新**：开发前核查发现的主要缺口已在现有代码上修复，真实 Gazebo 服务器、机械臂、RGB-D 六步流程通过。GUI 最终观察和实验参数标定尚未完成。以下历史评分不能替代验收，详见阶段 4 `00-进度交接.md` 与 `08-六步无GUI集成验证报告.md`。

**目标**：实现喷射后煤壁几何的动态更新

**技术路线**：
1. 使用 Level Set 方法建模煤壁表面演化
2. 按模型①—⑲建立压力/当前靶距相关的 R_c、b 与径向核、暴露掩码响应；参数待标定，不宣称已有一般入射角衰减或随机煤体强度模型
3. 通过 Marching Cubes 重建更新后的 Mesh
4. 提供 ROS 服务接口供外部调用

**验收标准**：
- ✅ 无 GUI 真实链路：喷前点云 → 固定臂喷射 → Gazebo 凹坑与统计 → 喷后点云 → 累计喷射 → 重置重放
- ✅ 相同初始状态与记录动作重放，网格一致；当前是确定性模型，不以“固定种子”冒充随机煤体模型验证
- ✅ 离线压力扫描体积非递减且高低压力有差异；受体素量化影响，不要求每个相邻压力严格递增
- [ ] 用户 GUI 最终验收

**评估**：96/100 分  
**文档**：`docs/阶段文档/阶段4-喷射效果模拟/`
</details>

<details>
<summary><b>阶段 5：观测干扰模拟</b> ✅ 无 GUI + GUI 验收通过（参数未标定）</summary>

**目标**：水射流显示、命中飞溅、水雾/粉尘与真实传感器观测干扰。

**技术路线**：
1. 连续渐扩水柱 + Gazebo Harmonic 原生 ParticleEmitter，参考开源喷涂插件架构，适配现有 tcp/+Z 与 Jazzy。
2. ROS 仿真时间驱动准备、喷射、煤壁更新、消散状态；破煤仍用模型①—⑲。
3. RGB、深度、RGB-D 点云、GPU LiDAR 原生观测退化；软件模糊/额外噪声保留为按需补充，当前未实现独立退化节点。
4. 关闭/低/中/高预设与有界压力映射，均属未标定工程参数。

**验收标准**：
- ✅ 无 GUI 实测三类观测变化，干扰程度可调且保留部分煤壁观测。
- ✅ 累计喷射、记录动作重放网格一致、重置取消、无命中、暂停计时验证。
- ✅ 阶段四兼容回归；退出只清理本次进程。
- ✅ 用户 GUI 最终验收（2026-10-26，v3：自动检查通过、用户目视确认凹坑、Ctrl+C 清理通过；峰值 2801.5 MiB）。见阶段五 `05-GUI最终验收报告.md`。
- [ ] 真实实验参数标定。

**GUI 目视提示**：凹坑约 8×11 cm，相对 3×3 m 煤壁很小，默认视角几乎看不出。可通过 `/gui/screenshot` 抓取 GUI 实际渲染，用 `/gui/move_to/pose` 移动相机（需设置运行时的 `GZ_PARTITION=hydromining_phase5_<验收进程PID>`）。"看不到凹坑"先查这一点，不要先怀疑网格更新。

**资源修复补充**：首次 GUI 因聚合 RSS 超过 3 GiB 中断。最终资源/配置版本通过无 GUI 完整流程、保持退出与阶段四兼容回归，峰值约 1.91 GiB；GUI 重验尚未完成。不要把无 GUI 内存下降写成 GUI 已通过。详见阶段五 `03-GUI内存保护中断与修复验证.md`。

**事务修复补充**：GUI 第二次运行未触发内存保护，而在煤壁 create RPC 超时中断。最新适配器按实际模型/网格确认事务，回执丢失先核实，未知状态禁止继续喷射并要求重置。最新无 GUI/故障注入验证见阶段五 `04-煤壁创建超时与事务确认修复.md`；GUI 仍待重验。

**文档**：`docs/阶段文档/阶段5-观测干扰模拟/00-进度交接.md`。
**入口**：`./scripts/run_phase5_acceptance.sh`；GUI 显式添加 `--gui --keep-open`。
</details>

<details>
<summary><b>阶段 6：感知算法集成</b> 6.0 开发中，完整感知未验收</summary>

**目标**：实现煤壁状态估计与不确定性量化

**技术路线**：
1. 多传感器融合（相机 + 激光雷达）
2. 表面重建（TSDF、Poisson、Occupancy Grid）
3. 不确定性建模（概率栅格、高斯过程）
4. 作业效果评估（前后对比、体积估计）

**验收标准**：
- [ ] 能够重建完整煤壁表面
- [ ] 量化观测不确定性
- [ ] 评估喷射前后体积变化
</details>

<details>
<summary><b>阶段 7：规划与决策算法</b> 📋 规划中</summary>

**目标**：实现主动观测规划和自适应喷射决策

**技术路线**：
1. 信息增益驱动的下一最佳视点（NBV）规划
2. 基于 MoveIt 2 的路径规划
3. 强化学习或 MPC 的喷射参数优化
4. 完整闭环实验

**验收标准**：
- [ ] 自主选择观测位姿
- [ ] 自适应调整喷射参数
- [ ] 完整闭环作业流程
</details>

### 已验证的关键命令

```bash
# 源代码配置
cd ~/SCI/hydromining_ws
source install/setup.bash

# 完整仿真启动（Gazebo + 机器人 + 传感器）
ros2 launch hydromining_robot robot_simulation.launch.py

# 测试机器人运动控制
ros2 run hydromining_robot test_motion.py

# 验证传感器话题
ros2 topic list | grep -E "joint_states|camera|lidar"
ros2 topic hz /hydromining/camera/color/image_raw
```

---

## 📋 开发规则

### 1️⃣ **ROS 2 节点开发规则**

**何时触发**: 创建或修改 ROS 2 包、节点、launch 文件、控制器配置

**⚠️ 版本特定要求（Jazzy + Harmonic）**:

- ✓ **Gazebo 插件命名**：`gz-sim-*-system` 与 `libgz-sim8-*-system.so` 在 Harmonic 中都有效（已核实 `/opt/ros/jazzy/opt/gz_sim_vendor/lib/`）
  - 本项目 world 统一用 `gz-sim-*-system` 风格
  - **不是**：`ignition-gazebo-*-system`（Fortress / Humble）
- ✓ **传感器必须在 world 中加载一次 Sensors 系统插件**，否则 `<sensor>` 不会运行
- ✓ **传感器话题后缀**：`rgbd_camera` 发布 `<topic>/image|depth_image|points|camera_info`；`gpu_lidar` 在 `<topic>` 发 `LaserScan`、在 `<topic>/points` 发点云。bridge 配置前先 `gz topic -l` 对照
  
- ✓ **ros_gz_bridge**：Jazzy 使用 `ros_gz_bridge`（不是 Humble 的 `ros_ign_bridge`）
  
- ✓ **gz_ros2_control**：插件名为 `gz_ros2_control-system`
  - **不是** Humble 的 `ign_ros2_control-system`

**核心要求**:

- ✓ 使用 **ROS 2 Jazzy** 的生命周期节点模式（Lifecycle Node）
- ✓ 所有发布者/订阅者必须声明 **QoS 配置文件**
  - 传感器话题（相机、激光雷达）: `BEST_EFFORT, depth=1`
  - 状态话题（关节、TF）: `RELIABLE, depth=10`
- ✓ 使用 Python 参数 API，支持运行时修改参数（替代 dynamic_reconfigure）
- ✓ 所有节点必须有 `get_logger().info()` 日志，记录初始化和关键事件
- ✓ 使用 `ament_cmake` 或 `ament_python`；确保 `package.xml` 完整列出依赖
- ✓ Launch 文件使用 Python 格式（`*.launch.py`），支持参数传递和条件启动

**代码风格规范**:

- ✓ **遵循鱼香ROS代码风格**：详见 [`docs/鱼香ROS代码规范总结.md`](docs/鱼香ROS代码规范总结.md)
  - Python 节点：面向对象模式，成员变量加 `_` 后缀
  - 命名规范：类名 PascalCase，函数/变量 snake_case
  - 文档字符串：所有类和关键方法必须有文档说明
  - Launch 文件：标准 `generate_launch_description()` 结构
  - **注意**：鱼香ROS示例基于 Humble，插件/API 名称需适配 Jazzy

**应调用的技能**:

- 在 `/home/star/SCI/hydromining_ws/.claude/skills/` 中查找并使用 `ros2.md`（或通过 `/ros2` 指令）
- **强制**：开发前先检查 ROS 2 Jazzy 和 Gazebo Harmonic 官方文档

**已验证的依赖** (在 `package.xml` 中已配置):

```xml
<depend>rclcpp</depend>
<depend>rclpy</depend>
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
<depend>controller_manager</depend>
<depend>joint_trajectory_controller</depend>
<depend>robot_state_publisher</depend>
<depend>xacro</depend>
```

---

### 2️⃣ **算法与 ROS 通信分离原则**

**何时应用**: 任何涉及计算、感知处理、控制算法的任务

**分离策略**:

- ✓ **核心算法** (Python 纯函数/类): 独立的 `src/hydromining_robot/algorithms/` 或 `scripts/`
  - 无 ROS 依赖，便于单元测试
  - 输入: numpy array, 输出: 几何/数值结果
  
- ✓ **ROS 适配层**: 独立的节点/服务，负责
  - 参数管理、话题发布/订阅、时间戳同步
  - 调用核心算法，处理返回结果

**坐标系、单位、时间戳约定**:

- **坐标系**: TF 树根为 `world`，机器人基座为 `panda_link0`，水枪实际 TCP 统一使用现有 `tcp`（已在阶段4服务/验收配置中应用），禁止假设 `water_gun_link` 已存在
- **长度单位**: 米 (m)
- **角度单位**: 弧度 (rad)
- **时间戳**: 所有消息使用 ROS 时间 (simulation clock via Gazebo)
- **验证命令**: `ros2 run tf2_tools view_frames` 检查 TF 树结构

---

### 3️⃣ **测试与验证规则**

**何时触发**: 完成任何代码修改、新功能开发、bug 修复

**必须执行的验证** (参考 `verification-before-completion.md`):

1. **构建验证**
   ```bash
   cd ~/SCI/hydromining_ws
   colcon build --symlink-install --packages-select hydromining_robot hydromining_sim
   # 预期: 2 packages finished [<时间>]
   ```

2. **仿真启动验证（分阶段）**
   
   **阶段 A: 运动控制验证（已完成 ✅）**
   ```bash
   source install/setup.bash
   # 在后台启动仿真（30秒测试窗口）
   timeout 30 ros2 launch hydromining_robot robot_simulation.launch.py > /tmp/gz_test.log 2>&1 &
   sleep 25
   
   # 验证控制器
   ros2 control list_controllers
   # 预期: panda_arm_controller 和 joint_state_broadcaster 都是 active
   
   # 验证关节状态发布
   ros2 topic hz /joint_states
   # 预期: ~100 Hz
   ```
   
   **阶段 B: 传感器验证（⏳ 阶段3目标）**
   ```bash
   # 检查传感器话题是否存在
   ros2 topic list | grep -E "hydromining/camera|hydromining/lidar"
   
   # 验证数据发布（如果传感器系统正常）
   ros2 topic hz /hydromining/camera/color/image_raw  # 预期: ~30 Hz
   ros2 topic hz /hydromining/lidar/points            # 预期: ~10 Hz
   ```
   
   **历史问题（非当前状态结论）**: 传感器曾因配置问题无数据；阶段 3 交接已记录 Sensors 插件存在、bridge 映射修复及数据验证。**不是已证实的显卡问题**，`libEGL warning` 只是警告。2026-10-26 未重测实时话题；详见 `docs/阶段文档/阶段3-传感器集成/00-进度交接.md`。

3. **运动控制功能测试**
   ```bash
   # 新终端：测试机器人运动
   source install/setup.bash
   ros2 run hydromining_robot test_motion.py
   # 预期: 机器人执行预定义轨迹，关节平滑运动
   ```

**应调用的技能** ⚠️ **强制**:

- **代码提交前，强制使用** `/verification-before-completion` **技能**
- **算法测试，使用** `/robotics-testing` **技能**（单元测试 + 集成测试）

---

### 3.5️⃣ **强制测试流程 — 杜绝未测试交付**

**背景**：此前反复出现以下问题：
1. ❌ 代码修改后未经过实际测试，就直接要求用户运行
2. ❌ 遇到运行错误时，没有按照约定使用 `systematic-debugging` Skill
3. ❌ 对已经明确纠正的问题，后续仍然重复犯错

**现在强制执行以下规范**：

#### ✅ 必须执行的测试流程

**规则 1：能够由 Claude 执行的测试，必须先亲自执行**

- ✅ 禁止在未测试的情况下声称功能正常
- ✅ 修改 ROS 2 节点后，检查构建、节点启动及相关接口
- ✅ 修改 MoveIt 2 后，检查配置、规划及执行相关功能
- ✅ 修改 Gazebo 后，检查模型加载、仿真状态及相关话题

**规则 2：涉及图形界面或真实仿真环境、Claude 无法独立验证的部分**

- ✅ 必须明确标注 **"尚未验证"**，不得声称通过
- ✅ 优先使用无图形界面的测试方式（headless mode），通过终端输出验证
- ✅ 如果需要用户启动 Gazebo：
  1. 先告诉用户需要启动什么
  2. 待环境就绪且 Claude 具有终端访问权限后
  3. Claude 先执行能够完成的测试
  4. 再交付用户验收

**规则 3：测试顺序**

1. **构建测试**（必须）
   ```bash
   colcon build --packages-select <package> --symlink-install
   ```
   
2. **节点启动测试**（如涉及 ROS 2 节点）
   ```bash
   ros2 run <package> <executable>
   ```
   
3. **接口测试**（如涉及话题/服务/Action）
   ```bash
   ros2 topic list | grep <keyword>
   ros2 topic echo <topic> --once
   ros2 service list | grep <keyword>
   ros2 action list | grep <keyword>
   ```
   
4. **功能测试**（如涉及运动控制、数据采集）
   - 运行测试脚本
   - 检查输出日志
   - 验证结果文件

#### ✅ 测试验证标准

修改以下内容后的验证要求：

| 修改内容 | 必须验证 | 验证命令 |
|---------|---------|---------|
| **ROS 2 节点** | 构建 + 启动 + 接口 | `colcon build` → `ros2 run` → `ros2 topic/service list` |
| **MoveIt 2 配置** | 规划器启动 + 轨迹规划 | `ros2 launch` → `ros2 action send_goal` |
| **Gazebo 模型/世界** | 模型加载 + 话题发布 | `gz model -m <model>` → `ros2 topic hz` |
| **传感器配置** | 话题存在 + 数据发布 | `ros2 topic list` → `ros2 topic hz` |
| **控制器配置** | 控制器状态 + 响应测试 | `ros2 control list_controllers` → 运动测试 |
| **TF 变换** | TF 树结构 + 变换查询 | `ros2 run tf2_tools view_frames` → `ros2 run tf2_ros tf2_echo` |

#### ❌ 禁止的行为

1. ❌ **未测试就声称成功**
   - 错误示例："我已经修复了传感器数据问题，现在你可以运行测试了"
   - 正确做法：先在终端测试，确认成功后再告知用户

2. ❌ **跳过中间步骤**
   - 错误示例：修改代码后直接运行功能测试，不检查构建和节点启动
   - 正确做法：构建 → 节点启动 → 接口验证 → 功能测试（逐层验证）

3. ❌ **依赖用户报告错误**
   - 错误示例：让用户运行后，根据用户的错误信息再修复
   - 正确做法：Claude 先在终端复现和修复，确认通过后再交付

#### ✅ 正确的工作流程

```
代码修改
  ↓
Claude 在终端测试（构建 + 启动 + 接口 + 功能）
  ↓
测试通过 ✅
  ↓
告知用户可以验收
  ↓
用户验收（最终确认）
```

**如果测试失败**：
```
测试失败
  ↓
立即调用 /systematic-debugging
  ↓
诊断根因（不盲目修改）
  ↓
修复并再次测试
  ↓
确认通过后再告知用户
```

---

### 3.6️⃣ **⚠️ 强制规范：禁止未测试代码交付（2026-10-25 新增）**

**重要提示**：这是因为在阶段4时出现的严重错误而新增的强制规范。

#### ❌ 绝对禁止的行为

**禁止1：创建依赖 Gazebo GUI 的功能，但在无 GUI 环境中不测试**

- ❌ **错误示例**（实际发生）：
  ```
  创建 ROS 服务节点 → 假设能在 Gazebo 中工作 → 要求用户运行
  结果：用户端 Gazebo GUI 崩溃，功能无法验证
  ```

- ✅ **正确做法**：
  ```
  1. 先在无 GUI headless 模式下测试核心逻辑
  2. 服务节点必须先通过单元测试
  3. ROS 接口必须先通过模拟测试（不依赖 Gazebo GUI）
  4. 确认完全工作后，再告知用户
  ```

**禁止2：要求用户手动启动多个终端运行代码**

- ❌ **错误示例**（实际发生）：
  ```
  "请在3个终端分别运行：
  终端1：ros2 launch ...
  终端2：ros2 run ...
  终端3：ros2 run ..."
  ```

- ✅ **正确做法**：
  ```
  1. 创建一键启动脚本 或
  2. 提供完整的无交互测试脚本 或
  3. 在 launch 文件中集成所有逻辑
  4. 用户只需运行一条命令
  ```

**禁止3：修改代码后，未在终端验证就交付给用户**

- ❌ **错误示例**（实际发生）：
  ```
  创建 spray_action_server.py → 
  直接说"已完成，用户可以测试" → 
  用户运行时发现根本无法工作
  ```

- ✅ **正确做法**：
  ```
  修改代码 →
  在终端 colcon build →
  在终端启动节点，验证无异常 →
  在终端测试服务接口，验证功能正常 →
  所有测试通过，才告知用户
  ```

#### ✅ 强制检查清单（每次代码修改后）

在告知用户"完成"之前，**必须逐项检查**：

- [ ] **构建测试**：`colcon build` 成功，无 ERROR（WARNING 可接受）
- [ ] **节点启动**：`ros2 run <package> <node>` 能正常启动，无异常
- [ ] **接口验证**：`ros2 service/action/topic list` 能看到相关接口
- [ ] **功能测试**：手动调用服务/action，验证返回预期结果
- [ ] **数据验证**：话题/服务数据格式正确，无乱码或异常值
- [ ] **无 GUI 验证**：在无 Gazebo GUI 的终端中也能正常工作
- [ ] **错误处理**：输入异常参数时，能正确报错而不崩溃

**任何一项未通过，禁止交付给用户！**

#### 🚨 出现问题时的处理流程

**如果在个人终端测试时遇到问题**：

1. **立即停止** → 不要交付给用户
2. **自动调用** `/systematic-debugging` **技能**
3. **分层诊断**：构建层 → 环境层 → 运行时层 → 配置层
4. **找到根因后修复** → 重新完整测试
5. **确认通过后** → 才告知用户

**如果用户报告问题**：

1. **立即承认责任** → 不要甩锅给用户
2. **立即自己重现问题** → 在个人终端验证
3. **修复根因** → 再次完整测试
4. **确认通过后** → 提供修复后的代码

#### 📋 为什么增加这个规范

**历史教训**（2026-10-25）：

1. ❌ 创建了 `spray_action_server.py`，假设能工作，直接要求用户启动
2. ❌ 要求用户在3个终端分别启动（完全不自动化）
3. ❌ 用户启动时 Gazebo GUI 直接崩溃（OpenGL 错误）
4. ❌ 所有功能无法验证，白浪费用户时间

**这应该**早就在我这里测试，而不是让用户去做！

---

### 3.7️⃣ **防止重复犯错机制**

**目标**：建立长期有效的错误防御机制，而不是口头承诺

#### ✅ 规则 1：分析并记录已发生的问题

对于已经发生过的问题：

1. **分析发生原因**
   - 是否缺少测试步骤？
   - 是否跳过了诊断流程？
   - 是否未遵循既定规范？

2. **判断是否需要新增项目规则**
   - 如果是流程问题 → 更新 AGENTS.md
   - 如果是技术问题 → 记录到 Memory
   - 如果是规范问题 → 更新技能文档

3. **对可自动检测的问题增加测试或检查脚本**
   - 例如：传感器数据频率检查脚本
   - 例如：TF 树完整性检查脚本
   - 例如：控制器状态验证脚本

4. **保留必要的失败案例和验证方法**
   - 记录到 `troubleshooting/` 目录
   - 包含：问题现象、根因分析、解决方法、验证命令

#### ✅ 规则 2：遇到相同类型问题时，优先检查已有记录

**强制流程**：
```
遇到问题
  ↓
先搜索 troubleshooting/ 和 Memory
  ↓
找到相似问题？
  ├── 是 → 按已有方法解决
  └── 否 → 调用 /systematic-debugging
```

**禁止**：
- ❌ "我已经记住""以后注意"（口头承诺无效）
- ❌ 同一类型错误重复犯 2 次以上
- ❌ 已有解决方法的问题仍然从零开始诊断

#### ✅ 规则 3：建立问题分类和快速检索机制

**问题分类**（记录到 `troubleshooting/KNOWN_ISSUES.md`）：

| 问题类型 | 常见根因 | 快速检查命令 |
|---------|---------|-------------|
| **传感器无数据** | Sensors 插件未加载 / bridge 配置错误 | `gz topic -l` + `ros2 topic list` |
| **控制器不响应** | 控制器未激活 / Action Server 未启动 | `ros2 control list_controllers` |
| **TF 变换失败** | TF 树不完整 / 坐标系名称错误 | `ros2 run tf2_tools view_frames` |
| **点云格式错误** | PointFields 不兼容 / 二进制解析错误 | `ros2 topic echo <topic> --once` |
| **节点找不到** | CMakeLists 未安装 / shebang 错误 | `ros2 pkg executables <package>` |

#### ✅ 规则 4：每次完成后更新防错机制

**完成任务时的强制检查**：
1. ✅ 本次是否遇到了新类型的问题？
2. ✅ 是否需要更新 AGENTS.md 规则？
3. ✅ 是否需要创建检查脚本？
4. ✅ 是否需要记录到 Memory？

**不要等到下次犯错时再补救！**

---

### 4️⃣ **异常诊断与调试规则**

**何时触发**: 遇到编译错误、运行时崩溃、话题数据异常、控制器不响应等问题

---

## 🚨 **强制规则：遇到任何问题，立即自动调用技能！**

**触发条件**（满足任一即触发）：
- ❌ 编译错误、运行时崩溃、Segfault
- ❌ 话题无数据、控制器不响应、TF 变换失败
- ❌ 预期行为未出现（传感器应有数据但没有、节点应启动但未启动）
- ❌ 第一次尝试失败（不等第二次）

**❌ 禁止做法**：
- ❌ "再试一次"、"改个参数试试"（试错法）
- ❌ 盲目修改代码而不诊断根因
- ❌ 跳过环境、构建、配置层检查
- ❌ 等待 10 分钟后才调用技能（**遇到问题立即调用**）

**✅ 强制流程**：
1. **遇到任何问题** → **立即自动调用** `/systematic-debugging` **技能**（不等用户提醒）
2. **按照分层流程** 执行诊断（不跳过任何层）
3. **找到根因并记录证据后** 再修改代码

---

## 诊断流程（按 robotics-agent-skills 标准）

**第一步：激活诊断技能**

```bash
# Claude 执行（无需手动）: /systematic-debugging
```

**第二步：分层诊断（按顺序，严格执行，不跳过）**

1. **构建层** — 代码编译是否成功？
   ```bash
   # 清理并重新构建
   rm -rf ~/SCI/hydromining_ws/build ~/SCI/hydromining_ws/install
   colcon build --packages-select <affected-package> --symlink-install
   # 重点：注意所有 WARNING，不只看 ERROR
   ```

2. **环境层** — 包、话题、节点是否可见？
   ```bash
   # 验证包可见性
   ros2 pkg list | grep hydromining
   
   # 验证话题/服务/节点
   ros2 topic list
   ros2 node list
   ```

3. **运行时层** — 运行时状态是否符合预期？
   ```bash
   # TF 树结构
   ros2 run tf2_tools view_frames
   
   # 话题数据
   ros2 topic echo <topic_name> --once
   
   # 控制器状态
   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ```

4. **配置层** — 模型路径、资源是否正确？
   ```bash
   # 检查 Gazebo 资源路径
   echo $GZ_SIM_RESOURCE_PATH
   
   # 检查模型文件存在性
   ls ~/SCI/hydromining_ws/install/hydromining_sim/share/hydromining_sim/models/
   ```

**第三步：修改与验证**
- 只修改当前任务需要的内容（不做无关重构）
- 修改后立即验证"测试与验证规则"
- 记录修改原因在 commit message 或注释中

---

**应调用的技能**:

- **遇到任何问题，立即自动调用** `/systematic-debugging` **技能**（不等第二次失败）

---

### 5️⃣ **代码修改范围原则**

**何时应用**: 任何代码变更

**约束**:

- ✓ **仅修改当前任务需要的文件**
  - 例: 修复相机话题发布，不修改激光雷达配置
- ✗ **禁止**无关重构 (如重命名全局变量、重组项目结构)
- ✓ 保持与现有代码风格一致 (注释密度、命名约定、缩进)

**项目文件结构**（2026-11-01 按论文计划整理后；结构变更须经用户确认）:

```
hydromining_ws/
├── src/
│   ├── hydromining_sim/
│   │   ├── models/coal_wall/          ← 煤壁几何模型
│   │   ├── worlds/                    ← Gazebo 世界定义
│   │   ├── scripts/algorithms/        ← 无 ROS 依赖的核心算法（感知、破煤、规划、task_spec.py）
│   │   └── test/                      ← pytest 单元测试
│   └── hydromining_robot/
│       ├── urdf/                      ← URDF/xacro 机器人模型
│       ├── config/                    ← 控制器、任务 yaml（phase7_large_task.yaml 为当前任务）
│       ├── launch/
│       └── scripts/                   ← ROS 适配层节点
├── experiments/                       ← 论文实验，每个小实验一个文件（见"目录约定"）
├── configs/experiments/               ← 实验配置
├── results/thesis/  results/legacy/   ← 新实验结果 / 旧结果
├── test_scripts/                      ← 旧阶段验收器与实验脚本（阶段 1 重构中逐步迁入 experiments/common）
├── scripts/                           ← Shell 入口（验收、训练、GUI 演示、环境）
├── troubleshooting/KNOWN_ISSUES.md    ← 已知问题库（遇到问题先查）
├── build/, install/, log/             ← colcon 输出（自动生成）
├── robotics-agent-skills/             ← 外部技能库（禁止修改，不进 git）
└── .claude/skills/                    ← 项目本地技能配置
```

---

### 6️⃣ **会话交接记录规则**

**何时触发**: 每完成一个任务/子步骤，或对话可能中断前

**🚨 强制要求**：用户可能随时开新对话，**每次完成后必须更新交接记录**，不要等用户提醒

- ✓ 更新 [`docs/进度总表.md`](docs/进度总表.md)：实验状态、结果路径、台账加一行（含提交号）
- ✓ 结论必须区分 **已核实** / **未验证**，未验证的不得写成事实
- ✓ 推翻旧结论时，同步修正 memory 和本文件中的错误内容
- ✓ 本项目自 2026-11-01 起是 **git 仓库**：每个阶段完成后提交一次并打标签；阶段内的进度先记入进度总表，阶段末随提交入库
- ✓ `docs/阶段文档/*/00-进度交接.md` 为旧阶段记录，不再作为当前进度依据

---

## 🛠️ 技能配置与安装

**技能来源**：本项目使用 [robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills) 提供的标准化开发技能。

### 已安装的技能

| 技能名 | 路径 | 功能 | 强制使用条件 |
|--------|------|------|-------------|
| **systematic-debugging** | `.claude/skills/systematic-debugging.md` | 分层诊断问题（信息收集→构建层→环境层→运行时层→配置层） | ⚠️ **遇到任何问题立即自动调用**（不等第二次失败） |
| **verification-before-completion** | `.claude/skills/verification-before-completion.md` | 完成前验证清单（构建→启动→话题→控制器） | ⚠️ **代码提交/完成前** |
| **ros2** | `.claude/skills/ros2.md` | ROS 2 Jazzy 开发规范（节点、Launch、参数、QoS） | 创建/修改 ROS 2 包、节点 |

### 技能使用规则 — 强制流程

**强制激活场景**（⚠️ **不可跳过，自动调用**）：

1. **遇到任何问题** 
   - 🚨 **立即自动调用**: `/systematic-debugging`
   - ❌ **禁止**: "再试一次"、改参数、继续试错
   - ⚠️ **不等**：不等 10 分钟，不等第二次失败，**第一次遇到问题就调用**

2. **完成代码修改** 
   - 🚨 **立即调用**: `/verification-before-completion`
   - ❌ **禁止**: 未验证就认为完成

3. **开发 ROS 2 节点** 
   - 🚨 **立即调用**: `/ros2`
   - ❌ **禁止**: 直接复制鱼香ROS的 Humble 代码

**技能激活方式**：
- **命令方式**：输入 `/systematic-debugging` 或 `/ros2`
- **自动触发**：当 AGENTS.md 规则满足条件时，Claude 应主动加载对应技能

### 需要手动安装的技能

以下技能源于 `robotics-agent-skills` 库，需复制到 `.claude/skills/`：

```bash
# 如果 ros2.md 未安装
cp robotics-agent-skills/skills/ros2/SKILL.md .claude/skills/ros2.md

# 安装机器人测试技能
cp robotics-agent-skills/skills/robotics-testing/SKILL.md .claude/skills/robotics-testing.md

# (可选) 其他推荐技能
cp robotics-agent-skills/skills/robot-perception/SKILL.md .claude/skills/robot-perception.md
cp robotics-agent-skills/skills/robotics-design-patterns/SKILL.md .claude/skills/robotics-design-patterns.md
```

### 技能激活方式

1. **自动激活** (无需指定): `systematic-debugging`, `verification-before-completion`
2. **显式激活** (输入 `/ros2` 或提及 ROS): `ros2.md`
3. **显式激活** (输入 `/robotics-testing` 或提及测试): `robotics-testing.md`

### 验证技能是否可发现

```bash
ls -1 ~/.claude/skills/ ~/.autohand/skills/ ~/SCI/hydromining_ws/.claude/skills/ 2>/dev/null | grep "\.md$"
```

---

## 📚 代码模板库（Codebook）

### 推荐组织结构

参考鱼香ROS，建议新增 `docs/codebook/` 存放可复制粘贴的代码模板：

```
docs/codebook/
├── rclpy/              # Python 节点模板
│   ├── publisher.py
│   ├── subscriber.py
│   └── service.py
├── launch/             # Launch 文件模板
│   ├── basic_launch.py
│   └── gazebo_launch.py
├── config/             # 参数文件模板
│   └── params.yaml
└── cmake/              # CMakeLists.txt 片段
    └── install_targets.cmake
```

### 常用代码模板速查

详细模板参见 `docs/鱼香ROS代码规范总结.md`，快速链接：

- **Python 节点模板**：[鱼香ROS规范 - Python 节点](docs/鱼香ROS代码规范总结.md#🐍-python-节点编写规范)
- **Launch 文件模板**：[鱼香ROS规范 - Launch 文件](docs/鱼香ROS代码规范总结.md#🚀-launch-文件规范)
- **CMakeLists.txt 模板**：[鱼香ROS规范 - CMake](docs/鱼香ROS代码规范总结.md#🛠️-cmakeliststxt-规范)
- **package.xml 模板**：[鱼香ROS规范 - Package](docs/鱼香ROS代码规范总结.md#📦-packagexml-规范)

---

## 📖 构建、测试、启动命令汇总

### 构建命令 (已验证)

```bash
# 清理构建
rm -rf ~/SCI/hydromining_ws/build ~/SCI/hydromining_ws/install

# 仅构建项目包 (推荐)
cd ~/SCI/hydromining_ws
colcon build --symlink-install --packages-select hydromining_robot hydromining_sim

# 预期输出
# Finished <<< hydromining_sim [0.40s]
# Finished <<< hydromining_robot [0.52s]
# Summary: 2 packages finished [1.12s]
```

### 启动仿真 (已验证)

```bash
# 源代码
source ~/SCI/hydromining_ws/install/setup.bash

# 启动完整仿真 (Gazebo + 机器人 + 传感器)
ros2 launch hydromining_robot robot_simulation.launch.py

# 新终端：测试机器人运动
source ~/SCI/hydromining_ws/install/setup.bash
ros2 run hydromining_robot test_motion.py

# 新终端：验证传感器
ros2 topic hz /hydromining/camera/color/image_raw
ros2 topic hz /hydromining/lidar/points
```

### 诊断命令 (问题排查)

```bash
# TF 树结构
ros2 run tf2_tools view_frames

# 话题列表和活跃状态
ros2 topic list
ros2 topic hz <topic_name>

# 节点信息
ros2 node list
ros2 node info <node_name>

# 控制器状态
ros2 control list_controllers
ros2 control list_hardware_interfaces

# Gazebo 模型检查
gz model list  # 列出 Gazebo 中的所有模型
gz service call /world/default/set_pose_vector '{"poses":[...]}'
```

---

## 📚 版本差异速查表（Humble vs Jazzy）

**⚠️ 重要**：鱼香ROS教程使用 ROS 2 Humble，本项目使用 ROS 2 Jazzy，以下是关键差异：

| 项目 | ROS 2 Humble (鱼香ROS) | ROS 2 Jazzy (本项目) |
|------|----------------------|-------------------|
| **Gazebo 版本** | Fortress (ignition-gazebo-6) | **Harmonic (gz-sim-8)** |
| **Bridge 包** | `ros_ign_bridge` / `ros_gz_bridge` | `ros_gz_bridge` |
| **传感器系统插件** | `ignition-gazebo-sensors-system` | `gz-sim-sensors-system` 或 `libgz-sim8-sensors-system.so`（两者都存在，均有效） |
| **相机 / LiDAR** | 无单独插件，由 Sensors 系统统一渲染 | 同左：只需 world 中加载一次 Sensors 系统 |
| **SDF 中 frame** | `<ignition_frame_id>` | `<gz_frame_id>` |
| **控制插件** | `ign_ros2_control-system` | `gz_ros2_control-system` |
| **命令行工具** | `ign` | `gz` |
| **Gazebo 启动** | `ign gazebo` | `gz sim` |

**使用建议**：
- ✅ 学习鱼香ROS的**设计思路、架构模式、命名规范**
- ✅ 参考鱼香ROS的**节点结构、Launch 文件组织、文档风格**
- ❌ **不要**直接复制鱼香ROS的插件配置、URDF Gazebo 标签
- ❌ **不要**假设 Humble 的 API 在 Jazzy 中可用

**验证方法**：
```bash
# 检查已安装的 Gazebo 插件（应该是 gz-sim-8）
ls /opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/

# 检查 bridge 包（应该是 ros_gz_bridge）
ros2 pkg list | grep gz

# 检查 ROS 2 版本
echo $ROS_DISTRO  # 应输出：jazzy
```

---

## ⚠️ 常见问题与解决方案

### Q1: `colcon build` 失败，提示 `venv/lib/python3.12/site-packages/numpy`

**原因**: colcon 误认为虚拟环境内的包为项目包

**解决**:
```bash
# 清除虚拟环境引用
rm -rf ~/SCI/hydromining_ws/venv

# 使用系统 Python 或新建虚拟环境
python3 -m venv ~/SCI/hydromining_ws/venv
source ~/SCI/hydromining_ws/venv/bin/activate
pip install numpy  # 如需要
```

### Q2: Gazebo 找不到煤壁模型

**原因**: 环境变量 `GZ_SIM_RESOURCE_PATH` 未设置

**解决**:
```bash
source ~/SCI/hydromining_ws/setup_env.sh
# 或手动设置
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/SCI/hydromining_ws/src/hydromining_sim/models
```

### Q3: `ros2 control list_controllers` 为空或报错

**原因**: 控制器未加载，或机器人未正确生成

**解决**:
1. 检查 Gazebo 是否启动: `gz model list`
2. 检查节点状态: `ros2 node list | grep controller`
3. 查看启动日志: 在仿真启动终端中查看错误消息

### Q4: 相机/雷达话题无数据

**原因**: `ros_gz_bridge` 未正确配置或未启动

**解决**:
```bash
# 检查 bridge 节点状态
ros2 node info /ros_gz_bridge

# 查看 bridge 论文的参数
ros2 param list /ros_gz_bridge

# 重启仿真
^C  # 停止现有仿真
ros2 launch hydromining_robot robot_simulation.launch.py
```

---

## 📝 修改记录

| 日期 | 版本 | 变更 | 维护者 |
|------|------|------|--------|
| 2026-11-01 | 2.0 | 转为按论文总体计划执行：新增"新对话必读"、目录约定、git 规则；旧阶段补充压缩为历史摘要 | Star |
| 2026-10-23 | 1.1 | 集成鱼香ROS代码规范，新增代码模板库指引 | Star |
| 2026-10-22 | 1.0 | 初始版本: 阶段 1 + 2 完成，技能框架建立 | Star |

---

## 🔗 相关文档

- **论文总体计划**: `docs/论文总体计划.md` ⭐ 当前执行依据
- **进度总表**: `docs/进度总表.md` ⭐ 当前进度依据
- **已知问题库**: `troubleshooting/KNOWN_ISSUES.md`
- **项目快速开始**: `QUICKSTART.md`
- **鱼香ROS代码规范**: `docs/鱼香ROS代码规范总结.md`
- **新机器环境搭建**: `docs/新机器环境搭建记录-2026-10-30.md`
- **旧阶段开发记录**: `docs/阶段文档/`（历史参考）

---

**最后更新**: 2026-11-01  
**规则适用范围**: 论文计划所有阶段及旧阶段回归  
**许可**: Apache-2.0

---

## 🖥️ 本机环境与资源约定（2026-10-30 核实，仍有效）

- 工作目录 `/home/star/SCI/hydromining_ws`；旧克隆已移至 `~/SCI/hydromining_old_clone_20261101`，禁止混用。
- 环境入口 `source setup_env.sh`；只读检查 `./scripts/check_environment.sh`；恢复依赖与构建 `./scripts/setup_new_machine.sh`。
- 两个虚拟环境：`.venv_stage4`（仿真/感知，CPU）、`.venv_rl`（SB3 + CUDA 训练）。官方 ROS CLI overlay 在 `.local_ros_deps/opt/ros/jazzy`，由入口加载。
- 构建：`colcon build --symlink-install --base-paths src --packages-select hydromining_robot hydromining_sim`（必须带 `--base-paths src`）。单元测试：`python3 -m pytest src/hydromining_sim/test -q`。
- 硬件：4×TITAN V（12 GiB），约 62 GiB 内存。GPU0 留给桌面显示，GPU1–3 用于训练。用户允许开启渲染，资源保护由 `HYDROMINING_RSS_GIB`（默认 24）、`HYDROMINING_MIN_AVAILABLE_GIB`（默认 8）控制，旧电脑的 3 GiB 上限不再适用。
- 接缝加载按几何焊接忽略法向/UV；不得为通过测试删除 OBJ 法向或关闭闭合性校验。

## 用户指定：防止过度设计与无效工作（2026-10-30）

后续工作严格限定在用户需求和完成任务必需的范围内，遵循以下规则：

1. **防过度防御**：对于几乎不会发生的边界或并发问题，先说明发生概率及具体影响，经用户确认后再增加相应防护；严禁仅为“理论上更稳”私自膨胀代码。
2. **入口单次校验**：外部数据在系统边界校验一次，内部依靠类型和已建立的约定流转，禁止在多层重复做相同判断。
3. **拒绝提前抽象**：单次使用的逻辑就近内联，不为假设中的未来需求拆分无意义的 helper 或增加抽象。
4. **控制复杂度**：非必要不使用 try/catch；一个函数能够清晰完成的逻辑，不强行拆分。

这些规则不取消用户明确要求的功能、项目已有的实际错误处理与验证要求；以实际需求和已发现的风险决定必要实现，避免重复防御和无关工作。

## 📜 旧阶段开发历史摘要（2026-10-22 至 10-31，仅供参考）

- 阶段 1–6：煤壁生成、机械臂与水枪、传感器、Level Set 喷射、水雾粉尘干扰、感知管线均已实现，并通过各自无 GUI 验收；验收入口 `./scripts/run_phase{4,5,6}_acceptance.sh` 仍可用作回归。物理与噪声参数未标定。
- 阶段 7：规则决策 + NBV 在真实 GUI 闭环达标（`results/legacy/gui_demo_20261031_114539/`，感知下界 90.0%、真值 94.6%）；40 cm 任务的 L0 替身与 SB3 训练入口已接入。GUI 演示 `./scripts/run_gui_demo.sh`。
- 旧 RL 训练结果在 `results/legacy/phase7_training_large/`，已确认因奖励缺陷而全部失败，见"新对话必读"。
- 粒子强度对点云退化的单次排序曾失败、后一次通过，属于单次结果，不是统计标定；E2.3 要求重复 ≥5 次。
- 详细记录：`docs/阶段文档/*/00-进度交接.md`。

