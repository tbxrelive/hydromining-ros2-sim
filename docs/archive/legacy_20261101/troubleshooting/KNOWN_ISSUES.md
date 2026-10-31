# 已知问题索引 — Hydromining 项目

## 2026-10-31 高保真评估贯穿后真值缺失

- **现象**：SAC L2 的候选喷射触发 `Through-cut detected`；对未提交候选前的表面做终局真值核对时，任务区存在无交点，导致 `truth does not cover the entire task region`，进而使调度器停止。原始记录在 `results/phase7_training_large/gpu3.log`。具体缺失几何原因尚未完全定位，不能把候选喷射当成已提交穿透。
- **处理**：贯穿终止且真值不全时记 `truth=unknown` 及有效覆盖率，精确达标比例和移除体积留空；其他路径仍严格拒绝缺失。汇总单列未知数量，不当作误达或凭空填值。
- **验证**：`test_perception_truth.py` 的缺失真值贯穿回归、`test_large_evaluation_missing_truth.py` 的汇总回归；真实 SAC seed 0 的 L2 单回合贯穿终止完成，但该回合真值完整，不等于 12 回合均已重验。恢复计划使用 `./scripts/run_phase7_training.sh --output results/phase7_training_large --dry-run` 核对；已完成的模型和评估应跳过。

## 2026-10-26 阶段五新增核实项

- **重放 UPDATING 的 create 超时、Host unreachable**：RPC 未收到回执不等于实体未创建。最新适配器用 typed `scene/info` 按精确模型名及目标网格 URI 核实；创建回执丢失时先读状态，确认缺失才一次禁止重命名的重发，未知状态要求重置，不盲目回滚。GUI 二次峰值 2579.707 MiB，不能归因于 3 GiB 保护。新增 `coal_wall_scene` 必须由 ROS 包索引传入安装路径，不能用 Python 源码文件的相邻位置猜测，symlink 安装会改变解析位置。见阶段五 04 报告及事务单元/真实服务故障注入测试。
- **控制器切换尚未结束就重发并报已 active**：服务调用默认等 10 秒，而 switch-timeout 设置 30 秒，前者可能先超时。启动器设置 service-call-timeout=40 秒覆盖切换预算；不通过改变控制器状态或全局杀进程掩盖。
- **GUI 重放准备时自动关闭、Memory guard**：先看 `failure.json`，不能当正常完成或直接定性为 OOM。用户首次记录为 3081.238 MiB，超过 3 GiB 脚本阈值。分进程记录显示计算节点在重复 BVH/NumPy 求解后保留大量工作区；验收进程树限制 glibc 分配区及大块分配/归还阈值，GUI 默认省去额外截图相机。最终无 GUI 全流程峰值从基线 2626.723 降到 1952.633 MiB，保留保护阈值；这不是 GUI 重验通过的证据。新增 `resources.jsonl` 与失败快照用于继续定位，详见阶段五 `03-GUI内存保护中断与修复验证.md`。
- **验收结果不清楚、保持窗口最后只剩平煤壁**：功能检查完成立即生成 `checks.json`；保持前额外重放一次并保存 `06_final_view`，退出清理后才生成最终 `summary.json`。保持期间继续监测资源，后续失败不得被覆盖为 passed。仅凭旧图片或中间统计不能认定本次完整验收通过。
- **有粒子实体/回执但传感器完全不变**：动态创建的模型必须通过 `SdfEntityCreator.SetParent(model, world)` 挂到世界。仅有 emitter_count 回执不足以证明渲染或传感器接收，必须做真实图像/点云对照。
- **雾粒子出现白色方块边缘**：Ogre2 PBS→unlit 复制的透明 blend block 使用 ONE / ONE_MINUS_SOURCE_ALPHA；纹理需采用预乘 RGB，量化为 alpha=0 的像素 RGB 也必须为 0。用本地生成脚本和资产回归检查，不修改全局渲染库。
- **喷射中 Ctrl+C 的节点段错误或 pybind 转换异常**：自动先关闭 context/直接在信号回调抛异常可能撞上在途消息转换。项目节点和验收器使用仅置位的信号处理；生产节点排空工作线程后才销毁 ROS 句柄。退出测试同时检查主进程和仿真子进程日志。
- **效果 parameter_bridge 在退出时偶发 std::system_error / SIGABRT**：先保留原生日志，不能因顶层脚本退出且无遗留就算通过。效果桥使用已安装 ros_gz_bridge 库的显式关闭包装：取消 Gazebo 订阅→释放桥接句柄→关闭 ROS context；验收先停止喷射生产者，再停止 launch。未改系统 ros_gz_bridge 安装。
- **低资源测试反而占满一个 CPU 核**：/clock 可能以 1 kHz 唤醒 spin_once，禁止每次消息都扫描全部进程树；资源监控节流到 0.5 秒，关节漂移检查仍保留。
- **冷启动控制器/生命周期等待超时**：首次离屏渲染和初始 SDF 建立可能跨过原 5/10 秒等待窗口。当前 switch-timeout=30 秒，生命周期首个查询等待 30 秒；须查看服务状态而非只看进程启动。

阶段五物理参数未标定、GUI 待用户验收。详见阶段五进度交接及验证报告。

## 2026-10-26 阶段4集成新增核实项

- **Ctrl+C后报 `rcl_shutdown already called` 且GUI未退出**：信号处理可能先关闭ROS context；无条件再次shutdown会中断finally并跳过进程清理。使用带原context的 `rclpy.try_shutdown()`，ROS收尾错误不得阻止本次子进程清理；用 `test_scripts/test_phase4_sigint.py` 实际覆盖无GUI保持运行后SIGINT退出，不能用正常结束代替Ctrl+C验收。详见阶段4的09报告。

- **新网格加载时 Harmonic/DART 崩溃**：若日志有 `normal count ... 0 ... vertex count`，先检查 OBJ 是否有与顶点数量一致的 `vn`，不要归因 `libEGL warning`。阶段1已有相同法向导出要求；阶段4 `MeshConverter.save_mesh()` 现显式使用 `include_normals=True`，算法回归增加法向数量断言。
- **点云“变到 world 后不在煤壁上”**：检查坐标轴和 frame 标签。当前 Gazebo RGB-D 点是本体 +X 前向，header 使用 `camera_link`，不能只标为 `camera_optical_frame` 而不旋转数据。采集需等待 TF 到达消息的精确时间戳，禁止用 latest TF 掩盖时间错误。
- **离线计算占内存过高**：禁止恢复体素点×全部顶点的 `cdist` 矩阵；当前使用单线程 BVH 三角形距离查询，网格/积分/墙钟/内存均设预算。构建限制 Make/CMake 两任务，不能只限制 colcon 包并行数。
- **退出后 gz 服务器仍占 CPU/内存**：`ros_gz_sim` 经 shell 启动，launch 退出不必然代表所有后代退出。验收脚本在停 launch 前保存自己的后代进程，随后仅清理这些已确认属于本次运行的进程，并检查无活动遗留。禁止全局 `pkill gz`。
- **重激活生命周期节点后几何与数值状态不一致**：激活应导出当前保留的 phi，重置才恢复 initial_phi；真实 ROS 接口回归覆盖停用拒绝与重新激活保留网格。

当前验证证据见 `docs/阶段文档/阶段4-喷射效果模拟/08-六步无GUI集成验证报告.md`。GUI 最终观察尚未验证，参数未标定。

**目的**：记录已发生的问题和解决方法，避免重复犯错。

---

## 问题分类索引

### 1. 传感器相关问题

#### 1.1 传感器无数据 / 话题不发布

**常见根因**：

1. ⚠️ **World 未加载 Sensors 系统插件**
   - 症状：`gz topic -l` 能看到话题，但 `ros2 topic list` 看不到
   - 解决：在 `*.sdf` 的 `<world>` 标签中添加：
     ```xml
     <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
       <render_engine>ogre2</render_engine>
     </plugin>
     ```

2. ⚠️ **ros_gz_bridge 配置错误**
   - 症状：`gz topic -l` 和 `ros2 topic list` 都有话题，但数据不通
   - 检查方法：
     ```bash
     gz topic -l                     # 查看 Gazebo 话题
     gz topic -e -t <topic_name>     # 监听 Gazebo 数据
     ros2 topic list                 # 查看 ROS 2 话题
     ros2 topic hz <topic_name>      # 检查 ROS 2 数据频率
     ```
   - 常见错误：
     - 话题名称不匹配（`/hydromining/lidar` vs `/hydromining/lidar/points`）
     - 消息类型不匹配（`sensor_msgs/LaserScan` vs `sensor_msgs/PointCloud2`）
   - 解决：对照 `gz topic -l` 的输出，修正 bridge 配置

3. ⚠️ **传感器未加载或配置错误**
   - 症状：`gz topic -l` 没有传感器话题
   - 检查方法：
     ```bash
     gz model -m <model_name> -i     # 查看模型详细信息
     ```
   - 解决：检查 URDF/xacro 中的 `<sensor>` 配置

**快速检查命令**：
```bash
# 完整诊断流程
gz topic -l                                    # 1. Gazebo 侧话题
ros2 topic list | grep hydromining             # 2. ROS 2 侧话题
ros2 topic hz /hydromining/lidar/points        # 3. 数据频率
ros2 topic echo /hydromining/lidar/points --once  # 4. 数据内容
```

**相关文档**：
- `docs/阶段文档/阶段3-传感器集成/00-进度交接.md`（已纠正的错误结论）
- Memory: `stage3-sensor-problem-review.md`

---

### 2. 控制器相关问题

#### 2.1 控制器不响应 / Action 调用超时

**常见根因**：

1. ⚠️ **控制器未激活**
   - 症状：`ros2 control list_controllers` 显示 `inactive`
   - 检查方法：
     ```bash
     ros2 control list_controllers
     # 预期: panda_arm_controller [joint_trajectory_controller/JointTrajectoryController] active
     ```
   - 解决：检查 `ros2_control` 配置和 `controller_manager` 启动

2. ⚠️ **Action Server 未启动**
   - 症状：`ros2 action send_goal` 超时
   - 检查方法：
     ```bash
     ros2 action list | grep panda
     # 预期: /panda_arm_controller/follow_joint_trajectory
     ```
   - 解决：检查控制器配置中的 `action_ns` 参数

**快速检查命令**：
```bash
ros2 control list_controllers           # 1. 控制器状态
ros2 control list_hardware_interfaces   # 2. 硬件接口
ros2 action list | grep panda           # 3. Action 接口
ros2 topic hz /joint_states             # 4. 关节状态发布频率
```

---

### 3. TF 变换相关问题

#### 3.1 TF 树不完整 / 坐标系找不到

**常见根因**：

1. ⚠️ **robot_state_publisher 未启动**
   - 症状：`ros2 run tf2_ros tf2_echo world panda_link0` 报错
   - 检查方法：
     ```bash
     ros2 node list | grep robot_state_publisher
     ros2 run tf2_tools view_frames  # 生成 frames.pdf
     ```
   - 解决：检查 launch 文件中的 `robot_state_publisher` 节点

2. ⚠️ **URDF 中缺少 link 或 joint**
   - 症状：TF 树缺少某些坐标系
   - 检查方法：
     ```bash
     ros2 run tf2_tools view_frames
     evince frames.pdf  # 查看 TF 树结构
     ```
   - 解决：检查 URDF/xacro 文件，确保所有 link 和 joint 定义完整

**快速检查命令**：
```bash
ros2 run tf2_tools view_frames                      # 1. 生成 TF 树
ros2 run tf2_ros tf2_echo world panda_link0         # 2. 查询特定变换
ros2 topic hz /tf /tf_static                        # 3. TF 发布频率
```

---

### 4. 点云数据相关问题

#### 4.1 点云格式错误 / 解析失败

**常见根因**：

1. ⚠️ **PointFields 不兼容**
   - 症状：`ros2 topic echo` 显示点云，但读取时报错
   - 检查方法：
     ```bash
     ros2 topic echo /hydromining/lidar/points --once | grep fields
     ```
   - 解决：手动解析 `PointCloud2` 消息（见阶段 3 实现）

2. ⚠️ **NaN 值未过滤**
   - 症状：点云包含无效点
   - 检查方法：读取点云时检查 `math.isnan(x)`
   - 解决：在转换为 numpy 数组时过滤 NaN

**快速检查命令**：
```bash
ros2 topic echo /hydromining/lidar/points --once | head -50  # 1. 查看消息格式
ros2 topic hz /hydromining/lidar/points                      # 2. 数据频率
```

**相关文档**：
- `src/hydromining_robot/scripts/pointcloud_capture.py`（手动解析实现）
- `docs/阶段文档/阶段3-传感器集成/03-阶段评估报告.md`

---

### 5. 构建相关问题

#### 5.1 节点找不到 / 可执行文件不存在

**常见根因**：

1. ⚠️ **CMakeLists.txt 未安装可执行文件**
   - 症状：`ros2 run <package> <node>` 报错找不到
   - 检查方法：
     ```bash
     ros2 pkg executables <package>
     ```
   - 解决：在 `CMakeLists.txt` 中添加：
     ```cmake
     install(PROGRAMS
       scripts/your_node.py
       DESTINATION lib/${PROJECT_NAME}
     )
     ```

2. ⚠️ **Python 脚本缺少 shebang 或执行权限**
   - 症状：`ros2 run` 报错权限不足
   - 检查方法：
     ```bash
     ls -l install/<package>/lib/<package>/<node>
     ```
   - 解决：
     ```bash
     chmod +x src/<package>/scripts/<node>.py
     ```
     并确保第一行有 `#!/usr/bin/env python3`

**快速检查命令**：
```bash
ros2 pkg executables <package>                # 1. 列出可执行文件
ros2 pkg list | grep <package>                # 2. 检查包是否可见
colcon list --packages-select <package>       # 3. 检查包配置
```

---

## 使用指南

### 遇到问题时的流程

1. **先查本索引**
   - 按问题类型快速定位
   - 执行"快速检查命令"

2. **如果找到相似问题**
   - 按照记录的解决方法执行
   - 验证是否解决

3. **如果未找到相似问题**
   - 立即调用 `/systematic-debugging`
   - 按照分层流程诊断（构建层 → 环境层 → 运行时层 → 配置层）
   - **解决后更新本索引**

### 添加新问题的格式

```markdown
#### X.Y 问题标题

**常见根因**：

1. ⚠️ **根因描述**
   - 症状：...
   - 检查方法：
     ```bash
     command_to_check
     ```
   - 解决：...

**快速检查命令**：
```bash
command1  # 1. 说明
command2  # 2. 说明
```

**相关文档**：
- 文档路径或 Memory 链接
```

---

**维护日期**：2026-10-24  
**下次更新**：每完成一个阶段或遇到新问题后更新


## 2026-10-30 新机器迁移核查

- 外层工作区没有 .claude/.git，但内层 SCI/ 有：先核对隐藏文件与实际克隆位置，不用复制后的源码目录状态推断远端内容丢失。此次 src/scripts/test_scripts/docs 核查开始时逐文件一致。
- ros2/gz 不在初始 PATH：source /opt/ros/jazzy/setup.bash 后可见，Gazebo 8.15.0。不要当作软件未安装。
- ros2 control 报 invalid choice：本次 CLI 扩展不可用，ros2 pkg prefix ros2controlcli 找不到包；Gazebo 日志仍显示两控制器激活成功。CLI 缺失与控制器加载失败须分开诊断。尚未安装修复。
- 构建成功但算法依赖缺失：本次两包编译通过，系统 Python 全量 pytest 收集仍因 trimesh/open3d 失败，.venv_stage4 不存在。必须分别检查构建、Python 导入、接口与功能，不能用 colcon 成功代替业务可运行。
- venv_test 已被 .gitignore 忽略却仍跟踪：git -C SCI ls-files venv_test 仍列出文件。新增 ignore 不会自动从索引移除；本次未执行索引或历史清理。
- 证据：results/project_audit_20261030/；完整诊断见 docs/项目全面核查汇报-2026-10-30.md。


## 2026-10-30 环境搭建修复记录

- **py4dgeo导入报matplotlib.tri.triangulation缺失**：系统Matplotlib3.6.3的nspkg.pth预加载系统mpl_toolkits，与venv Matplotlib3.9.4混用。统一使用系统3.6.3；不要盲目反复重装py4dgeo。实际mplot3d/py4dgeo导入与pip check通过。
- **煤壁报not watertight**：trimesh默认合并保留法向/UV差异。原资产按几何合并应为12800顶点闭合体；voxel_grid明确merge_norm=True/merge_tex=True后原煤壁保真回归与真实六步通过。保持1e-7m焊接精度、OBJ法向和闭合性拒绝，禁止删除校验掩盖问题。
- **ros2controlcli可发现但view_controller_chains缺graphviz**：ros2系统脚本使用系统解释器。虚拟环境生成ros2启动器，由source setup_env.sh激活后统一解释器。graphviz/dot和命令插件实际检查通过。
- **缺sudo且ROS声明依赖包缺失**：官方Jazzy纯Python包解包到.local_ros_deps/opt/ros/jazzy，本地local_setup与环境入口加载；不是系统apt注册安装，不要用dpkg Installed状态代替ROS overlay可见性判断。
- 本次证据：results/environment_setup_20261030/，其中保留首次失败与修复后日志；环境说明见docs/新机器环境搭建记录-2026-10-30.md。


## 2026-10-30 复现与训练防错补充

- NBV重复观测不能平均掉系统误差下限；precision融合只处理独立方差，再加systematic_sigma。回归覆盖“系统误差已达下限时gain=0”。真实可见性禁止全1或读取真值网格，射线采样轴与meshgrid xy对应(1,0)。
- L1慢主要在逐格点融合；单点栅格批量路径须与原标量数值等价，不能为提速删除Huber/误差下限。多点局部平面仍走原路径。
- SB3并行环境增加时gradient_steps=1会降低每样本更新比；优化配置明确gradient_steps=-1；预算截断保留bootstrap，达标同一步优先terminated。
- 恢复训练必须检查观测版本、算法/示范哈希和环境数；checkpoint完整代次原子发布，环境从新回合恢复。只看到zip可加载不足以证明回放/优化器可恢复。
- Gazebo GUI截图RPC接受StringMsg目录并异步写PNG；必须等待文件并实际查看。最终凹坑需近景；射流需在SPRAYING期间抓图。旧3GiB记录不再约束本机，使用可配置资源保护。
- 证据：results/reproduction_20261030/，最终144项测试及真实六工况/GUI均通过；长训未执行。


## 2026-10-30 GUI反复Ctrl+C导致清理中断

用户在GUI inspection ready后连续按Ctrl+C，首个中断进入finally，后续SIGINT在process.wait期间抛KeyboardInterrupt，跳过后续清理；gz sim GUI/子进程可能遗留。phase7_real_loop.py在finally入口忽略重复SIGINT、提示等待、执行原分阶段的本次进程清理；两次真实GUI进程的连续中断测试均无本次残留（keep-open测试的回合是短替身，非采煤验收）。证据results/gui_interrupt_fix_20261030_v2/与results/gui_keepopen_interrupt_20261030/。不要用全局pkill关闭其他用户的Gazebo。
