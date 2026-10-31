# 阶段4：六步无 GUI 集成验证报告

**日期**：2026-10-26  
**结论**：六步流程已在真实 Gazebo Harmonic 服务器、Panda 控制器和 RGB-D 传感器链路上验证通过；GUI 界面的最终观察仍待维护者验收。**冲蚀参数未标定，以下是仿真原型结果，不能作为真实煤体破坏深度预测。**

## 实际结果与证据

结果目录：`results/phase4_acceptance/2026-10-26/`。`summary.json` 是当次测试的数值记录，`simulation.log` 是真实仿真运行日志，`comparison.png` 上排来自 Gazebo RGB 相机实际离屏渲染，下排来自实际 RGB-D 点云差分。

| 验收步骤 | 当次证据 | 结果 |
|---|---|---|
| 1 初始煤壁与喷前点云 | `01_before_rgb.png`、`01_before.pcd`、`01_before.npz` | 71756 个有效实测点；按消息时间戳转换到 world |
| 2 固定机械臂并喷射 | 控制器轨迹成功；喷射输入读取实际 world→tcp TF | 固定期间 1626 帧关节状态的最大观测偏差 0 rad |
| 3 更新煤壁、输出统计 | 初始、首喷 OBJ；Gazebo remove/create 确认，渲染和点云均变化 | 首喷深度 0.05518037 m，体积 0.0004553611 m³ |
| 4 喷后点云观测凹坑 | `02_after_first.pcd/.npz`，ROI 2014 点 | 396 点沿射流方向退缩超过 5 mm；ROI 的 P95 退缩 0.02874921 m |
| 5 累计喷射 | `03_after_second.pcd/.npz`、第二次更新 OBJ | 累计深度 0.09299237 m，累计体积 0.0007968819 m³；532 点退缩超过 5 mm |
| 6 重置、重放 | `04_reset.*`、`05_replay.*`；首次与重放 OBJ 的 SHA256 比较 | 网格字节一致；重置点云 P95 差 5.17e-13 m，重放点云 P95 差 2.48e-16 m |

工况：每次 20 MPa、12 s，积分 60 步，dt=0.2 s，网格 96×96×64，等间距 0.03360404 m。体积采用占据体素计数，因此存在体素量化台阶；深度是沿固定射流轴线的一组平行射线首交点变化最大值。点云 P95 与网格最大深度不是同一指标，不能直接相等比较。

第六步重放的是第一次从实际 TCP 记录下来的 p/d 与相同压力、时长、开关状态；不以重新读取后的浮点位姿微小差异替代“相同输入”。

## 保留与修改范围

保留原煤壁 OBJ、生成工具、Panda 和水枪资产、原世界文件、ros2_control 控制链路、原有算法类及包结构。在这些基础上修正：

- `voxel_grid.py`：1e-7 m 接缝合并，拒绝无法封闭的网格；使用现有 Open3D BVH 的三角形符号距离；显式单线程与分块，移除 NxM cdist。
- `erosion_model.py`：保留原径向核及临时 R_c、b；逐步更新当前命中、靶距和自遮挡；最近表面点速度向窄带延拓。额外遮挡网格接口有单测覆盖。
- `level_set_evolver.py`：物理间距归一化的 Godunov 迎风离散、三维 CFL 上界，末步缩短以匹配请求时长。
- `geometry_utils.py`、`mesh_converter.py`：体积计数、平行射线深度；去重和去退化；OBJ 显式导出法向，避免 Harmonic/DART 载入碰撞网格崩溃。
- `spray_simulation.py`：集中维护初始/当前煤体；候选计算与提交分离，错误和超限不提交；重置可复现。
- `SprayAction.srv`、CMake/package.xml、`spray_controller_node.py`：补齐接口生成和依赖，使用生命周期节点，配置/激活、停用拒绝、重新激活保留当前几何、运行时 dt 校验、实际 TCP 坐标转换。
- `gazebo_coal_wall.py`：原位替换煤壁的 visual 与 collision 网格，独立文件名避免缓存；确认模型移除/创建后才提交算法状态。
- 相机安装朝向与点云 frame 修正：Gazebo 点云使用 +X 前向本体轴，声明为 camera_link；在相机本体与 world 之间使用消息时间戳对应 TF。仅改标签不会把点坐标旋转成光学轴。
- 单独增加 `coal_wall_acceptance.sdf` 与 `spray_acceptance.yaml`：使用同一原煤壁，转为可由机械臂近似正入射的竖直布置；原 `coal_wall_v02.sdf` 保留。TCP 使用现有实际帧 `tcp`。

未安装任何新软件，沿用现有 `.venv_stage4`。外部 `robotics-agent-skills/` 未修改。历史实验脚本仍保留，后续以本报告中的入口为准。

## 资源控制与测试退出

- 数值计算 OpenMP/BLAS/网格 BVH 单线程；完整验收进程组限制到两个 CPU 核心；构建串行处理包，Make/CMake 最多两个并行任务。
- 网格最多 100 万采样点，距离查询每块最多 32768 点；单次积分最多 600 步，墙钟计算预算 30 s。
- 验收实时监控进程总 RSS：超过 3 GiB 或系统可用 RAM 低于 1 GiB 时停止。本次总峰值约 2008.1 MiB，包含 Gazebo、ROS 和测试进程，RSS 相加会重复计入共享页。
- 只追踪并清理自己启动的后代进程，退出核查确认 `owned_processes_remaining=[]`。未全局 pkill。
- 日志中的负压力拒绝是故意执行的异常输入测试；部分 Gazebo 退出由 launch 超时升级到 SIGTERM，不是作业阶段崩溃。最后另行确认无活动遗留进程。

## 验证层级

1. 在全新的 `/tmp/hydromining_phase4_final_build` 完成两个包构建，串行包、两编译任务：2 packages finished [8.54s]。
2. 五项算法 pytest 通过：SDF/网格封闭与 OBJ 法向、迎风平面退缩与 CFL、累计/重置/时长、无命中/关闭/非法输入/遮挡、原煤壁零喷射几何误差。
3. 完整离线集成测试通过，重复输入距离场一致；5、10、15、20 MPa 的去除体积非递减且最高大于最低。离散体积可出现相邻压力结果相同。
4. 独立安装目录、从 `/tmp` 启动 ROS 节点并调用真实服务：有效喷射、非法启动 dt、非法运行时 dt、非单位方向、停用拒绝、重新激活、重置重放验证通过。详细见 `ros_interface_test.json`。
5. 上表六步真实无 GUI 集成通过，保留五组 PCD/NPZ/RGB PNG、网格、日志和图像。

## 后续使用

在工作区执行一条命令即可构建并完成无 GUI 验证：

```bash
./scripts/run_phase4_acceptance.sh
```

维护者最终 GUI 验收（本次没有执行此命令）：

```bash
./scripts/run_phase4_acceptance.sh --gui --keep-open --output /tmp/hydromining_gui_acceptance
```

该命令自动完成六步，最终保留 GUI 中的重放凹坑供观察，Ctrl+C 退出并清理本次进程。

算法回归：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv_stage4/bin/python -m pytest src/hydromining_sim/test/test_erosion_regression.py -q
```

离线集成：`.venv_stage4/bin/python test_scripts/test_phase4_integration.py`。真实 ROS 接口回归需先 source ROS/工作区，然后使用 `.venv_stage4/bin/python src/hydromining_robot/test/scripts/test_spray_interfaces.py`。

## 未验证与适用边界

- **GUI 尚未验证，等待维护者最终观察**；本次所有仿真测试均为 `-s --headless-rendering`。
- R_c、b 仍是临时经验函数，尚未做公式⑱⑲的实验联合标定；不能称为真实煤体性能预测。
- 固定近似正入射、两次短时喷射；裂纹、块体剥落、长期极限、一般斜入射与空间随机性不在已验证能力内。
- 外部遮挡网格输入的算法测试通过，但当前 ROS 适配层没有自动采集任意 Gazebo 实体几何。六步验收场景未设置额外射流遮挡物，不应外推为任意遮挡场景已通过。
- 阶段 3 历史固定 LiDAR 的 ICP 不等于移动多视角融合；本次新增的是固定末端相机的喷前/喷后 world 点云验证。
- 当前相机图像/CameraInfo 的通用 ROS 光学坐标约定尚未单独验收；此轮点云使用真实 camera_link 本体轴并验证了几何位置。
- 零喷射重建以网格尺度误差为验收条件，不声称逐顶点恢复原始 OBJ。

## 最终收尾审计（2026-10-26）

补强了全过程关节采样：不再把喷射结束时的单帧偏差称为全过程最大值。`joint_hold.npz` 保存固定期间收到的全部关节样本与参考位置；最终运行记录 1626 帧，最大观测偏差为 0 rad。

每次喷射结束时立即在终端输出本次与累计深度、体积，GUI 保持打开时也可查看；完成后 Ctrl+C 是正常退出。实际终端输出保存在 `acceptance_console.log`。

`test_scripts/audit_phase4_results.py` 独立重新读取五组 PCD/NPZ/RGB、关节记录和五份 OBJ，验证 world 坐标变换、保存精度、帧顺序、观测凹坑及累计变化、网格封闭性/正体积、重置与重放字节一致。审计结果为 `completion_audit.json`，六项自动化要求全部通过。

审计命令（不启动仿真）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv_stage4/bin/python test_scripts/audit_phase4_results.py results/phase4_acceptance/2026-10-26
```

按用户要求，本轮自动测试范围为无 GUI；GUI 的最终人工观察没有被声明为已通过，也没有提前开展阶段 5。标定及原先明确排除的模型扩展仍按上文跟踪。
