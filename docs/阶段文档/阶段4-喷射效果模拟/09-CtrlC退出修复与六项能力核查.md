# Ctrl+C 退出修复与六项能力核查

日期：2026-10-26。

## 六项能力的实际状态

| 能力 | 已核实的内容 | 验证边界 |
|---|---|---|
| 单次喷射破坏 | 原煤壁上产生局部凹坑；首喷深度约5.52 cm、体积455.4 cm³；实际RGB-D点云出现对应变化 | 临时参数的仿真结果，不是实煤标定预测 |
| 累计破坏 | 第二次从当前剩余煤体继续演化，累计深度约9.30 cm、体积796.9 cm³ | 固定近似正入射、两次短时喷射 |
| 传感器观测变化 | 实际RGB-D喷前/喷后world点云、参考环带及累计观测检查通过 | 不是仅用OBJ或画面代替传感器数据；尚非移动多视角融合 |
| 重置与重复运行 | 初始/重置网格字节一致；首喷/重放网格字节一致，点云差接近浮点精度 | 重放相同记录动作、相同初始算法状态 |
| 不同参数产生不同效果 | 离线5/10/15/20 MPa压力扫描体积为68.3/136.5/136.5/341.3 cm³，非递减且高低压力有差异 | 10与15 MPa体积相同是体素计数量化；时长、靶距未做系统联合扫描，模型参数未标定 |
| 碰撞几何同步更新 | `GazeboCoalWall.replace()` 为 collision 与 visual 写入同一份新网格，并删除/生成整个煤壁模型 | 已有实现与模型加载证据；尚缺独立的碰撞接触/穿透实验，不得表述为物理碰撞行为已全面验收 |

## 用户报告与根因

用户GUI验收已到 `05_replay`，Ctrl+C后出现：

```text
RCLError: failed to shutdown: rcl_shutdown already called on the given context
```

ROS的信号处理可能已关闭context，而验收脚本finally中再次无条件调用 `rclpy.shutdown()`；异常使随后关闭launch与子进程、写summary的代码无法执行。此前正常结束测试没有覆盖这个真实SIGINT路径，不能据此声称Ctrl+C退出已验证。

核查用户 `/tmp/hydromining_gui_acceptance` 中的五组点云/图片/网格：文件存在，首喷与累计网格不同，重置与初始一致，重放与首喷一致，重置/重放点云P95差为0；summary未生成。证据保存在 `results/phase4_sigint_regression/user_gui_artifact_audit.json`。这些证据证明业务流程已运行，但不能替代用户对GUI画面效果的最终确认。

## 修复

- 使用 `rclpy.try_shutdown(context=node.context)`，使已关闭context的再次收尾成为幂等操作。
- 把ROS对象收尾与本次仿真进程收尾分开，记录清理错误，单项错误不阻止继续清理。
- 持续保留本次launch后代进程身份，避免父进程先退出后丢失清理目标；只处理自己启动的进程。
- 同时处理 `KeyboardInterrupt` 与 `ExternalShutdownException`；收尾阶段忽略后续重复SIGINT，避免再次打断清理。
- `--keep-open` 支持无GUI测试，以便自动覆盖与GUI保持窗口时相同的Ctrl+C逻辑。GUI验收命令不变。

## 实际验证

1. 两个项目包构建通过。
2. 用真实rclpy context复现重复shutdown的RCLError；对同一已关闭context重复try_shutdown不报错。
3. 执行 `test_scripts/test_phase4_sigint.py`：真实无GUI Gazebo/机器人/传感器六步流程 → 保持运行 → 自动发送SIGINT → 检查返回值与所有已记录子进程。
4. 结果：退出码0，无重复shutdown错误、无Traceback、`cleanup_errors=[]`、无遗留子进程；RSS总峰值1978.8 MiB，两个CPU核心。此轮未启动GUI。

证据：`results/phase4_sigint_regression/sigint_regression.json`、`console.log`、`summary.json`、`simulation.log`。

测试入口：

```bash
.venv_stage4/bin/python test_scripts/test_phase4_sigint.py
```

维护者GUI验收入口保持原命令：

```bash
./scripts/run_phase4_acceptance.sh --gui --keep-open --output /tmp/hydromining_gui_acceptance
```

待补：独立物理碰撞接触验证、参数联合扫描/真实标定，以及用户对GUI视觉效果与退出行为的最终确认。
