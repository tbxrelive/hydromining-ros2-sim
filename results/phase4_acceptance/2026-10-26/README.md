# 阶段4验收结果（2026-10-26）

当前结论、方法与限制见工作区 `docs/阶段文档/阶段4-喷射效果模拟/08-六步无GUI集成验证报告.md`。

- `comparison.png`：真实相机渲染和实测点云对比。
- `summary.json`：六步验收数值及进程清理结果。
- `01_before`、`02_after_first`、`03_after_second`、`04_reset`、`05_replay`：每组保存 world PCD、含时间戳/TF/组织结构的 NPZ、RGB PNG。
- `meshes/coal_wall_0001..0005.obj`：初始、首喷、累计、重置、重放的实际加载网格。
- `clean_build.log`、`unit_tests.log`、`offline_integration.*`、`ros_interface_test.json`：分层验证证据。
- `simulation.log`、`ros_logs/`：当前仿真日志。

参数未标定；GUI 的最终观察尚待维护者验收。不要把点云 P95 退缩与网格最大深度混为同一指标。

- `joint_hold.npz`：固定期间收到的全部关节状态与参考位置。
- `completion_audit.json`：独立从保存数据重算的六项要求审计。
- `acceptance_console.log`：逐次喷射即时统计与最终结果。
