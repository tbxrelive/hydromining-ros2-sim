# 阶段五 GUI 最终验收报告

**日期**：2026-10-26  
**结论**：阶段五功能验收通过（无 GUI + GUI 自动检查 + 用户目视确认 + Ctrl+C 清理）。参数仍未标定，物理适用边界不变。

## 被测版本

18 个实现/测试文件的 SHA-256 与 `transaction_fix/implementation_sha256.json` 完全一致，即 04 报告中事务修复后的冻结版本，本次未修改任何源码。

## 验证结果

| 层级 | 命令/方式 | 结果 |
|---|---|---|
| 构建 | `colcon build --symlink-install --packages-select hydromining_sim hydromining_robot` | 2 包通过 |
| 回归 | `pytest` 算法/资产/事务/保护/接口 | 24 passed |
| 无 GUI 全流程 | `./scripts/run_phase5_acceptance.sh --output /tmp/hydromining_phase5_headless_v3` | passed，峰值 1946.6 MiB，无遗留进程 |
| GUI 全流程 | `./scripts/run_phase5_acceptance.sh --gui --keep-open --output /tmp/hydromining_phase5_gui_v3` | 打印 `All acceptance checks passed`；重放 UPDATING 无 create 超时；峰值 2801.5 MiB（未触发 3072 MiB 保护） |
| GUI 目视 | 用户观察 + `/gui/screenshot` | 见下文 |
| GUI 退出 | 对验收进程发送 SIGINT | 8 s 内退出；`summary.json` passed=true、`owned_processes_remaining=[]`、`cleanup_errors=[]`，无 failure.json |
| 阶段四兼容 | `./scripts/run_phase4_acceptance.sh --output /tmp/hydromining_phase4_regression_v3` | passed，重放网格一致，峰值 2190.0 MiB |

GUI 运行的关键数值：首喷点云 396 点退缩、P95 2.87 cm；累计 532 点、P95 6.28 cm；首喷最大深度 5.52 cm、体积 455.4 cm³；重置 P95 0 m；关节漂移 0 rad；off 预设零干扰，high 点云干扰 1.48% > low 1.38%，可见比例约 98.5%。

## GUI 目视问题与诊断

用户第一次反馈"看不到凹坑"。按 systematic-debugging 分层核查：

1. **数据层**：最终网格 `coal_wall_0016.obj` 相对重置网格有 24 个顶点偏移，最大 5.96 cm；凹坑范围约 8 × 11 cm，而煤壁约 3 × 3 m。每次替换都用唯一文件名，排除 GUI 网格缓存。
2. **渲染层**：通过 `/gui/screenshot` 获取 GUI 实际渲染。默认视角下凹坑位于喷嘴上方，只有几个像素的暗点。
3. **结论**：GUI 已正确显示更新后的网格。看不清的原因是凹坑相对煤壁太小、体素间距 3.36 cm 较粗，加上材质是均匀灰色无纹理，**不是功能缺陷**。用 `/gui/move_to/pose` 把相机移到凹坑附近后，用户确认能看到凹坑。

截图证据：`gui/gui_screenshot_default_view.png`、`gui/gui_screenshot_crater_closeup.png`。

## 风险与遗留

- GUI 内存峰值 2801.5 MiB，距 3072 MiB 保护线只剩约 270 MiB。桌面同时运行浏览器等程序时，系统可用内存仍可能触发 1 GiB 下限保护。
- 可视性改进（可选，未实施）：验收脚本在 keep-open 阶段自动把 GUI 相机移近凹坑，或在 GUI 中按深度着色。
- 未变：压力—雾化/粉尘关系、射流速度场、冲击力未标定；软件模糊/额外噪声模块未实现；喷射期间不支持边运动边切割。

## 证据

`results/phase5_acceptance/2026-10-26/gui_acceptance_v3/`：`gui/`、`headless/`、`phase4_regression/` 及 `provenance.json`（源码哈希核对）。
