# 第 2 章 仿真系统

阶段 2 已完成并冻结为 **sim-v1b**（`configs/sim_v1.lock.json`，标签 `sim-v1b`）。
阶段 5 追加 E2.2（非均质强度场）与 E2.4（多保真度一致性）后升为 sim-v2。

版本关系：`sim-v1`（仅把压力指数改为线性）→ **`sim-v1b`**（用户补充 4 篇全文后，
加阈值压力、0.22x 射流半宽与剖面指数 2）。**场景库内容在两版之间逐字节一致。**

## 文件

| 文件 | 实验 | 说明 |
|---|---|---|
| `e2_1a_erosion_scan.py` | E2.1a | 压力/靶距/时长全因子扫描（`--engine l0\|levelset`，`--kernel-json` 复现历史核） |
| `e2_1a_plot_kernels.py` | E2.1a | 三轮核（v0/v1/v1b）响应曲线对比图 |
| `e2_1b_sensitivity.py` | E2.1b | 层 1 Sobol（单次喷射）、层 2 Morris（任务级，含压力/靶距扰动） |
| `e2_1c_baseline.py` | E2.1c | 重建规则基线、单次喷射可行性、与 E0.3 比对 |
| `e2_3a_interference_pilot.py` | E2.3a | 干扰参数预实验：受扰比例 V-D-视角-噪声热力图 |
| `e2_3b_interference_levels.py` | E2.3b | 四等级定义与重复统计（相邻可分判据） |
| `e2_3c_particle_validation.py` | E2.3c | Gazebo 粒子档位验证与等效能见度映射 |
| `e2_3d_l1l2_interference.py` | E2.3d | 干扰等级接入 L1/L2 的渲染路径 |
| `e2_3e_particle_feasibility.py` | E2.3e | 粒子层可行性上限：发射器调到最强能到多少 |
| `e2_5_scenario_library.py` | E2.5 | 场景库生成、跨进程复现性、60° 倾角核对、全库规则基线 |

## 结果与结论

见 [`docs/阶段文档/阶段2-仿真基准/00-阶段总结.md`](../../docs/阶段文档/阶段2-仿真基准/00-阶段总结.md)
与 `results/thesis/E2.*/summary.json`。参数出处见 `docs/文献/E2.1_标定依据.md`。
