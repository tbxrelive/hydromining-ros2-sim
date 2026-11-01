# 第 2 章 仿真系统

阶段 2 已完成并冻结为 **sim-v1**（`configs/sim_v1.lock.json`，标签 `sim-v1`）。
阶段 5 追加 E2.2（非均质强度场）与 E2.4（多保真度一致性）后升为 sim-v2。

## 文件

| 文件 | 实验 | 说明 |
|---|---|---|
| `e2_1a_erosion_scan.py` | E2.1a | 压力/靶距/时长全因子扫描（`--engine l0\|levelset`），与文献趋势对照 |
| `e2_1b_sensitivity.py` | E2.1b | 层 1 Sobol（单次喷射）、层 2 Morris（任务级） |
| `e2_1c_baseline.py` | E2.1c | 在新核上重建规则基线并与 E0.3 逐回合比对、单次喷射可行性 |
| `e2_3a_interference_pilot.py` | E2.3a | 干扰参数预实验：受扰比例 V-D-视角-噪声热力图 |
| `e2_3b_interference_levels.py` | E2.3b | 四等级定义与重复统计（相邻可分判据） |
| `e2_3c_particle_validation.py` | E2.3c | Gazebo 粒子档位验证与等效能见度映射 |
| `e2_3d_l1l2_interference.py` | E2.3d | 干扰等级接入 L1/L2 的渲染路径 |
| `e2_5_scenario_library.py` | E2.5 | 场景库生成、跨进程复现性、60° 倾角核对、全库规则基线 |

## 结果与结论

见 [`docs/阶段文档/阶段2-仿真基准/00-阶段总结.md`](../../docs/阶段文档/阶段2-仿真基准/00-阶段总结.md)
与 `results/thesis/E2.*/summary.json`。参数出处见 `docs/文献/E2.1_标定依据.md`。
