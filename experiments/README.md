# experiments —— 论文实验

目录与命名规则见 [`docs/论文总体计划.md`](../docs/论文总体计划.md) 第四节，进度见 [`docs/进度总表.md`](../docs/进度总表.md)。

- 每个小实验一个文件，文件名以实验号开头：`e3_0_sigma_calibration.py`。
- 共用逻辑放 `common/`，不在各实验里重复实现。
- 配置放 `configs/experiments/<实验号>.yaml`。
- 结果写 `results/thesis/<实验号>/<run_id>/`，含 manifest.json、episodes.jsonl、summary.json、figures/。

新增实验时只在本目录按章节放入文件，不调整顶层结构。
