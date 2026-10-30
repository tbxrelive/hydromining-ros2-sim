#!/bin/bash
# 文档重组执行脚本

cd /home/star/SCI/hydromining_ws/docs

echo "创建新目录结构..."
mkdir -p 阶段文档/阶段1-煤壁生成
mkdir -p 阶段文档/阶段2-机器人集成
mkdir -p 阶段文档/阶段3-破坏响应模型
mkdir -p 阶段文档/模板与工具/煤壁生成工具
mkdir -p 阶段文档/模板与工具/阶段2模板
mkdir -p 参考资料
mkdir -p 技术笔记

echo "移动阶段 1 文档..."
[ -f coal_wall_generation.md ] && mv coal_wall_generation.md 阶段文档/阶段1-煤壁生成/02-使用指南.md
[ -f stage1_summary.md ] && mv stage1_summary.md 阶段文档/阶段1-煤壁生成/03-完成总结.md
[ -f ../troubleshooting/STAGE1_FIX_SUMMARY.md ] && cp ../troubleshooting/STAGE1_FIX_SUMMARY.md 阶段文档/阶段1-煤壁生成/04-故障修复.md

echo "移动阶段 2 文档..."
[ -f stage2_design.md ] && mv stage2_design.md 阶段文档/阶段2-机器人集成/01-设计方案.md
[ -f stage2_execution_guide.md ] && mv stage2_execution_guide.md 阶段文档/阶段2-机器人集成/02-执行指南.md
[ -f stage2_review.md ] && mv stage2_review.md 阶段文档/阶段2-机器人集成/03-评估报告.md

echo "移动模板文件..."
[ -d stage2_templates ] && mv stage2_templates 阶段文档/模板与工具/阶段2模板/

echo "移动参考资料..."
[ -f Agent_Skills分析.md ] && mv 参考资料/Agent_Skills分析.md 参考资料/ 2>/dev/null || true

echo "清理临时文件..."
rm -f EXECUTE_STAGE2.md QUICKSTART_STAGE2.txt STAGE2_DEPLOYMENT.md STAGE2_READY.md
rm -f create_stage2_files.py setup_stage2.sh

echo "✓ 文档重组完成"
echo ""
echo "新结构预览："
tree -L 2 -d
