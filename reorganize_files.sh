#!/bin/bash
# 整理项目文件结构
# 目标：将测试脚本、临时文件、文档分类存放

set -e

WS_DIR=~/SCI/hydromining_ws
cd $WS_DIR

echo "==========================================
  整理项目文件结构
=========================================="

# 1. 创建标准目录
mkdir -p test_scripts       # 测试脚本
mkdir -p temp_backup        # 临时备份（稍后可删除）

echo "
1. 移动测试脚本到 test_scripts/"
mv -v auto_test.sh test_scripts/ 2>/dev/null || true
mv -v run_observation_test.sh test_scripts/ 2>/dev/null || true
mv -v test_sensors.sh test_scripts/ 2>/dev/null || true
mv -v verify_with_venv.sh test_scripts/ 2>/dev/null || true

echo "
2. 移动临时/历史脚本到 temp_backup/"
mv -v scripts/测试运动.sh temp_backup/ 2>/dev/null || true
mv -v scripts/快速修复.sh temp_backup/ 2>/dev/null || true
mv -v scripts/启动仿真.sh temp_backup/ 2>/dev/null || true
mv -v scripts/完整构建并测试.sh temp_backup/ 2>/dev/null || true
mv -v scripts/一键解决所有问题.sh temp_backup/ 2>/dev/null || true
mv -v scripts/终极修复.sh temp_backup/ 2>/dev/null || true
mv -v troubleshooting/测试问题修复.md temp_backup/ 2>/dev/null || true
mv -v troubleshooting/阶段2测试完整指南.md temp_backup/ 2>/dev/null || true
mv -v troubleshooting/完全重建.sh temp_backup/ 2>/dev/null || true
mv -v troubleshooting/诊断并修复环境.sh temp_backup/ 2>/dev/null || true
mv -v troubleshooting/最简单测试方案.md temp_backup/ 2>/dev/null || true
mv -v troubleshooting/regenerate.sh temp_backup/ 2>/dev/null || true

echo "
3. 保留在项目根目录的文件："
echo "  ✓ AGENTS.md          — 项目规则"
echo "  ✓ QUICKSTART.md      — 快速开始指南"
echo "  ✓ setup_env.sh       — 环境配置脚本"
echo "  ✓ docs/              — 文档目录"
echo "  ✓ src/               — 源代码"
echo "  ✓ test_scripts/      — 测试脚本（新）"
echo "  ✓ troubleshooting/   — 故障排查文档"

echo "
4. 可以删除的目录/文件（如果确认不需要）："
echo "  • temp_backup/       — 历史临时脚本"
echo "  • scripts/           — 旧脚本目录（已空）"
echo "  • QUICK_TEST_GUIDE.md — 重复的测试指南"
echo "  • venv/              — 虚拟环境残留"
echo ""
echo "删除命令："
echo "  rm -rf temp_backup scripts QUICK_TEST_GUIDE.md venv"

echo "
=========================================="
echo "  整理完成！"
echo "=========================================="
echo ""
echo "当前测试脚本位置："
ls -1 test_scripts/
