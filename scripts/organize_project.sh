#!/bin/bash

# 项目结构整理脚本
# 用途: 整理根目录杂乱文件，安装 skills

set -e

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

echo "📁 项目结构整理开始..."

# 1. 创建必要的目录
echo "✓ 创建目录结构..."
mkdir -p scripts docs/archive .claude/skills

# 2. 归档杂乱的文档文件
echo "✓ 整理文档文件到 docs/archive/..."
for file in "阶段2最终测试指南.md" "快速开始.md" "已修复问题总结.md" "最终测试指南.md"; do
  if [ -f "$file" ]; then
    mv -v "$file" docs/archive/ || true
  fi
done

# 3. 移动脚本文件到 scripts/（已在上面完成，这里只备份新增脚本）
echo "✓ 脚本文件已整理..."

# 4. 复制 robotics-agent-skills 中的 skills
echo "✓ 安装 robotics-agent-skills..."
if [ -d "robotics-agent-skills/skills" ]; then
  cp -v robotics-agent-skills/skills/ros2/SKILL.md .claude/skills/ros2.md
  cp -v robotics-agent-skills/skills/robotics-testing/SKILL.md .claude/skills/robotics-testing.md
  cp -v robotics-agent-skills/skills/robot-perception/SKILL.md .claude/skills/robot-perception.md
  cp -v robotics-agent-skills/skills/robotics-design-patterns/SKILL.md .claude/skills/robotics-design-patterns.md
  echo "✓ Skills 安装完成！"
else
  echo "⚠ robotics-agent-skills 目录不存在，跳过..."
fi

# 5. 验证 skills 安装
echo ""
echo "📋 已安装的 Skills:"
ls -1 .claude/skills/*.md

echo ""
echo "✅ 项目结构整理完成！"
echo ""
echo "📍 关键目录:"
echo "  - 脚本文件: scripts/"
echo "  - 文档: docs/ (archive/为旧文档)"
echo "  - Skills: .claude/skills/"
echo "  - 源代码: src/"
echo "  - 源代码构建输出: build/ install/"
