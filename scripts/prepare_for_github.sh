#!/bin/bash
set -e

echo "========================================="
echo "GitHub 上传前检查"
echo "========================================="

cd ~/SCI/hydromining_ws

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

ERRORS=0
WARNINGS=0

# 1. 检查构建产物
echo -e "${YELLOW}[1/8] 检查构建产物...${NC}"
if [ -d build ] || [ -d install ]; then
    echo -e "${RED}❌ 发现构建产物目录（build/ 或 install/）${NC}"
    echo "   建议执行：rm -rf build/ install/ log/"
    ERRORS=$((ERRORS+1))
else
    echo -e "${GREEN}✅ 无构建产物${NC}"
fi

# 2. 检查 Python 缓存
echo -e "${YELLOW}[2/8] 检查 Python 缓存...${NC}"
PYCACHE_COUNT=$(find . -type d -name "__pycache__" 2>/dev/null | wc -l)
if [ $PYCACHE_COUNT -gt 0 ]; then
    echo -e "${YELLOW}⚠️  发现 $PYCACHE_COUNT 个 __pycache__ 目录${NC}"
    echo "   建议执行：find . -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true"
    WARNINGS=$((WARNINGS+1))
else
    echo -e "${GREEN}✅ 无 Python 缓存${NC}"
fi

# 3. 检查敏感信息
echo -e "${YELLOW}[3/8] 检查敏感信息...${NC}"
SENSITIVE=$(grep -r "password\|secret\|api_key\|token" src/ config/ algorithms/ --include="*.py" --include="*.yaml" 2>/dev/null | grep -v "# password" | wc -l)
if [ $SENSITIVE -gt 0 ]; then
    echo -e "${RED}❌ 发现可能的敏感信息（$SENSITIVE 处）${NC}"
    grep -r "password\|secret\|api_key\|token" src/ config/ algorithms/ --include="*.py" --include="*.yaml" 2>/dev/null | grep -v "# password" | head -5
    echo "   请手动检查并删除"
    ERRORS=$((ERRORS+1))
else
    echo -e "${GREEN}✅ 无明显敏感信息${NC}"
fi

# 4. 检查绝对路径
echo -e "${YELLOW}[4/8] 检查硬编码路径...${NC}"
HARDCODED=$(grep -r "/home/star" src/ config/ algorithms/ test_scripts/ --include="*.py" --include="*.launch.py" 2>/dev/null | wc -l)
if [ $HARDCODED -gt 0 ]; then
    echo -e "${YELLOW}⚠️  发现 $HARDCODED 处硬编码路径${NC}"
    grep -r "/home/star" src/ config/ algorithms/ test_scripts/ --include="*.py" --include="*.launch.py" 2>/dev/null | head -3
    echo "   建议使用 ROS 包路径或相对路径"
    WARNINGS=$((WARNINGS+1))
else
    echo -e "${GREEN}✅ 无硬编码路径${NC}"
fi

# 5. 检查大文件
echo -e "${YELLOW}[5/8] 检查大文件（>10MB）...${NC}"
LARGE_FILES=$(find . -type f -size +10M 2>/dev/null | grep -v ".git" | grep -v ".venv_stage4" | wc -l)
if [ $LARGE_FILES -gt 0 ]; then
    echo -e "${YELLOW}⚠️  发现 $LARGE_FILES 个大文件${NC}"
    find . -type f -size +10M -exec ls -lh {} \; 2>/dev/null | grep -v ".git" | grep -v ".venv_stage4" | head -5
    echo "   建议：1) 添加到 .gitignore  2) 使用 Git LFS  3) 移到外部存储"
    WARNINGS=$((WARNINGS+1))
else
    echo -e "${GREEN}✅ 无超大文件${NC}"
fi

# 6. 检查必需文件
echo -e "${YELLOW}[6/8] 检查必需文件...${NC}"
REQUIRED_FILES=(
    "README.md"
    ".gitignore"
    "AGENTS.md"
    "scripts/setup_new_machine.sh"
    "docs/GitHub部署指南.md"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        echo -e "${RED}❌ 缺少文件：$file${NC}"
        ERRORS=$((ERRORS+1))
    fi
done

if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✅ 所有必需文件存在${NC}"
fi

# 7. 检查文档完整性
echo -e "${YELLOW}[7/8] 检查阶段 7 文档...${NC}"
STAGE7_DOCS=(
    "docs/阶段文档/阶段7-规划决策/00-进度交接.md"
    "docs/阶段文档/阶段7-规划决策/01-设计方案.md"
    "docs/阶段文档/阶段7-规划决策/02-执行指南.md"
    "docs/阶段文档/阶段7-规划决策/03-评估报告.md"
)

for doc in "${STAGE7_DOCS[@]}"; do
    if [ ! -f "$doc" ]; then
        echo -e "${RED}❌ 缺少文档：$doc${NC}"
        ERRORS=$((ERRORS+1))
    fi
done

if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✅ 阶段 7 文档完整${NC}"
fi

# 8. 统计仓库大小
echo -e "${YELLOW}[8/8] 统计仓库大小...${NC}"
TOTAL_SIZE=$(du -sh . 2>/dev/null | cut -f1)
echo "   总大小：$TOTAL_SIZE"

CODE_SIZE=$(du -sh src/ algorithms/ test_scripts/ 2>/dev/null | awk '{sum+=$1} END {print sum}')
echo "   代码大小：约 $(du -sh src/ algorithms/ test_scripts/ 2>/dev/null | awk '{s+=$1} END {print s}') MB"

RESULTS_SIZE=$(du -sh results/ 2>/dev/null | cut -f1)
echo "   结果大小：$RESULTS_SIZE"

if [ -d .venv_stage4 ]; then
    VENV_SIZE=$(du -sh .venv_stage4/ 2>/dev/null | cut -f1)
    echo -e "${YELLOW}   ⚠️  虚拟环境：$VENV_SIZE（应在 .gitignore 中）${NC}"
fi

# 总结
echo ""
echo "========================================="
echo "检查完成"
echo "========================================="
echo -e "错误：${RED}$ERRORS${NC}"
echo -e "警告：${YELLOW}$WARNINGS${NC}"

if [ $ERRORS -gt 0 ]; then
    echo ""
    echo -e "${RED}❌ 发现错误，请修复后再上传${NC}"
    exit 1
elif [ $WARNINGS -gt 0 ]; then
    echo ""
    echo -e "${YELLOW}⚠️  有警告，建议检查后再上传${NC}"
    echo ""
    read -p "是否继续？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo ""
    echo -e "${GREEN}✅ 检查通过，可以上传到 GitHub${NC}"
fi

echo ""
echo "下一步："
echo "1. 初始化 Git 仓库（如尚未初始化）："
echo "   git init"
echo ""
echo "2. 添加所有文件："
echo "   git add ."
echo ""
echo "3. 首次提交："
echo "   git commit -m 'Initial commit: Hydromining ROS 2 simulation system'"
echo ""
echo "4. 关联 GitHub 仓库："
echo "   git remote add origin https://github.com/tbxrelive/hydromining-ros2-sim.git"
echo ""
echo "5. 推送："
echo "   git branch -M main"
echo "   git push -u origin main"
echo ""
