#!/bin/bash
set -e

echo "========================================="
echo "水力采煤仿真系统 - 新机器部署"
echo "========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 检查系统版本
echo -e "${YELLOW}检查系统版本...${NC}"
if ! grep -q "24.04" /etc/os-release; then
    echo -e "${RED}⚠️  警告：建议使用 Ubuntu 24.04 LTS${NC}"
    echo "当前系统："
    lsb_release -a
    read -p "是否继续？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo -e "${GREEN}✅ Ubuntu 24.04 LTS${NC}"
fi

# 2. 检查并安装 ROS 2 Jazzy
echo -e "${YELLOW}检查 ROS 2 Jazzy...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo "ROS 2 未安装，开始安装..."

    sudo apt update
    sudo apt install -y software-properties-common curl

    # 添加 ROS 2 apt 仓库
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz

    echo -e "${GREEN}✅ ROS 2 Jazzy 安装完成${NC}"
else
    ROS_VERSION=$(ros2 --version 2>&1 | head -n1)
    echo -e "${GREEN}✅ ROS 2 已安装: $ROS_VERSION${NC}"

    if [[ ! "$ROS_VERSION" =~ "jazzy" ]]; then
        echo -e "${RED}⚠️  警告：检测到非 Jazzy 版本，可能不兼容${NC}"
    fi
fi

# 3. 安装系统依赖
echo -e "${YELLOW}安装系统依赖...${NC}"
sudo apt install -y \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libopencv-dev \
    git \
    wget

echo -e "${GREEN}✅ 系统依赖安装完成${NC}"

# 4. 克隆或更新仓库
echo -e "${YELLOW}准备项目目录...${NC}"
if [ ! -d ~/SCI ]; then
    mkdir -p ~/SCI
fi

cd ~/SCI

if [ ! -d hydromining_ws ]; then
    echo "克隆仓库..."
    read -p "请输入 GitHub 仓库 URL（留空使用默认）: " REPO_URL
    REPO_URL=${REPO_URL:-"https://github.com/tbxrelive/hydromining-ros2-sim.git"}

    git clone "$REPO_URL" hydromining_ws
    echo -e "${GREEN}✅ 仓库克隆完成${NC}"
else
    echo "仓库已存在，拉取最新代码..."
    cd hydromining_ws
    git pull
    echo -e "${GREEN}✅ 代码更新完成${NC}"
fi

cd ~/SCI/hydromining_ws

# 5. 初始化 rosdep
echo -e "${YELLOW}初始化 rosdep...${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
echo -e "${GREEN}✅ rosdep 初始化完成${NC}"

# 6. 安装 ROS 依赖
echo -e "${YELLOW}安装 ROS 包依赖...${NC}"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys="gz_sim"
echo -e "${GREEN}✅ ROS 依赖安装完成${NC}"

# 7. 创建 Python 虚拟环境
echo -e "${YELLOW}创建 Python 虚拟环境...${NC}"
if [ ! -d .venv_stage4 ]; then
    python3 -m venv .venv_stage4
fi
source .venv_stage4/bin/activate

# 8. 安装 Python 依赖
echo -e "${YELLOW}安装 Python 依赖（可能需要 5-10 分钟）...${NC}"
pip install --upgrade pip

# 使用清华镜像加速（可选）
read -p "使用清华镜像加速下载？(Y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    PIP_INDEX="-i https://pypi.tuna.tsinghua.edu.cn/simple"
else
    PIP_INDEX="--index-url https://download.pytorch.org/whl/cpu"
fi

pip install \
    torch==2.5.1+cpu \
    gymnasium==1.0.0 \
    stable-baselines3==2.4.1 \
    numpy scipy matplotlib \
    open3d pytest \
    $PIP_INDEX

echo -e "${GREEN}✅ Python 依赖安装完成${NC}"

# 9. 构建 ROS 2 工作空间
echo -e "${YELLOW}构建 ROS 2 工作空间...${NC}"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hydromining_robot hydromining_sim

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ 工作空间构建成功${NC}"
else
    echo -e "${RED}❌ 构建失败，请检查错误信息${NC}"
    exit 1
fi

# 10. 验证安装
echo ""
echo "========================================="
echo "运行验证测试..."
echo "========================================="

source .venv_stage4/bin/activate
source install/setup.bash

# 单元测试
echo -e "${YELLOW}运行单元测试...${NC}"
if pytest algorithms/planning/test_*.py -v --tb=short > /tmp/pytest_output.log 2>&1; then
    echo -e "${GREEN}✅ 单元测试通过${NC}"
    grep -E "passed|failed" /tmp/pytest_output.log | tail -1
else
    echo -e "${YELLOW}⚠️  单元测试有警告，详见 /tmp/pytest_output.log${NC}"
fi

# 快速功能测试
echo -e "${YELLOW}运行快速功能测试（替身环境 L1）...${NC}"
if timeout 120 python3 test_scripts/phase7_surrogate_loop.py tri_r5 --level L1 > /tmp/phase7_test.log 2>&1; then
    echo -e "${GREEN}✅ 功能测试通过${NC}"
else
    echo -e "${YELLOW}⚠️  功能测试超时或失败，详见 /tmp/phase7_test.log${NC}"
fi

# 11. 生成环境配置脚本
echo -e "${YELLOW}生成快捷启动脚本...${NC}"
cat > ~/SCI/hydromining_ws/setup_env.sh << 'EOF'
#!/bin/bash
# 快捷启动脚本
source /opt/ros/jazzy/setup.bash
source ~/SCI/hydromining_ws/.venv_stage4/bin/activate
source ~/SCI/hydromining_ws/install/setup.bash
echo "✅ 水力采煤仿真环境已加载"
echo "快速测试命令："
echo "  python3 test_scripts/phase7_real_loop.py tri_r5"
EOF
chmod +x ~/SCI/hydromining_ws/setup_env.sh

echo -e "${GREEN}✅ 快捷脚本已生成: ~/SCI/hydromining_ws/setup_env.sh${NC}"

# 12. 完成总结
echo ""
echo "========================================="
echo -e "${GREEN}🎉 部署成功！${NC}"
echo "========================================="
echo ""
echo "下一步："
echo ""
echo "1. 激活环境（每次新终端都需要）："
echo "   source ~/SCI/hydromining_ws/setup_env.sh"
echo ""
echo "2. 运行真实 Gazebo 测试："
echo "   cd ~/SCI/hydromining_ws"
echo "   python3 test_scripts/phase7_real_loop.py tri_r5"
echo ""
echo "3. 查看完整文档："
echo "   docs/阶段文档/阶段7-规划决策/02-执行指南.md"
echo ""
echo "4. 批量测试所有工况："
echo "   ./scripts/run_phase7_batch.sh"
echo ""
echo "提示：首次运行 Gazebo 可能需要下载模型（~1-5 分钟）"
echo ""
