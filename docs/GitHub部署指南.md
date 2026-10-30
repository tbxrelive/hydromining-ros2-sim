# GitHub 部署指南

**目标**: 将水力采煤仿真系统上传到 GitHub，支持其他 Ubuntu 机器快速部署

**日期**: 2026-10-29  
**维护者**: Star (@tbxrelive)

---

## 📋 上传前检查清单

### 1. 清理构建产物和临时文件

```bash
cd ~/SCI/hydromining_ws

# 删除构建产物（会在新机器上重新构建）
rm -rf build/ install/ log/

# 删除 Python 缓存
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find . -type f -name "*.pyc" -delete

# 删除大文件测试结果（保留必要的验收报告）
# 保留最新验收结果，删除历史调试数据
du -sh results/phase*/ | sort -h
# 建议：保留 phase4/5/6/7 的 acceptance/ 和最新 real_*.json，删除其他
```

### 2. 检查敏感信息

```bash
# 检查是否有敏感配置（API 密钥、密码等）
grep -r "password\|secret\|api_key" src/ config/ --include="*.py" --include="*.yaml"

# 检查是否有绝对路径（应该使用相对路径或 ROS 包路径）
grep -r "/home/star" src/ config/ --include="*.py" --include="*.launch.py"
```

**如果发现绝对路径，修改为**：
```python
# ❌ 错误：硬编码路径
config_path = "/home/star/SCI/hydromining_ws/config/perception/acceptance.yaml"

# ✅ 正确：使用 ROS 包路径
from ament_index_python.packages import get_package_share_directory
pkg_dir = get_package_share_directory('hydromining_robot')
config_path = os.path.join(pkg_dir, 'config', 'perception', 'acceptance.yaml')
```

### 3. 验证 .gitignore

创建或更新 `.gitignore`：

```bash
cat > .gitignore << 'EOF'
# ROS 2 构建产物
build/
install/
log/

# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/
.venv_stage4/
venv/

# 测试结果（保留必要的验收报告）
results/**/debug_*
results/**/train_*
results/**/*.log
!results/phase*/acceptance/
!results/phase*/*_acceptance.json/

# Gazebo 临时文件
.gz/
.gazebo/

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# 大文件模型（使用 Git LFS 或外部存储）
*.zip
*.tar.gz
*.pth
*.onnx

# 私有配置
.env
*.local.yaml
EOF
```

### 4. 创建 Git LFS 配置（如有大文件）

```bash
# 安装 Git LFS
sudo apt-get install git-lfs
git lfs install

# 跟踪大文件（如训练好的 DDPG 模型）
git lfs track "*.pth"
git lfs track "*.zip"
git lfs track "*.dae"
git lfs track "*.obj"

# 添加 .gitattributes
git add .gitattributes
```

---

## 🚀 上传到 GitHub

### 方式一：全新仓库

```bash
cd ~/SCI/hydromining_ws

# 1. 初始化 Git 仓库
git init

# 2. 添加所有文件
git add .

# 3. 第一次提交
git commit -m "Initial commit: Hydromining ROS 2 simulation system

- 阶段 1-7 完整实现
- 规则决策基线验收通过
- 支持 ROS 2 Jazzy + Gazebo Harmonic
- 详见 docs/ 目录"

# 4. 创建 GitHub 仓库（在 GitHub 网站上）
# 仓库名建议：hydromining-ros2-sim

# 5. 关联远程仓库
git remote add origin https://github.com/tbxrelive/hydromining-ros2-sim.git

# 6. 推送
git branch -M main
git push -u origin main
```

### 方式二：已有仓库（更新）

```bash
cd ~/SCI/hydromining_ws

# 拉取最新代码
git pull origin main

# 添加新文件
git add docs/阶段文档/阶段7-规划决策/

# 提交
git commit -m "阶段 7 完成：规划与决策算法

- 规则决策基线 6/6 工况通过
- DDPG 训练基础设施完成
- 完整文档（设计/执行/评估）"

# 推送
git push origin main
```

---

## 📦 创建发布版本（Release）

### 在 GitHub 网站上操作

1. 进入仓库 → **Releases** → **Create a new release**

2. 填写信息：
   - **Tag**: `v0.7.0`
   - **Title**: `阶段 7：规划与决策算法（验收版）`
   - **Description**:
     ```markdown
     ## 主要功能
     - ✅ NBV 主动观测规划
     - ✅ 规则决策基线（6/6 工况通过）
     - ✅ DDPG 强化学习框架
     - ✅ 完整任务闭环（观测→决策→执行→收尾）
     
     ## 依赖
     - ROS 2 Jazzy
     - Gazebo Harmonic
     - Python 3.12+
     - PyTorch 2.5.1+cpu
     
     ## 快速开始
     参见 [docs/GitHub部署指南.md](docs/GitHub部署指南.md)
     ```

3. 上传附件（可选）：
   - 阶段 7 验收报告 PDF
   - 演示视频（如已录制）
   - 训练好的 DDPG 模型（如需共享）

4. **Publish release**

---

## 🖥️ 新机器部署指南

### 环境要求

| 组件 | 版本 | 安装方法 |
|-----|------|---------|
| **操作系统** | Ubuntu 24.04 LTS | - |
| **ROS 2** | Jazzy | [官方安装指南](https://docs.ros.org/en/jazzy/Installation.html) |
| **Gazebo** | Harmonic | `sudo apt install ros-jazzy-ros-gz` |
| **Python** | 3.12+ | 系统自带 |
| **Git** | 2.x | `sudo apt install git` |

### 快速部署脚本

创建 `scripts/setup_new_machine.sh`：

```bash
#!/bin/bash
set -e

echo "========================================="
echo "水力采煤仿真系统 - 新机器部署"
echo "========================================="

# 1. 检查系统版本
if ! grep -q "24.04" /etc/os-release; then
    echo "⚠️  警告：建议使用 Ubuntu 24.04 LTS"
fi

# 2. 安装 ROS 2 Jazzy（如未安装）
if ! command -v ros2 &> /dev/null; then
    echo "安装 ROS 2 Jazzy..."
    sudo apt update
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz
else
    echo "✅ ROS 2 已安装: $(ros2 --version)"
fi

# 3. 安装依赖包
echo "安装依赖包..."
sudo apt install -y \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libopencv-dev

# 4. 克隆仓库
if [ ! -d ~/SCI ]; then
    mkdir -p ~/SCI
fi

cd ~/SCI
if [ ! -d hydromining_ws ]; then
    echo "克隆仓库..."
    git clone https://github.com/tbxrelive/hydromining-ros2-sim.git hydromining_ws
else
    echo "✅ 仓库已存在，拉取最新代码..."
    cd hydromining_ws
    git pull
fi

cd ~/SCI/hydromining_ws

# 5. 初始化 rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 6. 安装 ROS 依赖
echo "安装 ROS 包依赖..."
rosdep install --from-paths src --ignore-src -r -y

# 7. 创建 Python 虚拟环境
echo "创建 Python 虚拟环境..."
python3 -m venv .venv_stage4
source .venv_stage4/bin/activate

# 8. 安装 Python 依赖
pip install --upgrade pip
pip install \
    torch==2.5.1+cpu \
    gymnasium==1.0.0 \
    stable-baselines3==2.4.1 \
    numpy scipy matplotlib \
    open3d pytest \
    --index-url https://download.pytorch.org/whl/cpu

# 9. 构建 ROS 2 工作空间
echo "构建 ROS 2 工作空间..."
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select hydromining_robot hydromining_sim

# 10. 验证安装
echo ""
echo "========================================="
echo "安装完成！运行验证测试..."
echo "========================================="

source install/setup.bash

# 单元测试
pytest algorithms/planning/test_*.py -v --tb=short

# 快速功能测试（替身环境，无需 GPU）
python3 test_scripts/phase7_surrogate_loop.py tri_r5 --level L1

echo ""
echo "✅ 部署成功！"
echo ""
echo "下一步："
echo "1. 真实 Gazebo 测试："
echo "   cd ~/SCI/hydromining_ws"
echo "   source .venv_stage4/bin/activate"
echo "   source install/setup.bash"
echo "   python3 test_scripts/phase7_real_loop.py tri_r5"
echo ""
echo "2. 查看文档："
echo "   docs/阶段文档/阶段7-规划决策/02-执行指南.md"
```

### 使用方法

在新机器上运行：

```bash
# 1. 下载部署脚本
wget https://raw.githubusercontent.com/tbxrelive/hydromining-ros2-sim/main/scripts/setup_new_machine.sh

# 2. 添加执行权限
chmod +x setup_new_machine.sh

# 3. 运行（约 15-30 分钟）
./setup_new_machine.sh
```

---

## 🔧 手动部署步骤（如自动脚本失败）

### 步骤 1：克隆仓库

```bash
mkdir -p ~/SCI
cd ~/SCI
git clone https://github.com/tbxrelive/hydromining-ros2-sim.git hydromining_ws
cd hydromining_ws
```

### 步骤 2：安装 ROS 2 Jazzy

参考官方指南：https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```bash
# 简化版（已有 universe 仓库）
sudo apt update && sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz
```

### 步骤 3：安装依赖

```bash
# 系统依赖
sudo apt install -y python3-pip python3-venv python3-colcon-common-extensions

# ROS 依赖
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 步骤 4：创建虚拟环境

```bash
cd ~/SCI/hydromining_ws
python3 -m venv .venv_stage4
source .venv_stage4/bin/activate

pip install --upgrade pip
pip install torch==2.5.1+cpu gymnasium==1.0.0 stable-baselines3==2.4.1 \
    numpy scipy matplotlib open3d pytest \
    --index-url https://download.pytorch.org/whl/cpu
```

### 步骤 5：构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 步骤 6：验证

```bash
source .venv_stage4/bin/activate
source install/setup.bash

# 单元测试
pytest algorithms/planning/test_*.py -v

# 功能测试
python3 test_scripts/phase7_surrogate_loop.py tri_r5 --level L1
```

---

## 📝 常见问题

### Q1: `colcon build` 失败，提示找不到包

**解决**：
```bash
# 重新安装 ROS 依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys="gz_sim"
```

### Q2: PyTorch 下载慢或校验失败

**解决**：
```bash
# 使用清华镜像
pip install torch==2.5.1+cpu \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    --trusted-host pypi.tuna.tsinghua.edu.cn
```

### Q3: Gazebo 启动报错 `libEGL warning`

**说明**：这只是警告，不影响功能（无 GUI 模式不需要 OpenGL）。

**如需 GUI**：
```bash
# 检查显卡驱动
nvidia-smi  # NVIDIA GPU
glxinfo | grep "OpenGL"  # 集成显卡

# 安装 Mesa 驱动（集成显卡）
sudo apt install -y mesa-utils libgl1-mesa-glx
```

### Q4: 新机器性能不足

**最低配置**：
- CPU: 4 核（推荐 8 核）
- 内存: 16 GB（推荐 32 GB）
- 存储: 50 GB 可用空间

**降级策略**：
- 只运行替身环境（L1 模式，无 Gazebo）
- 关闭 GUI：所有测试脚本不加 `--gui`
- 减少并行进程：一次只测试一个工况

---

## 📚 相关文档

- **项目总览**: `README.md`
- **开发规则**: `AGENTS.md`
- **快速开始**: `QUICKSTART.md`（建议创建）
- **阶段 7 执行指南**: `docs/阶段文档/阶段7-规划决策/02-执行指南.md`

---

## 🔗 GitHub 仓库示例结构

```
hydromining-ros2-sim/
├── README.md                    # 项目介绍
├── AGENTS.md                    # 开发规则
├── LICENSE                      # 开源协议（建议 Apache-2.0）
├── .gitignore
├── .gitattributes               # Git LFS 配置
├── src/                         # ROS 2 包源码
│   ├── hydromining_robot/
│   └── hydromining_sim/
├── algorithms/                  # 核心算法
├── test_scripts/                # 测试脚本
├── config/                      # 配置文件
├── launch/                      # Launch 文件
├── docs/                        # 文档
│   ├── GitHub部署指南.md
│   ├── 阶段文档/
│   └── 鱼香ROS代码规范总结.md
├── scripts/                     # 工具脚本
│   ├── setup_new_machine.sh
│   └── run_phase7_batch.sh
└── results/                     # 验收结果（精简版）
    └── phase7/acceptance/
```

---

**最后更新**: 2026-10-29  
**维护者**: Star (@tbxrelive)
