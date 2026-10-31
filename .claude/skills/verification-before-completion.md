# Verification Before Completion (完成前验证)

## 目的
在声称任务完成之前，通过系统化的验证流程确保功能正常工作。

## 何时使用
- 完成任何代码修改后
- 准备提交或推送代码前
- 声称"修复成功"前

## 核心原则
> **"It works on my machine" 不是完成标准**

完成 = 代码 + 测试 + 验证 + 文档

## 验证层级

### Level 1: 语法验证（2分钟）
```bash
# Python
python3 -m py_compile file.py

# xacro/URDF
xacro file.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf
```

### Level 2: 构建验证（5分钟）
```bash
# 清理重建
rm -rf build/pkg install/pkg
colcon build --packages-select pkg

# 检查错误和警告
grep -i error build.log
grep -i warning build.log
```

### Level 3: 单元验证（10分钟）
```bash
# 测试单个组件
ros2 run pkg node --ros-args -p test_mode:=true

# 检查话题
ros2 topic list
ros2 topic echo /topic --once
```

### Level 4: 集成验证（15分钟）
```bash
# 启动完整系统
ros2 launch pkg launch_file

# 检查节点
ros2 node list
ros2 topic hz /topic
```

### Level 5: 端到端验证
- 执行完整用户场景
- 观察实际效果
- 确认性能

## 完整检查清单

### 代码质量
- [ ] 代码遵循项目规范
- [ ] 有必要的注释
- [ ] 无 TODO/FIXME 遗留
- [ ] 无调试代码

### 构建与安装
- [ ] 清理构建成功
- [ ] 依赖已声明
- [ ] 文件正确安装

### 功能验证
- [ ] 单元测试通过
- [ ] 集成测试通过
- [ ] 端到端场景通过

### 文档
- [ ] README 已更新
- [ ] 使用说明准确

## 常见陷阱

❌ **只在 IDE 中测试**
- IDE 可能自动添加路径和环境变量

✅ **在干净终端测试**
```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

❌ **假设构建输出是最新的**
- 构建系统可能没检测到变化

✅ **总是清理重建**
```bash
rm -rf build/pkg install/pkg
colcon build --packages-select pkg
```

❌ **只测试"快乐路径"**
- 只测试参数正确、文件存在的情况

✅ **测试错误情况**
- 缺少参数、文件不存在、无效输入

❌ **依赖之前的状态**
- 系统"工作"因为之前的启动留下了状态

✅ **完全重启测试**
```bash
pkill -9 gz
rm -rf /tmp/gazebo-*
# 重新启动
```

## 验证模板

### 最小验证
```bash
#!/bin/bash
set -e

echo "[1/3] 构建..."
colcon build --packages-select pkg

echo "[2/3] 环境..."
source install/setup.bash
ros2 pkg list | grep pkg || exit 1

echo "[3/3] 启动..."
timeout 10 ros2 launch pkg launch_file || true

echo "✓ 最小验证通过"
```

### 完整验证
```bash
#!/bin/bash
set -e

echo "[1/5] 清理..."
rm -rf build/pkg install/pkg

echo "[2/5] 构建..."
colcon build --packages-select pkg

echo "[3/5] 语法..."
xacro install/pkg/share/pkg/urdf/robot.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf

echo "[4/5] 环境..."
source install/setup.bash
ros2 pkg list | grep pkg

echo "[5/5] 集成..."
timeout 30 ros2 launch pkg launch_file &
sleep 10
ros2 node list
pkill -f "ros2 launch"

echo "✓ 完整验证通过"
```

## 验证报告模板

```markdown
## 验证报告

**任务**: <描述>
**日期**: 2026-10-22

### 验证结果

- [x] Level 1: 语法验证通过
- [x] Level 2: 构建验证通过
- [x] Level 3: 单元验证通过
- [x] Level 4: 集成验证通过
- [x] Level 5: 端到端验证通过

### 遇到的问题
1. 问题描述
   - 解决方案

### 未测试的部分
- [ ] 长时间运行稳定性
- [ ] 边界条件

### 结论
✅ 任务完成，所有核心功能验证通过
```

---

**记住**：没有验证的"完成"不是真正的完成。
