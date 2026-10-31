# Systematic Debugging (系统化调试)

## 目的
提供一个分层的、系统化的调试方法，避免随机尝试和无效修复。

## 何时使用
- 系统无法正常工作
- 出现错误信息但原因不明
- 修复一个问题后出现新问题
- 花费超过 15 分钟仍未定位问题

## 工作流程

### 第 1 层：信息收集（5 分钟）
```bash
# 收集完整错误信息
# 不要只看第一行错误，获取完整堆栈

# 检查日志文件
ls ~/.ros/log/latest/
cat ~/.ros/log/latest/*.log

# 记录环境状态
env | grep ROS
env | grep GZ
env | grep GAZEBO
```

**检查点**：
- [ ] 完整错误信息已保存
- [ ] 所有相关日志已检查
- [ ] 环境变量已记录

### 第 2 层：分层诊断（10 分钟）
按顺序检查每一层，不要跳过：

#### 2.1 构建层
```bash
# 清理并重新构建
colcon build --packages-select <package> --cmake-clean-cache

# 检查构建输出
# 任何 WARNING 都要注意
```

#### 2.2 环境层
```bash
# 验证包可见性
ros2 pkg list | grep <your_package>

# 验证 topic/service/action
ros2 topic list
ros2 service list
ros2 action list
```

#### 2.3 配置层
```bash
# 验证文件存在性
ls install/<package>/share/<package>/

# 验证 xacro/URDF
xacro <file>.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf
```

#### 2.4 运行时层
```bash
# 启动并监控
ros2 launch <package> <launch_file> 2>&1 | tee debug.log

# 在另一个终端检查节点
ros2 node list
ros2 node info /<node_name>
```

**检查点**：
- [ ] 构建无错误无警告
- [ ] 包在 ROS 2 环境中可见
- [ ] 配置文件存在且有效
- [ ] 节点正常启动

### 第 3 层：根因分析（10 分钟）
找到**第一个**失败的地方：

1. **从错误信息向前追溯**
   - 最后一个错误通常不是根因
   - 找到导致它的上游错误

2. **隔离问题**
   ```bash
   # 最小可复现示例
   # 逐步删除组件直到问题消失
   ```

3. **验证假设**
   ```bash
   # 如果怀疑是 X，单独测试 X
   # 不要在完整系统中测试假设
   ```

**检查点**：
- [ ] 找到第一个失败点
- [ ] 理解失败原因
- [ ] 有可验证的假设

### 第 4 层：修复验证（5 分钟）
```bash
# 1. 只修复一个问题
# 2. 重新构建
colcon build --packages-select <package>

# 3. 验证修复
<your_test_command>

# 4. 如果失败，回滚并重新分析
git diff
git checkout -- <file>
```

**检查点**：
- [ ] 修复是最小的
- [ ] 修复已验证
- [ ] 没有引入新问题

## 检查清单

### 开始调试前
- [ ] 有完整的错误信息
- [ ] 知道上一次工作的状态
- [ ] 有时间限制（避免无限调试）

### 调试过程中
- [ ] 一次只改一个地方
- [ ] 每次修改后都验证
- [ ] 记录尝试过的方法

### 调试完成后
- [ ] 根因已确认
- [ ] 修复已验证
- [ ] 文档已更新

## 常见陷阱

### ❌ 随机尝试
```bash
# 错误做法
试试重新构建
试试重启
试试改这个参数
试试...
```

### ✅ 系统化方法
```bash
# 正确做法
1. 收集信息
2. 分层检查
3. 定位根因
4. 最小修复
5. 验证结果
```

### ❌ 跳过验证
```bash
# 错误做法
改了 3 个文件
直接启动完整系统
失败了也不知道哪个改错了
```

### ✅ 逐步验证
```bash
# 正确做法
改文件 A → 验证 → 通过
改文件 B → 验证 → 通过
改文件 C → 验证 → 失败 → 回滚 C → 重新分析
```

## 示例：Gazebo 模型加载失败

### 问题
```
[Err] Unable to find uri[model://coal_wall]
```

### 系统化诊断

#### 第 1 层：信息收集
```bash
# 获取完整错误
# 检查模型路径环境变量
echo $GZ_SIM_RESOURCE_PATH
echo $GAZEBO_MODEL_PATH

# 检查模型文件
ls src/hydromining_sim/models/coal_wall/
```

#### 第 2 层：分层诊断

**配置层**：
```bash
# 模型文件存在吗？
ls src/hydromining_sim/models/coal_wall/model.sdf
ls src/hydromining_sim/models/coal_wall/model.config

# 安装了吗？
ls install/hydromining_sim/share/hydromining_sim/models/coal_wall/
```

**环境层**：
```bash
# Gazebo 能找到模型路径吗？
source install/setup.bash
echo $GZ_SIM_RESOURCE_PATH | tr ':' '\n'
```

#### 第 3 层：根因分析
```
发现：模型文件存在，但没有安装到 install/
原因：CMakeLists.txt 没有 install(DIRECTORY models ...)
```

#### 第 4 层：修复验证
```bash
# 1. 添加 install 指令
# 2. 重新构建
colcon build --packages-select hydromining_sim

# 3. 验证安装
ls install/hydromining_sim/share/hydromining_sim/models/coal_wall/

# 4. 测试 Gazebo
gz sim <world_file>
```

## 时间预算

| 阶段 | 时间限制 | 如果超时 |
|-----|---------|---------|
| 信息收集 | 5 分钟 | 寻求帮助 |
| 分层诊断 | 10 分钟 | 重新审视问题 |
| 根因分析 | 10 分钟 | 考虑不同方向 |
| 修复验证 | 5 分钟 | 回滚并重新分析 |

**总计**: 30 分钟
**如果 30 分钟仍未解决**: 
1. 记录当前发现
2. 创建最小可复现示例
3. 寻求帮助或换个时间

## 相关 Skills
- `error-recovery.md` - 修复失败后的恢复
- `verification-before-completion.md` - 修复后的验证
- `writing-plans.md` - 复杂问题的规划

---

**记住**：系统化调试不是速度慢，而是避免在错误方向上浪费时间。
