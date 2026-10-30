# GitHub Skills 仓库搜索关键词

## 基于你提供的参考

1. **systematic-debugging** - 系统化调试方法
2. **verification-before-completion** - 完成前验证
3. **writing-plans** - 编写计划
4. **ros2** - ROS 2 相关技能
5. **robotics-software-principles** - 机器人软件原则
6. **robotics-testing** - 机器人测试

## 推荐的 GitHub 搜索查询

### 高优先级搜索

```
# AI Agent Skills for Robotics
"agent skills" robotics ROS2
"claude skills" robotics
"AI coding assistant" ROS2 skills

# Systematic Development
"systematic debugging" robotics
"test-driven" ROS2
"verification before completion"

# ROS 2 Best Practices
"ROS2 best practices" skills
"ROS2 development workflow"
"ROS2 testing framework"

# Robotics Software Engineering
"robotics software principles"
"robot development guidelines"
"robotics code quality"
```

### 具体仓库类型

1. **Skills/Prompts 仓库**
   - anthropics/anthropic-cookbook
   - patterns-ai/patterns
   - f/awesome-chatgpt-prompts

2. **ROS 2 开发仓库**
   - ros2/ros2
   - ros-planning/moveit2
   - gazebosim/ros_gz

3. **机器人测试仓库**
   - ament/ament_cmake
   - osrf/osrf_testing_tools_cpp

## 预期找到的内容

### Skills 结构示例
```
robotics-skills/
├── core/
│   ├── systematic-debugging.md
│   ├── verification-workflow.md
│   └── error-recovery.md
├── ros2/
│   ├── package-setup.md
│   ├── launch-files.md
│   └── gazebo-integration.md
└── testing/
    ├── unit-tests.md
    ├── integration-tests.md
    └── simulation-validation.md
```

### 我们需要的核心 Skills

1. **预验证流程**
   - 构建前检查依赖
   - 语法验证（xacro, URDF）
   - 路径验证

2. **系统化调试**
   - 分层诊断（构建 → 环境 → 运行时）
   - 日志收集和分析
   - 最小可复现示例

3. **完成前验证**
   - 单元测试
   - 集成测试
   - 仿真验证

4. **ROS 2 最佳实践**
   - 包结构规范
   - CMakeLists.txt 模板
   - Launch 文件模式

5. **回滚和恢复**
   - Git 分支策略
   - 已知良好状态检查点
   - 快速回滚脚本
