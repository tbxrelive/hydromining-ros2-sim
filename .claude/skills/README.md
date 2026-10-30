# Skills 技能库

这个文件夹包含了用于 AI Agent 的各种技能（Skills）。每个 Skill 都是一个独立的指导文件，定义了特定任务的最佳实践和工作流程。

---

## 📚 Skills 索引

### 🔧 核心开发技能

| Skill 名称 | 文件 | 用途 | 何时调用 |
|-----------|------|------|---------|
| **系统化调试** | `systematic-debugging.md` | 分层诊断和问题定位 | 出现错误、系统不工作时 |
| **完成前验证** | `verification-before-completion.md` | 提交前的完整测试流程 | 完成任何代码修改后 |
| **编写计划** | `writing-plans.md` | 创建详细的实施计划 | 开始新功能或大型重构前 |
| **预验证检查** | `pre-modification-checks.md` | 修改前的依赖和配置检查 | 修改任何配置或代码前 |

### 🤖 ROS 2 专项技能

| Skill 名称 | 文件 | 用途 | 何时调用 |
|-----------|------|------|---------|
| **ROS 2 开发** | `ros2-development.md` | ROS 2 包开发最佳实践 | 创建或修改 ROS 2 包时 |
| **Launch 文件** | `ros2-launch-files.md` | Launch 文件编写和调试 | 编写或修改 launch 文件时 |
| **URDF/xacro** | `ros2-urdf-xacro.md` | 机器人描述文件 | 创建或修改机器人模型时 |
| **ros2_control** | `ros2-control.md` | 控制器配置和调试 | 配置机器人控制系统时 |

### 🎮 Gazebo 仿真技能

| Skill 名称 | 文件 | 用途 | 何时调用 |
|-----------|------|------|---------|
| **Gazebo 集成** | `gazebo-integration.md` | Gazebo 配置和模型管理 | 设置仿真环境时 |
| **SDF 模型** | `gazebo-sdf-models.md` | SDF 文件编写和验证 | 创建 Gazebo 模型时 |
| **模型路径** | `gazebo-model-paths.md` | 模型资源路径配置 | 解决模型加载问题时 |

### 🧪 测试与质量技能

| Skill 名称 | 文件 | 用途 | 何时调用 |
|-----------|------|------|---------|
| **机器人测试** | `robotics-testing.md` | 机器人系统测试策略 | 编写测试或验证功能时 |
| **单元测试** | `unit-testing.md` | 单元测试编写 | 开发新功能时 |
| **集成测试** | `integration-testing.md` | 跨组件测试 | 多个模块集成时 |

---

## 🚀 快速开始

### 如何调用 Skill

在与 AI Agent 对话时，明确提到 Skill 名称：

```
"请按照 systematic-debugging.md 的流程诊断这个问题"
"使用 verification-before-completion.md 验证这个修改"
"根据 ros2-development.md 创建新包"
```

---

**版本**: 1.0.0  
**最后更新**: 2026-10-22  
**维护者**: Star & Claude Code
