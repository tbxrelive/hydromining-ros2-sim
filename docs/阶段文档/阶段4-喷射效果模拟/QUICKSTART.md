# 阶段4快速验证指南

**文档版本**: v1.0  
**最后更新**: 2026-10-25

---

## 🚀 5分钟快速验证

### 前提条件

```bash
# 确认在项目目录
cd ~/SCI/hydromining_ws

# 确认煤壁 Mesh 文件存在
ls -lh src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj
```

---

## ✅ 验证步骤

### 步骤1: 激活虚拟环境

```bash
source .venv_stage4/bin/activate

# 验证依赖
python3 -c "import trimesh, numpy, scipy, pymcubes; print('✅ 依赖正常')"
```

**预期输出**:
```
✅ 依赖正常
```

---

### 步骤2: 运行核心算法单元测试

```bash
python3 test_scripts/test_phase4_1_complete.py
```

**预期输出**（关键行）:
```
[TEST 4.1.1] VoxelGrid 类 ✅ PASS
[TEST 4.1.2] ErosionModel 类 ✅ PASS
[TEST 4.1.3] LevelSetEvolver 类 ✅ PASS
[TEST 4.1.4] GeometryUtils 类 ✅ PASS

✅ 所有4个子阶段测试通过！
```

**预计时间**: ~30秒

---

### 步骤3: 运行几何转换测试

```bash
python3 test_scripts/test_mesh_conversion.py
```

**预期输出**（关键行）:
```
✅ VoxelGrid 生成成功
✅ Marching Cubes 转换成功
✅ Mesh 信息:
   顶点数: 13440
   三角形数: 26876
   是否闭合: True
```

**预计时间**: ~15秒

---

### 步骤4: 运行完整集成测试（可选，耗时较长）

```bash
# 后台运行（约2-5分钟）
python3 test_scripts/test_phase4_integration.py > /tmp/phase4_test.log 2>&1 &

# 查看进度
tail -f /tmp/phase4_test.log
```

**预期输出**（关键行）:
```
================================================================================
Phase 4.4: 完整流程集成测试
================================================================================

[1/6] 加载 Mesh 并生成 VoxelGrid...
✅ VoxelGrid 生成

[2/6] 定义喷射参数...
✅ 喷射参数

[3/6] 计算冲蚀速度场...
✅ 冲蚀速度

[4/6] Level Set 演化...
✅ 演化完成

[5/6] 转换为 Mesh...
✅ Mesh 生成

[6/6] 保存结果...
✅ 已保存到: /tmp/phase4_tests/coal_wall_after_spray_seed42.obj

================================================================================
✅ 完整流程测试通过
================================================================================
```

**检查输出文件**:
```bash
ls -lh /tmp/phase4_tests/
cat /tmp/phase4_tests/integration_test_result.json
```

**预计时间**: ~2-5分钟（取决于分辨率）

---

## 🧪 ROS 服务验证（需 ROS 2 环境）

### 步骤1: 构建包

```bash
cd ~/SCI/hydromining_ws
colcon build --packages-select hydromining_robot --symlink-install
source install/setup.bash
```

**预期输出**:
```
Finished <<< hydromining_robot [4.2s]
Summary: 1 package finished [6.0s]
```

---

### 步骤2: 检查服务接口

```bash
ros2 interface show hydromining_robot/srv/SprayAction
```

**预期输出**:
```
float64 position_x
float64 position_y
float64 position_z
float64 direction_x
float64 direction_y
float64 direction_z
float64 pressure
float64 duration
float64 nozzle_diameter
int32 random_seed
---
bool success
string message
float64 volume_removed_m3
float64 max_depth_m
int32 vertices_removed
int32 vertices_displaced
string output_mesh_path
```

---

### 步骤3: 启动服务节点（可选）

**终端1 - 启动节点**:
```bash
source install/setup.bash
source .venv_stage4/bin/activate

ros2 run hydromining_robot spray_controller_node \
  --ros-args \
  -p coal_wall_mesh_path:=src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj \
  -p output_mesh_dir:=/tmp/hydromining_meshes
```

**预期输出**:
```
[INFO] [spray_controller_node]: ✅ 喷射控制器节点已启动
[INFO] [spray_controller_node]:    煤壁 Mesh: src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj
[INFO] [spray_controller_node]:    输出目录: /tmp/hydromining_meshes
[INFO] [spray_controller_node]:    体素分辨率: (64, 64, 32)
```

**终端2 - 测试服务**:
```bash
cd ~/SCI/hydromining_ws
source install/setup.bash

python3 test_scripts/test_spray_service.py
```

**预期输出**:
```
[INFO] [spray_action_client_test]: ⏳ 等待 /spray_action 服务...
[INFO] [spray_action_client_test]: ✅ 服务已连接
[INFO] [spray_action_client_test]: 📤 发送喷射请求...
[INFO] [spray_action_client_test]: ✅ 喷射模拟成功
[INFO] [spray_action_client_test]:    去除体积: 0.XXXXXX m³
[INFO] [spray_action_client_test]:    最大深度: 0.XXXX m
[INFO] [spray_action_client_test]:    输出 Mesh: /tmp/hydromining_meshes/coal_wall_updated_XXXXX.obj
```

---

## ❌ 常见问题排查

### 问题1: `ModuleNotFoundError: No module named 'trimesh'`

**原因**: 虚拟环境未激活或依赖未安装

**解决**:
```bash
source .venv_stage4/bin/activate
pip install trimesh pymcubes scipy numpy
```

---

### 问题2: `FileNotFoundError: Mesh not found`

**原因**: 煤壁 Mesh 文件路径错误

**解决**:
```bash
# 检查文件存在
ls src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj

# 或使用绝对路径
python3 test_scripts/test_phase4_integration.py
```

---

### 问题3: 测试运行很慢（>5分钟）

**原因**: 分辨率过高或 CPU 性能较低

**解决**:
```bash
# 编辑测试脚本，降低分辨率
# 将 resolution=(64, 64, 32) 改为 resolution=(32, 32, 16)
```

---

### 问题4: ROS 服务节点启动失败

**原因1**: 虚拟环境未激活
```bash
source .venv_stage4/bin/activate
```

**原因2**: 包未构建
```bash
colcon build --packages-select hydromining_robot --symlink-install
source install/setup.bash
```

**原因3**: 路径错误
```bash
# 检查 coal_wall.obj 路径
ls src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj
```

---

## 📊 成功标志

### 必须通过（核心功能）

- [x] 单元测试全部通过（test_phase4_1_complete.py）
- [x] 几何转换测试通过（test_mesh_conversion.py）
- [x] ROS 服务接口编译成功

### 可选通过（完整验证）

- [ ] 集成测试通过（test_phase4_integration.py）
- [ ] ROS 服务节点可启动并响应

---

## 🎯 下一步

### 如果所有测试通过 ✅

**恭喜！阶段4验收通过，可以进入阶段5**

查看下一阶段规划：
```bash
cat docs/阶段文档/阶段4-喷射效果模拟/00-进度交接.md
```

### 如果遇到问题 ❌

1. 查看详细错误日志
2. 参考 `docs/阶段文档/阶段4-喷射效果模拟/00-进度交接.md`
3. 检查 troubleshooting 部分

---

## 📞 技术支持

### 文档位置

- 进度交接: `docs/阶段文档/阶段4-喷射效果模拟/00-进度交接.md`
- 设计方案: `docs/阶段文档/阶段4-喷射效果模拟/01-设计方案.md`
- 执行指南: `docs/阶段文档/阶段4-喷射效果模拟/02-执行指南.md`
- 评估报告: `docs/阶段文档/阶段4-喷射效果模拟/03-评估报告.md`

### 联系方式

- 项目维护者: Star (@tbxrelive)
- GitHub: https://github.com/tbxrelive

---

**文档版本**: v1.0  
**最后更新**: 2026-10-25
