# 阶段4 Gazebo GUI 验收指南

**最后更新**: 2026-10-25

---

## 🎯 概述

阶段4实现了 **Gazebo 仿真中实时更新煤壁几何**，通过 ROS 服务触发喷射动作。

**核心功能**：
1. ✅ ROS 服务接口（`/spray_action`）
2. ✅ Level Set 演化算法
3. ✅ Marching Cubes 几何重建
4. ✅ Gazebo 模型动态更新
5. ✅ 完整验收测试脚本（6步标准）

---

## 🚀 快速启动

### 第1步：启动仿真

**终端1：启动 Gazebo + 机器人**

```bash
cd ~/SCI/hydromining_ws
source install/setup.bash
ros2 launch hydromining_robot robot_simulation.launch.py
```

**预期**：
- Gazebo GUI 窗口打开
- 看到 Panda 机械臂 + 水枪 + 初始煤壁
- 煤壁呈现凹凸不平的表面

---

### 第2步：启动喷射动作服务器

**终端2：启动服务节点**

```bash
cd ~/SCI/hydromining_ws
source install/setup.bash
source .venv_stage4/bin/activate  # 激活阶段4虚拟环境

ros2 run hydromining_robot spray_action_server.py
```

**预期输出**：
```
✅ 喷射动作服务器已启动
   服务: /spray_action
   输出目录: /tmp/phase4_gazebo
📦 加载煤壁模型: /home/star/SCI/hydromining_ws/src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj
✅ Level Set 初始化完成: (32, 32, 16)
```

---

### 第3步：运行验收测试

**终端3：执行验收测试**

```bash
cd ~/SCI/hydromining_ws
source install/setup.bash
source .venv_stage4/bin/activate

ros2 run hydromining_robot test_phase4_acceptance.py
```

---

## 📋 验收标准（6步）

测试脚本会自动执行以下步骤：

| 步骤 | 描述 | 在 Gazebo GUI 中观察 |
|------|------|---------------------|
| **1** | 展示初始煤壁，保存喷射前点云 | 煤壁完整，凹凸不平 |
| **2** | 固定机械臂，触发第1次喷射（10MPa, 5秒） | （等待处理） |
| **3** | 喷射结束后更新煤壁，展示凹坑 | **煤壁中心出现明显凹坑** |
| **4** | 保存喷射后点云 | 凹坑深度 ~5-15 cm |
| **5** | 再喷射一次，展示累计变化 | **凹坑加深，体积增大** |
| **6** | 重置煤壁，用相同参数重做 | （需要重启服务器节点） |

---

## 🔍 验收重点

### ✅ 必须观察到的现象

1. **第1次喷射后（步骤3）**：
   - Gazebo GUI 中煤壁模型发生变化
   - 在喷射点（世界坐标 `[0.0, 0.0, 1.0]`）附近出现凹坑
   - 终端输出：
     ```
     ✅ 喷射成功（第 1 次）
        去除体积: 0.006754 m³
        最大深度: 0.089 m
     ```

2. **第2次喷射后（步骤5）**：
   - 凹坑进一步加深
   - 累计去除体积增加
   - 终端输出：
     ```
     ✅ 喷射成功（第 2 次）
        第2次去除体积: 0.005123 m³
        累计去除体积: 0.011877 m³
     ```

### ⚠️ 常见问题

**问题1：Gazebo 中煤壁没有变化**

**诊断**：
```bash
# 检查服务是否可用
ros2 service list | grep spray_action

# 检查服务器节点日志
# 查看终端2的输出，是否有错误
```

**可能原因**：
- 服务器节点未启动
- Gazebo `gz service` 调用失败
- 虚拟环境未激活（缺少 numpy/scipy）

---

**问题2：构建失败**

**现象**：`KeyError: 'geometry_msgs'`

**解决**：已在 CMakeLists.txt 中修复，重新构建：
```bash
colcon build --packages-select hydromining_robot --symlink-install
```

---

**问题3：模块导入失败**

**现象**：`ModuleNotFoundError: No module named 'voxel_grid'`

**原因**：算法模块路径错误

**解决**：检查算法模块是否存在：
```bash
ls ~/SCI/hydromining_ws/src/hydromining_sim/scripts/*.py
# 应包含：voxel_grid.py, erosion_model.py, level_set_evolver.py, geometry_utils.py
```

---

## 📊 预期输出示例

### 终端2（服务器节点）

```
✅ 喷射动作服务器已启动
   服务: /spray_action
   输出目录: /tmp/phase4_gazebo
📦 加载煤壁模型: .../coal_wall.obj
✅ Level Set 初始化完成: (32, 32, 16)

🔧 收到喷射请求
   位置: [0.00, 0.00, 1.00]
   压力: 10.0 MPa
   时长: 5.0 秒
✅ Mesh 已保存: /tmp/phase4_gazebo/coal_wall_spray_1.obj
   去除体积: 0.006754 m³
   最大深度: 0.089 m
🔄 更新 Gazebo 模型...
✅ Gazebo 模型已更新
```

### 终端3（测试脚本）

```
🧪 阶段4验收测试开始
======================================================================
📋 步骤1: 展示初始煤壁，保存喷射前点云
   ▶ 请在 Gazebo GUI 中观察初始煤壁
💾 点云信息已保存: /tmp/phase4_acceptance/pointcloud_before_spray1.txt

📋 步骤2: 固定机械臂，触发一次喷射
🚀 触发喷射: 位置=[0.0, 0.0, 1.0], 压力=10.0MPa, 时长=5.0s
✅ 喷射成功（第 1 次）
   去除体积: 0.006754 m³
   最大深度: 0.089 m

📋 步骤3: 喷射结束后更新煤壁，展示凹坑
   ▶ 请在 Gazebo GUI 中观察煤壁凹坑
   ✅ 去除体积: 0.006754 m³
   ✅ 最大深度: 0.089 m

📋 步骤4: 保存喷射后点云，确认传感器观测到凹坑
💾 点云信息已保存: /tmp/phase4_acceptance/pointcloud_after_spray1.txt

📋 步骤5: 再喷射一次，展示累计变化
✅ 第2次去除体积: 0.005123 m³
✅ 累计去除体积: 0.011877 m³

======================================================================
✅ 阶段4验收测试完成
======================================================================
📊 测试结果：
   - 第1次喷射去除体积: 0.006754 m³
   - 第1次喷射最大深度: 0.089 m
   - 第2次喷射去除体积: 0.005123 m³
   - 累计去除体积: 0.011877 m³
📂 输出目录: /tmp/phase4_acceptance
```

---

## 🎮 手动测试（可选）

如果自动测试脚本有问题，可以手动调用服务：

```bash
# 手动触发一次喷射
ros2 service call /spray_action hydromining_robot/srv/SprayAction \
  "{target_position: {x: 0.0, y: 0.0, z: 1.0}, \
    spray_direction: {x: 0.0, y: 0.0, z: -1.0}, \
    pressure: 10.0, \
    duration: 5.0}"
```

**预期响应**：
```yaml
success: True
message: '喷射成功（第 1 次）'
removed_volume: 0.006754
max_depth: 0.089
mesh_path: '/tmp/phase4_gazebo/coal_wall_spray_1.obj'
```

---

## 📂 生成的文件

| 文件 | 路径 | 说明 |
|------|------|------|
| **更新后的煤壁 Mesh** | `/tmp/phase4_gazebo/coal_wall_spray_<N>.obj` | Marching Cubes 重建 |
| **临时 SDF 文件** | `/tmp/phase4_gazebo/coal_wall_spray_<N>.sdf` | Gazebo spawn 使用 |
| **点云元信息** | `/tmp/phase4_acceptance/pointcloud_*.txt` | 点云验证记录 |

---

## 🔧 调试技巧

### 检查 Gazebo 模型是否更新

```bash
# 列出当前 Gazebo 中的模型
gz model --list

# 应该看到 coal_wall（如果已更新，名称可能包含时间戳）
```

### 检查生成的 Mesh 文件

```bash
# 查看生成的 OBJ 文件
ls -lh /tmp/phase4_gazebo/

# 验证 OBJ 文件格式
head -20 /tmp/phase4_gazebo/coal_wall_spray_1.obj
```

### 监控 ROS 话题

```bash
# 查看传感器话题（验证点云数据）
ros2 topic hz /hydromining/lidar/points
```

---

## ✅ 验收通过标准

- [ ] Gazebo GUI 中看到初始煤壁
- [ ] 第1次喷射后，煤壁出现明显凹坑
- [ ] 终端输出去除体积 > 0（建议 > 0.005 m³）
- [ ] 终端输出最大深度 > 0（建议 > 0.05 m）
- [ ] 第2次喷射后，凹坑进一步加深
- [ ] 累计去除体积 = 第1次 + 第2次

---

## 📖 相关文档

- [阶段4设计方案](01-设计方案.md)
- [阶段4执行指南](02-执行指南.md)
- [阶段4评估报告](03-评估报告.md)
- [进度交接文档](00-进度交接.md)

---

**作者**: Star  
**日期**: 2026-10-25  
**版本**: 1.0
