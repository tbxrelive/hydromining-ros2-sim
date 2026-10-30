# 阶段 3 快速测试指南

**目标**：验证传感器集成和点云配准功能

---

## 🚀 快速开始（4 个终端）

### 终端 1：启动仿真

```bash
cd ~/SCI/hydromining_ws
./run_observation_test.sh sim
```

**预期现象**：
- Gazebo 窗口打开
- 显示煤壁和机器人
- 控制台无报错

---

### 终端 2：启动点云采集节点

```bash
cd ~/SCI/hydromining_ws
./run_observation_test.sh capture
```

**预期输出**：
```
[INFO] [pointcloud_capture]: 点云采集节点已启动
[INFO] [pointcloud_capture]: 服务准备就绪: /capture_pointcloud
[INFO] [pointcloud_capture]: 输出目录: .../test/data
```

---

### 终端 3：运行多视角观测测试

```bash
cd ~/SCI/hydromining_ws
./run_observation_test.sh test
```

**预期流程**：
```
[observation_test]: === 开始多视角观测测试 ===
[observation_test]: [1/5] 移动到观测位姿 A（正面）
[observation_test]: 发送目标位置: [0.0, -0.785, 0.0]...
[observation_test]: ✓ 运动完成
[observation_test]: [2/5] 采集点云 A
[pointcloud_capture]: 点云已保存: .../pointcloud_xxx.pcd
...
[observation_test]: === ✓ 观测序列完成 ===
[observation_test]: ✓✓✓ 测试成功！
```

---

### 终端 4：验证点云配准质量

**方法 1：使用自动虚拟环境脚本**（推荐）

```bash
cd ~/SCI/hydromining_ws
./verify_with_venv.sh
```

首次运行会自动创建虚拟环境并安装 Open3D（约 400 MB，需要几分钟）。

**方法 2：手动使用虚拟环境**

```bash
cd ~/SCI/hydromining_ws

# 创建虚拟环境（只需首次运行）
python3 -m venv venv_test
source venv_test/bin/activate
pip install open3d numpy

# 运行验证
python3 src/hydromining_robot/test/scripts/verify_registration.py \
  src/hydromining_robot/test/data

# 退出虚拟环境
deactivate
```

**预期输出**：
```
找到 3 个点云文件:
  - pointcloud_xxx0.pcd  (A)
  - pointcloud_xxx1.pcd  (B)
  - pointcloud_xxx2.pcd  (A baseline)

=== 点云配准评估 ===
预处理点云...
点云 A: 12000+ 点
点云 B: 11000+ 点

执行 ICP 配准...

配准适应度 (fitness): 0.50+ (目标 > 0.5)
配准 RMSE: 0.03-0.05 m (目标 < 0.05m)

✓ 配准质量良好（重叠区域对齐成功）
✓✓✓ 验证通过：传感器系统工作正常
```

会弹出 Open3D 窗口显示红色（A）和绿色（B 对齐后）点云。

---

## 📊 验证标准

| 测试项 | 验证标准 | 说明 |
|--------|----------|------|
| 仿真启动 | Gazebo 窗口正常显示 | 传感器插件已加载 |
| 采集节点 | 服务 `/capture_pointcloud` 可用 | 能接收采集请求 |
| 多视角观测 | 机器人移动到多个位姿并采集 | 运动控制 + 传感器数据正常 |
| 点云配准 | fitness > 0.5, RMSE < 0.05m | 点云质量足够用于 3D 重建 |

---

## 🔧 其他命令

```bash
# 清理点云数据（重新测试前）
./run_observation_test.sh clean

# 查看脚本用法
./run_observation_test.sh
```

---

## ❓ 常见问题

### Q1: `test_observation.py` 卡在 "等待 Action Server"

**原因**：Joint Trajectory Controller 未启动

**解决**：检查终端 1 的输出，确认控制器已 active：
```bash
# 在另一个终端检查
source ~/SCI/hydromining_ws/install/setup.bash
ros2 control list_controllers
```

应该看到：
```
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
```

---

### Q2: 点云文件数量不足

**原因**：观测测试未完成或失败

**解决**：
1. 检查终端 3 的输出，看是否有错误
2. 清理并重新运行：
   ```bash
   ./run_observation_test.sh clean
   ./run_observation_test.sh test
   ```

---

### Q3: Open3D 窗口不弹出

**原因**：可能没有显示服务器（SSH 会话）或虚拟环境配置问题

**解决**：
- 如果是 SSH 连接，添加 X11 转发：`ssh -X user@host`
- 检查 `$DISPLAY` 环境变量是否设置
- 验证结果主要看终端输出的数值（fitness 和 RMSE）

---

### Q4: 虚拟环境安装 Open3D 失败

**解决方法**：
```bash
# 删除旧的虚拟环境
rm -rf ~/SCI/hydromining_ws/venv_test

# 使用系统级安装（有风险，但简单）
pip install open3d --break-system-packages

# 然后直接运行验证（不需要虚拟环境）
cd ~/SCI/hydromining_ws
source install/setup.bash
python3 src/hydromining_robot/test/scripts/verify_registration.py \
  src/hydromining_robot/test/data
```

---

## 📝 测试完成后

如果所有测试通过：
1. ✅ 传感器数据流正常
2. ✅ 多视角观测成功
3. ✅ 点云配准质量达标

**阶段 3 完成！** 🎉

可以进入下一阶段：
- 阶段 4：煤壁识别与特征提取
- 阶段 5：路径规划与轨迹生成

---

## 🗑️ 清理（可选）

测试完成后，如果想释放磁盘空间：

```bash
# 删除虚拟环境（约 500 MB）
rm -rf ~/SCI/hydromining_ws/venv_test

# 删除测试点云数据（约几 MB）
./run_observation_test.sh clean
```
