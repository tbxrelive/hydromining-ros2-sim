# GUI 可视化使用说明

## 🎯 概述

阶段4提供了 **2 种可视化方式**：
1. **HTML 浏览器版**（推荐，无需 OpenGL）
2. **Open3D 桌面版**（需要图形界面）

---

## 🌐 方式1：HTML 浏览器版（推荐）

### 优点
- ✅ 无需 OpenGL，适合 SSH 远程环境
- ✅ 在任何浏览器中查看（Chrome, Firefox, Safari）
- ✅ 交互流畅，支持触摸屏
- ✅ 自动对比两个模型（左侧喷射前，右侧喷射后）

### 快速使用

```bash
cd ~/SCI/hydromining_ws
source .venv_stage4/bin/activate

# 1. 先生成喷射后模型
python3 test_scripts/test_phase4_integration_mini.py

# 2. 生成 HTML 可视化
python3 test_scripts/visualize_spray_html.py

# 3. 启动 HTTP 服务器
cd /tmp/phase4_visualization
python3 -m http.server 8080
```

### 打开方式

**方式 A：本地浏览器**
```
访问: http://localhost:8080/comparison.html
```

**方式 B：远程访问（SSH 端口转发）**
```bash
# 在本地机器执行
ssh -L 8080:localhost:8080 star@your_server_ip

# 然后在本地浏览器访问
http://localhost:8080/comparison.html
```

**方式 C：直接用文件路径打开**
```
在浏览器地址栏输入:
file:///tmp/phase4_visualization/comparison.html
```

---

## 🖼️ 交互操作

### 鼠标控制

| 操作 | 功能 |
|------|------|
| **鼠标左键拖动** | 旋转视角 |
| **鼠标滚轮** | 缩放（放大/缩小） |
| **鼠标右键拖动** | 平移视图 |

### 按钮功能

- **重置视角**：恢复初始相机位置
- **自动旋转**：模型自动旋转展示
- **线框模式**：切换实体/线框显示

---

## 🎨 颜色说明

- **灰色（左侧）**：喷射前煤壁
- **红色（右侧）**：喷射后煤壁

**观察要点**：
- 喷射后模型应在喷射点附近出现明显**凹陷**
- 凹陷形状呈**圆锥形或半球形**
- 其他区域保持原状

---

## 🖥️ 方式2：Open3D 桌面版

### 前置要求

```bash
cd ~/SCI/hydromining_ws
source .venv_stage4/bin/activate
pip install open3d
```

### 运行可视化

```bash
# 对比视图（默认）
python3 test_scripts/visualize_spray_result.py

# 仅查看喷射前
python3 test_scripts/visualize_spray_result.py --before

# 仅查看喷射后
python3 test_scripts/visualize_spray_result.py --after
```

### ⚠️ 常见问题

**问题：`Failed to create window` 或 `GLX: Failed to create context`**

**原因**：运行环境无 X11 图形界面（如 SSH 远程连接）

**解决**：使用**方式1（HTML 版）**，或启用 X11 转发：
```bash
ssh -X star@your_server_ip
```

---

## 🔍 验证要点

### 喷射前煤壁（灰色）
- 表面应呈现凹凸不平的粗糙几何
- 倾斜角度约 40-60°
- 整体完整，无缺失

### 喷射后煤壁（红色）
- 应在**喷射点附近**出现明显**凹陷**
- 凹陷形状呈**圆锥形或半球形**
- 其他区域保持原状

### 对比要点
- 将两个模型旋转到同一角度
- 重点观察喷射点（世界坐标 `[0.0, 0.0, 1.0]`）附近
- 预期深度：约 5-15 cm（取决于压力和时长）

---

## 📊 技术细节

### 模型文件路径

| 模型 | 路径 |
|------|------|
| **喷射前** | `src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj` |
| **喷射后** | `/tmp/phase4_visualization/coal_wall_after_spray.obj` |
| **HTML 可视化** | `/tmp/phase4_visualization/comparison.html` |

### 模型规格

**喷射前**：
- 顶点数：~13,440
- 三角形数：~26,880
- 尺寸：3m × 3m × 2m

**喷射后（32×32×16分辨率）**：
- 顶点数：~1,646
- 三角形数：~3,160
- 去除体积：约 0.0068 m³

### HTML 技术栈

- **渲染引擎**：Three.js r128
- **模型格式**：OBJ → JSON（嵌入式）
- **渲染器**：WebGL
- **文件大小**：约 2-3 MB（包含两个模型数据）

---

## 🛠️ 故障排查

### 问题1：HTTP 服务器无法访问

**现象**：浏览器显示"无法连接"

**解决**：
```bash
# 检查服务器是否运行
ps aux | grep "http.server"

# 如果未运行，重新启动
cd /tmp/phase4_visualization
python3 -m http.server 8080

# 验证端口
curl http://localhost:8080/comparison.html
```

---

### 问题2：模型加载失败或显示不完整

**现象**：浏览器显示空白或只有一个模型

**原因**：OBJ 文件格式错误或路径不对

**解决**：
```bash
# 重新生成模型
cd ~/SCI/hydromining_ws
source .venv_stage4/bin/activate
python3 test_scripts/test_phase4_integration_mini.py

# 验证文件存在
ls -lh /tmp/phase4_visualization/coal_wall_after_spray.obj

# 重新生成 HTML
python3 test_scripts/visualize_spray_html.py
```

---

### 问题3：浏览器控制台报错

**现象**：按 F12 查看控制台，有 JavaScript 错误

**常见错误及解决**：

| 错误信息 | 原因 | 解决方法 |
|---------|------|---------|
| `Failed to load resource: net::ERR_FILE_NOT_FOUND` | CDN 资源加载失败 | 检查网络连接，或使用本地 Three.js |
| `THREE is not defined` | Three.js 未加载 | 刷新页面，或检查 CDN 链接 |
| `WebGL not supported` | 浏览器不支持 WebGL | 更新浏览器或换用 Chrome/Firefox |

---

## 🎓 高级使用

### 自定义颜色

编辑 `visualize_spray_result.py`：

```python
# 第 38-39 行
mesh1.paint_uniform_color([0.7, 0.7, 0.7])  # 灰色 - 喷射前
mesh2.paint_uniform_color([0.9, 0.3, 0.3])  # 红色 - 喷射后

# 修改为其他颜色（RGB 范围 0-1）
mesh1.paint_uniform_color([0.2, 0.5, 0.8])  # 蓝色
mesh2.paint_uniform_color([0.8, 0.8, 0.2])  # 黄色
```

---

### 调整模型间距

编辑 `visualize_spray_result.py`：

```python
# 第 42 行
mesh2.translate([3.5, 0, 0])  # 向右平移 3.5m

# 修改平移距离
mesh2.translate([5.0, 0, 0])  # 更远
mesh2.translate([2.0, 0, 0])  # 更近
```

---

### 保存截图

在可视化窗口中：
1. 调整到满意的视角
2. 按 `Ctrl+P`（或菜单 File → Take Screenshot）
3. 保存为 PNG 图片

---

## 📝 相关文档

- **测试脚本**：`test_scripts/visualize_spray_result.py`
- **模型生成**：`test_scripts/test_phase4_integration_mini.py`
- **算法原理**：`docs/阶段文档/阶段4-喷射效果模拟/01-设计方案.md`

---

**最后更新**：2026-10-25  
**维护者**：Star (@tbxrelive)
