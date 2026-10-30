# 第一阶段修复总结

## 问题诊断

启动 Gazebo 时遇到的问题：

1. **煤壁显示为白色**而非深灰色
2. **侧面呈散状**，整体不连贯
3. **Gazebo 错误警告**：
   ```
   [Err] One of the submeshes does not have a normal count that matches its vertex count
   ```
4. **GPU 驱动警告**（非关键，但表明渲染压力）

## 根本原因

导出的 OBJ 文件**缺少法向量信息**（vn 行），导致：
- Gazebo 无法正确计算光照
- 材质信息被忽略，默认渲染为白色
- 网格拓扑问题导致侧面显示异常

## 实施的修复

### 1. 网格生成优化（generate_coal_wall.py）

**修改位置**：`generate()` 方法

```python
# 添加的步骤
print("  → 检查并填补孔洞...")
coal_wall.fill_holes()

print("  → 修复法向向量...")
coal_wall.fix_normals()

print("  → 优化网格拓扑...")
coal_wall = trimesh.util.concatenate([coal_wall])  # 重新合并
```

**作用**：
- `fix_normals()` 确保法向向量一致和正确指向
- 重新合并消除侧壁与前后表面的连接缝隙
- 输出额外的诊断信息（法向向量数、水密性检查）

### 2. 显式法向量导出（generate_coal_wall.py）

**修改位置**：`main()` 函数的导出部分

```python
# 关键修改
mesh.export(output_path, file_type='obj', include_normals=True)

# 验证导出结果
with open(output_path, 'r') as f:
    lines = f.readlines()
    vn_count = sum(1 for line in lines if line.startswith('vn '))
    v_count = sum(1 for line in lines if line.startswith('v '))

print(f"  OBJ 统计:")
print(f"    - 顶点 (v): {v_count}")
print(f"    - 法向 (vn): {vn_count}")
if vn_count == v_count:
    print(f"    ✓ 法向向量完整")
```

**作用**：
- `include_normals=True` 明确指示 trimesh 导出法向向量
- 验证 OBJ 文件确实包含了法向量（vn 数量 = v 数量）

### 3. 材质提升（model.sdf）

**修改位置**：visual 元素的 material 部分

```xml
<!-- 之前（太暗）-->
<ambient>0.12 0.12 0.12 1</ambient>
<diffuse>0.18 0.18 0.18 1</diffuse>

<!-- 之后（可见）-->
<ambient>0.35 0.35 0.35 1</ambient>
<diffuse>0.45 0.45 0.45 1</diffuse>
```

**作用**：
- 增加环境光（ambient）使暗处可见
- 增加漫反射（diffuse）使材质颜色更清晰
- 降低金属度（metalness 0.1→0.05）确保清晰的灰色

## 预期修复效果

执行修改后应看到：

✓ **深灰色煤壁**（而非白色）  
✓ **整体连贯**（无散状侧面）  
✓ **清晰的凹凸**（表面细节可见）  
✓ **无 Gazebo 法向错误**  
✓ **碰撞检测正常**  

## 执行步骤

### 方式1：使用预设（推荐）

```bash
cd /home/star/SCI/hydromining_ws
source venv/bin/activate
source setup_env.sh
cd src/hydromining_sim/scripts
python generate_from_preset.py default
```

### 方式2：直接运行脚本

```bash
cd /home/star/SCI/hydromining_ws
bash regenerate.sh
```

### 方式3：完全自定义

```bash
cd /home/star/SCI/hydromining_ws
source venv/bin/activate
python src/hydromining_sim/scripts/generate_coal_wall.py \
  --width 3.0 \
  --height 3.0 \
  --tilt 20 \
  --roughness 0.12 \
  --output src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj
```

### 启动 Gazebo 验证

```bash
cd /home/star/SCI/hydromining_ws
source venv/bin/activate
source setup_env.sh
gz sim src/hydromining_sim/worlds/coal_wall_v02.sdf
```

## 验收清单

启动 Gazebo 后检查：

- [ ] 煤壁呈**深灰色**（RGB ~0.4）
- [ ] 整体是**连贯的倾斜体**（无破碎感）
- [ ] 从不同角度看有**厚度和侧面**
- [ ] 表面凹凸**清晰可见**
- [ ] 没有红色或异常颜色
- [ ] 终端无法向错误（仅 GPU 驱动警告可接受）

## 技术细节

### OBJ 法向向量格式

修复后的 OBJ 文件应包含：
```
# 示例
v -1.5 -1.5 0.0       # 顶点位置
vn 0.0 0.0 1.0        # 顶点法向（对应）
f 1//1 2//2 3//3      # 面片索引（顶点//法向）
```

### Trimesh 处理流程

```
前表面 (with normals)
    ↓
后表面 (face indices reversed)
    ↓
侧壁 (连接边界)
    ↓
concatenate() ──→ 合并
    ↓
fill_holes() ────→ 填补缝隙
    ↓
fix_normals() ───→ 确保一致
    ↓
concatenate() ──→ 重新优化拓扑
    ↓
export(..., include_normals=True)  ──→ OBJ
```

## 如果问题仍未解决

**症状**：仍然显示白色或散状

**排查步骤**：

1. **检查 OBJ 文件**
   ```bash
   grep '^vn ' src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj | wc -l
   # 应该 > 0
   ```

2. **检查控制台输出**
   ```
   生成煤壁网格...
   → 检查并填补孔洞...
   → 修复法向向量...
   → 优化网格拓扑...
   ✓ 生成完成
     法向向量: XXXX   # 应该 > 0
     是否水密: True   # 应该是 True
   ```

3. **重新清理并生成**
   ```bash
   rm src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj
   python generate_coal_wall.py ...
   ```

4. **检查材质是否被加载**
   在 Gazebo 中右键点击煤壁 → 查看属性 → Material

## 修改文件清单

✓ `src/hydromining_sim/scripts/generate_coal_wall.py`
  - generate() 方法：添加填孔、法向修复、拓扑优化
  - main() 函数：显式导出法向、验证输出

✓ `src/hydromining_sim/models/coal_wall/model.sdf`
  - material 参数：提高 ambient/diffuse 值

✓ `regenerate.sh`（新建）
  - 便利脚本快速重建

---

**修复完成时间**: 2026-10-21  
**状态**: ✓ 代码修改已完成，待执行和验证
