#!/usr/bin/env python3
"""
生成喷射前后对比的 HTML 可视化页面

使用 Three.js 在浏览器中查看 3D 模型
无需 OpenGL，适合 SSH 远程环境
"""

import json
from pathlib import Path

def obj_to_json(obj_path: str) -> dict:
    """将 OBJ 文件转换为 JSON 格式"""
    vertices = []
    faces = []

    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()[1:]
                vertices.extend([float(x) for x in parts])
            elif line.startswith('f '):
                parts = line.strip().split()[1:]
                # 处理 OBJ 格式：可能是 "v", "v/vt", "v/vt/vn", "v//vn"
                for part in parts:
                    vertex_index = int(part.split('/')[0]) - 1  # OBJ 索引从1开始
                    faces.append(vertex_index)

    return {
        'vertices': vertices,
        'faces': faces
    }


def generate_html():
    """生成 HTML 可视化页面"""

    # 文件路径
    mesh_before = "src/hydromining_sim/models/coal_wall/meshes/coal_wall.obj"
    mesh_after = "/tmp/phase4_visualization/coal_wall_after_spray.obj"

    if not Path(mesh_before).exists():
        print(f"❌ 找不到文件: {mesh_before}")
        return 1

    if not Path(mesh_after).exists():
        print(f"❌ 找不到文件: {mesh_after}")
        print(f"   请先运行: python3 test_scripts/test_phase4_integration_mini.py")
        return 1

    print("加载模型...")
    data_before = obj_to_json(mesh_before)
    data_after = obj_to_json(mesh_after)

    print(f"  喷射前: {len(data_before['vertices'])//3} 顶点")
    print(f"  喷射后: {len(data_after['vertices'])//3} 顶点")

    # 生成 HTML
    html_content = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>煤壁喷射前后对比 - 3D 可视化</title>
    <style>
        :root {{
            --bg-dark: #0A0D12;
            --bg-mid: #161D2B;
            --text-light: #E5E7EB;
            --accent-cyan: #38BDF8;
        }}
        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }}
        body {{
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
            background: var(--bg-dark);
            color: var(--text-light);
            line-height: 1.6;
        }}
        .header {{
            padding: 1.5rem 2rem;
            background: var(--bg-mid);
            border-bottom: 1px solid rgba(56, 189, 248, 0.2);
        }}
        h1 {{
            font-size: clamp(1.25rem, 2.5vw, 1.75rem);
            font-weight: 600;
            letter-spacing: -0.02em;
        }}
        .subtitle {{
            margin-top: 0.5rem;
            font-size: 0.875rem;
            color: #9CA3AF;
        }}
        #canvas-container {{
            position: relative;
            width: 100%;
            height: calc(100vh - 160px);
        }}
        .controls {{
            padding: 1rem 2rem;
            background: var(--bg-mid);
            display: flex;
            gap: 1rem;
            flex-wrap: wrap;
            align-items: center;
        }}
        .btn {{
            padding: 0.5rem 1rem;
            background: rgba(56, 189, 248, 0.1);
            border: 1px solid var(--accent-cyan);
            color: var(--accent-cyan);
            border-radius: 999px;
            cursor: pointer;
            font-size: 0.875rem;
            transition: all 0.2s;
        }}
        .btn:hover {{
            background: rgba(56, 189, 248, 0.2);
        }}
        .info {{
            margin-left: auto;
            font-size: 0.75rem;
            color: #6B7280;
        }}
        .legend {{
            position: absolute;
            top: 1rem;
            right: 1rem;
            background: rgba(10, 13, 18, 0.9);
            padding: 1rem;
            border-radius: 8px;
            border: 1px solid rgba(56, 189, 248, 0.2);
            font-size: 0.875rem;
        }}
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 0.5rem;
            margin-bottom: 0.5rem;
        }}
        .legend-color {{
            width: 1rem;
            height: 1rem;
            border-radius: 3px;
        }}
    </style>
</head>
<body>
    <div class="header">
        <h1>煤壁喷射前后对比 - 3D 可视化</h1>
        <div class="subtitle">阶段4：喷射效果模拟 | Level Set 方法 + Marching Cubes</div>
    </div>

    <div id="canvas-container">
        <div class="legend">
            <div class="legend-item">
                <div class="legend-color" style="background: #9CA3AF;"></div>
                <span>喷射前煤壁（左侧）</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #EF4444;"></div>
                <span>喷射后煤壁（右侧）</span>
            </div>
        </div>
    </div>

    <div class="controls">
        <button class="btn" onclick="resetCamera()">重置视角</button>
        <button class="btn" onclick="toggleRotation()">自动旋转</button>
        <button class="btn" onclick="toggleWireframe()">线框模式</button>
        <span class="info">鼠标左键拖动旋转 | 滚轮缩放 | 右键平移</span>
    </div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>

    <script>
        // 数据
        const dataBefore = {json.dumps(data_before)};
        const dataAfter = {json.dumps(data_after)};

        // 场景设置
        const container = document.getElementById('canvas-container');
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x0A0D12);

        const camera = new THREE.PerspectiveCamera(
            60,
            container.clientWidth / container.clientHeight,
            0.1,
            1000
        );
        camera.position.set(5, 3, 5);

        const renderer = new THREE.WebGLRenderer({{ antialias: true }});
        renderer.setSize(container.clientWidth, container.clientHeight);
        container.appendChild(renderer.domElement);

        const controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;

        // 光照
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);

        const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight1.position.set(5, 5, 5);
        scene.add(dirLight1);

        const dirLight2 = new THREE.DirectionalLight(0xffffff, 0.4);
        dirLight2.position.set(-5, 3, -5);
        scene.add(dirLight2);

        // 创建 Mesh
        function createMesh(data, color, offset) {{
            const geometry = new THREE.BufferGeometry();

            const vertices = new Float32Array(data.vertices);
            const indices = new Uint32Array(data.faces);

            geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
            geometry.setIndex(new THREE.BufferAttribute(indices, 1));
            geometry.computeVertexNormals();

            const material = new THREE.MeshPhongMaterial({{
                color: color,
                shininess: 30,
                flatShading: false
            }});

            const mesh = new THREE.Mesh(geometry, material);
            mesh.position.x = offset;
            return mesh;
        }}

        const meshBefore = createMesh(dataBefore, 0x9CA3AF, -2);
        const meshAfter = createMesh(dataAfter, 0xEF4444, 2);

        scene.add(meshBefore);
        scene.add(meshAfter);

        // 坐标轴
        const axesHelper = new THREE.AxesHelper(1);
        scene.add(axesHelper);

        // 动画
        let autoRotate = false;
        function animate() {{
            requestAnimationFrame(animate);

            if (autoRotate) {{
                meshBefore.rotation.y += 0.005;
                meshAfter.rotation.y += 0.005;
            }}

            controls.update();
            renderer.render(scene, camera);
        }}
        animate();

        // 控制函数
        function resetCamera() {{
            camera.position.set(5, 3, 5);
            controls.reset();
        }}

        function toggleRotation() {{
            autoRotate = !autoRotate;
        }}

        function toggleWireframe() {{
            meshBefore.material.wireframe = !meshBefore.material.wireframe;
            meshAfter.material.wireframe = !meshAfter.material.wireframe;
        }}

        // 响应窗口调整
        window.addEventListener('resize', () => {{
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        }});
    </script>
</body>
</html>
"""

    # 保存 HTML
    output_path = Path("/tmp/phase4_visualization/comparison.html")
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html_content)

    print(f"\n✅ HTML 已生成: {output_path}")
    print(f"\n打开方式:")
    print(f"  1. 在浏览器中打开: file://{output_path}")
    print(f"  2. 或复制到本地: cp {output_path} ~/")
    print(f"  3. 或启动 HTTP 服务器:")
    print(f"     cd /tmp/phase4_visualization && python3 -m http.server 8080")
    print(f"     然后访问: http://localhost:8080/comparison.html")

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(generate_html())
