# 鱼香ROS代码规范总结

**来源**: 动手学ROS2 (d2l-ros2) - fishros  
**提取日期**: 2025-01-23  
**适用范围**: ROS 2 Humble/Jazzy 项目

---

## 📚 文档组织结构

### 三层架构模式
鱼香ROS采用**基础篇-入门篇-进阶篇**三层结构组织每个章节：

```
章节X/
├── basic/          # 基础篇：理论知识、概念解释
├── get_started/    # 入门篇：实战代码、快速上手
└── advanced/       # 进阶篇：深入原理、高级用法
```

**应用到我们项目**：
- `docs/阶段文档/` → 按阶段组织（已有）
- `docs/技术笔记/` → 对应"入门篇"（实战经验）
- `docs/参考资料/` → 对应"基础篇"+"进阶篇"

### 代码模板库（Codebook）
专门建立 `codebook/` 目录，存放可复制粘贴的代码模板：

```
codebook/
├── rclcpp/         # C++ 节点模板
├── rclpy/          # Python 节点模板
├── pages/          # Launch、CMake 模板
└── urdf/           # URDF/Xacro 模板
```

**建议新增**：`docs/codebook/` 或 `.claude/templates/`

---

## 🐍 Python 节点编写规范

### 1. 文件命名
- **节点文件**: `<功能>_<序号>.py`  
  例: `topic_publisher_02.py`, `topic_subscribe_02.py`
- **脚本文件**: `<描述性名称>.py`  
  例: `test_motion.py`, `generate_from_preset.py`

### 2. 标准节点结构（面向对象）

```python
#!/usr/bin/env python3
"""
模块文档字符串：简短描述节点功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NodePublisher02(Node):
    """类文档字符串：详细描述节点用途"""
    
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        
        # 创建发布者
        self.command_publisher_ = self.create_publisher(String, "command", 10) 
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        """
        定时器回调函数
        """
        msg = String()
        msg.data = 'backup'
        self.command_publisher_.publish(msg) 
        self.get_logger().info(f'发布了指令：{msg.data}')

def main(args=None):
    rclpy.init(args=args)              # 初始化rclpy
    node = NodePublisher02("topic_publisher_02")  # 新建一个节点
    rclpy.spin(node)                   # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()                   # 关闭rclpy

if __name__ == '__main__':
    main()
```

### 3. 命名规范

| 类型 | 规范 | 示例 |
|------|------|------|
| **类名** | PascalCase | `NodePublisher02`, `PandaMotionTest` |
| **函数/方法** | snake_case | `timer_callback`, `command_callback` |
| **成员变量** | snake_case + 后缀`_` | `command_publisher_`, `subscription_` |
| **话题名** | snake_case | `"command"`, `"/joint_states"` |

### 4. 日志输出规范
```python
# ✓ 良好的日志习惯
self.get_logger().info("大家好，我是%s!" % name)
self.get_logger().info(f'发布了指令：{msg.data}')
self.get_logger().error(f'错误: {e}')

# 使用 f-string 或 % 格式化，保持一致性
```

### 5. 订阅者回调模式
```python
def __init__(self, name):
    super().__init__(name)
    # 创建订阅者
    self.command_subscribe_ = self.create_subscription(
        String,
        "command",
        self.command_callback,  # 回调函数
        10                      # QoS 深度
    )

def command_callback(self, msg):
    """回调函数必须有文档字符串"""
    speed = 0.0
    if msg.data == "backup":
        speed = -0.2
    self.get_logger().info(f'收到[{msg.data}]命令，发送速度{speed}')
```

### 6. 关闭处理（Shutdown Handling）
```python
def main(args=None):
    rclpy.init(args=args)
    node = NodePublisher02("node_name")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # 清理发布者/订阅者
        rclpy.shutdown()
```

---

## 🚀 Launch 文件规范

### 1. 标准结构
```python
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 包路径获取
    pkg_name = get_package_share_directory('package_name')
    
    # 2. 参数声明
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 3. 文件路径
    urdf_file = os.path.join(pkg_name, 'urdf', 'robot.urdf.xacro')
    
    # 4. 节点定义
    node = Node(
        name='node_runtime_name',
        package='ros2_package_name',
        executable='name_of_executable',
        parameters=[{'name_of_int_param': 1,
                     'name_of_str_param': 'value'}],
        remappings=[('from', 'to')],
        output='screen',
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        node,
    ])
```

### 2. 从文件加载参数
```python
node_params = os.path.join(
    get_package_share_directory('package_name'),
    'config',
    'params.yaml'
)

Node(
    name='node_runtime_name',
    package='ros2_package_name',
    executable='name_of_executable',
    parameters=[{'another_param': 42.0},
                 node_params]
)
```

### 3. 包含其他 Launch 文件
```python
my_launch_file = os.path.join(
    get_package_share_directory('package_name'),
    'launch',
    'my_launch.launch.py'
)

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([my_launch_file])
),
```

---

## 🛠️ CMakeLists.txt 规范

### 1. 标准模板（ament_cmake）
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package_name)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 包含头文件
include_directories(include)

# 创建可执行文件
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node
  rclcpp
)

# 安装目录
install(DIRECTORY
  models
  urdf
  config
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行文件
install(TARGETS my_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 脚本
install(
  PROGRAMS
    scripts/my_script.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
```

### 2. 关键要点
- ✓ **依赖声明顺序**: `ament_cmake` 必须在最前
- ✓ **安装目标**: 所有资源必须 `install` 到 `share/${PROJECT_NAME}`
- ✓ **Python 脚本**: 使用 `PROGRAMS` 而非 `FILES`，保持可执行权限

---

## 📦 package.xml 规范

### 1. 标准结构
```xml
<?xml version="1.0"?>
<package format="3">
  <name>hydromining_robot</name>
  <version>0.1.0</version>
  <description>Hydromining robot with water gun for coal wall spraying</description>

  <maintainer email="tbxrelive@github.com">Star</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 运行时和构建时都需要 -->
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <!-- 仅运行时需要 -->
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 2. 依赖类型选择
| 依赖类型 | 使用场景 | 示例 |
|----------|----------|------|
| `<depend>` | 构建和运行时都需要 | `rclcpp`, `std_msgs` |
| `<build_depend>` | 仅构建时需要 | 消息生成工具 |
| `<exec_depend>` | 仅运行时需要 | `launch`, `launch_ros` |
| `<buildtool_depend>` | 构建工具 | `ament_cmake` |

---

## 🏗️ 项目结构规范

### 1. Python 包结构（ament_python）
```
example_py/
├── example_py/           # Python 模块目录
│   ├── __init__.py
│   ├── node_02.py
│   └── topic_publisher_02.py
├── package.xml
├── resource/
│   └── example_py
├── setup.cfg
├── setup.py             # 入口点配置
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

### 2. setup.py 配置
```python
from setuptools import setup

package_name = 'example_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Star',
    maintainer_email='tbxrelive@github.com',
    description='Example Python package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            "node_02 = example_py.node_02:main",
            "topic_publisher_02 = example_py.topic_publisher_02:main"
        ],
    },
)
```

### 3. C++ 包结构（ament_cmake）
```
example_cpp/
├── include/
│   └── example_cpp/
│       └── my_header.hpp
├── src/
│   ├── my_node.cpp
│   └── my_library.cpp
├── launch/
├── config/
├── CMakeLists.txt
└── package.xml
```

---

## 🔧 Colcon 构建规范

### 1. 常用命令
```bash
# 基础构建
colcon build

# 开发模式（推荐）
colcon build --symlink-install

# 清理 CMake 缓存（解决大部分构建问题）
colcon build --cmake-clean-cache

# 构建单个包
colcon build --packages-select <package_name>

# 添加 CMake 参数
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 2. 测试命令
```bash
# 运行测试
colcon test

# 显示详细结果
colcon test-result --verbose

# 测试单个包
colcon test --packages-select <package_name>
```

### 3. 输出格式化
```bash
# 将输出显示到屏幕
colcon build --event-handlers console_direct+
```

---

## 💬 注释与文档规范

### 1. Python 文档字符串
```python
#!/usr/bin/env python3
"""
模块级文档字符串：描述文件整体功能
可以多行描述
"""

class MyNode(Node):
    """类文档字符串：描述节点作用和设计意图"""
    
    def callback(self, msg):
        """
        方法文档字符串：描述函数功能
        
        Args:
            msg: 消息类型和含义
        
        Returns:
            返回值说明（如果有）
        """
        pass
```

### 2. 关键位置注释
```python
# ✓ 在关键步骤前加注释
# 创建订阅者
self.subscription_ = self.create_subscription(...)

# 关键：必须提供 velocities，否则控制器会拒绝执行
point.velocities = [0.0] * 7
```

### 3. 避免过度注释
```python
# ❌ 不要写显而易见的注释
x = 5  # 设置 x 为 5

# ✓ 解释"为什么"而非"是什么"
x = 5  # 传感器数量限制为 5 个
```

---

## 🎯 代码风格要点

### 1. 一致性原则
- **命名风格**: 全项目统一使用 snake_case（Python）或 camelCase（C++）
- **缩进**: Python 4 空格，C++ 2 空格（遵循 ROS 2 风格）
- **字符串格式化**: 统一使用 f-string 或 % 格式化

### 2. 可读性优先
```python
# ✓ 清晰的变量名
joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# ❌ 模糊的缩写
jp = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```

### 3. 模块化设计
- 每个节点一个文件
- 复杂逻辑提取为独立函数/类
- 配置参数使用 YAML 文件而非硬编码

---

## 📚 学习资源引用方式

### 在代码注释中引用
```python
"""
参考资料：
- 鱼香ROS《动手学ROS2》第3章：话题通信
- ROS2官方文档：http://docs.ros.org/en/humble/
"""
```

### 在文档中引用
```markdown
## 参考资料

- [动手学ROS2 - 鱼香ROS](https://fishros.com/d2lros2/)
- [ROS2 Cookbook](https://github.com/mikeferguson/ros2_cookbook)
```

---

## ✅ 应用建议

### 对当前项目的改进
1. **新增 `docs/codebook/`**：存放常用代码模板
2. **规范化节点命名**：统一使用 `<功能>_<版本>.py` 格式
3. **完善文档字符串**：所有节点类和回调函数添加文档
4. **Launch 文件注释**：每个 Node 定义前加注释说明
5. **setup.py 规范化**：保持与 package.xml 信息一致

### 新节点开发检查清单
- [ ] 使用面向对象方式（继承 `Node`）
- [ ] 类名使用 PascalCase，成员变量加 `_` 后缀
- [ ] 添加类和方法的文档字符串
- [ ] 日志使用 `self.get_logger().info/error()`
- [ ] 在 `setup.py` 中注册 `entry_points`
- [ ] 运行前执行 `colcon build --symlink-install`

---

**最后更新**: 2025-01-23  
**维护者**: Star (@tbxrelive)  
**来源**: d2l-ros2 项目分析
