"""阶段六噪声开关必须默认无影响，并且只在目标传感器中注入 SDF noise。"""
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[1]
URDF = WORKSPACE / 'src/hydromining_robot/urdf/panda_with_water_gun.urdf.xacro'


def expand(*arguments):
    """使用实际 xacro 展开整个机器人，而非只解析传感器片段。"""
    result = subprocess.run(['xacro', str(URDF), 'name:=panda', 'arm_id:=panda', *arguments],
                            capture_output=True, text=True, timeout=30, check=True)
    return ET.fromstring(result.stdout)


def normalized(root):
    """忽略缩进空白，只比较 URDF 元素、属性和有效文本。"""
    for element in root.iter():
        element.text = (element.text or '').strip()
        element.tail = ''
    return ET.tostring(root)


def test_noise_off_by_default_and_explicit_zero_is_identical():
    default = expand()
    zero = expand('depth_noise_stddev:=0', 'lidar_noise_stddev:=0')
    assert not default.findall('.//noise')
    assert ET.tostring(default) == ET.tostring(zero)


def test_noise_only_changes_noise_elements_in_rgbd_and_lidar():
    base = expand()
    noisy = expand('depth_noise_stddev:=0.01', 'lidar_noise_stddev:=0.02')
    camera = noisy.find(".//sensor[@name='rgbd_camera']/camera")
    lidar = noisy.find(".//sensor[@name='lidar_sensor']/lidar")
    assert camera.find('noise/type').text == 'gaussian'
    assert float(camera.find('noise/stddev').text) == 0.01
    assert float(lidar.find('noise/stddev').text) == 0.02
    for parent in (camera, lidar):
        parent.remove(parent.find('noise'))
    assert normalized(base) == normalized(noisy)
