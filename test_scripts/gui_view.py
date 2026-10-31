"""Actual Gazebo GUI camera service and rendered screenshot evidence (gz-gui8)."""
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from algorithms.perception.kinematics import look_at


def write_scene_world(source, output, coal_pose=None, low_light=False, status_panel=False):
    """Create the requested world from the existing Harmonic scene."""
    world = ET.parse(source)
    if coal_pose is not None:
        world.find('./world/include/pose').text = ' '.join(map(str, coal_pose))
    if low_light:
        world.find('./world/scene/ambient').text = '.04 .04 .04 1'
        world.find('./world/scene/background').text = '.03 .03 .03 1'
        for light in world.findall('./world/light'):
            light.find('diffuse').text = '.08 .08 .08 1'
            light.find('specular').text = '.02 .02 .02 1'
    if status_panel:
        config = (Path.home()/'.gz/sim/8/gui.config').read_text()
        defaults = ET.fromstring('<root>'+config.split('?>', 1)[-1]+'</root>')
        gui = ET.SubElement(world.find('./world'), 'gui', {'fullscreen': '0'})
        for plugin in defaults.findall('plugin'):
            gui.append(plugin)
        plugin = ET.SubElement(gui, 'plugin', {'filename': 'HydrominingStatus', 'name': 'Hydromining status'})
        settings = ET.SubElement(plugin, 'gz-gui')
        ET.SubElement(settings, 'property', {'key': 'state', 'type': 'string'}).text = 'floating'
        ET.SubElement(settings, 'property', {'key': 'width', 'type': 'double'}).text = '340'
        ET.SubElement(settings, 'property', {'key': 'height', 'type': 'double'}).text = '220'
        ET.SubElement(settings, 'property', {'key': 'y', 'type': 'double'}).text = '90'
    world.write(output, encoding='utf-8', xml_declaration=True)


def request(service, kind, message):
    result = subprocess.run(['gz','service','-s',service,'--reqtype',kind,'--reptype',
                             'gz.msgs.Boolean','--timeout','5000','--req',message],
                            capture_output=True,text=True,timeout=8)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError(f'GUI service {service}: {result.stdout} {result.stderr}')


def capture_gui(output, center, label='final', move=True):
    """Move camera close to the task region and require a real PNG to appear."""
    output=Path(output)/'gui'/label;output.mkdir(parents=True,exist_ok=True)
    if move:
        center=np.asarray(center,float)
        offset = [-1.,-.65,.4] if label in ('before','spraying') else [-.55,-.3,.22]
        pose=look_at(center+np.array(offset),center)
        q=Rotation.from_matrix(pose[:3,:3]).as_quat();p=pose[:3,3]
        message='pose {position {x:%g y:%g z:%g} orientation {x:%g y:%g z:%g w:%g}}'%(*p,*q)
        request('/gui/move_to/pose','gz.msgs.GUICamera',message)
        # CameraTracking animates the view over roughly one second.
        time.sleep(1.5)
    before=set(output.glob('*.png'))
    request('/gui/screenshot','gz.msgs.StringMsg','data: '+__import__('json').dumps(str(output.resolve())))
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        files=set(output.glob('*.png'))-before
        if files:
            return {'screenshot':str(max(files,key=lambda p:p.stat().st_mtime)), 'actual_gui_render':True}
        time.sleep(.1)
    raise TimeoutError('GUI accepted screenshot request but no PNG was rendered')
