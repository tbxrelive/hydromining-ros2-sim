"""仅依赖 numpy/scipy 的串联运动链正逆运动学，从展开后的 URDF 读取。

只用于离线生成观测视点，不替代 Gazebo/ros2_control。不做自碰撞检测。
"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def _tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = xyz
    return T


class Chain:
    """base→tip 的关节序列，仅支持固定关节和有限位的旋转关节。"""

    def __init__(self, urdf_xml, base, tip):
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            ax = j.find('axis')
            lim = j.find('limit')
            by_child[j.find('child').get('link')] = dict(
                name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
                T=_tf(xyz, rpy), axis=np.array([float(v) for v in (ax.get('xyz') if ax is not None else '0 0 1').split()]),
                lower=float(lim.get('lower', -np.pi)) if lim is not None else -np.pi,
                upper=float(lim.get('upper', np.pi)) if lim is not None else np.pi)
        path, link = [], tip
        visited = set()
        while link != base:
            if link in visited or link not in by_child:
                raise ValueError(f'No kinematic path from {base} to {tip}')
            visited.add(link)
            if by_child[link]['type'] not in ('fixed', 'revolute'):
                raise ValueError('Only fixed and revolute joints are supported')
            path.append(by_child[link])
            link = by_child[link]['parent']
        self.steps_ = path[::-1]
        self.names_ = [s['name'] for s in self.steps_ if s['type'] == 'revolute']
        self.lower_ = np.array([s['lower'] for s in self.steps_ if s['type'] == 'revolute'])
        self.upper_ = np.array([s['upper'] for s in self.steps_ if s['type'] == 'revolute'])

    def fk(self, q):
        """返回 tip 相对 base 的 4x4 变换。"""
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.names_),) or not np.isfinite(q).all():
            raise ValueError('joint vector must match the chain and contain finite values')
        T, i = np.eye(4), 0
        for s in self.steps_:
            T = T @ s['T']
            if s['type'] == 'revolute':
                T = T @ np.block([[Rotation.from_rotvec(s['axis'] * q[i]).as_matrix(), np.zeros((3, 1))],
                                  [np.zeros((1, 3)), np.ones((1, 1))]])
                i += 1
        return T

    def ik(self, target, seed, orientation_weight=0.3, margin=0.05):
        """数值逆解；返回 (关节角, 位置误差 m, 姿态误差 rad)。关节限位内缩 margin。"""
        lo, hi = self.lower_ + margin, self.upper_ - margin

        def residual(q):
            T = self.fk(q)
            dr = Rotation.from_matrix(target[:3, :3] @ T[:3, :3].T).as_rotvec()
            return np.r_[T[:3, 3] - target[:3, 3], orientation_weight * dr]
        sol = least_squares(residual, np.clip(seed, lo, hi), bounds=(lo, hi), xtol=1e-10, ftol=1e-10)
        T = self.fk(sol.x)
        err_r = np.linalg.norm(Rotation.from_matrix(target[:3, :3] @ T[:3, :3].T).as_rotvec())
        return sol.x, float(np.linalg.norm(T[:3, 3] - target[:3, 3])), float(err_r)


def look_at(position, target, up=(0.0, 0.0, 1.0)):
    """相机 +X 指向 target、+Z 尽量朝上的位姿（Gazebo 相机沿 +X 看）。"""
    x = np.asarray(target, float) - np.asarray(position, float)
    x /= np.linalg.norm(x)
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    T = np.eye(4)
    T[:3, :3] = np.c_[x, y, z]
    T[:3, 3] = position
    return T
