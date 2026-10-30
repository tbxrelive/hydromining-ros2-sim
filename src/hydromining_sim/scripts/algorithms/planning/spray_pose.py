"""阶段七喷射位姿生成：喷嘴沿射流轴指向墙面目标点，解算关节角。

TCP +Z 为射流轴（与 spray_controller 的 use_tcp 方向一致）；喷嘴出口在
TCP 后方 nozzle_offset 处。所有几何在 world 中表达，IK 在机械臂基座系求解。
"""
import numpy as np
from scipy.spatial.transform import Rotation


def aim_rotation(direction, up=(0.0, 0.0, 1.0)):
    """构造 +Z 沿 direction 的姿态；up 投影定 +X 之外的自由度。"""
    z = np.asarray(direction, float)
    if z.shape != (3,) or not np.isfinite(z).all() or np.linalg.norm(z) < 1e-9:
        raise ValueError('jet direction must be a finite nonzero vector')
    z /= np.linalg.norm(z)
    y = np.cross(np.asarray(up, float), z)
    if np.linalg.norm(y) < 1e-6:
        y = np.cross([0.0, 1.0, 0.0], z)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.c_[x, y, z]


def tcp_target_world(target, direction, standoff, nozzle_offset=0.05):
    """喷嘴出口位于 target 前 standoff 处的 TCP world 位姿（4x4）。

    source = tcp - nozzle_offset*direction（与 hit_geometry 一致），
    故 tcp = target - (standoff - nozzle_offset)*direction。
    """
    target = np.asarray(target, float)
    if target.shape != (3,) or not np.isfinite(target).all():
        raise ValueError('target must be three finite coordinates')
    if not np.isfinite(standoff) or standoff <= nozzle_offset:
        raise ValueError('standoff must be finite and exceed the nozzle offset')
    T = np.eye(4)
    T[:3, :3] = aim_rotation(direction)
    T[:3, 3] = target - (standoff - nozzle_offset) * T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return T


def solve_spray_pose(chain, T_world_base, target, direction, standoff, seed,
                     position_tolerance=0.005, rotation_tolerance=0.02):
    """返回 {'joints','reachable','position_error_m','rotation_error_rad','tcp_world'}。

    不保证无自碰撞/环境碰撞；调用方负责核实位姿安全裕度。
    """
    T_world_base = np.asarray(T_world_base, float)
    if T_world_base.shape != (4, 4) or not np.isfinite(T_world_base).all():
        raise ValueError('world to base transform must be a finite 4x4 matrix')
    desired = tcp_target_world(target, direction, standoff)
    joints, position_error, rotation_error = chain.ik(np.linalg.inv(T_world_base) @ desired, seed)
    return {'joints': joints, 'reachable': bool(position_error <= position_tolerance and
                                              rotation_error <= rotation_tolerance),
            'position_error_m': position_error, 'rotation_error_rad': rotation_error,
            'tcp_world': desired}
