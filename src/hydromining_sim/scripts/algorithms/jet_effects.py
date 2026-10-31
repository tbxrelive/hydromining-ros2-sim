"""Bounded engineering effects and observation metrics; no ROS dependencies."""
import numpy as np


PRESETS = {'off': 0.0, 'low': 0.4, 'medium': 1.0, 'high': 1.8}


def jet_hydraulics(pressure, diameter=0.002, density=1000., velocity_coefficient=0.95):
    """Nozzle outlet estimates in SI units; coal response remains uncalibrated."""
    velocity = velocity_coefficient * np.sqrt(2. * pressure * 1e6 / density)
    flow = np.pi * diameter ** 2 / 4. * velocity
    return {'pressure_mpa': float(pressure), 'nozzle_diameter_m': float(diameter),
            'outlet_velocity_m_s': float(velocity), 'flow_m3_s': float(flow),
            'hydraulic_power_w': float(pressure * 1e6 * flow)}


def effect_strength(pressure, preset, reference_pressure=20.0):
    """Uncalibrated display mapping, separate from erosion Rc and b (MPa)."""
    if (preset not in PRESETS or not np.isfinite(pressure) or not 0 <= pressure <= 60
            or not np.isfinite(reference_pressure) or not 1 <= reference_pressure <= 60):
        raise ValueError('Require a known preset and pressure in [0,60] MPa')
    return float(min(2.0, PRESETS[preset] * pressure / reference_pressure))


def hit_geometry(simulation, action, rotation, translation, nozzle_offset=0.05):
    """Use the current numerical surface, before the candidate field is committed."""
    import open3d as o3d
    g = simulation.grid
    scene = simulation.model.surface_scene(g.phi, g.origin, g.spacing)
    ray = np.r_[action.position, action.direction].astype(np.float32)[None, :]
    result = scene.cast_rays(o3d.core.Tensor(ray), nthreads=1)
    distance = float(result['t_hit'].numpy()[0])
    valid = np.isfinite(distance) and distance > 1e-6
    direction = rotation @ action.direction
    tcp = rotation @ action.position + translation
    hit = tcp + distance * direction if valid else tcp + 2.0 * direction
    normal = rotation @ result['primitive_normals'].numpy()[0] if valid else -direction
    if valid and np.dot(normal, direction) > 0:
        normal = -normal
    # Legacy actions start at TCP; newer actions can start at the nozzle itself.
    return {'source': (tcp - nozzle_offset * direction).tolist(), 'direction': direction.tolist(),
            'hit': hit.tolist(), 'normal': normal.tolist(), 'has_hit': bool(valid)}


def command_packet(sequence, mode, geometry, strength, eroding, flags, radius, settings=None,
                   outlet_velocity=None):
    """v3 adds the modeled outlet speed; v1/v2 remain usable by old callers."""
    values = [1, sequence, mode, *geometry['source'], *geometry['direction'],
              *geometry['hit'], *geometry['normal'], int(geometry['has_hit']),
              strength, int(eroding), *map(int, flags), radius]
    if len(values) != 23 or not np.isfinite(values).all():
        raise ValueError('Invalid jet command')
    if settings is not None:
        settings = np.asarray(settings, dtype=float)
        if (settings.shape != (6,) or not np.isfinite(settings).all()
                or np.any(settings[:3] < 0) or np.any(settings[:3] > 1)
                or np.any(settings[3:] < .02) or np.any(settings[3:] > .5)):
            raise ValueError('Require three rate scales in [0,1] and three spreads in [.02,.5] m')
        values[0] = 2
        values.extend(settings)
    if outlet_velocity is not None:
        if settings is None or not np.isfinite(outlet_velocity) or outlet_velocity < 0:
            raise ValueError('Outlet velocity requires effect settings and must be nonnegative')
        values[0] = 3
        values.append(float(outlet_velocity))
    return ' '.join(format(float(x), '.12g') for x in values)


def cloud_change(reference, observed, mask, tolerance=0.01):
    """Registered same-ray clouds: partial visibility and disturbed return fraction."""
    a, b = np.asarray(reference), np.asarray(observed)
    mask = np.asarray(mask, dtype=bool) & np.isfinite(a).all(axis=1)
    if mask.sum() < 10:
        raise ValueError('Insufficient clean target returns')
    error = np.linalg.norm(b[mask] - a[mask], axis=1)
    visible = np.isfinite(error) & (error <= tolerance)
    return {'target_returns': int(mask.sum()), 'visible_fraction': float(visible.mean()),
            'disturbed_fraction': float((~visible).mean()),
            'finite_fraction': float(np.isfinite(error).mean())}
