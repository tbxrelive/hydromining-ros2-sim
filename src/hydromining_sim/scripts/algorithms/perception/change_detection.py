"""喷前后固定坐标系变化与有效观测体积；不对变化表面做 ICP。"""
import numpy as np


def compare_surfaces(baseline, current, cell_area, registration_sigma=0.001,
                     confidence_k=1.645):
    """传播独立测量及配准误差；未可靠观测区域始终为未知。"""
    if cell_area <= 0 or registration_sigma < 0 or confidence_k <= 0:
        raise ValueError('invalid change detection parameters')
    if baseline['height'].shape != current['height'].shape:
        raise ValueError('baseline and current grids must match')
    valid = (baseline['coverage'] == 2) & (current['coverage'] == 2)
    valid &= np.isfinite(baseline['sigma']) & np.isfinite(current['sigma'])
    valid &= np.isfinite(baseline['height']) & np.isfinite(current['height'])
    valid &= (baseline['sigma'] >= 0) & (current['sigma'] >= 0)
    delta = np.where(valid, baseline['height'] - current['height'], np.nan)
    sigma = np.where(valid, np.sqrt(baseline['sigma'] ** 2 + current['sigma'] ** 2 +
                                   registration_sigma ** 2), np.nan)
    significant = valid & (delta - confidence_k * sigma > 0)
    return {'delta': delta, 'sigma': sigma, 'valid': valid, 'significant': significant}


def volume_summary(change, area_weights, coal_density=1400., confidence_k=1.96):
    """仅积分可靠重叠区域；以完全相关误差上界给体积区间，不用格数稀释共模误差。"""
    area = np.asarray(area_weights, float)
    if coal_density <= 0 or not np.isfinite(coal_density) or np.any(area < 0):
        raise ValueError('invalid density or area weights')
    valid = change['valid'] & (area > 0)
    volume = float(np.sum(np.maximum(change['delta'][valid], 0) * area[valid]))
    sigma = float(np.sum(change['sigma'][valid] * area[valid]))
    return {
        'removed_volume_m3': volume, 'volume_sigma_m3': sigma,
        'volume_lower_m3': max(0., volume - confidence_k * sigma),
        'volume_upper_m3': volume + confidence_k * sigma,
        'removed_mass_kg': volume * coal_density,
        'observed_area_m2': float(area[valid].sum()),
        'unobserved_area_m2': float(area[~valid].sum()),
        'density_kg_m3': coal_density, 'density_calibrated': False,
        'scope': 'positive retreat in reliably observed area only',
        'volume_uncertainty': 'fully correlated conservative bound',
    }


def external_change_alarm(change, region_fraction, cell_area, minimum_retreat=0.01,
                          confidence_k=1.645):
    """区域外显著变化单独告警，不改变区域内的合格面积。"""
    outside = 1 - np.asarray(region_fraction, float)
    changed = (change['valid'] & (outside > 0) &
               (change['delta'] - confidence_k * change['sigma'] >= minimum_retreat))
    return {'triggered': bool(changed.any()),
            'area_m2': float(np.sum(outside[changed]) * cell_area),
            'maximum_retreat_m': float(np.max(change['delta'][changed])) if changed.any() else 0.,
            'cells': np.argwhere(changed).tolist()}
