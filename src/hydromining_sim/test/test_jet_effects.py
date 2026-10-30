"""Effects must remain bounded, independent from physical erosion parameters."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from algorithms.jet_effects import effect_strength, command_packet, cloud_change


def test_pressure_mapping_is_bounded_and_separate():
    """Saturating pressure mapping and independently ordered presets."""
    for preset in ('off', 'low', 'medium', 'high'):
        values = [effect_strength(p, preset) for p in (0, 5, 10, 20, 40, 60)]
        assert values == sorted(values) and min(values) >= 0 and max(values) <= 2
    assert effect_strength(20, 'off') == 0
    assert effect_strength(20, 'low') < effect_strength(20, 'medium') < effect_strength(20, 'high')
    for pressure in (-1, np.nan, 61):
        with pytest.raises(ValueError):
            effect_strength(pressure, 'medium')


def test_nozzle_command_does_not_overload_erosion_radius():
    geometry = {'source': [0,0,0], 'direction': [0,0,1], 'hit': [0,0,1],
                'normal': [0,0,-1], 'has_hit': True}
    data = list(map(float, command_packet(4, 1, geometry, 1, True, [True]*4, .008).split()))
    assert len(data) == 23 and data[:3] == [1,4,1] and data[-1] == .008
    assert data[15:18] == [1,1,1]


def test_configured_effects_keep_native_particle_budget():
    """v2 tuning cannot increase the declared maximum particle count."""
    geometry = {'source':[0,0,0], 'direction':[0,0,1], 'hit':[0,0,1],
                'normal':[0,0,-1], 'has_hit':True}
    settings = [.5, .25, 0., .24, .15, .20]
    packet = list(map(float, command_packet(5,1,geometry,1,True,[True]*4,.008,settings).split()))
    assert len(packet)==29 and packet[0]==2 and packet[23:]==settings
    for settings in ([1.1,1,1,.24,.1,.2], [1,1,1,0,.1,.2], [1,1,1,.24,.1,np.nan]):
        with pytest.raises(ValueError):
            command_packet(5,1,geometry,1,True,[True]*4,.008,settings)
    assert effect_strength(10,'medium',10)==effect_strength(20,'medium',20)
    assert effect_strength(60,'high',1)==2
    with pytest.raises(ValueError):
        effect_strength(20,'medium',0)


def test_missing_and_outlier_returns_reduce_visibility():
    reference = np.tile([1.,0.,0.], (100,1))
    observed = reference.copy()
    observed[:20] = np.nan
    observed[20:40,0] += .1
    result = cloud_change(reference, observed, np.ones(100, dtype=bool))
    assert result['visible_fraction'] == .6 and result['disturbed_fraction'] == .4


def test_mist_border_is_premultiplied_and_lifetime_fades():
    """Transparent texels must not add white squares with Ogre's ONE blend factor."""
    from PIL import Image
    textures = Path(__file__).parents[1] / 'models/water_jet/materials/textures'
    sprite = np.asarray(Image.open(textures / 'mist.png'))
    assert np.all(sprite[sprite[:,:,3] == 0,:3] == 0)
    assert 0 < sprite[:,:,3].max() < 128
    fade = np.asarray(Image.open(textures / 'fade.png'))
    assert np.all(np.diff(fade[0,:,3].astype(int)) <= 0)
    assert fade[0,0,3] == 255 and np.all(fade[0,-1] == 0)
