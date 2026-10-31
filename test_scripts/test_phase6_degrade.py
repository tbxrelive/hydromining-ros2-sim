"""离线数据生成的来源保护、标签隔离、全因子与可复现回归。"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from phase6_degrade import generate, read_config, sha256
from audit_phase6_environment import audit


@pytest.fixture
def recording(tmp_path):
    source = tmp_path / 'source'
    frames = source / 'dataset/baseline'
    frames.mkdir(parents=True)
    (source / 'summary.json').write_text(json.dumps(
        {'passed': True, 'noise': 'off', 'effects': 'off'}))
    metadata = {
        'depth': np.ones((3, 4), np.float32), 'K': np.eye(3),
        'lidar_local': np.array([[1., 0, 0], [0, 1., 0]], np.float32),
        'depth_stamp': np.array([3, 5]), 'lidar_stamp': np.array([3, 6]),
        'tf_depth': np.array([0, 0, 0, 0, 0, 0, 1.]),
        'tf_lidar': np.array([0, 0, 0, 0, 0, 0, 1.]),
        'depth_frame': 'camera', 'lidar_frame': 'lidar', 'joints': np.zeros(7),
    }
    np.savez_compressed(frames / 'frame.npz', **metadata)
    config = tmp_path / 'config.yaml'
    config.write_text(yaml.safe_dump({
        'schema_version': 1, 'seed': 8, 'environments': ['clean', 'dim_dust_heavy'],
        'noise_scales': [0.0, 1.0], 'r_min_m': 0.1}))
    return source, config


def test_generate_reproducible_matrix_preserves_sources_and_metadata(recording, tmp_path):
    source, config = recording
    before = {str(p): sha256(p) for p in source.rglob('*') if p.is_file()}
    first, second = tmp_path / 'first', tmp_path / 'second'
    report = generate(source, first, config)
    generate(source, second, config)
    assert audit(first)['passed']
    assert report['completed'] and report['case_count'] == 4
    assert report['generated_snapshot_count'] == 4
    assert before == {str(p): sha256(p) for p in source.rglob('*') if p.is_file()}
    manifest = json.loads((first / 'manifest.json').read_text())
    assert manifest['calibrated'] is False
    assert manifest['source_start_provenance_available'] is False
    for p in first.rglob('*.npz'):
        with np.load(p) as a, np.load(second / p.relative_to(first)) as b:
            assert set(a.files) == set(b.files)
            for key in a.files:
                np.testing.assert_array_equal(a[key], b[key])
    with np.load(source / 'dataset/baseline/frame.npz') as raw:
        with np.load(first / 'clean_noise0/observations/baseline/frame.npz') as clean:
            assert set(raw.files) == set(clean.files)
            for key in raw.files:
                np.testing.assert_array_equal(raw[key], clean[key])
        with np.load(first / 'dim_dust_heavy_noise1/observations/baseline/frame.npz') as dust:
            assert not any('label' in key for key in dust.files)
            for key in raw.files:
                if key not in ('depth', 'lidar_local'):
                    np.testing.assert_array_equal(raw[key], dust[key])
    with pytest.raises(ValueError, match='empty'):
        generate(source, first, config)


@pytest.mark.parametrize('noise,effects,passed', [
    ({'depth': 0.01, 'lidar': 0.0}, 'off', True),
    ('off', 'on', True), ('off', 'off', False), (None, 'off', True)])
def test_reject_nonclean_source(recording, tmp_path, noise, effects, passed):
    source, config = recording
    (source / 'summary.json').write_text(json.dumps(
        {'noise': noise, 'effects': effects, 'passed': passed}))
    with pytest.raises(ValueError, match='source'):
        generate(source, tmp_path / 'out', config)
    assert not (tmp_path / 'out').exists()


def test_reject_output_inside_source(recording):
    source, config = recording
    with pytest.raises(ValueError, match='disjoint'):
        generate(source, source / 'derived', config)
    assert not (source / 'derived').exists()


def test_audit_detects_source_and_observation_tampering(recording, tmp_path):
    source, config = recording
    output = tmp_path / 'out'
    generate(source, output, config)
    observation = output / 'clean_noise0/observations/baseline/frame.npz'
    original = observation.read_bytes()
    observation.write_bytes(b'corrupted derived data')
    with pytest.raises(ValueError, match='derived'):
        audit(output)
    observation.write_bytes(original)
    (source / 'summary.json').write_text('{}')
    with pytest.raises(ValueError, match='source changed'):
        audit(output)


def test_audit_rejects_incomplete_matrix(recording, tmp_path):
    source, config = recording
    output = tmp_path / 'out'
    generate(source, output, config)
    path = output / 'summary.json'
    summary = json.loads(path.read_text())
    summary['cases'].pop()
    path.write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='experiment cases'):
        audit(output)


@pytest.mark.parametrize('key,value', [
    ('seed', -1), ('seed', True), ('environments', []),
    ('environments', ['unknown']), ('environments', ['clean', 'clean']),
    ('noise_scales', [-1.0]), ('noise_scales', [float('nan')]),
    ('noise_scales', [0, 0]), ('noise_scales', [True]), ('r_min_m', 0)])
def test_invalid_config_rejected(recording, key, value):
    _, config = recording
    values = yaml.safe_load(config.read_text())
    values[key] = value
    config.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError):
        read_config(config)
