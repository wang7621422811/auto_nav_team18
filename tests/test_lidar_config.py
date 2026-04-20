# Copyright 2026 team18
"""Configuration tests for LiDAR driver parameter files."""

from pathlib import Path

import yaml


_ROOT = Path(__file__).resolve().parents[1]


def _load_yaml(relpath: str) -> dict:
    with (_ROOT / relpath).open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def test_lidar_driver_params_match_runtime_node_names() -> None:
    cfg = _load_yaml('config/lidar.yaml')

    assert 'sick_scan' in cfg
    assert 'lakibeam_scan' in cfg

    sick_params = cfg['sick_scan']['ros__parameters']
    lakibeam_params = cfg['lakibeam_scan']['ros__parameters']

    assert sick_params['frame_id'] == 'laser'
    assert sick_params['scanner_type']
    assert 'hostname' in sick_params
    assert 'port' in sick_params
    assert 'sick' not in sick_params

    assert lakibeam_params['frame_id'] == 'laser'
    assert 'hostname' in lakibeam_params
    assert 'port' in lakibeam_params
    assert 'lakibeam' not in lakibeam_params


def test_real_overrides_target_runtime_lidar_nodes() -> None:
    cfg = _load_yaml('config/real.yaml')

    assert 'sick_scan' in cfg
    assert 'lakibeam_scan' in cfg

    sick_params = cfg['sick_scan']['ros__parameters']
    lakibeam_params = cfg['lakibeam_scan']['ros__parameters']

    assert sick_params['frame_id'] == 'laser'
    assert sick_params['scanner_type']
    assert lakibeam_params['frame_id'] == 'laser'
