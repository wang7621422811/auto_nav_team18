# Copyright 2026 team18
"""Tests for robot.yaml extrinsics parsing."""

from pathlib import Path

import pytest

from auto_nav.robot_extrinsics import (
    load_sensor_xyz_from_files,
    load_sensor_xyz_from_robot_yaml,
    static_transform_arguments,
)


def test_load_sensor_xyz_from_robot_yaml(tmp_path: Path) -> None:
    cfg = tmp_path / 'robot.yaml'
    cfg.write_text(
        '\n'.join(
            [
                '/**:',
                '  ros__parameters:',
                '    laser_x: 0.1',
                '    laser_y: -0.02',
                '    laser_z: 0.3  # hi',
                '    camera_x: 0.25',
                '    camera_y: 0',
                '    camera_z: "0.22"',
                '',
            ]
        ),
        encoding='utf-8',
    )
    out = load_sensor_xyz_from_robot_yaml(cfg)
    assert out['laser'] == (0.1, -0.02, 0.3)
    assert out['camera'] == (0.25, 0.0, 0.22)


def test_load_missing_key(tmp_path: Path) -> None:
    cfg = tmp_path / 'robot.yaml'
    cfg.write_text('laser_x: 1\n', encoding='utf-8')
    with pytest.raises(ValueError, match='missing keys'):
        load_sensor_xyz_from_robot_yaml(cfg)


def test_load_sensor_xyz_from_layered_files(tmp_path: Path) -> None:
    base_cfg = tmp_path / 'robot.yaml'
    override_cfg = tmp_path / 'real.yaml'
    base_cfg.write_text(
        '\n'.join(
            [
                '/**:',
                '  ros__parameters:',
                '    laser_x: 0.20',
                '    laser_y: 0.00',
                '    laser_z: 0.281',
                '    camera_x: 0.24',
                '    camera_y: 0.00',
                '    camera_z: 0.261',
                '',
            ]
        ),
        encoding='utf-8',
    )
    override_cfg.write_text(
        '\n'.join(
            [
                '/**:',
                '  ros__parameters:',
                '    laser_z: 0.295',
                '    camera_x: 0.245',
                '',
            ]
        ),
        encoding='utf-8',
    )

    out = load_sensor_xyz_from_files([base_cfg, override_cfg])
    assert out['laser'] == (0.2, 0.0, 0.295)
    assert out['camera'] == (0.245, 0.0, 0.261)


def test_static_transform_arguments() -> None:
    args = static_transform_arguments((1.0, 2.0, 3.0), child='laser')
    assert args == ['1.0', '2.0', '3.0', '0.0', '0.0', '0.0', 'base_link', 'laser']
