# Copyright 2026 team18
"""Tests for lightweight ROS parameter parsing helpers."""

from pathlib import Path

import pytest

from auto_nav.config_params import load_ros_param_from_files


def test_load_ros_param_from_layered_files_prefers_later_value(tmp_path: Path) -> None:
    base_cfg = tmp_path / 'lidar.yaml'
    override_cfg = tmp_path / 'real.yaml'

    base_cfg.write_text(
        '\n'.join(
            [
                'sick_scan:',
                '  ros__parameters:',
                '    scanner_type: "sick_lms_5xx"',
                '',
            ]
        ),
        encoding='utf-8',
    )
    override_cfg.write_text(
        '\n'.join(
            [
                'sick_scan:',
                '  ros__parameters:',
                '    scanner_type: "sick_tim_7xx"',
                '',
            ]
        ),
        encoding='utf-8',
    )

    value = load_ros_param_from_files(
        [base_cfg, override_cfg],
        node_name='sick_scan',
        key='scanner_type',
    )
    assert value == 'sick_tim_7xx'


def test_load_ros_param_from_files_reads_wildcard_section(tmp_path: Path) -> None:
    cfg = tmp_path / 'robot.yaml'
    cfg.write_text(
        '\n'.join(
            [
                '/**:',
                '  ros__parameters:',
                '    frame_id: "laser"',
                '',
            ]
        ),
        encoding='utf-8',
    )

    value = load_ros_param_from_files([cfg], node_name='sick_scan', key='frame_id')
    assert value == 'laser'


def test_load_ros_param_from_files_raises_for_missing_key(tmp_path: Path) -> None:
    cfg = tmp_path / 'lidar.yaml'
    cfg.write_text('sick_scan:\n  ros__parameters:\n    port: 2111\n', encoding='utf-8')

    with pytest.raises(ValueError, match='missing ros__parameters key'):
        load_ros_param_from_files([cfg], node_name='sick_scan', key='scanner_type')
