# Copyright 2026 team18
"""Read laser/camera XYZ from config files for static TF (no PyYAML)."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

_KEYS = (
    'laser_x',
    'laser_y',
    'laser_z',
    'camera_x',
    'camera_y',
    'camera_z',
    'imu_x',
    'imu_y',
    'imu_z',
)


def _collect_scalars(paths: Iterable[str | Path]) -> dict[str, float]:
    """Parse selected scalar keys from one or more YAML-like config files.

    Later files win, which matches ROS parameter-file layering in launch.
    """
    found: dict[str, float] = {}
    for path in paths:
        text = Path(path).read_text(encoding='utf-8')
        for raw in text.splitlines():
            line = raw.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, _, rest = line.partition(':')
            key = key.strip()
            if key not in _KEYS:
                continue
            val = rest.strip()
            if val.startswith('"') and val.endswith('"'):
                val = val[1:-1]
            elif val.startswith("'") and val.endswith("'"):
                val = val[1:-1]
            found[key] = float(val)

    return found


def load_sensor_xyz_from_files(paths: Iterable[str | Path]) -> dict[str, tuple[float, float, float]]:
    """Parse laser_*, camera_*, and imu_* metres from layered config files."""
    found = _collect_scalars(paths)

    missing = [k for k in _KEYS if k not in found]
    if missing:
        joined = ', '.join(str(Path(path)) for path in paths)
        raise ValueError(f'{joined}: missing keys: {", ".join(missing)}')

    return {
        'laser': (found['laser_x'], found['laser_y'], found['laser_z']),
        'camera': (found['camera_x'], found['camera_y'], found['camera_z']),
        'imu': (found['imu_x'], found['imu_y'], found['imu_z']),
    }


def load_sensor_xyz_from_robot_yaml(path: str | Path) -> dict[str, tuple[float, float, float]]:
    """Backward-compatible wrapper for reading just robot.yaml."""
    return load_sensor_xyz_from_files([path])


def static_transform_arguments(
    xyz: tuple[float, float, float],
    *,
    parent: str = 'base_link',
    child: str,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> list[str]:
    """Build argv for ``tf2_ros static_transform_publisher`` (x y z yaw pitch roll parent child)."""
    x, y, z = xyz
    return [
        str(x),
        str(y),
        str(z),
        str(roll),
        str(pitch),
        str(yaw),
        parent,
        child,
]
