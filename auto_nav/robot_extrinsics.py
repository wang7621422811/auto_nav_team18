# Copyright 2026 team18
"""Read laser/camera XYZ from config/robot.yaml for static TF (no PyYAML)."""

from __future__ import annotations

from pathlib import Path

_KEYS = (
    'laser_x',
    'laser_y',
    'laser_z',
    'camera_x',
    'camera_y',
    'camera_z',
)


def load_sensor_xyz_from_robot_yaml(path: str | Path) -> dict[str, tuple[float, float, float]]:
    """Parse laser_* and camera_* metres from robot.yaml.

    Expects scalar lines like ``laser_x: 0.20`` (optional quotes, inline # comments).
    """
    text = Path(path).read_text(encoding='utf-8')
    found: dict[str, float] = {}
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

    missing = [k for k in _KEYS if k not in found]
    if missing:
        raise ValueError(f'{path}: missing keys: {", ".join(missing)}')

    return {
        'laser': (found['laser_x'], found['laser_y'], found['laser_z']),
        'camera': (found['camera_x'], found['camera_y'], found['camera_z']),
    }


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
