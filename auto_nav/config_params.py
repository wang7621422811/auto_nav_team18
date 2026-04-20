# Copyright 2026 team18
"""Minimal scalar ROS parameter readers for simple YAML config files."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable


def _strip_inline_comment(raw: str) -> str:
    """Remove trailing comments from a YAML scalar line."""
    return raw.split('#', 1)[0].rstrip()


def load_ros_param_from_files(
    paths: Iterable[str | Path],
    *,
    node_name: str,
    key: str,
) -> str:
    """Read a scalar ROS parameter from layered YAML files without PyYAML.

    The parser supports this repo's config style only:
      <node_name>:
        ros__parameters:
          <key>: <scalar>

    The wildcard section ``/**:`` is also recognised. Later matches override
    earlier ones, which mirrors ROS parameter-file layering in launch.
    """
    value: str | None = None

    for path in paths:
        in_target_node = False
        in_ros_params = False

        for raw in Path(path).read_text(encoding='utf-8').splitlines():
            line = _strip_inline_comment(raw)
            stripped = line.strip()
            if not stripped:
                continue

            indent = len(line) - len(line.lstrip(' '))

            if indent == 0 and stripped.endswith(':'):
                in_target_node = stripped in (f'{node_name}:', '/**:')
                in_ros_params = False
                continue

            if not in_target_node:
                continue

            if indent == 2 and stripped == 'ros__parameters:':
                in_ros_params = True
                continue

            if indent <= 2 and stripped.endswith(':'):
                in_ros_params = False
                continue

            if not in_ros_params or indent < 4 or ':' not in stripped:
                continue

            current_key, _, rest = stripped.partition(':')
            if current_key.strip() != key:
                continue

            parsed = rest.strip()
            if parsed.startswith('"') and parsed.endswith('"'):
                parsed = parsed[1:-1]
            elif parsed.startswith("'") and parsed.endswith("'"):
                parsed = parsed[1:-1]
            value = parsed

    if value is None:
        joined = ', '.join(str(Path(path)) for path in paths)
        raise ValueError(
            f'{joined}: missing ros__parameters key "{key}" for node "{node_name}"'
        )

    return value
