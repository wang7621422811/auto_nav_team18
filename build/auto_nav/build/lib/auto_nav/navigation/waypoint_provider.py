"""
WaypointProvider — loads and normalises a waypoint list from a YAML file.

Supported coordinate formats per waypoint entry:
  type: xy   →  x: <float>,  y: <float>   (already in local ENU / odom frame)
  type: gps  →  lat: <float>, lon: <float> (converted to ENU via GeoLocalizer)

YAML schema (config/waypoints.yaml, data section):

  # Optional GPS origin — required when any waypoint uses type: gps
  origin:
    lat: 51.4788
    lon: -0.0106

  waypoints:
    - name: wp1
      type: xy
      x: 8.0
      y: 0.0
    - name: wp2
      type: gps
      lat: 51.4789
      lon: -0.0107

The file may also contain ROS2 parameter sections (e.g. path_follower.ros__parameters);
they are silently ignored by this class.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import yaml

from .geo_localizer import GeoLocalizer


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float  # metres, local ENU east
    y: float  # metres, local ENU north


class WaypointProvider:
    """Load waypoints from a YAML file and expose as Waypoint(name, x, y) objects."""

    def __init__(self, yaml_path: str | Path) -> None:
        self._path = Path(yaml_path)
        self._waypoints: list[Waypoint] = []
        self._localizer = GeoLocalizer()
        self._load()

    # ------------------------------------------------------------------
    def _load(self) -> None:
        with open(self._path) as fh:
            data = yaml.safe_load(fh)

        if not isinstance(data, dict):
            raise ValueError(f"WaypointProvider: {self._path} is not a YAML mapping")

        # Optional GPS origin
        if "origin" in data:
            origin = data["origin"]
            self._localizer.set_origin(
                float(origin["lat"]),
                float(origin["lon"]),
            )

        raw_list = data.get("waypoints", [])
        if not isinstance(raw_list, list):
            raise ValueError(
                f"WaypointProvider: 'waypoints' key in {self._path} must be a list"
            )

        for idx, entry in enumerate(raw_list):
            wp_type = str(entry.get("type", "xy")).lower()
            name    = str(entry.get("name", f"wp{idx}"))

            if wp_type == "gps":
                if not self._localizer.has_origin:
                    raise ValueError(
                        f"Waypoint '{name}' is GPS type but no 'origin' is defined "
                        f"in {self._path}"
                    )
                east_m, north_m = self._localizer.gps_to_enu(
                    float(entry["lat"]),
                    float(entry["lon"]),
                )
                self._waypoints.append(Waypoint(name=name, x=east_m, y=north_m))

            else:  # xy (default)
                self._waypoints.append(
                    Waypoint(
                        name=name,
                        x=float(entry["x"]),
                        y=float(entry["y"]),
                    )
                )

    # ------------------------------------------------------------------
    @property
    def waypoints(self) -> list[Waypoint]:
        """Return a copy of the waypoint list."""
        return list(self._waypoints)

    def __len__(self) -> int:
        return len(self._waypoints)

    def __getitem__(self, idx: int) -> Waypoint:
        return self._waypoints[idx]

    def __iter__(self):
        return iter(self._waypoints)
