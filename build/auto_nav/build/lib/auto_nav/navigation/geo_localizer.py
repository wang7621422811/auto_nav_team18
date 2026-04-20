"""
GeoLocalizer — converts GPS (lat/lon) to local ENU (East-North-Up) coordinates.

Uses a flat-earth approximation valid for distances up to ~10 km — more than
sufficient for a competition field.

Call set_origin() once to fix the local frame origin, then use gps_to_enu()
to convert any subsequent GPS fix into (east_m, north_m) metres.
"""

from __future__ import annotations

import math


class GeoLocalizer:
    """Convert WGS-84 lat/lon to local East-North-Up (x=east, y=north) in metres."""

    _EARTH_RADIUS_M: float = 6_371_000.0  # mean Earth radius

    def __init__(self) -> None:
        self._origin_lat: float | None = None
        self._origin_lon: float | None = None
        self._cos_lat0: float = 1.0

    # ------------------------------------------------------------------
    def set_origin(self, lat: float, lon: float) -> None:
        """Fix the ENU origin (decimal degrees).  Must be called before gps_to_enu."""
        self._origin_lat = lat
        self._origin_lon = lon
        self._cos_lat0 = math.cos(math.radians(lat))

    @property
    def has_origin(self) -> bool:
        """True once set_origin() has been called."""
        return self._origin_lat is not None

    # ------------------------------------------------------------------
    def gps_to_enu(self, lat: float, lon: float) -> tuple[float, float]:
        """
        Return (east_m, north_m) of the given GPS coordinate relative to the
        stored origin.  Raises RuntimeError if set_origin() has not been called.
        """
        if not self.has_origin:
            raise RuntimeError(
                "GeoLocalizer: origin not set — call set_origin() first"
            )

        dlat = lat - self._origin_lat   # type: ignore[operator]
        dlon = lon - self._origin_lon   # type: ignore[operator]

        rad_per_deg = math.pi / 180.0
        north_m = dlat * rad_per_deg * self._EARTH_RADIUS_M
        east_m  = dlon * rad_per_deg * self._EARTH_RADIUS_M * self._cos_lat0

        return east_m, north_m
