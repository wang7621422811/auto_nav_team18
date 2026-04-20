"""
FinalApproachController — computes a lateral pass point so the robot
approaches an orange marker from the left, guaranteeing the marker
remains on the robot's RIGHT side throughout the final pass.

Geometry (2-D, odom/ENU frame):
  Given:
    p_robot  = (rx, ry)   robot's current position
    p_marker = (mx, my)   detected marker position

  Steps:
    u        = normalise(p_marker − p_robot)    approach direction vector
    n_left   = rotate u by +90° CCW             left-hand normal = (−u.y, u.x)
    p_pass   = p_marker + pass_offset · n_left

  Driving toward p_pass makes the robot travel to the LEFT of p_marker,
  so p_marker ends up on the robot's right side as it passes.

Usage:
    ctrl = FinalApproachController(pass_offset_m=0.8)
    pass_pt = ctrl.compute_pass_point(
        p_robot  = (robot_x, robot_y),
        p_marker = (marker_x, marker_y),
    )
    # then drive toward pass_pt.x, pass_pt.y
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


class FinalApproachController:
    """Compute the lateral pass point that keeps the marker on the robot's right."""

    def __init__(self, pass_offset_m: float = 0.8) -> None:
        """
        Parameters
        ----------
        pass_offset_m:
            Lateral distance (metres) between the marker and the pass point.
            Must be positive.  Larger values give more clearance to the right.
        """
        if pass_offset_m <= 0:
            raise ValueError("pass_offset_m must be positive")
        self._pass_offset = pass_offset_m

    # ------------------------------------------------------------------
    def compute_pass_point(
        self,
        p_robot: tuple[float, float],
        p_marker: tuple[float, float],
    ) -> Point2D:
        """
        Return the pass point the robot should drive toward.

        Parameters
        ----------
        p_robot  : (x, y) robot position in the local frame (metres)
        p_marker : (x, y) marker position in the local frame (metres)

        Returns
        -------
        Point2D with (x, y) of the pass point, offset to the left of the marker
        as seen along the robot→marker direction.
        """
        rx, ry = p_robot
        mx, my = p_marker

        dx = mx - rx
        dy = my - ry
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            # Robot is already at the marker — offset leftward in global frame
            return Point2D(x=mx - self._pass_offset, y=my)

        # Normalised approach direction vector u
        ux = dx / dist
        uy = dy / dist

        # Left normal: rotate u by +90° CCW → n_left = (−u.y, u.x)
        n_left_x = -uy
        n_left_y =  ux

        return Point2D(
            x=mx + self._pass_offset * n_left_x,
            y=my + self._pass_offset * n_left_y,
        )

    # ------------------------------------------------------------------
    def approach_bearing(
        self,
        p_robot: tuple[float, float],
        p_marker: tuple[float, float],
    ) -> float:
        """
        Return desired heading (radians, ENU convention) for the robot to
        face directly toward the marker.
        """
        dx = p_marker[0] - p_robot[0]
        dy = p_marker[1] - p_robot[1]
        return math.atan2(dy, dx)

    # ------------------------------------------------------------------
    @property
    def pass_offset_m(self) -> float:
        return self._pass_offset
