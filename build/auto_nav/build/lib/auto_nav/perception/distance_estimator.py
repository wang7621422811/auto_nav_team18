"""
DistanceEstimatorNode — computes the 3-D distance between a detected coloured
object and the orange-cone waypoint marker, using OAK-D depth data.

Algorithm (primary — depth available)
--------------------------------------
  1. Receive /marker/detection  → extract cone 3-D point p_cone.
  2. Receive /object/detection  → extract best-candidate object 3-D point p_obj.
  3. d = ||p_obj − p_cone||

  Where the 3-D point for an entity with pixel centre (cx, cy) and depth z is:
      X = z * (cx − cx_principal) / fx
      Y = z * (cy − cy_principal) / fy
      Z = z

Fallback (depth unavailable / NaN)
------------------------------------
  If either z is NaN:
  - Use the in-frame pixel-distance scaled by a size-based depth estimate
    (very rough; sets distance_confidence = "low").
  - If both are NaN, output distance_m = null, confidence = "none".

Publishes:
  /perception/distance  (std_msgs/String)  JSON:
    {
      "distance_m": <float|null>,
      "confidence": "high"|"low"|"none",
      "cone_range_m": <float|null>,
      "obj_range_m": <float|null>,
      "obj_color": <str>,
      "obj_shape": <str>
    }

Subscribes:
  /marker/detection   (geometry_msgs/PoseStamped)  — from ConeDetectorNode
  /marker/bbox        (std_msgs/String)             — JSON bbox from ConeDetectorNode
  /object/detection   (std_msgs/String)             — JSON list from ObjectDetectorNode

Parameters:
  fx, fy             — focal lengths in pixels (default OAK-D: 883.0)
  cx_principal       — principal point x (default 640.0)
  cy_principal       — principal point y (default 360.0)
  max_detection_age_s — discard stale detections (default 1.0 s)
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class DistanceEstimatorNode(Node):
    """Compute marker–object 3-D distance from depth + detection messages."""

    def __init__(self) -> None:
        super().__init__("distance_estimator")

        self.declare_parameter("fx",                 883.0)
        self.declare_parameter("fy",                 883.0)
        self.declare_parameter("cx_principal",       640.0)
        self.declare_parameter("cy_principal",       360.0)
        self.declare_parameter("max_detection_age_s", 1.0)

        gp = self.get_parameter
        self._fx    = gp("fx").get_parameter_value().double_value
        self._fy    = gp("fy").get_parameter_value().double_value
        self._cx0   = gp("cx_principal").get_parameter_value().double_value
        self._cy0   = gp("cy_principal").get_parameter_value().double_value
        self._max_age = gp("max_detection_age_s").get_parameter_value().double_value

        # Cached latest detections
        self._cone_pose: PoseStamped | None = None
        self._cone_pose_time: float         = 0.0
        self._cone_bbox: dict | None        = None

        self._obj_list: list[dict]          = []
        self._obj_time: float               = 0.0

        self.create_subscription(PoseStamped, "/marker/detection", self._on_cone_pose, 10)
        self.create_subscription(String,      "/marker/bbox",      self._on_cone_bbox, 10)
        self.create_subscription(String,      "/object/detection", self._on_objects,   10)

        self._pub = self.create_publisher(String, "/perception/distance", 10)

        self.get_logger().info("DistanceEstimatorNode ready.")

    # ------------------------------------------------------------------
    def _on_cone_pose(self, msg: PoseStamped) -> None:
        self._cone_pose      = msg
        self._cone_pose_time = time.monotonic()
        self._try_estimate()

    def _on_cone_bbox(self, msg: String) -> None:
        try:
            self._cone_bbox = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Cone bbox parse: {exc}")

    def _on_objects(self, msg: String) -> None:
        try:
            self._obj_list = json.loads(msg.data)
            self._obj_time = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Object detection parse: {exc}")
        self._try_estimate()

    # ------------------------------------------------------------------
    def _try_estimate(self) -> None:
        now = time.monotonic()
        if self._cone_pose is None:
            return
        if (now - self._cone_pose_time) > self._max_age:
            return
        if not self._obj_list:
            return
        if (now - self._obj_time) > self._max_age:
            return

        # Pick best object candidate (largest area)
        best_obj = max(self._obj_list, key=lambda o: o.get("w", 0) * o.get("h", 0))

        cone_range = self._cone_pose.pose.position.z   # metres or NaN
        obj_range  = best_obj.get("range_m")           # metres or None → NaN
        if obj_range is None:
            obj_range = float("nan")

        distance_m, confidence = self._compute_distance(
            cone_range, obj_range, best_obj
        )

        payload = {
            "distance_m":   round(distance_m, 3) if math.isfinite(distance_m) else None,
            "confidence":   confidence,
            "cone_range_m": round(cone_range, 3) if math.isfinite(cone_range) else None,
            "obj_range_m":  round(obj_range, 3)  if math.isfinite(obj_range)  else None,
            "obj_color":    best_obj.get("color", "unknown"),
            "obj_shape":    best_obj.get("shape", "unknown"),
        }
        self._pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------
    def _compute_distance(
        self,
        cone_z: float,
        obj_z:  float,
        obj:    dict,
    ) -> tuple[float, str]:
        """
        Returns (distance_metres, confidence_str).

        Primary path: both depths valid → full 3-D vector.
        Fallback: use in-plane pixel offset + average depth.
        """
        cone_cx = float(self._cone_bbox.get("cx", self._cx0)) if self._cone_bbox else self._cx0
        cone_cy = float(self._cone_bbox.get("cy", self._cy0)) if self._cone_bbox else self._cy0
        obj_cx  = float(obj.get("cx", self._cx0))
        obj_cy  = float(obj.get("cy", self._cy0))

        if math.isfinite(cone_z) and math.isfinite(obj_z) and cone_z > 0 and obj_z > 0:
            # Full 3-D: project both to camera frame
            p_cone = self._deproject(cone_cx, cone_cy, cone_z)
            p_obj  = self._deproject(obj_cx,  obj_cy,  obj_z)
            dist   = math.sqrt(sum((a - b) ** 2 for a, b in zip(p_cone, p_obj)))
            return dist, "high"

        # Fallback: use whichever depth we have as average, compute 2-D distance
        if math.isfinite(cone_z) and cone_z > 0:
            z_avg = cone_z
        elif math.isfinite(obj_z) and obj_z > 0:
            z_avg = obj_z
        else:
            return float("nan"), "none"

        # Pixel offset → angular offset → approximate arc length
        dx_px = obj_cx - cone_cx
        dy_px = obj_cy - cone_cy
        dx_m  = dx_px * z_avg / self._fx
        dy_m  = dy_px * z_avg / self._fy
        dist  = math.sqrt(dx_m ** 2 + dy_m ** 2)
        return dist, "low"

    # ------------------------------------------------------------------
    def _deproject(self, cx: float, cy: float, z: float) -> tuple[float, float, float]:
        """Back-project pixel + depth into camera-frame 3-D point (X, Y, Z)."""
        X = z * (cx - self._cx0) / self._fx
        Y = z * (cy - self._cy0) / self._fy
        return X, Y, z


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistanceEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
