"""
ConeDetectorNode — detects orange traffic-cone waypoint markers.

Pipeline per frame:
  BGR → HSV → orange threshold → morphological cleanup →
  contour filtering → depth lookup → publish detection

Publishes:
  /marker/detection  (geometry_msgs/PoseStamped)  — cone position in camera_link frame
                     header.frame_id = "camera_link"
                     pose.position.x  = bearing angle [rad]  (used as proxy when no depth)
                     pose.position.z  = range [m]
  /marker/bbox       (std_msgs/String)             — JSON: {cx, cy, w, h, range, bearing}

Subscribes:
  /camera/color/image_raw   (sensor_msgs/Image)
  /camera/depth/image_raw   (sensor_msgs/Image)   — optional; graceful if absent

Parameters (all in camera.yaml under cone_detector namespace):
  hsv_h_low, hsv_h_high  — hue band for orange (default 5–25 for 0-179 scale)
  hsv_s_low              — min saturation (default 120)
  hsv_v_low              — min value/brightness (default 80)
  min_area_px            — minimum contour area in pixels (default 400)
  max_area_px            — maximum contour area in pixels (default 80000)
  min_aspect_ratio       — min h/w ratio, rejects wide flat blobs (default 0.4)
  max_aspect_ratio       — max h/w ratio (default 4.0)
  depth_roi_px           — half-size of depth sampling square (default 5)
  camera_hfov_deg        — horizontal field of view in degrees (default 69.0 for OAK-D)
  image_width            — expected image width  (default 1280)
  image_height           — expected image height (default 720)
"""

from __future__ import annotations

import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ConeDetectorNode(Node):
    """ROS 2 node: detect orange cone marker and publish position."""

    def __init__(self) -> None:
        super().__init__("cone_detector")

        # --- parameters -------------------------------------------------
        self.declare_parameter("hsv_h_low",        5)
        self.declare_parameter("hsv_h_high",       25)
        self.declare_parameter("hsv_s_low",        120)
        self.declare_parameter("hsv_v_low",        80)
        self.declare_parameter("min_area_px",      400)
        self.declare_parameter("max_area_px",      80000)
        self.declare_parameter("min_aspect_ratio", 0.4)
        self.declare_parameter("max_aspect_ratio", 4.0)
        self.declare_parameter("depth_roi_px",     5)
        self.declare_parameter("camera_hfov_deg",  69.0)
        self.declare_parameter("image_width",      1280)
        self.declare_parameter("image_height",     720)

        self._p = self._get_params()

        self._bridge        = CvBridge()
        self._depth_image   = None   # latest depth frame (np.ndarray float32 metres)
        self._seen_color    = False

        best_effort = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # --- subscribers ------------------------------------------------
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self._on_color, best_effort)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self._on_depth, best_effort)

        # --- publishers -------------------------------------------------
        self._pub_pose = self.create_publisher(PoseStamped, "/marker/detection", 10)
        # publish msg for the object detector
        self._pub_bbox = self.create_publisher(String,       "/marker/bbox",      10)
        self._diag_timer = self.create_timer(5.0, self._diag_tick)

        self.get_logger().info("ConeDetectorNode ready — orange HSV "
                               f"H:[{self._p['h_low']},{self._p['h_high']}]")

    # ------------------------------------------------------------------
    def _get_params(self) -> dict:
        gp = self.get_parameter
        return dict(
            h_low        = gp("hsv_h_low").get_parameter_value().integer_value,
            h_high       = gp("hsv_h_high").get_parameter_value().integer_value,
            s_low        = gp("hsv_s_low").get_parameter_value().integer_value,
            v_low        = gp("hsv_v_low").get_parameter_value().integer_value,
            min_area     = gp("min_area_px").get_parameter_value().integer_value,
            max_area     = gp("max_area_px").get_parameter_value().integer_value,
            min_ar       = gp("min_aspect_ratio").get_parameter_value().double_value,
            max_ar       = gp("max_aspect_ratio").get_parameter_value().double_value,
            depth_roi    = gp("depth_roi_px").get_parameter_value().integer_value,
            hfov_rad     = math.radians(gp("camera_hfov_deg").get_parameter_value().double_value),
            img_w        = gp("image_width").get_parameter_value().integer_value,
            img_h        = gp("image_height").get_parameter_value().integer_value,
        )

    # ------------------------------------------------------------------
    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Depth decode error: {exc}", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    def _on_color(self, msg: Image) -> None:
        self._seen_color = True
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Color decode error: {exc}", throttle_duration_sec=5.0)
            return

        detection = self._detect_cone(bgr)
        if detection is None:
            return

        cx, cy, w, h, rng, bearing = detection
        self._publish(msg.header, cx, cy, w, h, rng, bearing)

    # ------------------------------------------------------------------
    def _diag_tick(self) -> None:
        """Warn when the detector is alive but no color frames are arriving."""
        if self._seen_color:
            return
        self.get_logger().warn(
            "No frames received on /camera/color/image_raw yet. "
            "perception.launch.py does not start the camera driver; "
            "launch bringup.launch.py with use_camera:=true or publish the topic externally.",
            throttle_duration_sec=10.0,
        )

    # ------------------------------------------------------------------
    def _detect_cone(self, bgr: np.ndarray):
        """Return (cx, cy, w, h, range_m, bearing_rad) or None."""
        p    = self._p
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Orange hue can wrap; support both a single band and the wrap-around case
        mask1 = cv2.inRange(hsv,
                            (p["h_low"],  p["s_low"], p["v_low"]),
                            (p["h_high"], 255,        255))
        # Wrap-around near hue=0 (red-orange): also check 0–5 and 170–179
        mask2 = cv2.inRange(hsv, (0, p["s_low"], p["v_low"]), (5, 255, 255))
        mask3 = cv2.inRange(hsv, (170, p["s_low"], p["v_low"]), (179, 255, 255))
        mask  = cv2.bitwise_or(mask1, cv2.bitwise_or(mask2, mask3))

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=2)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = None
        best_score = -1.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (p["min_area"] <= area <= p["max_area"]):
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            ar = ch / cw if cw > 0 else 0.0
            if not (p["min_ar"] <= ar <= p["max_ar"]):
                continue
            # Score: prefer large, central contours
            cx_ = x + cw / 2
            center_score = 1.0 - abs(cx_ / p["img_w"] - 0.5) * 2.0
            score = area * center_score
            if score > best_score:
                best_score = score
                best = (x, y, cw, ch)

        if best is None:
            return None

        bx, by, bw, bh = best
        cx = bx + bw / 2
        cy = by + bh / 2

        bearing = self._pixel_to_bearing(cx, p)
        rng     = self._estimate_range(cx, cy, p)

        return cx, cy, bw, bh, rng, bearing

    # ------------------------------------------------------------------
    @staticmethod
    def _pixel_to_bearing(cx_px: float, p: dict) -> float:
        """Convert pixel x-coordinate to bearing angle [rad]. +ve = left."""
        normalised = (cx_px / p["img_w"]) - 0.5   # [-0.5, +0.5]
        return -normalised * p["hfov_rad"]          # left of centre → positive bearing

    # ------------------------------------------------------------------
    def _estimate_range(self, cx: float, cy: float, p: dict) -> float:
        """Sample depth image around (cx, cy). Returns NaN on failure."""
        if self._depth_image is None:
            return float("nan")
        h, w = self._depth_image.shape[:2]
        r    = p["depth_roi"]
        x0   = max(0, int(cx) - r)
        x1   = min(w, int(cx) + r + 1)
        y0   = max(0, int(cy) - r)
        y1   = min(h, int(cy) + r + 1)
        roi  = self._depth_image[y0:y1, x0:x1].astype(np.float32)
        roi  = roi[np.isfinite(roi) & (roi > 0.01)]
        if roi.size == 0:
            return float("nan")
        # Use median to reject noise / holes
        depth_mm_or_m = float(np.median(roi))
        # OAK-D depth is in mm when using uint16; float32 may already be metres.
        # Heuristic: if median > 20 assume millimetres.
        return depth_mm_or_m / 1000.0 if depth_mm_or_m > 20 else depth_mm_or_m

    # ------------------------------------------------------------------
    def _publish(self, header, cx, cy, w, h, rng, bearing) -> None:
        pose                    = PoseStamped()
        pose.header             = header
        pose.header.frame_id    = "camera_link"
        # Encode bearing and range into position fields (no full 3-D pose needed here)
        pose.pose.position.x    = bearing   # [rad]  lateral angle
        pose.pose.position.y    = float(cx) # [px]   pixel centre x
        pose.pose.position.z    = rng       # [m]    range (NaN if unknown)
        pose.pose.orientation.w = 1.0
        self._pub_pose.publish(pose)

        bbox_data = json.dumps({
            "cx": round(cx, 1), "cy": round(cy, 1),
            "w": round(w, 1),   "h": round(h, 1),
            "range_m":  round(rng, 3) if math.isfinite(rng) else None,
            "bearing_rad": round(bearing, 4),
        })
        self._pub_bbox.publish(String(data=bbox_data))


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConeDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
