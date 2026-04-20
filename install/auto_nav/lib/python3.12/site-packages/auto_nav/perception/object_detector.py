"""
ObjectDetectorNode — detects coloured geometric objects near a waypoint marker.

Strategy:
  1. Wait for a cone marker detection on /marker/bbox to obtain an ROI in the image.
  2. Search a padded region around the marker for non-orange coloured blobs.
  3. For each candidate blob, forward its crop to ShapeClassifier and publish.

Publishes:
  /object/detection  (std_msgs/String)  JSON list:
    [{color, cx, cy, w, h, range_m, bearing_rad}, ...]

Subscribes:
  /camera/color/image_raw  (sensor_msgs/Image)
  /camera/depth/image_raw  (sensor_msgs/Image)
  /marker/bbox             (std_msgs/String)   — JSON from ConeDetectorNode

Parameters (all in camera.yaml under object_detector namespace):
  roi_padding_px     — extra pixels around marker bbox to include (default 300)
  min_area_px        — min candidate contour area (default 300)
  max_area_px        — max candidate contour area (default 60000)
  depth_roi_px       — depth sampling half-size (default 5)
  camera_hfov_deg    — camera horizontal FOV in degrees (default 69.0)
  image_width        — expected image width (default 1280)
  image_height       — expected image height (default 720)
  marker_timeout_s   — discard marker info older than this many seconds (default 2.0)

Colour definitions (HSV, 0-179/0-255 scale) come from camera.yaml:
  colors:
    red:    [[0,120,70],[10,255,255]]  and [[170,120,70],[179,255,255]]
    green:  [[35,80,60],[85,255,255]]
    blue:   [[100,80,60],[130,255,255]]
    yellow: [[20,120,80],[35,255,255]]
    purple: [[125,60,60],[160,255,255]]
    white:  [[0,0,200],[179,30,255]]
"""

from __future__ import annotations

import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Default HSV colour table: name → list of (lower, upper) tuples
_DEFAULT_COLORS: dict[str, list[tuple]] = {
    "red":    [((0,   120, 70),  (10,  255, 255)),
               ((170, 120, 70),  (179, 255, 255))],
    "green":  [((35,  80,  60),  (85,  255, 255))],
    "blue":   [((100, 80,  60),  (130, 255, 255))],
    "yellow": [((20,  120, 80),  (35,  255, 255))],
    "purple": [((125, 60,  60),  (160, 255, 255))],
    "white":  [((0,   0,  200),  (179, 30,  255))],
}


class ObjectDetectorNode(Node):
    """ROS 2 node: detect coloured objects near the cone marker."""

    def __init__(self) -> None:
        super().__init__("object_detector")

        self.declare_parameter("roi_padding_px",  300)
        self.declare_parameter("min_area_px",     300)
        self.declare_parameter("max_area_px",     60000)
        self.declare_parameter("depth_roi_px",    5)
        self.declare_parameter("camera_hfov_deg", 69.0)
        self.declare_parameter("image_width",     1280)
        self.declare_parameter("image_height",    720)
        self.declare_parameter("marker_timeout_s", 2.0)

        self._p = self._get_params()

        self._bridge       = CvBridge()
        self._depth_image  = None
        self._marker_bbox  = None   # dict from /marker/bbox JSON
        self._marker_time  = 0.0   # timestamp of last marker update

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image,  "/camera/color/image_raw", self._on_color, best_effort)
        self.create_subscription(Image,  "/camera/depth/image_raw", self._on_depth, best_effort)
        self.create_subscription(String, "/marker/bbox",            self._on_marker, 10)

        self._pub = self.create_publisher(String, "/object/detection", 10)

        self.get_logger().info("ObjectDetectorNode ready.")

    # ------------------------------------------------------------------
    def _get_params(self) -> dict:
        gp = self.get_parameter
        return dict(
            roi_pad    = gp("roi_padding_px").get_parameter_value().integer_value,
            min_area   = gp("min_area_px").get_parameter_value().integer_value,
            max_area   = gp("max_area_px").get_parameter_value().integer_value,
            depth_roi  = gp("depth_roi_px").get_parameter_value().integer_value,
            hfov_rad   = math.radians(gp("camera_hfov_deg").get_parameter_value().double_value),
            img_w      = gp("image_width").get_parameter_value().integer_value,
            img_h      = gp("image_height").get_parameter_value().integer_value,
            marker_to  = gp("marker_timeout_s").get_parameter_value().double_value,
        )

    # ------------------------------------------------------------------
    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Depth decode error: {exc}", throttle_duration_sec=5.0)

    def _on_marker(self, msg: String) -> None:
        try:
            self._marker_bbox = json.loads(msg.data)
            self._marker_time = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Marker bbox parse error: {exc}")

    # ------------------------------------------------------------------
    def _on_color(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Color decode error: {exc}", throttle_duration_sec=5.0)
            return

        results = self._detect_objects(bgr)
        self._pub.publish(String(data=json.dumps(results)))

    # ------------------------------------------------------------------
    def _detect_objects(self, bgr: np.ndarray) -> list[dict]:
        p   = self._p
        h_img, w_img = bgr.shape[:2]

        # Build ROI: if marker is fresh, crop around it; else full frame
        if (self._marker_bbox is not None
                and (time.monotonic() - self._marker_time) < p["marker_to"]):
            b    = self._marker_bbox
            cx_m = int(b.get("cx", w_img / 2))
            cy_m = int(b.get("cy", h_img / 2))
            bw_m = int(b.get("w",  100))
            bh_m = int(b.get("h",  100))
            pad  = p["roi_pad"]
            x0   = max(0, cx_m - bw_m // 2 - pad)
            x1   = min(w_img, cx_m + bw_m // 2 + pad)
            y0   = max(0, cy_m - bh_m // 2 - pad)
            y1   = min(h_img, cy_m + bh_m // 2 + pad)
        else:
            x0, y0, x1, y1 = 0, 0, w_img, h_img

        roi_bgr = bgr[y0:y1, x0:x1]
        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

        detections: list[dict] = []
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        for color_name, ranges in _DEFAULT_COLORS.items():
            mask = np.zeros(roi_bgr.shape[:2], dtype=np.uint8)
            for (lo, hi) in ranges:
                mask |= cv2.inRange(roi_hsv, lo, hi)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (p["min_area"] <= area <= p["max_area"]):
                    continue
                rx, ry, rw, rh = cv2.boundingRect(cnt)
                # Convert ROI-local coords back to full-image coords
                abs_cx = x0 + rx + rw / 2
                abs_cy = y0 + ry + rh / 2
                bearing = self._pixel_to_bearing(abs_cx, p)
                rng     = self._estimate_range(abs_cx, abs_cy, p)

                # Pass contour crop to shape classifier (imported lazily to avoid cycles)
                crop = roi_bgr[ry:ry + rh, rx:rx + rw]
                shape = _classify_shape(cnt, crop)

                detections.append({
                    "color":       color_name,
                    "shape":       shape,
                    "cx":          round(abs_cx, 1),
                    "cy":          round(abs_cy, 1),
                    "w":           rw,
                    "h":           rh,
                    "range_m":     round(rng, 3) if math.isfinite(rng) else None,
                    "bearing_rad": round(bearing, 4),
                })

        return detections

    # ------------------------------------------------------------------
    def _pixel_to_bearing(self, cx_px: float, p: dict) -> float:
        return -((cx_px / p["img_w"]) - 0.5) * p["hfov_rad"]

    def _estimate_range(self, cx: float, cy: float, p: dict) -> float:
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
        val = float(np.median(roi))
        return val / 1000.0 if val > 20 else val


# ---------------------------------------------------------------------------
# Inline lightweight shape classification (avoids import cycle with
# shape_classifier.py which is also a standalone node).
# ---------------------------------------------------------------------------

def _classify_shape(contour: np.ndarray, crop: np.ndarray) -> str:  # noqa: ARG001
    """Classify contour shape without importing the full ShapeClassifier node."""
    from auto_nav.perception.shape_classifier import classify_contour  # lazy import
    return classify_contour(contour)


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
