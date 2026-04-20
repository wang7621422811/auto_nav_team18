"""
PhotoCaptureNode — saves annotated photos when a waypoint is in final-approach.

Trigger:
  Listens to /waypoint/status (std_msgs/String).  When the value contains
  "FINAL_APPROACH:<index>", it captures the next available colour frame and
  saves three files:
      <photo_dir>/wp_<XX>_marker.jpg   — raw frame (cone detected)
      <photo_dir>/wp_<XX>_object.jpg   — raw frame (object bounding box drawn)
      <photo_dir>/wp_<XX>_annotated.jpg — both bboxes drawn on same frame

  Each waypoint index is saved only ONCE per mission to prevent duplicate writes.
  On detection failure the node still writes a best-effort image and logs "unknown".

Subscribes:
  /camera/color/image_raw  (sensor_msgs/Image)
  /waypoint/status         (std_msgs/String)   — e.g. "FINAL_APPROACH:0"
  /marker/bbox             (std_msgs/String)   — JSON from ConeDetectorNode
  /object/detection        (std_msgs/String)   — JSON list from ObjectDetectorNode

Publishes:
  /photo/saved  (std_msgs/String)   — path of most recently written annotated image

Parameters:
  photo_dir         — directory to save images (default "artifacts/photos")
  save_raw_frames   — also save the unannotated marker/object frames (default true)
  jpeg_quality      — JPEG compression quality 0-100 (default 90)
"""

from __future__ import annotations

import json
import os
import re

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PhotoCaptureNode(Node):
    """Save marker and object photos when entering final approach."""

    def __init__(self) -> None:
        super().__init__("photo_capture")

        self.declare_parameter("photo_dir",       "artifacts/photos")
        self.declare_parameter("save_raw_frames", True)
        self.declare_parameter("jpeg_quality",    90)

        gp = self.get_parameter
        self._photo_dir   = gp("photo_dir").get_parameter_value().string_value
        self._save_raw    = gp("save_raw_frames").get_parameter_value().bool_value
        self._jpeg_q      = gp("jpeg_quality").get_parameter_value().integer_value

        os.makedirs(self._photo_dir, exist_ok=True)

        self._bridge          = CvBridge()
        self._latest_bgr      = None
        self._marker_bbox     = None   # dict or None
        self._obj_list: list  = []
        self._captured: set   = set()  # wp indices already photographed

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image,  "/camera/color/image_raw",
                                 self._on_color, best_effort)
        self.create_subscription(String, "/waypoint/status",
                                 self._on_wp_status, 10)
        self.create_subscription(String, "/marker/bbox",
                                 self._on_marker_bbox, 10)
        self.create_subscription(String, "/object/detection",
                                 self._on_objects, 10)

        self._pub_photo = self.create_publisher(String, "/photo/saved", 10)

        self.get_logger().info(
            f"PhotoCaptureNode ready — saving to '{self._photo_dir}'"
        )

    # ------------------------------------------------------------------
    def _on_color(self, msg: Image) -> None:
        try:
            self._latest_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Color decode error: {exc}", throttle_duration_sec=5.0)

    def _on_marker_bbox(self, msg: String) -> None:
        try:
            self._marker_bbox = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            self._marker_bbox = None

    def _on_objects(self, msg: String) -> None:
        try:
            self._obj_list = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            self._obj_list = []

    # ------------------------------------------------------------------
    def _on_wp_status(self, msg: String) -> None:
        status = msg.data.strip()
        match  = re.match(r"FINAL_APPROACH:(\d+)", status)
        if not match:
            return
        wp_idx = int(match.group(1))
        if wp_idx in self._captured:
            return   # already photographed this waypoint
        if self._latest_bgr is None:
            self.get_logger().warn(f"WP{wp_idx}: final approach but no image yet.")
            return

        self._save_waypoint_photos(wp_idx, self._latest_bgr.copy())
        self._captured.add(wp_idx)

    # ------------------------------------------------------------------
    def _save_waypoint_photos(self, wp_idx: int, bgr: np.ndarray) -> None:
        tag    = f"wp_{wp_idx:02d}"
        prefix = os.path.join(self._photo_dir, tag)
        params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q]

        annotated = bgr.copy()
        marker_saved = False
        object_saved = False

        # ---- marker image ----
        if self._marker_bbox:
            b   = self._marker_bbox
            cx  = int(b.get("cx", 0))
            cy  = int(b.get("cy", 0))
            bw  = int(b.get("w",  50))
            bh  = int(b.get("h",  50))
            x1m = cx - bw // 2
            y1m = cy - bh // 2
            x2m = cx + bw // 2
            y2m = cy + bh // 2
            cv2.rectangle(annotated, (x1m, y1m), (x2m, y2m), (0, 165, 255), 2)
            rng = b.get("range_m")
            lbl = f"cone {rng:.2f}m" if rng is not None else "cone"
            cv2.putText(annotated, lbl, (x1m, y1m - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            if self._save_raw:
                path_marker = f"{prefix}_marker.jpg"
                cv2.imwrite(path_marker, bgr, params)
                marker_saved = True
                self.get_logger().info(f"Saved: {path_marker}")
        else:
            self.get_logger().warn(f"WP{wp_idx}: no marker bbox — writing raw frame.")
            if self._save_raw:
                path_marker = f"{prefix}_marker.jpg"
                cv2.imwrite(path_marker, bgr, params)
                marker_saved = True

        # ---- object image ----
        if self._obj_list:
            best = max(self._obj_list, key=lambda o: o.get("w", 0) * o.get("h", 0))
            ocx = int(best.get("cx", 0))
            ocy = int(best.get("cy", 0))
            ow  = int(best.get("w",  50))
            oh  = int(best.get("h",  50))
            x1o = ocx - ow // 2
            y1o = ocy - oh // 2
            x2o = ocx + ow // 2
            y2o = ocy + oh // 2
            color_bgr = (255, 0, 0)   # blue box for detected object
            cv2.rectangle(annotated, (x1o, y1o), (x2o, y2o), color_bgr, 2)
            lbl = f"{best.get('color','?')} {best.get('shape','?')}"
            cv2.putText(annotated, lbl, (x1o, y1o - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
            if self._save_raw:
                obj_frame = bgr.copy()
                cv2.rectangle(obj_frame, (x1o, y1o), (x2o, y2o), color_bgr, 2)
                cv2.putText(obj_frame, lbl, (x1o, y1o - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
                path_obj = f"{prefix}_object.jpg"
                cv2.imwrite(path_obj, obj_frame, params)
                object_saved = True
                self.get_logger().info(f"Saved: {path_obj}")
        else:
            self.get_logger().warn(
                f"WP{wp_idx}: no object detected — writing frame without object box."
            )
            if self._save_raw:
                path_obj = f"{prefix}_object.jpg"
                cv2.imwrite(path_obj, bgr, params)
                object_saved = True

        # ---- annotated composite ----
        path_ann = f"{prefix}_annotated.jpg"
        cv2.imwrite(path_ann, annotated, params)
        self.get_logger().info(f"Saved annotated: {path_ann} "
                               f"(marker={marker_saved}, object={object_saved})")
        self._pub_photo.publish(String(data=path_ann))


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoCaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
