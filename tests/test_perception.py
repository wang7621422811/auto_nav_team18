"""
Unit tests for Step-4 perception modules.

Tests run without a live ROS2 daemon or camera hardware:
  - shape_classifier  — pure Python, no ROS dependency
  - cone_detector     — HSV pipeline tested on synthetic numpy images
  - distance_estimator logic — pure maths tested directly

All ROS2 / cv_bridge / sensor_msgs imports are stubbed where needed.
"""

from __future__ import annotations

import json
import math
import sys
import types
import unittest
from unittest.mock import MagicMock

import numpy as np


# ---------------------------------------------------------------------------
# ① Stub heavy optional dependencies that may not be installed in CI
# ---------------------------------------------------------------------------

def _ensure_cv2():
    """If OpenCV is not installed, provide a minimal stub so imports pass."""
    try:
        import cv2  # noqa: F401
        return
    except ImportError:
        pass

    cv2_stub = types.ModuleType("cv2")

    # Constants used by perception code
    cv2_stub.COLOR_BGR2HSV   = 0
    cv2_stub.MORPH_ELLIPSE   = 1
    cv2_stub.MORPH_OPEN      = 2
    cv2_stub.MORPH_CLOSE     = 3
    cv2_stub.RETR_EXTERNAL   = 0
    cv2_stub.CHAIN_APPROX_SIMPLE = 1
    cv2_stub.FONT_HERSHEY_SIMPLEX = 0
    cv2_stub.IMWRITE_JPEG_QUALITY = 1

    cv2_stub.cvtColor             = lambda img, _: img
    cv2_stub.inRange              = lambda img, lo, hi: np.zeros(img.shape[:2], np.uint8)
    cv2_stub.bitwise_or           = lambda a, b: a | b
    cv2_stub.getStructuringElement = lambda *a, **kw: np.ones((5, 5), np.uint8)
    cv2_stub.morphologyEx         = lambda img, *a, **kw: img
    cv2_stub.findContours         = lambda *a, **kw: ([], None)
    cv2_stub.contourArea          = lambda c: 0.0
    cv2_stub.arcLength            = lambda c, closed: 0.0
    cv2_stub.approxPolyDP         = lambda c, eps, closed: c
    cv2_stub.boundingRect         = lambda c: (0, 0, 1, 1)
    cv2_stub.rectangle            = lambda *a, **kw: None
    cv2_stub.putText              = lambda *a, **kw: None
    cv2_stub.imwrite              = lambda *a, **kw: True

    sys.modules["cv2"] = cv2_stub


def _build_ros_stubs():
    """Register minimal ROS2 / sensor_msgs stubs in sys.modules."""
    rclpy = types.ModuleType("rclpy")
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.qos  = types.ModuleType("rclpy.qos")

    class _QoS:
        def __init__(self, **kw): pass

    class _Rel:
        BEST_EFFORT = "BEST_EFFORT"

    rclpy.qos.QoSProfile        = _QoS
    rclpy.qos.ReliabilityPolicy = _Rel
    rclpy.init         = MagicMock()
    rclpy.spin         = MagicMock()
    rclpy.try_shutdown = MagicMock()

    class _Node:
        def __init__(self, name):
            self._name   = name
            self._logger = MagicMock()
            self._params = {}

        def get_logger(self):
            return self._logger

        def create_publisher(self, *a, **kw):
            return MagicMock()

        def create_subscription(self, *a, **kw):
            return MagicMock()

        def create_timer(self, *a, **kw):
            return MagicMock()

        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            val = self._params.get(name)
            p = MagicMock()
            if isinstance(val, bool):
                p.get_parameter_value.return_value.bool_value = val
            elif isinstance(val, int):
                p.get_parameter_value.return_value.integer_value = val
            elif isinstance(val, float):
                p.get_parameter_value.return_value.double_value = val
            else:
                p.get_parameter_value.return_value.string_value = val or ""
            return p

    rclpy.node.Node = _Node

    geo = types.ModuleType("geometry_msgs")
    geo.msg = types.ModuleType("geometry_msgs.msg")

    class _PoseStamped:
        def __init__(self):
            self.header = MagicMock()
            self.pose   = MagicMock()
            self.pose.position = MagicMock(x=0.0, y=0.0, z=0.0)

    geo.msg.PoseStamped = _PoseStamped

    sensor = types.ModuleType("sensor_msgs")
    sensor.msg = types.ModuleType("sensor_msgs.msg")
    sensor.msg.Image = MagicMock

    std = types.ModuleType("std_msgs")
    std.msg = types.ModuleType("std_msgs.msg")

    class _String:
        def __init__(self, data=""):
            self.data = data

    std.msg.String = _String

    cv_bridge = types.ModuleType("cv_bridge")
    cv_bridge.CvBridge = MagicMock

    for name, mod in [
        ("rclpy",                rclpy),
        ("rclpy.node",           rclpy.node),
        ("rclpy.qos",            rclpy.qos),
        ("geometry_msgs",        geo),
        ("geometry_msgs.msg",    geo.msg),
        ("sensor_msgs",          sensor),
        ("sensor_msgs.msg",      sensor.msg),
        ("std_msgs",             std),
        ("std_msgs.msg",         std.msg),
        ("cv_bridge",            cv_bridge),
    ]:
        sys.modules.setdefault(name, mod)

    return _String, _PoseStamped


_ensure_cv2()
_String, _PoseStamped = _build_ros_stubs()


# Now safe to import our modules
from auto_nav.perception.shape_classifier import classify_contour  # noqa: E402
from auto_nav.perception.cone_detector import ConeDetectorNode  # noqa: E402
from auto_nav.perception.distance_estimator import DistanceEstimatorNode  # noqa: E402


# ---------------------------------------------------------------------------
# ② Shape classifier tests (pure Python, no mocks needed)
# ---------------------------------------------------------------------------

def _make_contour_from_points(pts: list[tuple[int, int]]) -> np.ndarray:
    """Wrap a list of (x,y) tuples into an OpenCV contour array."""
    return np.array([[[x, y]] for x, y in pts], dtype=np.int32)


class TestShapeClassifier(unittest.TestCase):

    def _circle_contour(self, r: int = 60, cx: int = 100, cy: int = 100) -> np.ndarray:
        pts = [(int(cx + r * math.cos(t)), int(cy + r * math.sin(t)))
               for t in np.linspace(0, 2 * math.pi, 64, endpoint=False)]
        return _make_contour_from_points(pts)

    def _rect_contour(self, w: int, h: int) -> np.ndarray:
        return _make_contour_from_points([(0, 0), (w, 0), (w, h), (0, h)])

    def _triangle_contour(self) -> np.ndarray:
        return _make_contour_from_points([(0, 0), (100, 0), (50, 86)])

    def _pentagon_contour(self) -> np.ndarray:
        r = 60
        pts = [(int(r * math.cos(t)), int(r * math.sin(t)))
               for t in np.linspace(0, 2 * math.pi, 5, endpoint=False)]
        return _make_contour_from_points(pts)

    # --- actual tests ---

    def test_circle_detected(self):
        cnt = self._circle_contour()
        result = classify_contour(cnt)
        self.assertEqual(result, "circle",
                         msg=f"Expected 'circle', got '{result}'")

    def test_square_detected(self):
        cnt = self._rect_contour(100, 100)
        result = classify_contour(cnt)
        self.assertEqual(result, "square",
                         msg=f"Expected 'square', got '{result}'")

    def test_rectangle_detected(self):
        cnt = self._rect_contour(200, 80)
        result = classify_contour(cnt)
        self.assertEqual(result, "rectangle",
                         msg=f"Expected 'rectangle', got '{result}'")

    def test_triangle_detected(self):
        cnt = self._triangle_contour()
        result = classify_contour(cnt)
        self.assertEqual(result, "triangle",
                         msg=f"Expected 'triangle', got '{result}'")

    def test_pentagon_detected(self):
        cnt = self._pentagon_contour()
        result = classify_contour(cnt)
        self.assertIn(result, ("pentagon", "circle"),
                      msg=f"Expected 'pentagon' or 'circle', got '{result}'")

    def test_empty_contour_returns_unknown(self):
        cnt = np.array([], dtype=np.int32).reshape(0, 1, 2)
        result = classify_contour(cnt)
        self.assertEqual(result, "unknown")

    def test_degenerate_contour_returns_unknown(self):
        cnt = _make_contour_from_points([(0, 0), (1, 0)])
        result = classify_contour(cnt)
        self.assertEqual(result, "unknown")


# ---------------------------------------------------------------------------
# ③ ConeDetectorNode — test internal helpers without ROS spin
# ---------------------------------------------------------------------------

class TestConeDetectorHelpers(unittest.TestCase):

    def _make_node(self) -> ConeDetectorNode:
        node = ConeDetectorNode.__new__(ConeDetectorNode)
        node._logger = MagicMock()
        node._params = {
            "hsv_h_low":        5,
            "hsv_h_high":       25,
            "hsv_s_low":        120,
            "hsv_v_low":        80,
            "min_area_px":      400,
            "max_area_px":      80000,
            "min_aspect_ratio": 0.4,
            "max_aspect_ratio": 4.0,
            "depth_roi_px":     5,
            "camera_hfov_deg":  69.0,
            "image_width":      1280,
            "image_height":     720,
        }
        node._p = dict(
            h_low    = 5,
            h_high   = 25,
            s_low    = 120,
            v_low    = 80,
            min_area = 400,
            max_area = 80000,
            min_ar   = 0.4,
            max_ar   = 4.0,
            depth_roi = 5,
            hfov_rad = math.radians(69.0),
            img_w    = 1280,
            img_h    = 720,
        )
        node._depth_image  = None
        node._bridge       = MagicMock()
        node._pub_pose     = MagicMock()
        node._pub_bbox     = MagicMock()
        return node

    def test_bearing_centre_is_zero(self):
        node = self._make_node()
        # Pixel at exact centre → bearing 0
        bearing = node._pixel_to_bearing(640.0, node._p)
        self.assertAlmostEqual(bearing, 0.0, places=5)

    def test_bearing_left_of_centre_is_positive(self):
        node = self._make_node()
        # Pixel left of centre (low x) → positive bearing (object to the left)
        bearing = node._pixel_to_bearing(0.0, node._p)
        self.assertGreater(bearing, 0.0)

    def test_bearing_right_of_centre_is_negative(self):
        node = self._make_node()
        bearing = node._pixel_to_bearing(1280.0, node._p)
        self.assertLess(bearing, 0.0)

    def test_range_returns_nan_without_depth(self):
        node = self._make_node()
        node._depth_image = None
        rng = node._estimate_range(640.0, 360.0, node._p)
        self.assertTrue(math.isnan(rng))

    def test_range_from_depth_mm(self):
        """Depth image with value 2000 (mm) should give range ≈ 2.0 m."""
        node = self._make_node()
        node._depth_image = np.full((720, 1280), 2000.0, dtype=np.float32)
        rng = node._estimate_range(640.0, 360.0, node._p)
        self.assertAlmostEqual(rng, 2.0, places=1)

    def test_range_from_depth_metres(self):
        """Depth image with value 1.5 (m) should give range ≈ 1.5 m."""
        node = self._make_node()
        node._depth_image = np.full((720, 1280), 1.5, dtype=np.float32)
        rng = node._estimate_range(640.0, 360.0, node._p)
        self.assertAlmostEqual(rng, 1.5, places=1)

    def test_publish_encodes_json_bbox(self):
        import json as _json
        node = self._make_node()
        node._pub_pose = MagicMock()
        node._pub_bbox = MagicMock()

        header = MagicMock()
        node._publish(header, cx=640, cy=360, w=80, h=120, rng=2.5, bearing=0.1)

        call_args = node._pub_bbox.publish.call_args[0][0]
        data = _json.loads(call_args.data)
        self.assertIn("cx", data)
        self.assertIn("bearing_rad", data)
        self.assertAlmostEqual(data["range_m"], 2.5, places=2)


# ---------------------------------------------------------------------------
# ④ DistanceEstimatorNode — test _compute_distance logic
# ---------------------------------------------------------------------------

class TestDistanceEstimator(unittest.TestCase):

    def _make_node(self) -> DistanceEstimatorNode:
        node = DistanceEstimatorNode.__new__(DistanceEstimatorNode)
        node._logger  = MagicMock()
        node._fx      = 883.0
        node._fy      = 883.0
        node._cx0     = 640.0
        node._cy0     = 360.0
        node._max_age = 1.0
        node._cone_pose       = None
        node._cone_pose_time  = 0.0
        node._cone_bbox       = None
        node._obj_list        = []
        node._obj_time        = 0.0
        node._pub             = MagicMock()
        return node

    def test_same_position_distance_is_zero(self):
        node = self._make_node()
        node._cone_bbox = {"cx": 640, "cy": 360}
        obj = {"cx": 640, "cy": 360, "w": 50, "h": 50}
        dist, conf = node._compute_distance(cone_z=2.0, obj_z=2.0, obj=obj)
        self.assertAlmostEqual(dist, 0.0, places=3)
        self.assertEqual(conf, "high")

    def test_high_confidence_with_both_depths(self):
        node = self._make_node()
        node._cone_bbox = {"cx": 640, "cy": 360}
        # Object offset by 1 m in x only (pixel offset ≈ fx pixels)
        obj = {"cx": 640 + 883, "cy": 360, "w": 50, "h": 50}
        dist, conf = node._compute_distance(cone_z=2.0, obj_z=2.0, obj=obj)
        self.assertEqual(conf, "high")
        # At z=2m, 883px → 2m lateral offset → 3D dist from same z ≈ 2m
        self.assertAlmostEqual(dist, 2.0, places=0)

    def test_low_confidence_when_one_depth_missing(self):
        node = self._make_node()
        node._cone_bbox = {"cx": 640, "cy": 360}
        obj = {"cx": 640, "cy": 360, "w": 50, "h": 50}
        dist, conf = node._compute_distance(cone_z=float("nan"), obj_z=2.0, obj=obj)
        self.assertEqual(conf, "low")

    def test_no_confidence_when_both_depths_missing(self):
        node = self._make_node()
        node._cone_bbox = {"cx": 640, "cy": 360}
        obj = {"cx": 640, "cy": 360, "w": 50, "h": 50}
        dist, conf = node._compute_distance(
            cone_z=float("nan"), obj_z=float("nan"), obj=obj
        )
        self.assertEqual(conf, "none")
        self.assertTrue(math.isnan(dist))

    def test_deproject_centre_pixel(self):
        node = self._make_node()
        X, Y, Z = node._deproject(640.0, 360.0, 3.0)
        self.assertAlmostEqual(X, 0.0, places=5)
        self.assertAlmostEqual(Y, 0.0, places=5)
        self.assertAlmostEqual(Z, 3.0, places=5)

    def test_deproject_right_pixel(self):
        node = self._make_node()
        # 1 fx pixel right of principal → X = z * 1/fx * fx = z
        X, Y, Z = node._deproject(640.0 + 883.0, 360.0, 2.0)
        self.assertAlmostEqual(X, 2.0, places=3)


# ---------------------------------------------------------------------------
# ⑤ PhotoCaptureNode — test waypoint index deduplication
# ---------------------------------------------------------------------------

class TestPhotoCaptureDedup(unittest.TestCase):

    def test_same_waypoint_not_captured_twice(self):
        """PhotoCaptureNode must skip re-capture if wp_idx already in _captured."""
        # Import lazily so stub is already in place
        from auto_nav.perception.photo_capture import PhotoCaptureNode

        node = PhotoCaptureNode.__new__(PhotoCaptureNode)
        node._logger       = MagicMock()
        node._photo_dir    = "/tmp"
        node._save_raw     = False
        node._jpeg_q       = 90
        node._bridge       = MagicMock()
        node._latest_bgr   = np.zeros((10, 10, 3), dtype=np.uint8)
        node._marker_bbox  = None
        node._obj_list     = []
        node._captured     = set()
        node._pub_photo    = MagicMock()

        saved_calls = []
        node._save_waypoint_photos = lambda idx, bgr: saved_calls.append(idx)

        # Simulate receiving FINAL_APPROACH:0 twice
        msg = MagicMock()
        msg.data = "FINAL_APPROACH:0"
        node._on_wp_status(msg)
        node._on_wp_status(msg)

        self.assertEqual(len(saved_calls), 1,
                         "Expected exactly one save call for the same waypoint index")


if __name__ == "__main__":
    unittest.main()
