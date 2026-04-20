"""
Unit tests for HomePoseRecorderNode.

Tests the business logic (pose capture once, YAML content) with mocked ROS2.
"""

import os
import sys
import tempfile
import types
import unittest
import yaml
from unittest.mock import MagicMock


# ---------------------------------------------------------------------------
# Minimal ROS2 stubs (same pattern as test_watchdog.py)
# ---------------------------------------------------------------------------

def _make_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')
    rclpy.qos  = types.ModuleType('rclpy.qos')

    class _QoSProfile:
        def __init__(self, **kw): pass

    class _Reliability:
        RELIABLE = 'RELIABLE'

    class _Durability:
        TRANSIENT_LOCAL = 'TRANSIENT_LOCAL'

    rclpy.qos.QoSProfile        = _QoSProfile
    rclpy.qos.ReliabilityPolicy = _Reliability
    rclpy.qos.DurabilityPolicy  = _Durability

    class _Node:
        def __init__(self, name): pass
        def get_logger(self):   return MagicMock()
        def get_clock(self):    return MagicMock()
        def declare_parameter(self, *a, **kw): pass
        def get_parameter(self, name): return MagicMock()
        def create_publisher(self, *a, **kw): return MagicMock()
        def create_subscription(self, *a, **kw): return MagicMock()

    rclpy.node.Node    = _Node
    rclpy.init         = MagicMock()
    rclpy.spin         = MagicMock()
    rclpy.try_shutdown = MagicMock()

    # geometry_msgs stubs
    geo = types.ModuleType('geometry_msgs')
    geo.msg = types.ModuleType('geometry_msgs.msg')

    class _Point:
        x = y = z = 0.0

    class _Quaternion:
        x = y = z = 0.0
        w = 1.0

    class _Pose:
        def __init__(self):
            self.position    = _Point()
            self.orientation = _Quaternion()

    class _PoseStamped:
        def __init__(self):
            self.header = MagicMock()
            self.pose   = _Pose()

    geo.msg.PoseStamped = _PoseStamped

    # nav_msgs stubs
    nav = types.ModuleType('nav_msgs')
    nav.msg = types.ModuleType('nav_msgs.msg')

    class _Header:
        frame_id = 'odom'
        stamp    = MagicMock()

    class _PoseWithCovariance:
        def __init__(self):
            self.pose = _Pose()

    class _Odometry:
        def __init__(self):
            self.header = _Header()
            self.pose   = _PoseWithCovariance()

    nav.msg.Odometry = _Odometry

    for mod, name in [
        (rclpy,    'rclpy'),
        (rclpy.node, 'rclpy.node'),
        (rclpy.qos,  'rclpy.qos'),
        (geo,        'geometry_msgs'),
        (geo.msg,    'geometry_msgs.msg'),
        (nav,        'nav_msgs'),
        (nav.msg,    'nav_msgs.msg'),
    ]:
        sys.modules.setdefault(name, mod)

    return rclpy, _Odometry, _PoseStamped


_rclpy, _Odometry, _PoseStamped = _make_stubs()

from auto_nav.mission.home_pose_recorder import HomePoseRecorderNode  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_node(home_pose_file: str) -> HomePoseRecorderNode:
    """Instantiate node with injected state (no live ROS2)."""
    node = HomePoseRecorderNode.__new__(HomePoseRecorderNode)
    node._home_pose_file = home_pose_file
    node._recorded = False
    node._pub      = MagicMock()

    # Bind a single logger mock so all get_logger() calls return the same object
    logger = MagicMock()
    node._logger = logger
    node.get_logger = lambda: logger

    mock_clock = MagicMock()
    mock_now   = MagicMock()
    mock_now.to_msg.return_value = MagicMock(sec=0, nanosec=0)
    mock_clock.now.return_value  = mock_now
    node.get_clock = lambda: mock_clock

    return node


def _make_odom(x: float, y: float, z: float, frame_id: str = 'odom') -> _Odometry:
    msg = _Odometry()
    msg.header.frame_id   = frame_id
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.w = 1.0
    return msg


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestHomePoseRecorderCapturesOnce(unittest.TestCase):

    def test_first_odom_triggers_publish(self):
        with tempfile.TemporaryDirectory() as d:
            node = _make_node(os.path.join(d, 'home_pose.yaml'))
            node._odom_cb(_make_odom(1.0, 2.0, 0.0))
            node._pub.publish.assert_called_once()

    def test_subsequent_odom_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            node = _make_node(os.path.join(d, 'home_pose.yaml'))
            node._odom_cb(_make_odom(1.0, 2.0, 0.0))
            node._odom_cb(_make_odom(5.0, 6.0, 0.0))
            # Still only called once
            node._pub.publish.assert_called_once()

    def test_recorded_flag_set_after_first(self):
        with tempfile.TemporaryDirectory() as d:
            node = _make_node(os.path.join(d, 'home_pose.yaml'))
            self.assertFalse(node._recorded)
            node._odom_cb(_make_odom(0.0, 0.0, 0.0))
            self.assertTrue(node._recorded)


class TestHomePoseRecorderYAML(unittest.TestCase):

    def _record_and_load(self, x, y, z, frame_id='odom'):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'logs', 'home_pose.yaml')
            node = _make_node(path)
            node._odom_cb(_make_odom(x, y, z, frame_id))
            with open(path) as f:
                return yaml.safe_load(f)

    def test_yaml_position_values(self):
        data = self._record_and_load(1.5, -2.3, 0.1)
        self.assertAlmostEqual(data['position']['x'], 1.5)
        self.assertAlmostEqual(data['position']['y'], -2.3)
        self.assertAlmostEqual(data['position']['z'], 0.1)

    def test_yaml_frame_id(self):
        data = self._record_and_load(0.0, 0.0, 0.0, frame_id='map')
        self.assertEqual(data['frame_id'], 'map')

    def test_yaml_contains_orientation(self):
        data = self._record_and_load(0.0, 0.0, 0.0)
        self.assertIn('orientation', data)
        self.assertIn('w', data['orientation'])

    def test_yaml_file_created_in_nested_dir(self):
        """Output file path may not exist yet — node must create parent dirs."""
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'a', 'b', 'c', 'home.yaml')
            node = _make_node(path)
            node._odom_cb(_make_odom(0.0, 0.0, 0.0))
            self.assertTrue(os.path.exists(path))


class TestHomePoseRecorderSaveError(unittest.TestCase):

    def test_io_error_logged_not_raised(self):
        """A bad output path must not crash the node."""
        # Use a path inside a file (impossible directory) to trigger OSError
        with tempfile.NamedTemporaryFile() as f:
            bad_path = os.path.join(f.name, 'impossible', 'home.yaml')
            node = _make_node(bad_path)
            # Should not raise
            node._odom_cb(_make_odom(0.0, 0.0, 0.0))
            node._logger.error.assert_called()


if __name__ == '__main__':
    unittest.main()
