# Copyright 2026 team18
"""Unit tests for GPS/odom fusion in outdoor_pose_fuser."""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock


def _install_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')
    rclpy.qos = types.ModuleType('rclpy.qos')

    class _Node:
        def __init__(self, name):  # pragma: no cover - constructor bypassed
            self._name = name

        def create_subscription(self, *args, **kwargs):  # pragma: no cover
            return MagicMock()

        def create_publisher(self, *args, **kwargs):  # pragma: no cover
            return MagicMock()

        def get_logger(self):
            return MagicMock()

    rclpy.node.Node = _Node
    rclpy.qos.QoSProfile = MagicMock()
    rclpy.qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE='RELIABLE')
    rclpy.qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL='TRANSIENT_LOCAL')
    rclpy.init = MagicMock()
    rclpy.spin = MagicMock()
    rclpy.try_shutdown = MagicMock()

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs.msg = types.ModuleType('geometry_msgs.msg')

    class _Header:
        def __init__(self):
            self.stamp = types.SimpleNamespace(sec=0, nanosec=0)
            self.frame_id = 'odom'

    class Quaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Pose:
        def __init__(self):
            self.position = _Point()
            self.orientation = Quaternion()

    class PoseStamped:
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class TransformStamped:
        def __init__(self):
            self.header = _Header()
            self.child_frame_id = ''
            self.transform = types.SimpleNamespace(
                translation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                rotation=Quaternion(),
            )

    geometry_msgs.msg.Quaternion = Quaternion
    geometry_msgs.msg.PoseStamped = PoseStamped
    geometry_msgs.msg.Twist = Twist
    geometry_msgs.msg.TransformStamped = TransformStamped

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs.msg = types.ModuleType('nav_msgs.msg')

    class Odometry:
        def __init__(self):
            self.header = _Header()
            self.child_frame_id = 'base_link'
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(),
                )
            )
            self.twist = types.SimpleNamespace(
                twist=types.SimpleNamespace(
                    linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                )
            )

    nav_msgs.msg.Odometry = Odometry

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs.msg = types.ModuleType('sensor_msgs.msg')

    class NavSatStatus:
        STATUS_NO_FIX = -1
        STATUS_FIX = 0

        def __init__(self):
            self.status = self.STATUS_FIX

    class NavSatFix:
        def __init__(self):
            self.header = _Header()
            self.status = NavSatStatus()
            self.latitude = 0.0
            self.longitude = 0.0

    class Imu:
        def __init__(self):
            self.orientation = Quaternion()

    sensor_msgs.msg.NavSatStatus = NavSatStatus
    sensor_msgs.msg.NavSatFix = NavSatFix
    sensor_msgs.msg.Imu = Imu

    std_msgs = types.ModuleType('std_msgs')
    std_msgs.msg = types.ModuleType('std_msgs.msg')

    class String:
        def __init__(self):
            self.data = ''

    class Bool:
        def __init__(self):
            self.data = False

    std_msgs.msg.String = String
    std_msgs.msg.Bool = Bool

    tf2_ros = types.ModuleType('tf2_ros')
    tf2_ros.TransformBroadcaster = MagicMock()

    for mod, name in [
        (rclpy, 'rclpy'),
        (rclpy.node, 'rclpy.node'),
        (rclpy.qos, 'rclpy.qos'),
        (geometry_msgs, 'geometry_msgs'),
        (geometry_msgs.msg, 'geometry_msgs.msg'),
        (nav_msgs, 'nav_msgs'),
        (nav_msgs.msg, 'nav_msgs.msg'),
        (sensor_msgs, 'sensor_msgs'),
        (sensor_msgs.msg, 'sensor_msgs.msg'),
        (std_msgs, 'std_msgs'),
        (std_msgs.msg, 'std_msgs.msg'),
        (tf2_ros, 'tf2_ros'),
    ]:
        sys.modules.setdefault(name, mod)

    return Odometry, NavSatFix, Imu


_Odometry, _NavSatFix, _Imu = _install_stubs()

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from auto_nav.navigation.geo_localizer import GeoLocalizer  # noqa: E402
from auto_nav.navigation.outdoor_pose_fuser import OutdoorPoseFuserNode, _yaw_to_quat  # noqa: E402


def _make_node() -> OutdoorPoseFuserNode:
    node = OutdoorPoseFuserNode.__new__(OutdoorPoseFuserNode)
    node._gps_alpha = 0.1
    node._gps_jump_reject_m = 8.0
    node._fix_timeout_s = 2.0
    node._use_imu_yaw = True
    node._publish_tf = True
    node._align_on_first_valid_fix = True
    node._localizer = GeoLocalizer()
    node._localizer.set_origin(-31.98, 115.82)
    node._raw_odom = None
    node._latest_imu_orientation = None
    node._pending_gps_enu = None
    node._latest_gps_aligned_xy = None
    node._last_fix_time_s = None
    node._alignment_ready = False
    node._gps_init_e = 0.0
    node._gps_init_n = 0.0
    node._odom_init_x = 0.0
    node._odom_init_y = 0.0
    node._last_nav_xy = None
    node._status = 'WAITING_FOR_FIX'
    node._pub_odom = MagicMock()
    node._pub_status = MagicMock()
    node._tf_pub = MagicMock()
    logger = MagicMock()
    node.get_logger = lambda: logger
    return node


def _make_odom(x: float, y: float, *, sec: int = 0, yaw: float = 0.0) -> _Odometry:
    msg = _Odometry()
    msg.header.stamp.sec = sec
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation = _yaw_to_quat(yaw)
    return msg


def _make_fix(lat: float, lon: float, *, sec: int = 0, valid: bool = True) -> _NavSatFix:
    msg = _NavSatFix()
    msg.header.stamp.sec = sec
    msg.latitude = lat
    msg.longitude = lon
    msg.status.status = 0 if valid else -1
    return msg


def _make_imu(yaw: float) -> _Imu:
    msg = _Imu()
    msg.orientation = _yaw_to_quat(yaw)
    return msg


class TestOutdoorPoseFuser(unittest.TestCase):

    def test_first_valid_fix_establishes_alignment(self):
        node = _make_node()
        node._odom_cb(_make_odom(5.0, -2.0, sec=1))

        fix = _make_fix(-31.97995, 115.82004, sec=1)
        node._fix_cb(fix)

        gps_e, gps_n = node._localizer.gps_to_enu(fix.latitude, fix.longitude)
        self.assertTrue(node._alignment_ready)
        self.assertAlmostEqual(node._gps_init_e, gps_e)
        self.assertAlmostEqual(node._gps_init_n, gps_n)
        self.assertAlmostEqual(node._odom_init_x, 5.0)
        self.assertAlmostEqual(node._odom_init_y, -2.0)

    def test_aligned_fix_matches_odom_offset_before_fusion(self):
        node = _make_node()
        node._odom_cb(_make_odom(10.0, 3.0, sec=1))

        first_fix = _make_fix(-31.97995, 115.82004, sec=1)
        second_fix = _make_fix(-31.97990, 115.82008, sec=2)
        node._fix_cb(first_fix)
        node._fix_cb(second_fix)

        gps_e, gps_n = node._localizer.gps_to_enu(second_fix.latitude, second_fix.longitude)
        aligned_x, aligned_y = node._align_gps_to_odom(gps_e, gps_n)

        self.assertEqual(node._latest_gps_aligned_xy, (aligned_x, aligned_y))
        self.assertGreater(aligned_x, 10.0)
        self.assertGreater(aligned_y, 3.0)

    def test_invalid_fix_falls_back_to_odom_and_imu(self):
        node = _make_node()
        node._imu_cb(_make_imu(1.2))

        odom = _make_odom(2.0, 4.0, sec=5, yaw=0.0)
        node._odom_cb(odom)
        node._fix_cb(_make_fix(-31.97995, 115.82004, sec=5, valid=False))

        nav_msg = node._pub_odom.publish.call_args.args[0]
        self.assertAlmostEqual(nav_msg.pose.pose.position.x, 2.0)
        self.assertAlmostEqual(nav_msg.pose.pose.position.y, 4.0)
        self.assertAlmostEqual(nav_msg.pose.pose.orientation.z, _yaw_to_quat(1.2).z)
        status_msg = node._pub_status.publish.call_args.args[0]
        self.assertEqual(status_msg.data, 'WAITING_FOR_FIX')

    def test_fresh_fix_recovers_smoothly_after_gps_lost(self):
        node = _make_node()
        node._odom_cb(_make_odom(0.0, 0.0, sec=1))
        node._fix_cb(_make_fix(-31.97995, 115.82004, sec=1))
        node._fix_cb(_make_fix(-31.97990, 115.82008, sec=2))

        node._odom_cb(_make_odom(1.0, 1.0, sec=6))
        lost_status = node._pub_status.publish.call_args.args[0]
        self.assertEqual(lost_status.data, 'GPS_LOST')

        node._fix_cb(_make_fix(-31.97989, 115.82009, sec=7))
        nav_msg = node._pub_odom.publish.call_args.args[0]
        self.assertGreater(nav_msg.pose.pose.position.x, 1.0)
        self.assertGreater(nav_msg.pose.pose.position.y, 1.0)
        self.assertLess(nav_msg.pose.pose.position.x, 3.0)
        ready_status = node._pub_status.publish.call_args.args[0]
        self.assertEqual(ready_status.data, 'GPS_READY')

    def test_tf_matches_nav_odom_pose(self):
        node = _make_node()
        node._imu_cb(_make_imu(0.7))
        node._odom_cb(_make_odom(4.0, -1.5, sec=1))
        node._fix_cb(_make_fix(-31.97995, 115.82004, sec=1))

        nav_msg = node._pub_odom.publish.call_args.args[0]
        tf_msg = node._tf_pub.sendTransform.call_args.args[0]

        self.assertAlmostEqual(tf_msg.transform.translation.x, nav_msg.pose.pose.position.x)
        self.assertAlmostEqual(tf_msg.transform.translation.y, nav_msg.pose.pose.position.y)
        self.assertAlmostEqual(tf_msg.transform.rotation.z, nav_msg.pose.pose.orientation.z)
        self.assertAlmostEqual(tf_msg.transform.rotation.w, nav_msg.pose.pose.orientation.w)


if __name__ == '__main__':
    unittest.main()
