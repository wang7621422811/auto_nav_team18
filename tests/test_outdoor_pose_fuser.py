# Copyright 2026 team18
"""Unit tests for the GPS outdoor pose fuser."""

from __future__ import annotations

import math
import sys
import types
import unittest
from unittest.mock import MagicMock


def _install_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')
    rclpy.qos = types.ModuleType('rclpy.qos')

    class _QoSProfile:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class _ReliabilityPolicy:
        RELIABLE = 'RELIABLE'

    class _DurabilityPolicy:
        TRANSIENT_LOCAL = 'TRANSIENT_LOCAL'

    rclpy.qos.QoSProfile = _QoSProfile
    rclpy.qos.ReliabilityPolicy = _ReliabilityPolicy
    rclpy.qos.DurabilityPolicy = _DurabilityPolicy

    class _Node:
        def __init__(self, name):  # pragma: no cover - tests bypass __init__
            self._name = name

        def create_subscription(self, *args, **kwargs):  # pragma: no cover
            return MagicMock()

        def create_publisher(self, *args, **kwargs):  # pragma: no cover
            return MagicMock()

        def get_logger(self):
            return MagicMock()

        def get_clock(self):
            return MagicMock()

    rclpy.node.Node = _Node
    rclpy.init = MagicMock()
    rclpy.spin = MagicMock()
    rclpy.try_shutdown = MagicMock()

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs.msg = types.ModuleType('geometry_msgs.msg')

    class _Header:
        def __init__(self):
            self.stamp = types.SimpleNamespace(sec=0, nanosec=0)
            self.frame_id = ''

    class _Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Quaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class _Pose:
        def __init__(self):
            self.position = _Point()
            self.orientation = _Quaternion()

    class PoseStamped:
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class _Transform:
        def __init__(self):
            self.translation = _Point()
            self.rotation = _Quaternion()

    class TransformStamped:
        def __init__(self):
            self.header = _Header()
            self.child_frame_id = ''
            self.transform = _Transform()

    geometry_msgs.msg.PoseStamped = PoseStamped
    geometry_msgs.msg.Twist = Twist
    geometry_msgs.msg.TransformStamped = TransformStamped

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs.msg = types.ModuleType('nav_msgs.msg')

    class _PoseWithCovariance:
        def __init__(self):
            self.pose = _Pose()
            self.covariance = [0.0] * 36

    class _TwistWithCovariance:
        def __init__(self):
            self.twist = MagicMock()
            self.covariance = [0.0] * 36

    class Odometry:
        def __init__(self):
            self.header = _Header()
            self.child_frame_id = ''
            self.pose = _PoseWithCovariance()
            self.twist = _TwistWithCovariance()

    nav_msgs.msg.Odometry = Odometry

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs.msg = types.ModuleType('sensor_msgs.msg')

    class Imu:
        def __init__(self):
            self.orientation = _Quaternion()

    class _Status:
        def __init__(self):
            self.status = 0

    class NavSatFix:
        def __init__(self):
            self.status = _Status()
            self.latitude = 0.0
            self.longitude = 0.0

    sensor_msgs.msg.Imu = Imu
    sensor_msgs.msg.NavSatFix = NavSatFix

    std_msgs = types.ModuleType('std_msgs')
    std_msgs.msg = types.ModuleType('std_msgs.msg')

    class String:
        def __init__(self, data=''):
            self.data = data

    class Bool:
        def __init__(self, data=False):
            self.data = data

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

    return Odometry, Imu, NavSatFix


_Odometry, _Imu, _NavSatFix = _install_stubs()

from auto_nav.navigation.outdoor_pose_fuser import (  # noqa: E402
    NavState,
    OutdoorPoseFuserNode,
    _quat_to_yaw,
)
from auto_nav.navigation.geo_localizer import GeoLocalizer  # noqa: E402


_EARTH_RADIUS_M = GeoLocalizer._EARTH_RADIUS_M


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _lat_for_north_m(origin_lat: float, north_m: float) -> float:
    return origin_lat + math.degrees(north_m / _EARTH_RADIUS_M)


def _lon_for_east_m(origin_lat: float, origin_lon: float, east_m: float) -> float:
    cos_lat = math.cos(math.radians(origin_lat))
    return origin_lon + math.degrees(east_m / (_EARTH_RADIUS_M * cos_lat))


def _make_node(
    *,
    origin_lat: float = 51.4788,
    origin_lon: float = -0.0106,
    alpha: float = 0.1,
    jump_reject_m: float = 8.0,
    fix_timeout_s: float = 2.0,
    use_imu_yaw: bool = True,
    imu_yaw_offset_deg: float = 0.0,
    heading_align_min_dist_m: float = 2.0,
    heading_align_alpha: float = 0.25,
) -> OutdoorPoseFuserNode:
    node = OutdoorPoseFuserNode.__new__(OutdoorPoseFuserNode)

    node._gps_alpha = alpha
    node._gps_jump_reject_m = jump_reject_m
    node._fix_timeout_s = fix_timeout_s
    node._use_imu_yaw = use_imu_yaw
    node._imu_yaw_offset = math.radians(imu_yaw_offset_deg)
    node._align_on_first_fix = True
    node._heading_align_min_dist_m = heading_align_min_dist_m
    node._heading_align_alpha = heading_align_alpha
    node._publish_tf = True

    node._localizer = GeoLocalizer()
    node._localizer.set_origin(origin_lat, origin_lon)

    node._last_odom_msg = None
    node._last_raw_odom_x = None
    node._last_raw_odom_y = None
    node._imu_yaw = None
    node._last_raw_yaw = None
    node._nav_x = None
    node._nav_y = None

    node._aligned = False
    node._gps_init_e = None
    node._gps_init_n = None
    node._odom_init_x = None
    node._odom_init_y = None
    node._heading_offset = 0.0
    node._heading_aligned = False
    node._last_valid_fix_time = None
    node._state = NavState.WAITING_FOR_FIX

    node._pub_odom = MagicMock()
    node._pub_status = MagicMock()
    node._tf_pub = MagicMock()

    logger = MagicMock()
    node.get_logger = lambda: logger

    node._clock_s = 0.0

    class _Clock:
        def __init__(self, owner):
            self._owner = owner

        def now(self):
            return types.SimpleNamespace(nanoseconds=int(self._owner._clock_s * 1e9))

    node.get_clock = lambda: _Clock(node)
    return node


def _make_odom(x: float, y: float, *, yaw: float = 0.0) -> _Odometry:
    msg = _Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    return msg


def _make_imu(*, yaw: float) -> _Imu:
    msg = _Imu()
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    return msg


def _make_fix(lat: float, lon: float, *, status: int = 0) -> _NavSatFix:
    msg = _NavSatFix()
    msg.status.status = status
    msg.latitude = lat
    msg.longitude = lon
    return msg


def _last_nav_msg(node: OutdoorPoseFuserNode):
    return node._pub_odom.publish.call_args.args[0]


def _last_status(node: OutdoorPoseFuserNode) -> str:
    return node._pub_status.publish.call_args.args[0].data


class TestOutdoorPoseFuser(unittest.TestCase):

    def test_first_valid_fix_aligns_to_current_odom(self):
        node = _make_node(alpha=1.0)

        node._odom_cb(_make_odom(10.0, 5.0))
        node._fix_cb(_make_fix(51.4788, -0.0106))

        nav_msg = _last_nav_msg(node)
        self.assertTrue(node._aligned)
        self.assertAlmostEqual(node._odom_init_x, 10.0)
        self.assertAlmostEqual(node._odom_init_y, 5.0)
        self.assertAlmostEqual(nav_msg.pose.pose.position.x, 0.0)
        self.assertAlmostEqual(nav_msg.pose.pose.position.y, 0.0)
        self.assertEqual(_last_status(node), NavState.GPS_ALIGNING.value)

    def test_aligned_fix_uses_odom_aligned_enu_coordinates(self):
        node = _make_node(alpha=1.0, jump_reject_m=20.0, use_imu_yaw=False)
        origin_lat = 51.4788
        origin_lon = -0.0106

        node._odom_cb(_make_odom(4.0, 6.0))
        node._fix_cb(_make_fix(origin_lat, origin_lon))

        node._odom_cb(_make_odom(14.0, 6.0, yaw=0.0))
        east_lon = _lon_for_east_m(origin_lat, origin_lon, 10.0)
        node._clock_s = 0.1
        node._fix_cb(_make_fix(origin_lat, east_lon))

        nav_msg = _last_nav_msg(node)
        self.assertAlmostEqual(nav_msg.pose.pose.position.x, 10.0, places=3)
        self.assertAlmostEqual(nav_msg.pose.pose.position.y, 0.0, places=3)
        self.assertEqual(_last_status(node), NavState.GPS_READY.value)

    def test_invalid_fix_falls_back_to_odom_plus_imu_yaw(self):
        node = _make_node(use_imu_yaw=True)

        node._imu_cb(_make_imu(yaw=1.2))
        node._odom_cb(_make_odom(1.0, 2.0, yaw=0.1))
        node._fix_cb(_make_fix(51.4788, -0.0106, status=-1))

        nav_msg = _last_nav_msg(node)
        fused_yaw = _quat_to_yaw(
            nav_msg.pose.pose.orientation.x,
            nav_msg.pose.pose.orientation.y,
            nav_msg.pose.pose.orientation.z,
            nav_msg.pose.pose.orientation.w,
        )
        self.assertAlmostEqual(nav_msg.pose.pose.position.x, 1.0)
        self.assertAlmostEqual(nav_msg.pose.pose.position.y, 2.0)
        self.assertAlmostEqual(fused_yaw, 1.2, places=3)
        self.assertEqual(node._state, NavState.ODOM_IMU_ONLY)

    def test_imu_yaw_offset_is_applied_before_fusing_orientation(self):
        node = _make_node(use_imu_yaw=True, imu_yaw_offset_deg=180.0)

        node._imu_cb(_make_imu(yaw=math.radians(40.0)))
        node._odom_cb(_make_odom(1.0, 2.0, yaw=0.0))

        nav_msg = _last_nav_msg(node)
        fused_yaw = _quat_to_yaw(
            nav_msg.pose.pose.orientation.x,
            nav_msg.pose.pose.orientation.y,
            nav_msg.pose.pose.orientation.z,
            nav_msg.pose.pose.orientation.w,
        )
        self.assertAlmostEqual(math.degrees(fused_yaw), -140.0, places=3)

    def test_valid_fix_recovery_returns_smoothly_after_timeout(self):
        node = _make_node(alpha=0.5, jump_reject_m=20.0, fix_timeout_s=1.0, use_imu_yaw=False)
        origin_lat = 51.4788
        origin_lon = -0.0106

        node._odom_cb(_make_odom(0.0, 0.0))
        node._fix_cb(_make_fix(origin_lat, origin_lon))

        node._odom_cb(_make_odom(10.0, 0.0, yaw=0.0))
        east_lon = _lon_for_east_m(origin_lat, origin_lon, 10.0)
        node._clock_s = 0.2
        node._fix_cb(_make_fix(origin_lat, east_lon))
        self.assertAlmostEqual(_last_nav_msg(node).pose.pose.position.x, 10.0, places=3)

        node._clock_s = 2.0
        node._odom_cb(_make_odom(12.0, 0.0, yaw=0.0))
        self.assertEqual(_last_status(node), NavState.GPS_LOST.value)
        self.assertAlmostEqual(_last_nav_msg(node).pose.pose.position.x, 12.0, places=3)

        node._clock_s = 2.1
        node._fix_cb(_make_fix(origin_lat, east_lon))
        self.assertEqual(_last_status(node), NavState.GPS_READY.value)
        self.assertAlmostEqual(_last_nav_msg(node).pose.pose.position.x, 11.0, places=3)

    def test_heading_alignment_rotates_odom_motion_and_yaw_into_enu(self):
        node = _make_node(alpha=1.0, jump_reject_m=20.0, use_imu_yaw=False)
        origin_lat = 51.4788
        origin_lon = -0.0106

        node._odom_cb(_make_odom(0.0, 0.0, yaw=math.pi / 2.0))
        node._fix_cb(_make_fix(origin_lat, origin_lon))

        node._odom_cb(_make_odom(0.0, 10.0, yaw=math.pi / 2.0))
        east_lon = _lon_for_east_m(origin_lat, origin_lon, 10.0)
        node._clock_s = 0.1
        node._fix_cb(_make_fix(origin_lat, east_lon))

        self.assertTrue(node._heading_aligned)
        self.assertAlmostEqual(node._heading_offset, -math.pi / 2.0, places=3)

        node._clock_s = 0.2
        node._odom_cb(_make_odom(0.0, 20.0, yaw=math.pi / 2.0))
        nav_msg = _last_nav_msg(node)
        fused_yaw = _quat_to_yaw(
            nav_msg.pose.pose.orientation.x,
            nav_msg.pose.pose.orientation.y,
            nav_msg.pose.pose.orientation.z,
            nav_msg.pose.pose.orientation.w,
        )
        self.assertAlmostEqual(nav_msg.pose.pose.position.x, 20.0, places=3)
        self.assertAlmostEqual(nav_msg.pose.pose.position.y, 0.0, places=3)
        self.assertAlmostEqual(fused_yaw, 0.0, places=3)

    def test_tf_matches_published_nav_odom(self):
        node = _make_node(alpha=1.0)
        origin_lat = 51.4788
        origin_lon = -0.0106

        node._odom_cb(_make_odom(2.0, 3.0, yaw=0.3))
        node._fix_cb(_make_fix(origin_lat, origin_lon))

        node._clock_s = 0.1
        fix_lat = _lat_for_north_m(origin_lat, 4.0)
        fix_lon = _lon_for_east_m(origin_lat, origin_lon, 7.0)
        node._fix_cb(_make_fix(fix_lat, fix_lon))

        nav_msg = _last_nav_msg(node)
        tf_msg = node._tf_pub.sendTransform.call_args.args[0]

        self.assertEqual(tf_msg.header.frame_id, nav_msg.header.frame_id)
        self.assertEqual(tf_msg.child_frame_id, nav_msg.child_frame_id)
        self.assertAlmostEqual(tf_msg.transform.translation.x, nav_msg.pose.pose.position.x)
        self.assertAlmostEqual(tf_msg.transform.translation.y, nav_msg.pose.pose.position.y)
        self.assertAlmostEqual(tf_msg.transform.rotation.z, nav_msg.pose.pose.orientation.z)
        self.assertAlmostEqual(tf_msg.transform.rotation.w, nav_msg.pose.pose.orientation.w)

    def test_status_tick_republishes_current_state(self):
        node = _make_node()
        node._state = NavState.GPS_READY

        node._status_tick()

        self.assertEqual(_last_status(node), NavState.GPS_READY.value)


if __name__ == '__main__':
    unittest.main()
