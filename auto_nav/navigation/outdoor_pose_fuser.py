"""Fuse wheel odom, IMU yaw, and GPS fixes into a navigation odometry topic.

This node implements the GPS step plan in ``docs/steps/gps_function.md``:

- GPS fixes are converted to ENU with ``GeoLocalizer``
- the first valid fix is aligned to the robot's current odom pose
- fused pose is published on ``/nav/odom``
- in GPS mode this node also owns the ``odom -> base_link`` TF publication

The implementation intentionally stays lightweight: no EKF, no new dependency,
and a simple configurable blend between raw odom position and aligned GPS
position.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .geo_localizer import GeoLocalizer


class OutdoorPoseFuserNode(Node):
    """Publish GPS-aligned odom for outdoor navigation."""

    def __init__(self) -> None:
        super().__init__('outdoor_pose_fuser')

        self.declare_parameter('gps_fix_topic', '/fix')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_in_topic', '/odom')
        self.declare_parameter('odom_out_topic', '/nav/odom')
        self.declare_parameter('status_topic', '/nav/status')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('gps_position_alpha', 0.1)
        self.declare_parameter('gps_jump_reject_m', 8.0)
        self.declare_parameter('fix_timeout_s', 2.0)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('align_on_first_valid_fix', True)
        self.declare_parameter('gps_origin_lat', float('nan'))
        self.declare_parameter('gps_origin_lon', float('nan'))

        def _str(name: str) -> str:
            return self.get_parameter(name).get_parameter_value().string_value

        def _dbl(name: str) -> float:
            return self.get_parameter(name).get_parameter_value().double_value

        def _bool(name: str) -> bool:
            return self.get_parameter(name).get_parameter_value().bool_value

        self._gps_fix_topic = _str('gps_fix_topic')
        self._imu_topic = _str('imu_topic')
        self._odom_in_topic = _str('odom_in_topic')
        self._odom_out_topic = _str('odom_out_topic')
        self._status_topic = _str('status_topic')
        self._publish_tf = _bool('publish_tf')
        self._gps_alpha = min(max(_dbl('gps_position_alpha'), 0.0), 1.0)
        self._gps_jump_reject_m = _dbl('gps_jump_reject_m')
        self._fix_timeout_s = _dbl('fix_timeout_s')
        self._use_imu_yaw = _bool('use_imu_yaw')
        self._align_on_first_valid_fix = _bool('align_on_first_valid_fix')

        self._localizer = GeoLocalizer()
        origin_lat = _dbl('gps_origin_lat')
        origin_lon = _dbl('gps_origin_lon')
        if math.isfinite(origin_lat) and math.isfinite(origin_lon):
            self._localizer.set_origin(origin_lat, origin_lon)
        else:
            self.get_logger().warn(
                'OutdoorPoseFuser: gps_origin_lat/lon not configured; '
                'GPS fixes will be ignored until an origin is provided'
            )

        self._raw_odom: Optional[Odometry] = None
        self._latest_imu_orientation: Optional[Quaternion] = None
        self._pending_gps_enu: Optional[tuple[float, float]] = None
        self._latest_gps_aligned_xy: Optional[tuple[float, float]] = None
        self._last_fix_time_s: Optional[float] = None
        self._alignment_ready = False
        self._gps_init_e = 0.0
        self._gps_init_n = 0.0
        self._odom_init_x = 0.0
        self._odom_init_y = 0.0
        self._last_nav_xy: Optional[tuple[float, float]] = None
        self._status = 'WAITING_FOR_FIX'

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub_odom = self.create_publisher(Odometry, self._odom_out_topic, 10)
        self._pub_status = self.create_publisher(String, self._status_topic, latched_qos)
        self._tf_pub = TransformBroadcaster(self)

        self.create_subscription(Odometry, self._odom_in_topic, self._odom_cb, 10)
        self.create_subscription(Imu, self._imu_topic, self._imu_cb, 10)
        self.create_subscription(NavSatFix, self._gps_fix_topic, self._fix_cb, 10)

        self.get_logger().info(
            'OutdoorPoseFuser ready '
            f'odom_in={self._odom_in_topic} odom_out={self._odom_out_topic} '
            f'gps_alpha={self._gps_alpha:.2f} fix_timeout={self._fix_timeout_s:.1f}s'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._raw_odom = msg

        if self._pending_gps_enu is not None and not self._alignment_ready:
            self._establish_alignment(*self._pending_gps_enu)
            self._latest_gps_aligned_xy = self._align_gps_to_odom(*self._pending_gps_enu)
            self._pending_gps_enu = None

        self._publish_nav(msg)

    def _imu_cb(self, msg: Imu) -> None:
        self._latest_imu_orientation = msg.orientation

    def _fix_cb(self, msg: NavSatFix) -> None:
        if not self._is_valid_fix(msg):
            return
        if not self._localizer.has_origin:
            return

        gps_e, gps_n = self._localizer.gps_to_enu(msg.latitude, msg.longitude)
        if not self._alignment_ready:
            if self._raw_odom is None or not self._align_on_first_valid_fix:
                self._pending_gps_enu = (gps_e, gps_n)
                self._status = 'GPS_ALIGNING'
                return
            self._establish_alignment(gps_e, gps_n)

        aligned_xy = self._align_gps_to_odom(gps_e, gps_n)
        if self._reject_jump(aligned_xy):
            return

        self._latest_gps_aligned_xy = aligned_xy
        self._last_fix_time_s = self._stamp_to_seconds(msg.header.stamp)

        if self._raw_odom is not None:
            self._publish_nav(self._raw_odom)

    def _establish_alignment(self, gps_e: float, gps_n: float) -> None:
        if self._raw_odom is None:
            return
        self._gps_init_e = gps_e
        self._gps_init_n = gps_n
        self._odom_init_x = self._raw_odom.pose.pose.position.x
        self._odom_init_y = self._raw_odom.pose.pose.position.y
        self._alignment_ready = True
        self.get_logger().info(
            'OutdoorPoseFuser: GPS/odom alignment established '
            f'gps_init=({gps_e:.3f}, {gps_n:.3f}) '
            f'odom_init=({self._odom_init_x:.3f}, {self._odom_init_y:.3f})'
        )

    def _align_gps_to_odom(self, gps_e: float, gps_n: float) -> tuple[float, float]:
        aligned_x = self._odom_init_x + (gps_e - self._gps_init_e)
        aligned_y = self._odom_init_y + (gps_n - self._gps_init_n)
        return aligned_x, aligned_y

    def _reject_jump(self, aligned_xy: tuple[float, float]) -> bool:
        if self._last_nav_xy is None:
            return False
        jump = math.hypot(
            aligned_xy[0] - self._last_nav_xy[0],
            aligned_xy[1] - self._last_nav_xy[1],
        )
        if jump <= self._gps_jump_reject_m:
            return False
        self.get_logger().warn(
            f'OutdoorPoseFuser: rejected GPS jump {jump:.2f}m '
            f'(threshold {self._gps_jump_reject_m:.2f}m)'
        )
        return True

    def _publish_nav(self, raw_odom: Odometry) -> None:
        nav_msg = Odometry()
        nav_msg.header = raw_odom.header
        nav_msg.child_frame_id = raw_odom.child_frame_id
        nav_msg.pose.pose = raw_odom.pose.pose

        pose = raw_odom.pose.pose
        raw_x = pose.position.x
        raw_y = pose.position.y
        raw_z = pose.position.z
        stamp_s = self._stamp_to_seconds(raw_odom.header.stamp)

        if self._gps_is_fresh(stamp_s) and self._latest_gps_aligned_xy is not None:
            gps_x, gps_y = self._latest_gps_aligned_xy
            nav_x = raw_x + self._gps_alpha * (gps_x - raw_x)
            nav_y = raw_y + self._gps_alpha * (gps_y - raw_y)
            self._status = 'GPS_READY'
        else:
            nav_x = raw_x
            nav_y = raw_y
            if self._alignment_ready:
                self._status = 'GPS_LOST'
            elif self._pending_gps_enu is not None:
                self._status = 'GPS_ALIGNING'
            else:
                self._status = 'WAITING_FOR_FIX'

        nav_msg.pose.pose.position.x = nav_x
        nav_msg.pose.pose.position.y = nav_y
        nav_msg.pose.pose.position.z = raw_z
        nav_msg.pose.pose.orientation = self._select_orientation(pose.orientation)

        if hasattr(raw_odom, 'twist'):
            nav_msg.twist = raw_odom.twist

        self._pub_odom.publish(nav_msg)
        self._publish_status(self._status)
        self._last_nav_xy = (nav_x, nav_y)

        if self._publish_tf:
            self._publish_tf_msg(nav_msg)

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

    def _publish_tf_msg(self, nav_msg: Odometry) -> None:
        parent = (nav_msg.header.frame_id or '').strip() or 'odom'
        child = (nav_msg.child_frame_id or '').strip() or 'base_link'

        tf_msg = TransformStamped()
        tf_msg.header.stamp = nav_msg.header.stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = nav_msg.pose.pose.position.x
        tf_msg.transform.translation.y = nav_msg.pose.pose.position.y
        tf_msg.transform.translation.z = nav_msg.pose.pose.position.z
        tf_msg.transform.rotation = nav_msg.pose.pose.orientation
        self._tf_pub.sendTransform(tf_msg)

    def _select_orientation(self, raw_orientation: Quaternion) -> Quaternion:
        if self._use_imu_yaw and self._latest_imu_orientation is not None:
            yaw = _quat_to_yaw(
                self._latest_imu_orientation.x,
                self._latest_imu_orientation.y,
                self._latest_imu_orientation.z,
                self._latest_imu_orientation.w,
            )
            return _yaw_to_quat(yaw)
        return raw_orientation

    def _gps_is_fresh(self, now_s: float) -> bool:
        if self._last_fix_time_s is None:
            return False
        return (now_s - self._last_fix_time_s) <= self._fix_timeout_s

    @staticmethod
    def _is_valid_fix(msg: NavSatFix) -> bool:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return False
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return False
        return True

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(getattr(stamp, 'sec', 0)) + float(getattr(stamp, 'nanosec', 0)) * 1e-9


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quat(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OutdoorPoseFuserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
