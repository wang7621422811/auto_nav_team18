# Copyright 2026 team18
"""Fuse GPS, IMU yaw, and wheel odometry into a navigation odometry stream.

This node implements the GPS-mode design from ``docs/steps/gps_function.md``:

1. GPS fixes are converted into a shared ENU frame with ``GeoLocalizer``.
2. The first valid fix anchors the raw odom stream to the shared ENU frame.
3. Wheel-odom deltas are rotated into ENU once enough motion exists to learn the
   odom-to-ENU heading offset.
4. Accepted GPS fixes pull the fused pose back toward the ENU-aligned GPS
   position while refining that heading offset.
5. The node publishes both ``/nav/odom`` and a matching ``odom -> base_link`` TF.

The implementation intentionally stays lightweight: no EKF, no extra
dependencies, and no changes to existing navigation-node interfaces.
"""

from __future__ import annotations

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .geo_localizer import GeoLocalizer


class NavState(str, Enum):
    """Published navigation-status values for GPS outdoor mode."""

    WAITING_FOR_FIX = 'WAITING_FOR_FIX'
    ODOM_IMU_ONLY = 'ODOM_IMU_ONLY'
    GPS_ALIGNING = 'GPS_ALIGNING'
    GPS_READY = 'GPS_READY'
    GPS_LOST = 'GPS_LOST'


class OutdoorPoseFuserNode(Node):
    """Publish a GPS-aligned odometry stream for the existing navigation stack."""

    def __init__(self) -> None:
        super().__init__('outdoor_pose_fuser')

        # Topic / publish configuration
        self.declare_parameter('gps_fix_topic', '/fix')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_in_topic', '/odom')
        self.declare_parameter('odom_out_topic', '/nav/odom')
        self.declare_parameter('status_topic', '/nav/status')
        self.declare_parameter('publish_tf', True)

        # GPS fusion tuning
        self.declare_parameter('gps_origin_lat', 51.4788)
        self.declare_parameter('gps_origin_lon', -0.0106)
        self.declare_parameter('gps_position_alpha', 0.1)
        self.declare_parameter('gps_jump_reject_m', 8.0)
        self.declare_parameter('fix_timeout_s', 2.0)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('imu_yaw_offset_deg', 0.0)
        self.declare_parameter('align_on_first_valid_fix', True)
        self.declare_parameter('heading_alignment_min_dist_m', 2.0)
        self.declare_parameter('heading_alignment_alpha', 0.25)

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
        self._imu_yaw_offset = math.radians(_dbl('imu_yaw_offset_deg'))
        self._align_on_first_fix = _bool('align_on_first_valid_fix')
        self._heading_align_min_dist_m = max(_dbl('heading_alignment_min_dist_m'), 0.0)
        self._heading_align_alpha = min(max(_dbl('heading_alignment_alpha'), 0.0), 1.0)

        self._localizer = GeoLocalizer()
        self._localizer.set_origin(
            _dbl('gps_origin_lat'),
            _dbl('gps_origin_lon'),
        )

        # Raw sensor state
        self._last_odom_msg: Odometry | None = None
        self._last_raw_odom_x: float | None = None
        self._last_raw_odom_y: float | None = None
        self._imu_yaw: float | None = None
        self._last_raw_yaw: float | None = None

        # Fused navigation pose
        self._nav_x: float | None = None
        self._nav_y: float | None = None

        # GPS alignment state
        self._aligned = False
        self._gps_init_e: float | None = None
        self._gps_init_n: float | None = None
        self._odom_init_x: float | None = None
        self._odom_init_y: float | None = None
        self._heading_offset: float = 0.0
        self._heading_aligned = False
        self._last_valid_fix_time: float | None = None

        self._state = NavState.WAITING_FOR_FIX

        # Transient-local status keeps the latest state available to late
        # subscribers, while the heartbeat helps default volatile CLI echo
        # subscribers observe the current state during field debugging.
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_odom = self.create_publisher(Odometry, self._odom_out_topic, 10)
        self._pub_status = self.create_publisher(String, self._status_topic, status_qos)
        self._tf_pub = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(Odometry, self._odom_in_topic, self._odom_cb, 10)
        self.create_subscription(Imu, self._imu_topic, self._imu_cb, 10)
        self.create_subscription(NavSatFix, self._gps_fix_topic, self._fix_cb, 10)
        self._status_timer = self.create_timer(1.0, self._status_tick)

        self._publish_status()
        self.get_logger().info(
            f'OutdoorPoseFuser ready  odom_in={self._odom_in_topic}  '
            f'odom_out={self._odom_out_topic}  gps_fix={self._gps_fix_topic}  '
            f'publish_tf={self._publish_tf}'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        """Propagate the fused pose with wheel-odometry increments."""
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        raw_yaw = _quat_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        if self._nav_x is None or self._nav_y is None:
            self._nav_x = raw_x
            self._nav_y = raw_y
        elif self._last_raw_odom_x is not None and self._last_raw_odom_y is not None:
            delta_x = raw_x - self._last_raw_odom_x
            delta_y = raw_y - self._last_raw_odom_y
            if self._aligned and self._heading_aligned:
                delta_x, delta_y = _rotate_xy(delta_x, delta_y, self._heading_offset)
            self._nav_x += delta_x
            self._nav_y += delta_y

        self._last_raw_odom_x = raw_x
        self._last_raw_odom_y = raw_y
        self._last_raw_yaw = raw_yaw
        self._last_odom_msg = msg

        if self._last_valid_fix_time is None:
            self._set_state(NavState.ODOM_IMU_ONLY)
        elif self._gps_fix_timed_out():
            self._set_state(NavState.GPS_LOST)

        self._publish_nav_odom(msg.header.stamp)

    def _imu_cb(self, msg: Imu) -> None:
        """Cache a planar yaw estimate from IMU orientation."""
        q = msg.orientation
        if not _quat_is_finite(q.x, q.y, q.z, q.w):
            return
        self._imu_yaw = _angle_wrap(
            _quat_to_yaw(q.x, q.y, q.z, q.w) + self._imu_yaw_offset
        )

    def _fix_cb(self, msg: NavSatFix) -> None:
        """Blend the fused pose back toward the aligned GPS position."""
        if not self._is_valid_fix(msg):
            return

        if self._last_odom_msg is None:
            self.get_logger().warn(
                'Received GPS fix before odometry; waiting for /odom before alignment',
                throttle_duration_sec=2.0,
            )
            return

        east_m, north_m = self._localizer.gps_to_enu(msg.latitude, msg.longitude)

        if not self._aligned:
            self._initialise_alignment(east_m, north_m)

        if not self._aligned or self._gps_init_e is None or self._gps_init_n is None:
            return

        if self._nav_x is None or self._nav_y is None:
            self._nav_x = east_m
            self._nav_y = north_m

        heading_aligned_now = self._update_heading_alignment(east_m, north_m)
        target_x = east_m
        target_y = north_m

        has_previous_fix = self._last_valid_fix_time is not None
        jump_m = math.hypot(target_x - self._nav_x, target_y - self._nav_y)
        if has_previous_fix and jump_m > self._gps_jump_reject_m:
            self.get_logger().warn(
                f'Rejecting GPS correction jump={jump_m:.2f}m '
                f'(threshold={self._gps_jump_reject_m:.2f}m)',
                throttle_duration_sec=1.0,
            )
            return

        self._blend_toward(
            target_x,
            target_y,
            force=(not has_previous_fix) or heading_aligned_now,
        )
        self._last_valid_fix_time = self._now_s()
        self._set_state(NavState.GPS_READY if has_previous_fix else NavState.GPS_ALIGNING)

        self._publish_nav_odom(self._last_odom_msg.header.stamp)

    # ------------------------------------------------------------------
    # Fusion helpers
    # ------------------------------------------------------------------

    def _initialise_alignment(self, east_m: float, north_m: float) -> None:
        """Anchor the raw odom stream to the shared GPS ENU frame."""
        if self._last_odom_msg is None:
            return

        raw_pose = self._last_odom_msg.pose.pose
        if self._align_on_first_fix:
            self._gps_init_e = east_m
            self._gps_init_n = north_m
            self._odom_init_x = raw_pose.position.x
            self._odom_init_y = raw_pose.position.y
        else:
            self._gps_init_e = 0.0
            self._gps_init_n = 0.0
            self._odom_init_x = raw_pose.position.x
            self._odom_init_y = raw_pose.position.y

        self._aligned = True
        self._nav_x = east_m
        self._nav_y = north_m
        self.get_logger().info(
            'GPS alignment initialised at '
            f'enu=({east_m:.2f}, {north_m:.2f})'
        )

    def _update_heading_alignment(self, east_m: float, north_m: float) -> bool:
        """Estimate the raw-odom to ENU heading offset from observed motion."""
        if (
            self._odom_init_x is None
            or self._odom_init_y is None
            or self._last_odom_msg is None
            or self._gps_init_e is None
            or self._gps_init_n is None
        ):
            return False

        odom_dx = self._last_odom_msg.pose.pose.position.x - self._odom_init_x
        odom_dy = self._last_odom_msg.pose.pose.position.y - self._odom_init_y
        gps_dx = east_m - self._gps_init_e
        gps_dy = north_m - self._gps_init_n

        odom_dist = math.hypot(odom_dx, odom_dy)
        gps_dist = math.hypot(gps_dx, gps_dy)
        if min(odom_dist, gps_dist) < self._heading_align_min_dist_m:
            return False

        candidate = _angle_wrap(math.atan2(gps_dy, gps_dx) - math.atan2(odom_dy, odom_dx))
        if not self._heading_aligned:
            self._heading_offset = candidate
            self._heading_aligned = True
            self.get_logger().info(
                'Heading alignment initialised  '
                f'offset={self._heading_offset:.3f}rad'
            )
            return True

        delta = _angle_wrap(candidate - self._heading_offset)
        self._heading_offset = _angle_wrap(
            self._heading_offset + self._heading_align_alpha * delta
        )
        return False

    def _blend_toward(self, target_x: float, target_y: float, *, force: bool) -> None:
        """Move the fused pose toward the GPS-aligned target."""
        if force or self._nav_x is None or self._nav_y is None:
            self._nav_x = target_x
            self._nav_y = target_y
            return

        self._nav_x += self._gps_alpha * (target_x - self._nav_x)
        self._nav_y += self._gps_alpha * (target_y - self._nav_y)

    def _publish_nav_odom(self, stamp) -> None:
        """Publish the fused odometry and, optionally, the matching TF."""
        if self._last_odom_msg is None or self._nav_x is None or self._nav_y is None:
            return

        raw_msg = self._last_odom_msg
        raw_pose = raw_msg.pose.pose
        fused_orientation = self._resolve_orientation(raw_pose.orientation)

        nav_msg = Odometry()
        nav_msg.header.stamp = stamp
        nav_msg.header.frame_id = (raw_msg.header.frame_id or 'odom').strip() or 'odom'
        nav_msg.child_frame_id = (raw_msg.child_frame_id or 'base_link').strip() or 'base_link'
        nav_msg.pose.pose.position.x = self._nav_x
        nav_msg.pose.pose.position.y = self._nav_y
        nav_msg.pose.pose.position.z = raw_pose.position.z
        nav_msg.pose.pose.orientation.x = fused_orientation[0]
        nav_msg.pose.pose.orientation.y = fused_orientation[1]
        nav_msg.pose.pose.orientation.z = fused_orientation[2]
        nav_msg.pose.pose.orientation.w = fused_orientation[3]

        if hasattr(nav_msg.pose, 'covariance') and hasattr(raw_msg.pose, 'covariance'):
            nav_msg.pose.covariance = list(raw_msg.pose.covariance)
        if hasattr(nav_msg, 'twist') and hasattr(raw_msg, 'twist'):
            nav_msg.twist = raw_msg.twist

        self._pub_odom.publish(nav_msg)

        if self._tf_pub is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = nav_msg.header.frame_id
            tf_msg.child_frame_id = nav_msg.child_frame_id
            tf_msg.transform.translation.x = self._nav_x
            tf_msg.transform.translation.y = self._nav_y
            tf_msg.transform.translation.z = raw_pose.position.z
            tf_msg.transform.rotation.x = fused_orientation[0]
            tf_msg.transform.rotation.y = fused_orientation[1]
            tf_msg.transform.rotation.z = fused_orientation[2]
            tf_msg.transform.rotation.w = fused_orientation[3]
            self._tf_pub.sendTransform(tf_msg)

    def _resolve_orientation(self, raw_orientation) -> tuple[float, float, float, float]:
        """Publish yaw in the same ENU frame used by the fused position."""
        if self._use_imu_yaw and self._imu_yaw is not None:
            yaw = self._imu_yaw
        elif self._last_raw_yaw is not None:
            yaw = self._last_raw_yaw
        else:
            return (
                raw_orientation.x,
                raw_orientation.y,
                raw_orientation.z,
                raw_orientation.w,
            )

        if self._heading_aligned:
            yaw = _angle_wrap(yaw + self._heading_offset)
        return _yaw_to_quat(yaw)

    def _is_valid_fix(self, msg: NavSatFix) -> bool:
        """Accept fixes with a valid status and finite coordinates."""
        if msg.status.status < 0:
            return False
        return (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
        )

    def _gps_fix_timed_out(self) -> bool:
        """Return True when the last accepted GPS fix is older than fix_timeout_s."""
        if self._last_valid_fix_time is None:
            return False
        return (self._now_s() - self._last_valid_fix_time) > self._fix_timeout_s

    def _set_state(self, state: NavState) -> None:
        if state == self._state:
            return
        self._state = state
        self._publish_status()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = str(self._state.value)
        self._pub_status.publish(msg)

    def _status_tick(self) -> None:
        """Re-publish the latest state for default CLI subscribers."""
        self._publish_status()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x * cos_yaw - y * sin_yaw,
        x * sin_yaw + y * cos_yaw,
    )


def _angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _quat_is_finite(qx: float, qy: float, qz: float, qw: float) -> bool:
    return (
        math.isfinite(qx)
        and math.isfinite(qy)
        and math.isfinite(qz)
        and math.isfinite(qw)
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OutdoorPoseFuserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
