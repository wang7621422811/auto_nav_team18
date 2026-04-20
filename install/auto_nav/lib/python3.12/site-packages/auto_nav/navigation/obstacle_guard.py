"""
ObstacleGuardNode — LiDAR-based emergency-stop safety layer.

Monitors the front sector of the LiDAR scan for obstacles closer than
safety_distance_m and publishes /emergency_stop=True when one is found.
The emergency_stop signal is consumed by CmdGateNode, which zeroes the
chassis command regardless of operating mode.

Target-cone whitelist (final-approach mode)
-------------------------------------------
When the robot is in FINAL_APPROACH state the target marker (orange traffic
cone) will be very close and LiDAR would trigger a false emergency stop.
To avoid this, the node maintains a whitelist: scan rays that point toward
the known marker position (within whitelist_radius_m) are excluded from the
safety check.  All other obstacles in the front sector are still checked.

Subscribes:
  /scan               (sensor_msgs/LaserScan)    — LiDAR scan
  /odom               (nav_msgs/Odometry)         — robot pose (for whitelist transform)
  /waypoint/status    (std_msgs/String)           — NAVIGATING / FINAL_APPROACH / etc.
  /marker/detection   (geometry_msgs/PoseStamped) — target cone position (odom frame)

Publishes:
  /emergency_stop     (std_msgs/Bool)             — True = stop; consumed by cmd_gate

Parameters (obstacle_guard section in lidar.yaml):
  safety_distance_m        minimum allowed obstacle distance in front (default 0.5)
  front_sector_deg         total angular width of the front safety zone (default 120.0)
  whitelist_radius_m       exclusion radius around the target cone     (default 0.7)
  estop_cooldown_s         seconds to hold estop=True after clear      (default 0.3)
  guard_rate_hz            watchdog publish rate                       (default 10.0)
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class ObstacleGuardNode(Node):

    _FINAL_APPROACH_STATE = 'FINAL_APPROACH'

    def __init__(self) -> None:
        super().__init__('obstacle_guard')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('safety_distance_m',  0.5)
        self.declare_parameter('front_sector_deg',   120.0)
        self.declare_parameter('whitelist_radius_m', 0.7)
        self.declare_parameter('estop_cooldown_s',   0.3)
        self.declare_parameter('guard_rate_hz',      10.0)

        def _dbl(n: str) -> float:
            return self.get_parameter(n).get_parameter_value().double_value

        self._safety_dist    = _dbl('safety_distance_m')
        self._front_half_rad = math.radians(_dbl('front_sector_deg') / 2.0)
        self._whitelist_r    = _dbl('whitelist_radius_m')
        self._cooldown       = _dbl('estop_cooldown_s')
        rate_hz              = _dbl('guard_rate_hz')

        # ---- State -------------------------------------------------------
        self._robot_x:   float = 0.0
        self._robot_y:   float = 0.0
        self._robot_yaw: float = 0.0
        self._wp_status: str = ''
        self._marker_x: Optional[float] = None
        self._marker_y: Optional[float] = None
        self._estop: bool = False
        self._last_trigger_time: Optional[float] = None   # rclpy clock seconds

        # ---- QoS ---------------------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- Publishers --------------------------------------------------
        self._pub_estop = self.create_publisher(Bool, '/emergency_stop', latched_qos)

        # ---- Subscribers -------------------------------------------------
        self.create_subscription(LaserScan,   '/scan',             self._scan_cb,   10)
        self.create_subscription(Odometry,    '/odom',             self._odom_cb,   10)
        self.create_subscription(String,      '/waypoint/status',  self._status_cb, latched_qos)
        self.create_subscription(PoseStamped, '/marker/detection', self._marker_cb, 10)

        # Watchdog timer: re-publish estop state even if scan is quiet
        self._timer = self.create_timer(1.0 / rate_hz, self._watchdog_tick)

        self.get_logger().info(
            f'ObstacleGuard ready  safety_dist={self._safety_dist}m  '
            f'sector=±{math.degrees(self._front_half_rad):.0f}°  '
            f'whitelist_r={self._whitelist_r}m'
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._robot_x   = p.position.x
        self._robot_y   = p.position.y
        self._robot_yaw = _quat_to_yaw(
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w,
        )

    def _status_cb(self, msg: String) -> None:
        self._wp_status = msg.data

    def _marker_cb(self, msg: PoseStamped) -> None:
        self._marker_x = msg.pose.position.x
        self._marker_y = msg.pose.position.y

    def _scan_cb(self, msg: LaserScan) -> None:
        triggered = self._check_scan(msg)
        now = self.get_clock().now().nanoseconds * 1e-9

        if triggered:
            self._last_trigger_time = now

        # Cooldown: keep estop active for estop_cooldown_s after last trigger
        if self._last_trigger_time is not None:
            if now - self._last_trigger_time <= self._cooldown:
                self._set_estop(True)
            else:
                self._set_estop(False)
        else:
            self._set_estop(False)

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _watchdog_tick(self) -> None:
        """Re-publish current estop state at guard_rate_hz."""
        msg = Bool()
        msg.data = self._estop
        self._pub_estop.publish(msg)

    # ------------------------------------------------------------------
    # Core safety check
    # ------------------------------------------------------------------

    def _check_scan(self, scan: LaserScan) -> bool:
        """
        Return True if any front-sector scan range is closer than
        safety_distance_m (after applying the target-cone whitelist).
        """
        in_final_approach = (self._wp_status == self._FINAL_APPROACH_STATE)
        whitelist_angle_half = self._compute_whitelist_angle() if in_final_approach else None

        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # Only check the front sector
            if abs(angle) > self._front_half_rad:
                continue

            # In final approach: skip rays pointing toward the target cone
            if whitelist_angle_half is not None:
                cone_angle_robot, cone_half = whitelist_angle_half
                if abs(_angle_wrap(angle - cone_angle_robot)) < cone_half:
                    continue

            if r < self._safety_dist:
                self.get_logger().warn(
                    f'ObstacleGuard: obstacle at {r:.2f}m angle={math.degrees(angle):.1f}°  '
                    f'(threshold={self._safety_dist}m)',
                    throttle_duration_sec=1.0,
                )
                return True

        return False

    # ------------------------------------------------------------------
    # Whitelist helper
    # ------------------------------------------------------------------

    def _compute_whitelist_angle(self) -> Optional[tuple[float, float]]:
        """
        Compute the cone's angle in robot frame and the angular half-width
        of the exclusion zone.  Returns None if marker not yet received.
        """
        if self._marker_x is None or self._marker_y is None:
            return None

        dx_odom = self._marker_x - self._robot_x
        dy_odom = self._marker_y - self._robot_y

        # Rotate into robot frame (rotate by -robot_yaw)
        cos_yaw = math.cos(-self._robot_yaw)
        sin_yaw = math.sin(-self._robot_yaw)
        dx_robot = dx_odom * cos_yaw - dy_odom * sin_yaw
        dy_robot = dx_odom * sin_yaw + dy_odom * cos_yaw

        dist = math.hypot(dx_robot, dy_robot)
        if dist < 0.1:
            # Robot is essentially at the cone — exclude a full hemisphere
            return (0.0, math.pi / 2.0)

        cone_angle_robot = math.atan2(dy_robot, dx_robot)
        # Angular half-width of the whitelist exclusion cone
        half_angle = math.atan2(self._whitelist_r, dist)

        return (cone_angle_robot, half_angle)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _set_estop(self, value: bool) -> None:
        if value != self._estop:
            self._estop = value
            self.get_logger().info(
                f'ObstacleGuard: emergency_stop → {value}'
            )
        msg = Bool()
        msg.data = self._estop
        self._pub_estop.publish(msg)


# ---------------------------------------------------------------------------
# Pure utilities
# ---------------------------------------------------------------------------

def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_wrap(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
