"""
WeavePlannerNode — corridor-aware gap planner for the WP[0]→WP[1] weave segment.

Only active when /navigation/segment == "1" (navigating toward waypoint index 1,
i.e., the second waypoint).  In that segment the robot must weave through a cone
corridor rather than taking a wide detour around the entire cone field.

How it works
------------
1.  A corridor is defined by the straight line from WP[0] to WP[1].
2.  Candidate gaps are scored with the same formula as GapPlannerNode, but with
    an additional heavy penalty proportional to how far outside the corridor the
    chosen heading would take the robot:

      score =
          w_goal      * cos(gap_angle - angle_to_waypoint)
        + w_clearance * (gap_width_rad / max_gap_width_rad)
        + w_progress  * cos(gap_angle)          # forward progress
        - w_smooth    * |gap_angle - prev_angle| # smoothness
        - w_corridor  * max(0, |corridor_lateral_dist| - corridor_half_width_m)

    The corridor penalty dominates when a gap would steer outside the corridor,
    keeping the robot threading through the cone field instead of going wide.

3.  Published lookahead point is additionally clamped: if the unclamped target
    is outside the corridor it is projected back onto the corridor centre-line.

Subscribes:
  /scan                  (sensor_msgs/LaserScan)    — LiDAR scan
  /odom                  (nav_msgs/Odometry)         — robot pose
  /waypoint/current      (geometry_msgs/PoseStamped) — current target (WP[1])
  /navigation/segment    (std_msgs/String)           — "1" = active, else silent

Publishes:
  /weave/local_target    (geometry_msgs/PoseStamped) — corridor-constrained
                           lookahead point; published only while segment == "1"

Parameters (weave_planner section in lidar.yaml):
  waypoints_file            path to waypoints YAML (to read WP[0] and WP[1])
  gap_min_clearance_m       same as GapPlanner             (default 0.8)
  gap_min_width_rad         same as GapPlanner             (default 0.30)
  corridor_half_width_m     half-width of the weave corridor in m (default 2.0)
  lookahead_m               lookahead distance for target   (default 1.2)
  w_goal                    weight — goal alignment         (default 3.0)
  w_clearance               weight — gap openness           (default 1.5)
  w_progress                weight — forward bias           (default 1.0)
  w_smooth                  weight — smoothness             (default 0.5)
  w_corridor                weight — corridor penalty       (default 5.0)
"""

from __future__ import annotations

import math
from typing import List, NamedTuple, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .waypoint_provider import WaypointProvider


# ---------------------------------------------------------------------------
# Data class
# ---------------------------------------------------------------------------

class Gap(NamedTuple):
    center_angle: float
    width_rad:    float
    mean_range:   float
    score:        float


# ---------------------------------------------------------------------------
# WeavePlannerNode
# ---------------------------------------------------------------------------

class WeavePlannerNode(Node):

    _ACTIVE_SEGMENT = '1'   # segment index that triggers weave mode

    def __init__(self) -> None:
        super().__init__('weave_planner')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('waypoints_file',       '')
        self.declare_parameter('gap_min_clearance_m',  0.8)
        self.declare_parameter('gap_min_width_rad',    0.30)
        self.declare_parameter('corridor_half_width_m', 2.0)
        self.declare_parameter('lookahead_m',          1.2)
        self.declare_parameter('w_goal',      3.0)
        self.declare_parameter('w_clearance', 1.5)
        self.declare_parameter('w_progress',  1.0)
        self.declare_parameter('w_smooth',    0.5)
        self.declare_parameter('w_corridor',  5.0)

        def _dbl(n: str) -> float:
            return self.get_parameter(n).get_parameter_value().double_value

        self._min_clearance    = _dbl('gap_min_clearance_m')
        self._min_width_rad    = _dbl('gap_min_width_rad')
        self._corridor_half_w  = _dbl('corridor_half_width_m')
        self._lookahead        = _dbl('lookahead_m')
        self._w_goal           = _dbl('w_goal')
        self._w_clearance      = _dbl('w_clearance')
        self._w_progress       = _dbl('w_progress')
        self._w_smooth         = _dbl('w_smooth')
        self._w_corridor       = _dbl('w_corridor')

        # ---- Load waypoints ----------------------------------------------
        wp_file = (
            self.get_parameter('waypoints_file').get_parameter_value().string_value
        )
        # Corridor endpoints (WP[0] → WP[1])
        self._corridor_start: Optional[Tuple[float, float]] = None
        self._corridor_end:   Optional[Tuple[float, float]] = None

        if wp_file:
            try:
                provider = WaypointProvider(wp_file)
                if len(provider) >= 2:
                    wp0 = provider[0]
                    wp1 = provider[1]
                    self._corridor_start = (wp0.x, wp0.y)
                    self._corridor_end   = (wp1.x, wp1.y)
                    self.get_logger().info(
                        f'WeavePlanner: corridor  '
                        f'WP0=({wp0.x:.2f},{wp0.y:.2f})  '
                        f'WP1=({wp1.x:.2f},{wp1.y:.2f})  '
                        f'half_w={self._corridor_half_w}m'
                    )
                else:
                    self.get_logger().warn(
                        'WeavePlanner: fewer than 2 waypoints — corridor disabled'
                    )
            except Exception as exc:
                self.get_logger().error(f'WeavePlanner: failed to load waypoints: {exc}')
        else:
            self.get_logger().warn(
                'WeavePlanner: waypoints_file not set — corridor not configured'
            )

        # ---- State -------------------------------------------------------
        self._robot_x:   float = 0.0
        self._robot_y:   float = 0.0
        self._robot_yaw: float = 0.0
        self._wp_x: Optional[float] = None
        self._wp_y: Optional[float] = None
        self._segment: str = ''
        self._prev_gap_angle: float = 0.0
        self._active: bool = False

        # ---- QoS ---------------------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- Publishers --------------------------------------------------
        self._pub_target = self.create_publisher(
            PoseStamped, '/weave/local_target', 10
        )

        # ---- Subscribers -------------------------------------------------
        self.create_subscription(LaserScan,   '/scan',               self._scan_cb,    10)
        self.create_subscription(Odometry,    '/odom',               self._odom_cb,    10)
        self.create_subscription(PoseStamped, '/waypoint/current',   self._wp_cb,      latched_qos)
        self.create_subscription(String,      '/navigation/segment', self._segment_cb, latched_qos)

        self.get_logger().info(
            f'WeavePlanner ready  active on segment={self._ACTIVE_SEGMENT}  '
            f'corridor_half_w={self._corridor_half_w}m  lookahead={self._lookahead}m'
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

    def _wp_cb(self, msg: PoseStamped) -> None:
        self._wp_x = msg.pose.position.x
        self._wp_y = msg.pose.position.y

    def _segment_cb(self, msg: String) -> None:
        self._segment = msg.data
        was_active = self._active
        self._active = (self._segment == self._ACTIVE_SEGMENT)
        if self._active != was_active:
            self.get_logger().info(
                f'WeavePlanner: {"ACTIVATED" if self._active else "deactivated"}'
            )

    def _scan_cb(self, msg: LaserScan) -> None:
        """Main processing — only runs when segment is active."""
        if not self._active:
            return

        gaps = self._find_gaps(msg)
        if not gaps:
            return

        best = self._score_and_pick(gaps, msg)
        if best is None:
            return

        self._prev_gap_angle = best.center_angle
        self._publish_local_target(best.center_angle, best.mean_range)

    # ------------------------------------------------------------------
    # Gap finding (shared with GapPlanner)
    # ------------------------------------------------------------------

    def _find_gaps(self, scan: LaserScan) -> List[Gap]:
        n = len(scan.ranges)
        if n == 0:
            return []

        free: List[bool] = []
        valid_ranges: List[float] = []
        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                r = scan.range_max
            is_free = (r > self._min_clearance and r <= scan.range_max)
            free.append(is_free)
            valid_ranges.append(r)

        gaps: List[Gap] = []
        i = 0
        while i < n:
            if not free[i]:
                i += 1
                continue

            j = i
            while j < n and free[j]:
                j += 1

            width_rad = (j - i) * scan.angle_increment
            if width_rad >= self._min_width_rad:
                center_idx   = (i + j) // 2
                center_angle = scan.angle_min + center_idx * scan.angle_increment
                mean_r       = sum(valid_ranges[i:j]) / (j - i)
                gaps.append(Gap(center_angle, width_rad, mean_r, 0.0))
            i = j

        return gaps

    # ------------------------------------------------------------------
    # Gap scoring with corridor penalty
    # ------------------------------------------------------------------

    def _score_and_pick(self, gaps: List[Gap], scan: LaserScan) -> Optional[Gap]:
        total_scan_width = scan.angle_max - scan.angle_min
        max_gap_width = total_scan_width if total_scan_width > 0 else math.pi

        angle_to_wp = self._angle_to_waypoint()

        scored: List[Gap] = []
        for g in gaps:
            goal_score      = math.cos(_angle_wrap(g.center_angle - angle_to_wp))
            clearance_score = min(g.width_rad / max_gap_width, 1.0)
            progress_score  = math.cos(g.center_angle)
            smooth_penalty  = abs(_angle_wrap(g.center_angle - self._prev_gap_angle))

            # Corridor penalty: how far outside the corridor the gap heading leads
            corridor_penalty = self._corridor_penalty(g.center_angle, g.mean_range)

            score = (
                self._w_goal      * goal_score
                + self._w_clearance * clearance_score
                + self._w_progress  * progress_score
                - self._w_smooth    * smooth_penalty
                - self._w_corridor  * corridor_penalty
            )
            scored.append(Gap(g.center_angle, g.width_rad, g.mean_range, score))

        return max(scored, key=lambda g: g.score) if scored else None

    # ------------------------------------------------------------------
    # Corridor penalty
    # ------------------------------------------------------------------

    def _corridor_penalty(self, gap_angle_robot: float, gap_range: float) -> float:
        """
        Compute how far outside the WP0→WP1 corridor the projected gap point
        would land.  Returns metres beyond the corridor boundary (0 if inside).
        """
        if self._corridor_start is None or self._corridor_end is None:
            return 0.0

        dist = min(self._lookahead, gap_range * 0.9)
        global_angle = self._robot_yaw + gap_angle_robot
        proj_x = self._robot_x + dist * math.cos(global_angle)
        proj_y = self._robot_y + dist * math.sin(global_angle)

        lateral_dist = _point_to_line_dist(
            proj_x, proj_y,
            self._corridor_start[0], self._corridor_start[1],
            self._corridor_end[0],   self._corridor_end[1],
        )

        return max(0.0, lateral_dist - self._corridor_half_w)

    # ------------------------------------------------------------------
    # Target publishing
    # ------------------------------------------------------------------

    def _publish_local_target(self, gap_angle: float, gap_range: float) -> None:
        dist = min(self._lookahead, gap_range * 0.9)

        global_angle = self._robot_yaw + gap_angle
        target_x = self._robot_x + dist * math.cos(global_angle)
        target_y = self._robot_y + dist * math.sin(global_angle)

        # Clamp target to corridor if corridor is configured
        if self._corridor_start is not None and self._corridor_end is not None:
            target_x, target_y = _clamp_to_corridor(
                target_x, target_y,
                self._corridor_start, self._corridor_end,
                self._corridor_half_w,
            )

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = float(target_x)
        msg.pose.position.y = float(target_y)
        msg.pose.orientation.w = 1.0
        self._pub_target.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _angle_to_waypoint(self) -> float:
        if self._wp_x is None or self._wp_y is None:
            return 0.0
        dx = self._wp_x - self._robot_x
        dy = self._wp_y - self._robot_y
        return _angle_wrap(math.atan2(dy, dx) - self._robot_yaw)


# ---------------------------------------------------------------------------
# Pure geometry utilities
# ---------------------------------------------------------------------------

def _point_to_line_dist(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> float:
    """Perpendicular distance from point (px, py) to the infinite line through A–B."""
    ab_x = bx - ax
    ab_y = by - ay
    ab_len = math.hypot(ab_x, ab_y)
    if ab_len < 1e-9:
        return math.hypot(px - ax, py - ay)
    # Signed lateral distance = cross product / length
    cross = (px - ax) * ab_y - (py - ay) * ab_x
    return abs(cross) / ab_len


def _clamp_to_corridor(
    px: float, py: float,
    corridor_start: tuple[float, float],
    corridor_end:   tuple[float, float],
    half_width: float,
) -> tuple[float, float]:
    """
    If (px, py) is outside the corridor (perpendicular distance > half_width),
    project it back onto the corridor boundary.
    """
    ax, ay = corridor_start
    bx, by = corridor_end
    ab_x = bx - ax
    ab_y = by - ay
    ab_len = math.hypot(ab_x, ab_y)
    if ab_len < 1e-9:
        return (px, py)

    # Unit vectors along and perpendicular to corridor
    u_x = ab_x / ab_len
    u_y = ab_y / ab_len
    n_x = -u_y   # left normal
    n_y =  u_x

    # Signed lateral distance from corridor centre-line
    signed_lat = (px - ax) * n_x + (py - ay) * n_y

    if abs(signed_lat) <= half_width:
        return (px, py)   # already inside — no clamping

    # Clamp to ±half_width
    clamped_lat = math.copysign(half_width, signed_lat)
    delta = clamped_lat - signed_lat
    return (px + delta * n_x, py + delta * n_y)


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
    node = WeavePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
