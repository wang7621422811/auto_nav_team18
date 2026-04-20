"""
GapPlannerNode — LiDAR-based gap selection for obstacle avoidance.

Reads /scan, identifies passable gaps in the scan, scores them with a
weighted formula, and publishes the best gap direction as a local
navigation target (a point ~lookahead_m ahead in the odom frame).

Gap scoring formula
-------------------
  score =
      w_goal      * cos(gap_center_angle - angle_to_waypoint)  # goal alignment
    + w_clearance * (gap_width_rad / max_gap_width_rad)        # gap openness
    + w_progress  * cos(gap_center_angle)                      # forward progress
    - w_smooth    * |gap_center_angle - prev_gap_angle|        # smoothness

Subscribes:
  /scan                (sensor_msgs/LaserScan)    — LiDAR scan
  /odom                (nav_msgs/Odometry)         — robot pose (odom frame)
  /waypoint/current    (geometry_msgs/PoseStamped) — current waypoint target

Publishes:
  /gap/local_target    (geometry_msgs/PoseStamped) — best-gap lookahead point
                         in the odom frame; not published when no valid gap

Parameters (gap_planner section in lidar.yaml):
  gap_min_clearance_m   minimum range to treat a ray as "free"    (default 0.8)
  gap_min_width_rad     minimum angular width for a valid gap      (default 0.30)
  lookahead_m           lookahead distance for the target point    (default 1.5)
  w_goal                weight — goal-direction alignment           (default 3.0)
  w_clearance           weight — gap openness                       (default 1.5)
  w_progress            weight — forward progress (cos angle)       (default 1.0)
  w_smooth              weight — turn-smoothness penalty            (default 0.5)
"""

from __future__ import annotations

import math
from typing import List, NamedTuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Data class
# ---------------------------------------------------------------------------

class Gap(NamedTuple):
    center_angle: float   # rad — in robot/laser frame
    width_rad: float      # rad — angular width of the gap
    mean_range: float     # m   — mean free range inside the gap
    score: float          # higher is better


# ---------------------------------------------------------------------------
# GapPlannerNode
# ---------------------------------------------------------------------------

class GapPlannerNode(Node):

    def __init__(self) -> None:
        super().__init__('gap_planner')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('gap_min_clearance_m', 0.8)
        self.declare_parameter('gap_min_width_rad',   0.30)
        self.declare_parameter('lookahead_m',         1.5)
        self.declare_parameter('w_goal',      3.0)
        self.declare_parameter('w_clearance', 1.5)
        self.declare_parameter('w_progress',  1.0)
        self.declare_parameter('w_smooth',    0.5)

        def _dbl(n: str) -> float:
            return self.get_parameter(n).get_parameter_value().double_value

        self._min_clearance = _dbl('gap_min_clearance_m')
        self._min_width_rad = _dbl('gap_min_width_rad')
        self._lookahead     = _dbl('lookahead_m')
        self._w_goal        = _dbl('w_goal')
        self._w_clearance   = _dbl('w_clearance')
        self._w_progress    = _dbl('w_progress')
        self._w_smooth      = _dbl('w_smooth')

        # ---- State -------------------------------------------------------
        self._robot_x:   float = 0.0
        self._robot_y:   float = 0.0
        self._robot_yaw: float = 0.0
        self._wp_x: Optional[float] = None
        self._wp_y: Optional[float] = None
        self._prev_gap_angle: float = 0.0   # smoothness across ticks

        # ---- QoS ---------------------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- Publishers --------------------------------------------------
        self._pub_target = self.create_publisher(
            PoseStamped, '/gap/local_target', 10
        )

        # ---- Subscribers -------------------------------------------------
        self.create_subscription(LaserScan,   '/scan',             self._scan_cb,  10)
        self.create_subscription(Odometry,    '/odom',             self._odom_cb,  10)
        self.create_subscription(PoseStamped, '/waypoint/current', self._wp_cb,    latched_qos)

        self.get_logger().info(
            f'GapPlanner ready  clearance={self._min_clearance}m  '
            f'min_width={math.degrees(self._min_width_rad):.1f}°  '
            f'lookahead={self._lookahead}m'
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

    def _scan_cb(self, msg: LaserScan) -> None:
        """Main processing — runs on every new scan."""
        gaps = self._find_gaps(msg)
        if not gaps:
            return   # no gap found; path_follower falls back to direct driving

        best = self._score_and_pick(gaps, msg)
        if best is None:
            return

        self._prev_gap_angle = best.center_angle
        self._publish_local_target(best.center_angle, best.mean_range)

    # ------------------------------------------------------------------
    # Gap finding
    # ------------------------------------------------------------------

    def _find_gaps(self, scan: LaserScan) -> List[Gap]:
        """
        Identify contiguous angular sectors where all ranges exceed
        gap_min_clearance_m. Each such sector is a candidate gap.
        """
        ranges = scan.ranges
        n = len(ranges)
        if n == 0:
            return []

        free: List[bool] = []
        valid_ranges: List[float] = []
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                r_use = scan.range_max
            else:
                r_use = r
            is_free = (
                r_use > self._min_clearance
                and r_use <= scan.range_max
            )
            free.append(is_free)
            valid_ranges.append(r_use)

        gaps: List[Gap] = []
        i = 0
        while i < n:
            if not free[i]:
                i += 1
                continue

            # Start of a free run
            j = i
            while j < n and free[j]:
                j += 1

            # Gap spans indices [i, j)
            width_rad = (j - i) * scan.angle_increment
            if width_rad >= self._min_width_rad:
                center_idx = (i + j) // 2
                center_angle = scan.angle_min + center_idx * scan.angle_increment
                mean_r = sum(valid_ranges[i:j]) / (j - i)
                gaps.append(Gap(
                    center_angle=center_angle,
                    width_rad=width_rad,
                    mean_range=mean_r,
                    score=0.0,          # filled below
                ))
            i = j

        return gaps

    # ------------------------------------------------------------------
    # Gap scoring
    # ------------------------------------------------------------------

    def _score_and_pick(self, gaps: List[Gap], scan: LaserScan) -> Optional[Gap]:
        """Score all candidate gaps and return the highest-scoring one."""
        # Maximum possible gap width (entire scan) — for normalisation
        total_scan_width = scan.angle_max - scan.angle_min
        max_gap_width = total_scan_width if total_scan_width > 0 else math.pi

        # Angle from robot to waypoint in robot frame
        angle_to_wp = self._angle_to_waypoint()

        scored: List[Gap] = []
        for g in gaps:
            # Goal alignment: +1 when gap points toward waypoint
            goal_score = math.cos(
                _angle_wrap(g.center_angle - angle_to_wp)
            )

            # Clearance: normalised gap width
            clearance_score = min(g.width_rad / max_gap_width, 1.0)

            # Progress: prefer forward direction (angle ≈ 0)
            progress_score = math.cos(g.center_angle)

            # Smoothness penalty: penalise large turn changes
            smooth_penalty = abs(_angle_wrap(g.center_angle - self._prev_gap_angle))

            score = (
                self._w_goal      * goal_score
                + self._w_clearance * clearance_score
                + self._w_progress  * progress_score
                - self._w_smooth    * smooth_penalty
            )

            scored.append(Gap(
                center_angle=g.center_angle,
                width_rad=g.width_rad,
                mean_range=g.mean_range,
                score=score,
            ))

        if not scored:
            return None

        return max(scored, key=lambda g: g.score)

    # ------------------------------------------------------------------
    # Target publishing
    # ------------------------------------------------------------------

    def _publish_local_target(self, gap_angle: float, gap_range: float) -> None:
        """
        Project a point `lookahead_m` ahead along gap_angle in the robot frame,
        then transform to odom frame and publish.
        """
        # Use the lesser of lookahead_m and 90 % of gap range for safety
        dist = min(self._lookahead, gap_range * 0.9)

        # Gap angle is in robot frame; transform to odom frame
        global_angle = self._robot_yaw + gap_angle
        target_x = self._robot_x + dist * math.cos(global_angle)
        target_y = self._robot_y + dist * math.sin(global_angle)

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
        """
        Angle from robot to current waypoint in the robot's local frame (rad).
        Returns 0.0 if no waypoint received yet.
        """
        if self._wp_x is None or self._wp_y is None:
            return 0.0   # default: prefer straight ahead
        dx = self._wp_x - self._robot_x
        dy = self._wp_y - self._robot_y
        # Global bearing to waypoint
        global_bearing = math.atan2(dy, dx)
        # Convert to robot frame
        return _angle_wrap(global_bearing - self._robot_yaw)


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
    node = GapPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
