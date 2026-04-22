"""
Unit tests for Step 3: LiDAR avoidance + weaving components.

Tests are pure-Python (no ROS2 runtime needed) and cover:
  - GapPlannerNode._find_gaps: gap detection from scan data
  - GapPlannerNode._score_and_pick: gap scoring formula
  - ObstacleGuardNode._check_scan: front-sector obstacle detection
  - ObstacleGuardNode._compute_whitelist_angle: cone exclusion zone
  - WeavePlannerNode._corridor_penalty: corridor penalty geometry
  - weave_planner._point_to_line_dist: perpendicular distance utility
  - weave_planner._clamp_to_corridor: corridor clamping geometry
  - path_follower: /navigation/segment published correctly (via state inspection)
"""

from __future__ import annotations

import math
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Minimal ROS2 stubs so modules can be imported without a running ROS2 node
# ---------------------------------------------------------------------------

def _make_ros_stubs():
    """Create minimal rclpy / ROS2 message stubs."""
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod.init = lambda *a, **kw: None
    rclpy_mod.spin = lambda *a, **kw: None
    rclpy_mod.try_shutdown = lambda: None

    node_mod = types.ModuleType('rclpy.node')
    class FakeNode:
        def __init__(self, name):
            self._name = name
            self._logger = _FakeLogger()
        def declare_parameter(self, name, default=None): pass
        def get_parameter(self, name):
            return _FakeParam(0.0)
        def create_publisher(self, *a, **kw): return MagicMock()
        def create_subscription(self, *a, **kw): return MagicMock()
        def create_timer(self, *a, **kw): return MagicMock()
        def get_logger(self): return self._logger
        def get_clock(self):
            clk = MagicMock()
            clk.now.return_value.nanoseconds = 0
            return clk
        def destroy_node(self): pass
    node_mod.Node = FakeNode

    class _FakeLogger:
        def info(self, *a, **kw): pass
        def warn(self, *a, **kw): pass
        def error(self, *a, **kw): pass

    class _FakeParam:
        def __init__(self, val): self._val = val
        def get_parameter_value(self):
            v = MagicMock()
            v.double_value = self._val
            v.string_value = ''
            return v

    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile       = MagicMock(return_value=MagicMock())
    qos_mod.ReliabilityPolicy = MagicMock()
    qos_mod.DurabilityPolicy  = MagicMock()

    # geometry / nav / sensor / std messages
    def _make_msg_mod(name, *classes):
        mod = types.ModuleType(name)
        for cls in classes:
            setattr(mod, cls, type(cls, (), {'__init__': lambda s: None}))
        return mod

    geo_mod = types.ModuleType('geometry_msgs.msg')
    class PoseStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id='')
            self.pose   = types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
    class Twist:
        def __init__(self):
            self.linear  = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
    geo_mod.PoseStamped = PoseStamped
    geo_mod.Twist       = Twist

    nav_mod = types.ModuleType('nav_msgs.msg')
    class Odometry:
        def __init__(self):
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
    nav_mod.Odometry = Odometry

    sensor_mod = types.ModuleType('sensor_msgs.msg')
    class LaserScan:
        def __init__(self):
            self.angle_min       = -math.pi / 2
            self.angle_max       =  math.pi / 2
            self.angle_increment = math.pi / 180  # 1 degree
            self.range_min       = 0.1
            self.range_max       = 10.0
            self.ranges          = []
    sensor_mod.LaserScan = LaserScan

    std_mod = types.ModuleType('std_msgs.msg')
    class Bool:
        def __init__(self): self.data = False
    class String:
        def __init__(self): self.data = ''
    std_mod.Bool   = Bool
    std_mod.String = String

    # ament_index stub (needed by launch file but not by unit tests directly)
    ament_mod = types.ModuleType('ament_index_python.packages')
    ament_mod.get_package_share_directory = lambda pkg: '/tmp'

    for mod_name, mod in [
        ('rclpy',                      rclpy_mod),
        ('rclpy.node',                 node_mod),
        ('rclpy.qos',                  qos_mod),
        ('geometry_msgs',              types.ModuleType('geometry_msgs')),
        ('geometry_msgs.msg',          geo_mod),
        ('nav_msgs',                   types.ModuleType('nav_msgs')),
        ('nav_msgs.msg',               nav_mod),
        ('sensor_msgs',                types.ModuleType('sensor_msgs')),
        ('sensor_msgs.msg',            sensor_mod),
        ('std_msgs',                   types.ModuleType('std_msgs')),
        ('std_msgs.msg',               std_mod),
        ('ament_index_python',         types.ModuleType('ament_index_python')),
        ('ament_index_python.packages', ament_mod),
    ]:
        sys.modules[mod_name] = mod

_make_ros_stubs()


# ---------------------------------------------------------------------------
# Import modules under test (after stubs installed)
# ---------------------------------------------------------------------------

import importlib, pathlib, os

_NAV_DIR = pathlib.Path(__file__).parent.parent / 'auto_nav' / 'navigation'
sys.path.insert(0, str(_NAV_DIR.parent.parent))  # workspace root

# Import the pure-Python utility functions directly from source files
from auto_nav.navigation.gap_planner     import GapPlannerNode, _angle_wrap as gap_wrap
from auto_nav.navigation.obstacle_guard  import ObstacleGuardNode, _angle_wrap as obs_wrap
from auto_nav.navigation.weave_planner   import (
    WeavePlannerNode,
    _point_to_line_dist,
    _clamp_to_corridor,
    _angle_wrap as weave_wrap,
)


# ---------------------------------------------------------------------------
# Helpers for building fake LaserScan objects
# ---------------------------------------------------------------------------

def _make_scan(
    ranges,
    angle_min=-math.pi / 2,
    angle_max=math.pi / 2,
    range_min=0.1,
    range_max=10.0,
):
    from sensor_msgs.msg import LaserScan
    scan = LaserScan()
    n = len(ranges)
    scan.angle_min       = angle_min
    scan.angle_max       = angle_max
    scan.angle_increment = (angle_max - angle_min) / max(n - 1, 1)
    scan.range_min       = range_min
    scan.range_max       = range_max
    scan.ranges          = list(ranges)
    return scan


def _make_gap_node():
    """Instantiate a GapPlannerNode with default parameters."""
    node = GapPlannerNode.__new__(GapPlannerNode)
    node._min_clearance = 0.8
    node._min_width_rad = 0.30
    node._lookahead     = 1.5
    node._goal_clear_half_rad = math.radians(6.0)
    node._goal_clear_min_fraction = 0.8
    node._w_goal        = 3.0
    node._w_clearance   = 1.5
    node._w_progress    = 1.0
    node._w_smooth      = 0.5
    node._robot_x       = 0.0
    node._robot_y       = 0.0
    node._robot_yaw     = 0.0
    node._wp_x          = 5.0   # waypoint 5 m ahead
    node._wp_y          = 0.0
    node._prev_gap_angle = 0.0
    return node


def _make_obs_node():
    """Instantiate an ObstacleGuardNode with default parameters."""
    node = ObstacleGuardNode.__new__(ObstacleGuardNode)
    node._safety_dist    = 0.5
    node._front_half_rad = math.radians(60.0)   # ±60° (120° total)
    node._whitelist_r    = 0.7
    node._cooldown       = 0.3
    node._scan_min_range = 0.0
    node._min_hit_rays   = 1
    node._estop          = False
    node._last_trigger_time = None
    node._robot_x   = 0.0
    node._robot_y   = 0.0
    node._robot_yaw = 0.0
    node._wp_status = ''
    node._marker_x  = None
    node._marker_y  = None
    # Stub logger and publisher
    node._logger = MagicMock()
    node._pub_estop = MagicMock()
    # Attach real logger via a fake get_logger
    node.get_logger = lambda: MagicMock(
        warn=lambda *a, **kw: None,
        info=lambda *a, **kw: None,
    )
    return node


# ===========================================================================
# Tests: GapPlannerNode gap finding
# ===========================================================================

class TestGapFinding(unittest.TestCase):

    def setUp(self):
        self.node = _make_gap_node()

    def test_all_free_returns_one_gap(self):
        """A completely clear scan produces at least one gap."""
        scan = _make_scan([5.0] * 181)   # 181 rays, all 5 m
        gaps = self.node._find_gaps(scan)
        self.assertGreater(len(gaps), 0)

    def test_all_blocked_returns_no_gap(self):
        """A scan with everything at 0.3 m (< clearance 0.8) produces no gaps."""
        scan = _make_scan([0.3] * 181)
        gaps = self.node._find_gaps(scan)
        self.assertEqual(len(gaps), 0)

    def test_obstacle_in_centre_splits_into_two_gaps(self):
        """A single obstacle cluster in the middle produces two side gaps."""
        ranges = [5.0] * 181
        # Block 10 rays around centre (index ~90)
        for i in range(82, 100):
            ranges[i] = 0.2
        scan = _make_scan(ranges)
        gaps = self.node._find_gaps(scan)
        self.assertGreaterEqual(len(gaps), 2)

    def test_gap_width_filter(self):
        """Gaps narrower than gap_min_width_rad are rejected."""
        # Very narrow free corridor: only 5 rays out of 181
        ranges = [0.2] * 181
        for i in range(88, 93):   # 5 rays ≈ 5° ≈ 0.087 rad
            ranges[i] = 5.0
        scan = _make_scan(ranges)
        # min_width_rad=0.30 → 5 rays @ ~0.017 rad/ray = 0.085 rad < 0.30
        gaps = self.node._find_gaps(scan)
        self.assertEqual(len(gaps), 0)

    def test_inf_ranges_treated_as_free(self):
        """Inf ranges (open sky) are treated as free (range_max)."""
        scan = _make_scan([float('inf')] * 181)
        gaps = self.node._find_gaps(scan)
        self.assertGreater(len(gaps), 0)

    def test_nan_ranges_excluded(self):
        """NaN ranges are replaced by range_max and treated as free."""
        scan = _make_scan([float('nan')] * 181)
        gaps = self.node._find_gaps(scan)
        self.assertGreater(len(gaps), 0)


# ===========================================================================
# Tests: GapPlannerNode gap scoring
# ===========================================================================

class TestGapScoring(unittest.TestCase):

    def setUp(self):
        self.node = _make_gap_node()

    def test_forward_gap_preferred_when_waypoint_is_ahead(self):
        """With waypoint straight ahead, gap at angle 0 should score highest."""
        scan = _make_scan([5.0] * 181)
        # Simulate two gaps: one straight ahead, one to the side
        from auto_nav.navigation.gap_planner import Gap
        gaps = [
            Gap(center_angle=0.0,              width_rad=0.5, mean_range=5.0, score=0.0),
            Gap(center_angle=math.radians(60), width_rad=0.5, mean_range=5.0, score=0.0),
        ]
        best = self.node._score_and_pick(gaps, scan)
        self.assertAlmostEqual(best.center_angle, 0.0, places=3)

    def test_scores_are_finite(self):
        """Scoring must not produce NaN or Inf."""
        scan = _make_scan([5.0] * 181)
        from auto_nav.navigation.gap_planner import Gap
        gaps = [Gap(center_angle=a, width_rad=0.4, mean_range=5.0, score=0.0)
                for a in [0.0, 0.5, -0.5, 1.0, -1.0]]
        best = self.node._score_and_pick(gaps, scan)
        self.assertTrue(math.isfinite(best.score))

    def test_no_gaps_returns_none(self):
        scan = _make_scan([5.0] * 181)
        result = self.node._score_and_pick([], scan)
        self.assertIsNone(result)

    def test_guided_gap_angle_biases_toward_waypoint_inside_gap(self):
        """When the chosen gap already contains the waypoint bearing, follow that bearing."""
        from auto_nav.navigation.gap_planner import Gap
        self.node._wp_x = 0.0
        self.node._wp_y = 5.0   # waypoint at +90° in robot frame
        gap = Gap(center_angle=0.0, width_rad=math.pi, mean_range=5.0, score=0.0)

        guided = self.node._guided_gap_angle(gap)

        self.assertAlmostEqual(guided, math.pi / 2, places=6)

    def test_guided_gap_angle_clamps_to_gap_edge(self):
        """When waypoint bearing lies outside the gap, clamp to nearest valid edge."""
        from auto_nav.navigation.gap_planner import Gap
        self.node._wp_x = 0.0
        self.node._wp_y = 5.0   # waypoint at +90°
        gap = Gap(center_angle=0.0, width_rad=math.radians(40.0), mean_range=5.0, score=0.0)

        guided = self.node._guided_gap_angle(gap)

        self.assertAlmostEqual(guided, math.radians(20.0), places=6)

    def test_goal_direction_range_accepts_clear_waypoint_bearing(self):
        """If waypoint window is clear enough, prefer direct waypoint tracking."""
        scan = _make_scan([5.0] * 181)
        self.node._wp_x = 0.0
        self.node._wp_y = 5.0

        goal_range = self.node._goal_direction_range(scan)

        self.assertIsNotNone(goal_range)
        self.assertGreater(goal_range, self.node._lookahead)

    def test_goal_direction_range_rejects_blocked_waypoint_bearing(self):
        """Blocked waypoint bearing must fall back to gap competition."""
        ranges = [5.0] * 181
        for i in range(174, 181):
            ranges[i] = 0.2
        scan = _make_scan(ranges)
        self.node._wp_x = 0.0
        self.node._wp_y = 5.0

        goal_range = self.node._goal_direction_range(scan)

        self.assertIsNone(goal_range)


# ===========================================================================
# Tests: ObstacleGuardNode safety check
# ===========================================================================

class TestObstacleGuard(unittest.TestCase):

    def setUp(self):
        self.node = _make_obs_node()

    def test_no_obstacle_no_estop(self):
        """Clear scan → no emergency stop."""
        scan = _make_scan([5.0] * 181)
        result = self.node._check_scan(scan)
        self.assertFalse(result)

    def test_close_obstacle_front_triggers_estop(self):
        """Obstacle at 0.3 m straight ahead → emergency stop."""
        ranges = [5.0] * 181
        ranges[89] = 0.3
        ranges[90] = 0.3   # centre ray = 0° = straight ahead
        ranges[91] = 0.3
        scan = _make_scan(ranges)
        result = self.node._check_scan(scan)
        self.assertTrue(result)

    def test_close_obstacle_behind_no_estop(self):
        """Obstacle behind the robot (outside front sector) must not trigger estop."""
        # front_half_rad = 60°, so |angle| > 60° is ignored
        # With 181 rays from -90° to +90°, indices 0 and 180 are at ±90°
        # Behind: we need angle > 90° but our scan only goes ±90°
        # Test with a narrow sector: obstacle at index 0 = -90° = outside ±60°
        ranges = [5.0] * 181
        ranges[0] = 0.3   # at -90° — outside the ±60° front sector
        scan = _make_scan(ranges)
        result = self.node._check_scan(scan)
        self.assertFalse(result)

    def test_obstacle_at_sector_edge_no_estop(self):
        """Obstacle exactly at the sector boundary is not checked."""
        # Sector half = 60°. Ray at index corresponding to ~61° should be ignored.
        # With 181 rays from -90° to +90°, angle = -90° + idx * (180°/180) = -90° + idx
        # angle = 61° → idx = 61 + 90 = 151
        ranges = [5.0] * 181
        ranges[151] = 0.3   # at +61° — just outside ±60° sector
        scan = _make_scan(ranges)
        result = self.node._check_scan(scan)
        self.assertFalse(result)

    def test_whitelist_excludes_cone_in_final_approach(self):
        """In FINAL_APPROACH, obstacle at cone location must not trigger estop."""
        self.node._wp_status = 'FINAL_APPROACH'
        # Cone is 1 m directly ahead of robot (in odom frame = same as robot frame here)
        self.node._marker_x = 1.0
        self.node._marker_y = 0.0

        ranges = [5.0] * 181
        # Place a close obstacle straight ahead (where cone is)
        ranges[90] = 0.3
        scan = _make_scan(ranges)
        result = self.node._check_scan(scan)
        # Should NOT trigger because the ray is whitelisted
        self.assertFalse(result)

    def test_whitelist_does_not_mask_other_obstacles(self):
        """Whitelist must only exclude the cone direction; other obstacles still trigger."""
        self.node._wp_status = 'FINAL_APPROACH'
        # Cone 1 m ahead
        self.node._marker_x = 1.0
        self.node._marker_y = 0.0

        ranges = [5.0] * 181
        # Obstacle NOT at cone direction (45° to the side, index ~135)
        ranges[135] = 0.3   # at +45° — outside whitelist, inside front sector
        scan = _make_scan(ranges)
        result = self.node._check_scan(scan)
        self.assertTrue(result)

    def test_single_hit_spike_does_not_trigger_when_min_hit_rays_is_three(self):
        """A 1-ray spike should be treated as noise when clustering is enabled."""
        self.node._min_hit_rays = 3
        ranges = [5.0] * 181
        ranges[90] = 0.3
        scan = _make_scan(ranges)
        result = self.node._check_scan(scan)
        self.assertFalse(result)

    def test_three_adjacent_hits_trigger_when_min_hit_rays_is_three(self):
        """A contiguous obstacle cluster must still trigger emergency stop."""
        self.node._min_hit_rays = 3
        ranges = [5.0] * 181
        ranges[89] = 0.3
        ranges[90] = 0.2
        ranges[91] = 0.3
        scan = _make_scan(ranges)
        result = self.node._check_scan(scan)
        self.assertTrue(result)

    def test_scan_min_range_filters_ultra_close_spike(self):
        """Returns inside the blind zone should be ignored before clustering."""
        self.node._scan_min_range = 0.25
        ranges = [5.0] * 181
        ranges[89] = 0.05
        ranges[90] = 0.05
        ranges[91] = 0.05
        scan = _make_scan(ranges, range_min=0.05)
        result = self.node._check_scan(scan)
        self.assertFalse(result)


# ===========================================================================
# Tests: obstacle_guard whitelist angle computation
# ===========================================================================

class TestWhitelistAngle(unittest.TestCase):

    def setUp(self):
        self.node = _make_obs_node()

    def test_no_marker_returns_none(self):
        self.node._marker_x = None
        self.node._marker_y = None
        result = self.node._compute_whitelist_angle()
        self.assertIsNone(result)

    def test_cone_ahead_gives_angle_zero(self):
        """Cone directly ahead in odom frame → robot-frame angle ≈ 0."""
        self.node._robot_yaw = 0.0
        self.node._marker_x  = 2.0
        self.node._marker_y  = 0.0
        result = self.node._compute_whitelist_angle()
        self.assertIsNotNone(result)
        cone_angle, half_angle = result
        self.assertAlmostEqual(cone_angle, 0.0, places=3)
        self.assertGreater(half_angle, 0.0)

    def test_cone_to_the_right_gives_negative_angle(self):
        """Cone to the right of robot → negative robot-frame angle."""
        self.node._robot_yaw = 0.0
        self.node._marker_x  = 0.0
        self.node._marker_y  = -2.0   # right side in ENU (y negative = east)
        result = self.node._compute_whitelist_angle()
        self.assertIsNotNone(result)
        cone_angle, _ = result
        self.assertLess(cone_angle, 0.0)


# ===========================================================================
# Tests: weave_planner geometry utilities
# ===========================================================================

class TestPointToLineDist(unittest.TestCase):

    def test_point_on_line_has_zero_distance(self):
        # Line from (0,0) to (10,0); point (5,0) is on the line
        d = _point_to_line_dist(5.0, 0.0,  0.0, 0.0,  10.0, 0.0)
        self.assertAlmostEqual(d, 0.0, places=6)

    def test_perpendicular_point(self):
        # Line from (0,0) to (10,0); point (5,3) is 3 m above
        d = _point_to_line_dist(5.0, 3.0,  0.0, 0.0,  10.0, 0.0)
        self.assertAlmostEqual(d, 3.0, places=6)

    def test_diagonal_line(self):
        # Line from (0,0) to (4,4); point (0,4): distance = 4/sqrt(2) ≈ 2.828
        d = _point_to_line_dist(0.0, 4.0,  0.0, 0.0,  4.0, 4.0)
        self.assertAlmostEqual(d, 4.0 / math.sqrt(2), places=5)

    def test_degenerate_line_falls_back_to_distance(self):
        # A and B are the same point
        d = _point_to_line_dist(3.0, 4.0,  1.0, 1.0,  1.0, 1.0)
        expected = math.hypot(3.0 - 1.0, 4.0 - 1.0)
        self.assertAlmostEqual(d, expected, places=6)


class TestClampToCorridor(unittest.TestCase):

    def test_point_inside_returns_unchanged(self):
        pt = _clamp_to_corridor(
            5.0, 0.5,
            corridor_start=(0.0, 0.0),
            corridor_end=(10.0, 0.0),
            half_width=1.0,
        )
        self.assertAlmostEqual(pt[0], 5.0, places=6)
        self.assertAlmostEqual(pt[1], 0.5, places=6)

    def test_point_outside_right_clamped(self):
        # Point 3 m to the right of a horizontal corridor; half_width=1
        pt = _clamp_to_corridor(
            5.0, -3.0,
            corridor_start=(0.0, 0.0),
            corridor_end=(10.0, 0.0),
            half_width=1.0,
        )
        # Should be clamped to y = -1.0
        self.assertAlmostEqual(pt[1], -1.0, places=5)

    def test_point_outside_left_clamped(self):
        pt = _clamp_to_corridor(
            5.0, 4.0,
            corridor_start=(0.0, 0.0),
            corridor_end=(10.0, 0.0),
            half_width=1.0,
        )
        self.assertAlmostEqual(pt[1], 1.0, places=5)


# ===========================================================================
# Tests: WeavePlannerNode corridor penalty
# ===========================================================================

class TestWeavePlannerCorridorPenalty(unittest.TestCase):

    def _make_weave_node(self, corridor_half_w=2.0):
        node = WeavePlannerNode.__new__(WeavePlannerNode)
        node._min_clearance   = 0.8
        node._min_width_rad   = 0.30
        node._corridor_half_w = corridor_half_w
        node._lookahead       = 1.2
        node._goal_clear_half_rad = math.radians(6.0)
        node._goal_clear_min_fraction = 0.8
        node._w_goal          = 3.0
        node._w_clearance     = 1.5
        node._w_progress      = 1.0
        node._w_smooth        = 0.5
        node._w_corridor      = 5.0
        node._robot_x         = 0.0
        node._robot_y         = 0.0
        node._robot_yaw       = 0.0
        node._wp_x            = 0.0
        node._wp_y            = 10.0
        node._corridor_start  = (0.0, 0.0)
        node._corridor_end    = (0.0, 10.0)  # vertical corridor along y-axis
        node._prev_gap_angle  = 0.0
        node._segment         = '1'
        node._active          = True
        return node

    def test_forward_gap_no_penalty(self):
        """Gap heading straight along the corridor (angle=π/2 for y-axis corridor) → zero penalty."""
        node = self._make_weave_node()
        # Corridor is along y-axis; forward from robot (robot_yaw=0) is angle=0 in robot frame
        # which maps to +x in odom. We need forward along corridor (+y).
        # Set robot_yaw = π/2 so forward (robot angle 0) = +y globally.
        node._robot_yaw = math.pi / 2
        penalty = node._corridor_penalty(0.0, 3.0)   # gap straight ahead = along corridor
        self.assertAlmostEqual(penalty, 0.0, places=3)

    def test_lateral_gap_incurs_penalty(self):
        """Gap heading 90° sideways out of the corridor → positive penalty."""
        node = self._make_weave_node(corridor_half_w=1.0)
        # robot_yaw=π/2, gap angle=π/2 in robot frame → global angle=π → heading in -x direction
        # projected point = (0, 0) + 1.2 * (cos(π), sin(π)) = (-1.2, 0)
        # lateral distance from y-axis corridor: |x|=1.2 > half_width=1.0 → penalty=0.2
        node._robot_yaw = math.pi / 2
        penalty = node._corridor_penalty(math.pi / 2, 3.0)
        self.assertGreater(penalty, 0.0)

    def test_no_corridor_configured_returns_zero(self):
        """If corridor endpoints are None, penalty is always 0."""
        node = self._make_weave_node()
        node._corridor_start = None
        node._corridor_end   = None
        penalty = node._corridor_penalty(0.0, 3.0)
        self.assertAlmostEqual(penalty, 0.0)

    def test_guided_gap_angle_biases_toward_waypoint_inside_gap(self):
        node = self._make_weave_node()
        from auto_nav.navigation.weave_planner import Gap

        node._wp_x = 3.0
        node._wp_y = 3.0   # waypoint at +45° in robot frame
        gap = Gap(center_angle=0.0, width_rad=math.pi, mean_range=5.0, score=0.0)

        guided = node._guided_gap_angle(gap)

        self.assertAlmostEqual(guided, math.pi / 4, places=6)

    def test_goal_direction_range_accepts_clear_waypoint_bearing(self):
        node = self._make_weave_node()
        scan = _make_scan([5.0] * 181)
        node._wp_x = 0.0
        node._wp_y = 5.0

        goal_range = node._goal_direction_range(scan)

        self.assertIsNotNone(goal_range)


# ===========================================================================
# Tests: angle_wrap utilities
# ===========================================================================

class TestAngleWrap(unittest.TestCase):

    def test_wrap_identity(self):
        self.assertAlmostEqual(gap_wrap(0.0), 0.0)

    def test_wrap_over_pi(self):
        self.assertAlmostEqual(gap_wrap(math.pi + 0.1), -math.pi + 0.1, places=6)

    def test_wrap_under_neg_pi(self):
        self.assertAlmostEqual(gap_wrap(-math.pi - 0.1), math.pi - 0.1, places=6)

    def test_wrap_full_circle(self):
        self.assertAlmostEqual(gap_wrap(2 * math.pi), 0.0, places=6)


# ===========================================================================

if __name__ == '__main__':
    unittest.main()
