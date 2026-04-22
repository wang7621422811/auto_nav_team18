"""
Unit tests for Step-2 navigation modules:
  GeoLocalizer, WaypointProvider, FinalApproachController, PathFollowerNode.

All tests run WITHOUT a live ROS2 daemon by stubbing rclpy and message types,
following the same pattern as test_teleop.py / test_home_pose_recorder.py.

Completion criteria checked (from docs/steps/step_2.md):
  ✓ 能加载 waypoint 列表
  ✓ 能自动走到单个 waypoint 再返回 home (state-machine logic)
  ✓ 切到下一个 waypoint 的逻辑正确
  ✓ home pose 由 mission 开始时记录，而不是程序启动时记录
"""

import json
import math
import os
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from unittest.mock import MagicMock


# ---------------------------------------------------------------------------
# Minimal ROS2 / message stubs
# ---------------------------------------------------------------------------

def _build_ros_stubs():
    rclpy      = types.ModuleType('rclpy')
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
    rclpy.init         = MagicMock()
    rclpy.spin         = MagicMock()
    rclpy.try_shutdown = MagicMock()

    # --- Node stub ----------------------------------------------------------
    class _Node:
        def __init__(self, name):
            self._name   = name
            self._logger = MagicMock()
            self._params: dict = {}

        def get_logger(self):                    return self._logger
        def create_publisher(self, *a, **kw):    return MagicMock()
        def create_subscription(self, *a, **kw): return MagicMock()
        def create_timer(self, *a, **kw):        return MagicMock()
        def destroy_node(self):                  pass

        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            val = self._params.get(name)
            p   = MagicMock()
            if isinstance(val, bool):
                p.get_parameter_value.return_value.bool_value = val
            elif isinstance(val, int):
                p.get_parameter_value.return_value.integer_value = val
            elif isinstance(val, float):
                p.get_parameter_value.return_value.double_value = val
            else:
                p.get_parameter_value.return_value.string_value = str(val) if val is not None else ''
            return p

        def get_clock(self):
            clock = MagicMock()
            ts    = MagicMock()
            ts.to_msg.return_value = MagicMock(sec=0, nanosec=0)
            clock.now.return_value = ts
            return clock

    rclpy.node.Node = _Node

    # --- geometry_msgs ------------------------------------------------------
    geo     = types.ModuleType('geometry_msgs')
    geo.msg = types.ModuleType('geometry_msgs.msg')

    class _Point:
        def __init__(self): self.x = self.y = self.z = 0.0

    class _Quaternion:
        def __init__(self): self.x = self.y = self.z = 0.0; self.w = 1.0

    class _Pose:
        def __init__(self):
            self.position    = _Point()
            self.orientation = _Quaternion()

    class _PoseStamped:
        def __init__(self):
            self.header = MagicMock()
            self.pose   = _Pose()

    class _Twist:
        def __init__(self):
            self.linear  = MagicMock(x=0.0, y=0.0, z=0.0)
            self.angular = MagicMock(x=0.0, y=0.0, z=0.0)

    geo.msg.Pose        = _Pose
    geo.msg.PoseStamped = _PoseStamped
    geo.msg.Twist       = _Twist

    # --- nav_msgs -----------------------------------------------------------
    nav     = types.ModuleType('nav_msgs')
    nav.msg = types.ModuleType('nav_msgs.msg')

    class _PoseWithCov:
        def __init__(self): self.pose = _Pose()

    class _Odometry:
        def __init__(self):
            self.header = MagicMock(frame_id='odom')
            self.pose   = _PoseWithCov()

    nav.msg.Odometry = _Odometry

    # --- std_msgs -----------------------------------------------------------
    std     = types.ModuleType('std_msgs')
    std.msg = types.ModuleType('std_msgs.msg')

    class _String:
        def __init__(self, data=''): self.data = data

    class _Bool:
        def __init__(self, data=False): self.data = data

    std.msg.String = _String
    std.msg.Bool   = _Bool

    for name, mod in [
        ('rclpy',               rclpy),
        ('rclpy.node',          rclpy.node),
        ('rclpy.qos',           rclpy.qos),
        ('geometry_msgs',       geo),
        ('geometry_msgs.msg',   geo.msg),
        ('nav_msgs',            nav),
        ('nav_msgs.msg',        nav.msg),
        ('std_msgs',            std),
        ('std_msgs.msg',        std.msg),
    ]:
        sys.modules.setdefault(name, mod)

    return _Odometry, _PoseStamped, _String, _Twist


_Odometry, _PoseStamped, _String, _Twist = _build_ros_stubs()

# Now safe to import navigation modules
from auto_nav.navigation.geo_localizer    import GeoLocalizer          # noqa: E402
from auto_nav.navigation.waypoint_provider import WaypointProvider, Waypoint  # noqa: E402
from auto_nav.navigation.final_approach   import FinalApproachController       # noqa: E402
from auto_nav.navigation.path_follower    import (                             # noqa: E402
    PathFollowerNode, State, _quat_to_yaw, _angle_wrap,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _write_waypoints_yaml(d: str, waypoints: list[dict], origin: dict | None = None) -> str:
    """Write a minimal waypoints YAML file, return its path."""
    data: dict = {'waypoints': waypoints}
    if origin:
        data['origin'] = origin
    path = os.path.join(d, 'waypoints.yaml')
    with open(path, 'w') as f:
        yaml.dump(data, f)
    return path


def _make_follower(wp_file: str, **param_overrides) -> PathFollowerNode:
    """Instantiate PathFollowerNode with stubbed internals."""
    node = PathFollowerNode.__new__(PathFollowerNode)

    defaults = {
        'waypoints_file':          wp_file,
        'coarse_arrival_radius_m': 2.5,
        'final_arrival_radius_m':  1.0,
        'home_arrival_radius_m':   1.0,
        'pass_offset_m':           0.8,
        'max_linear_vel':          0.4,
        'max_angular_vel':         1.0,
        'k_angular':               1.5,
        'k_linear':                0.4,
        'control_rate_hz':        10.0,
        'local_target_max_heading_deg': 85.0,
        'rotate_in_place_angle_deg': 25.0,
        'require_marker':         True,
        'emit_journey_events':    False,
    }
    defaults.update(param_overrides)

    node._wp_file     = defaults['waypoints_file']
    node._coarse_r    = defaults['coarse_arrival_radius_m']
    node._final_r     = defaults['final_arrival_radius_m']
    node._home_r      = defaults['home_arrival_radius_m']
    node._pass_offset = defaults['pass_offset_m']
    node._max_lin     = defaults['max_linear_vel']
    node._max_ang     = defaults['max_angular_vel']
    node._k_ang       = defaults['k_angular']
    node._k_lin       = defaults['k_linear']
    node._local_target_max_heading = math.radians(
        defaults['local_target_max_heading_deg']
    )
    node._rotate_in_place_angle = math.radians(
        defaults['rotate_in_place_angle_deg']
    )
    node._require_marker = defaults['require_marker']
    node._emit_journey_events = defaults['emit_journey_events']

    from auto_nav.navigation.waypoint_provider import WaypointProvider
    from auto_nav.navigation.final_approach    import FinalApproachController

    node._fa_ctrl  = FinalApproachController(pass_offset_m=node._pass_offset)

    if node._wp_file:
        provider = WaypointProvider(node._wp_file)
        node._waypoints = provider.waypoints
    else:
        node._waypoints = []

    node._state         = State.IDLE
    node._mode          = 'MANUAL'
    node._home_recorded = False
    node._wp_idx        = 0
    node._robot_x       = 0.0
    node._robot_y       = 0.0
    node._robot_yaw     = 0.0
    node._home_x        = 0.0
    node._home_y        = 0.0
    node._marker_x      = None
    node._marker_y      = None
    node._target_x      = 0.0
    node._target_y      = 0.0
    node._estop         = False

    node._pub_cmd     = MagicMock()
    node._pub_wp      = MagicMock()
    node._pub_status  = MagicMock()
    node._pub_segment = MagicMock()  # added by Step 3 (/navigation/segment)
    node._pub_event   = MagicMock()

    # Step 3 state variables
    node._local_target_time    = 0.0    # timestamp of last /local_target (0 = never)
    node._local_target_timeout = 0.5    # seconds before local target goes stale
    node._pending_advance      = False  # flag set when waypoint complete, cleared on deadman
    node._mission_hold         = False  # deadman-hold flag (Step 3)

    logger = MagicMock()
    node._logger = logger
    node.get_logger = lambda: logger

    clock = MagicMock()
    ts    = MagicMock()
    ts.to_msg.return_value = MagicMock(sec=0, nanosec=0)
    clock.now.return_value = ts
    node.get_clock = lambda: clock

    return node


def _make_odom(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> _Odometry:
    msg = _Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    # Encode yaw as quaternion (only z/w matter for planar yaw)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def _make_marker(x: float, y: float) -> _PoseStamped:
    msg = _PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    return msg


# ---------------------------------------------------------------------------
# GeoLocalizer
# ---------------------------------------------------------------------------

class TestGeoLocalizer(unittest.TestCase):

    def test_raises_without_origin(self):
        gl = GeoLocalizer()
        with self.assertRaises(RuntimeError):
            gl.gps_to_enu(51.0, -0.01)

    def test_origin_at_zero_offset(self):
        gl = GeoLocalizer()
        gl.set_origin(51.4788, -0.0106)
        east, north = gl.gps_to_enu(51.4788, -0.0106)
        self.assertAlmostEqual(east,  0.0, places=3)
        self.assertAlmostEqual(north, 0.0, places=3)

    def test_north_positive(self):
        """Moving north (lat+) should give positive north_m."""
        gl = GeoLocalizer()
        gl.set_origin(0.0, 0.0)
        _, north = gl.gps_to_enu(0.001, 0.0)
        self.assertGreater(north, 0.0)

    def test_east_positive(self):
        """Moving east (lon+) should give positive east_m."""
        gl = GeoLocalizer()
        gl.set_origin(0.0, 0.0)
        east, _ = gl.gps_to_enu(0.0, 0.001)
        self.assertGreater(east, 0.0)

    def test_100m_north_approximately(self):
        gl = GeoLocalizer()
        gl.set_origin(0.0, 0.0)
        # 1 degree of latitude ≈ 111 320 m → 0.0009 deg ≈ 100 m
        _, north = gl.gps_to_enu(0.0009, 0.0)
        self.assertAlmostEqual(north, 100.19, delta=1.0)

    def test_has_origin_property(self):
        gl = GeoLocalizer()
        self.assertFalse(gl.has_origin)
        gl.set_origin(51.0, 0.0)
        self.assertTrue(gl.has_origin)


# ---------------------------------------------------------------------------
# WaypointProvider
# ---------------------------------------------------------------------------

class TestWaypointProviderXY(unittest.TestCase):

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()

    def test_load_two_xy_waypoints(self):
        path = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'wp1', 'type': 'xy', 'x': 5.0,  'y': 0.0},
            {'name': 'wp2', 'type': 'xy', 'x': 10.0, 'y': 5.0},
        ])
        prov = WaypointProvider(path)
        self.assertEqual(len(prov), 2)
        self.assertEqual(prov[0].name, 'wp1')
        self.assertAlmostEqual(prov[0].x, 5.0)
        self.assertAlmostEqual(prov[0].y, 0.0)
        self.assertEqual(prov[1].name, 'wp2')
        self.assertAlmostEqual(prov[1].x, 10.0)

    def test_default_type_is_xy(self):
        """Waypoints without 'type' key default to xy."""
        path = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'a', 'x': 3.0, 'y': 4.0},
        ])
        prov = WaypointProvider(path)
        self.assertAlmostEqual(prov[0].x, 3.0)

    def test_auto_name_when_missing(self):
        path = _write_waypoints_yaml(self._tmpdir, [
            {'type': 'xy', 'x': 1.0, 'y': 2.0},
        ])
        prov = WaypointProvider(path)
        self.assertEqual(prov[0].name, 'wp0')

    def test_empty_waypoints(self):
        path = _write_waypoints_yaml(self._tmpdir, [])
        prov = WaypointProvider(path)
        self.assertEqual(len(prov), 0)

    def test_gps_without_origin_raises(self):
        path = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'gps1', 'type': 'gps', 'lat': 51.0, 'lon': -0.01},
        ])
        with self.assertRaises(ValueError):
            WaypointProvider(path)

    def test_gps_with_origin_converts(self):
        path = _write_waypoints_yaml(
            self._tmpdir,
            [{'name': 'g1', 'type': 'gps', 'lat': 51.4788, 'lon': -0.0106}],
            origin={'lat': 51.4788, 'lon': -0.0106},
        )
        prov = WaypointProvider(path)
        self.assertAlmostEqual(prov[0].x, 0.0, places=2)
        self.assertAlmostEqual(prov[0].y, 0.0, places=2)

    def test_iteration(self):
        path = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'a', 'type': 'xy', 'x': 1.0, 'y': 0.0},
            {'name': 'b', 'type': 'xy', 'x': 2.0, 'y': 0.0},
        ])
        names = [wp.name for wp in WaypointProvider(path)]
        self.assertEqual(names, ['a', 'b'])

    def test_richer_yaml_file_ignored_sections(self):
        """A file with extra top-level keys (like ROS params) should load fine."""
        data = {
            'path_follower': {'ros__parameters': {'coarse_arrival_radius_m': 2.5}},
            'waypoints': [
                {'name': 'wp1', 'type': 'xy', 'x': 7.0, 'y': 3.0},
            ],
        }
        path = os.path.join(self._tmpdir, 'full.yaml')
        with open(path, 'w') as f:
            yaml.dump(data, f)
        prov = WaypointProvider(path)
        self.assertEqual(len(prov), 1)
        self.assertAlmostEqual(prov[0].x, 7.0)

    def test_repo_real_gps_waypoints_file_loads(self):
        """The repository GPS sample file should stay loadable by WaypointProvider."""
        prov = WaypointProvider(REPO_ROOT / 'config' / 'waypoints_real_gps.yaml')
        self.assertGreaterEqual(len(prov), 2)

    def test_repo_real_gps_waypoints_file_declares_origin(self):
        path = REPO_ROOT / 'config' / 'waypoints_real_gps.yaml'
        with open(path) as f:
            data = yaml.safe_load(f)
        self.assertIn('origin', data)
        self.assertIn('waypoints', data)
        self.assertTrue(all(wp.get('type') == 'gps' for wp in data['waypoints']))

    def test_repo_gps_config_origin_matches_real_gps_waypoints(self):
        with open(REPO_ROOT / 'config' / 'gps.yaml') as f:
            gps_cfg = yaml.safe_load(f)
        with open(REPO_ROOT / 'config' / 'waypoints_real_gps.yaml') as f:
            waypoint_cfg = yaml.safe_load(f)

        gps_params = gps_cfg['outdoor_pose_fuser']['ros__parameters']
        waypoint_origin = waypoint_cfg['origin']

        self.assertEqual(gps_params['gps_origin_lat'], waypoint_origin['lat'])
        self.assertEqual(gps_params['gps_origin_lon'], waypoint_origin['lon'])

    def test_repo_waypoints_match_new_world_orange_cones(self):
        """Keep config/waypoints_data.yaml aligned with orange cones in new_world.sdf."""
        world_path = REPO_ROOT / 'auto_nav' / 'simulation' / 'sim_worlds' / 'new_world.sdf'
        waypoints_path = REPO_ROOT / 'config' / 'waypoints_data.yaml'

        tree = ET.parse(world_path)
        root = tree.getroot()

        orange_cones = set()
        for model in root.findall('.//model'):
            name = model.attrib.get('name', '')
            if not name.startswith('orange_cone_'):
                continue
            pose_text = (model.findtext('pose') or '').strip()
            pose_vals = [float(v) for v in pose_text.split()]
            orange_cones.add((name, pose_vals[0], pose_vals[1]))

        provider = WaypointProvider(waypoints_path)
        waypoint_set = {(wp.name, wp.x, wp.y) for wp in provider.waypoints}

        self.assertEqual(waypoint_set, orange_cones)


# ---------------------------------------------------------------------------
# FinalApproachController
# ---------------------------------------------------------------------------

class TestFinalApproachController(unittest.TestCase):

    def _ctrl(self, offset=1.0):
        return FinalApproachController(pass_offset_m=offset)

    def test_marker_on_right_when_approaching_from_south(self):
        """
        Robot at (0,0), marker at (0,5) — robot approaches north.
        Left normal is (-1,0) → pass_pt is to the left (negative x).
        The marker (0,5) ends up to the right of the path.
        """
        ctrl     = self._ctrl(offset=1.0)
        pass_pt  = ctrl.compute_pass_point((0.0, 0.0), (0.0, 5.0))
        # u = (0,1), n_left = (-1,0) → pass_pt = (0-1, 5) = (-1, 5)
        self.assertAlmostEqual(pass_pt.x, -1.0, places=5)
        self.assertAlmostEqual(pass_pt.y,  5.0, places=5)

    def test_marker_on_right_when_approaching_from_west(self):
        """
        Robot at (0,0), marker at (5,0) — robot approaches east.
        u = (1,0), n_left = (0,1) → pass_pt = (5, 0+1) = (5, 1)
        """
        ctrl    = self._ctrl(offset=1.0)
        pass_pt = ctrl.compute_pass_point((0.0, 0.0), (5.0, 0.0))
        self.assertAlmostEqual(pass_pt.x, 5.0, places=5)
        self.assertAlmostEqual(pass_pt.y, 1.0, places=5)

    def test_pass_offset_scales_correctly(self):
        ctrl    = self._ctrl(offset=2.0)
        pass_pt = ctrl.compute_pass_point((0.0, 0.0), (0.0, 5.0))
        self.assertAlmostEqual(pass_pt.x, -2.0, places=5)

    def test_robot_at_marker_returns_left_offset(self):
        """When robot is at the marker, no crash — returns a fallback offset."""
        ctrl    = self._ctrl(offset=1.0)
        pass_pt = ctrl.compute_pass_point((3.0, 3.0), (3.0, 3.0))
        self.assertIsNotNone(pass_pt)

    def test_invalid_offset_raises(self):
        with self.assertRaises(ValueError):
            FinalApproachController(pass_offset_m=0.0)
        with self.assertRaises(ValueError):
            FinalApproachController(pass_offset_m=-1.0)

    def test_n_left_is_perpendicular_to_approach(self):
        """Pass vector from marker to pass_pt should be perpendicular to approach."""
        ctrl    = self._ctrl(offset=1.0)
        robot   = (0.0, 0.0)
        marker  = (3.0, 4.0)
        pass_pt = ctrl.compute_pass_point(robot, marker)

        ux = marker[0] - robot[0]
        uy = marker[1] - robot[1]
        norm = math.hypot(ux, uy)
        ux /= norm; uy /= norm

        # Vector from marker to pass_pt
        vx = pass_pt.x - marker[0]
        vy = pass_pt.y - marker[1]

        dot = ux * vx + uy * vy
        self.assertAlmostEqual(dot, 0.0, places=5)

    def test_approach_bearing(self):
        ctrl    = self._ctrl()
        bearing = ctrl.approach_bearing((0.0, 0.0), (1.0, 0.0))
        self.assertAlmostEqual(bearing, 0.0, places=5)

        bearing = ctrl.approach_bearing((0.0, 0.0), (0.0, 1.0))
        self.assertAlmostEqual(bearing, math.pi / 2, places=5)


# ---------------------------------------------------------------------------
# PathFollowerNode — state machine
# ---------------------------------------------------------------------------

class TestPathFollowerStateIdle(unittest.TestCase):

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        self._wp_file = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'wp1', 'type': 'xy', 'x': 10.0, 'y': 0.0},
        ])

    def test_starts_in_idle(self):
        node = _make_follower(self._wp_file)
        self.assertEqual(node._state, State.IDLE)

    def test_tick_in_idle_does_not_move(self):
        node = _make_follower(self._wp_file)
        node._mode = 'AUTO'   # but state is still IDLE
        node._tick()
        node._pub_cmd.publish.assert_not_called()


class TestPathFollowerMissionStart(unittest.TestCase):
    """✓ home pose 由 mission 开始时记录，而不是程序启动时记录"""

    def setUp(self):
        self._tmpdir  = tempfile.mkdtemp()
        self._wp_file = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'wp1', 'type': 'xy', 'x': 10.0, 'y': 0.0},
        ])

    def test_home_not_recorded_at_init(self):
        node = _make_follower(self._wp_file)
        self.assertFalse(node._home_recorded)

    def test_home_recorded_on_first_auto(self):
        node = _make_follower(self._wp_file)
        node._robot_x = 3.0
        node._robot_y = 4.0
        node._mode_cb(_String(data='AUTO'))
        self.assertTrue(node._home_recorded)
        self.assertAlmostEqual(node._home_x, 3.0)
        self.assertAlmostEqual(node._home_y, 4.0)

    def test_home_not_re_recorded_on_second_auto(self):
        node = _make_follower(self._wp_file)
        node._robot_x = 3.0; node._robot_y = 4.0
        node._mode_cb(_String(data='AUTO'))
        # Robot moves, toggle mode
        node._robot_x = 9.0; node._robot_y = 9.0
        node._mode_cb(_String(data='MANUAL'))
        node._mode_cb(_String(data='AUTO'))
        # Home must still be the first position
        self.assertAlmostEqual(node._home_x, 3.0)
        self.assertAlmostEqual(node._home_y, 4.0)

    def test_mission_start_transitions_to_navigating(self):
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        self.assertEqual(node._state, State.NAVIGATING)

    def test_mission_start_sets_target_to_first_wp(self):
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        self.assertAlmostEqual(node._target_x, 10.0)
        self.assertAlmostEqual(node._target_y,  0.0)

    def test_mission_with_empty_wp_list_goes_done(self):
        empty_file = _write_waypoints_yaml(self._tmpdir, [])
        node = _make_follower(empty_file)
        node._mode_cb(_String(data='AUTO'))
        self.assertEqual(node._state, State.DONE)


class TestPathFollowerNavigation(unittest.TestCase):
    """✓ 能自动走到单个 waypoint 再返回 home"""

    def setUp(self):
        self._tmpdir  = tempfile.mkdtemp()
        self._wp_file = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'wp1', 'type': 'xy', 'x': 10.0, 'y': 0.0},
        ])

    def test_navigating_far_from_wp_publishes_cmd(self):
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        # Robot still at origin, wp at (10, 0)
        node._tick()
        node._pub_cmd.publish.assert_called()

    def test_navigating_arrives_coarse(self):
        """Within coarse radius → switch to COARSE_ARRIVED."""
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        # Place robot within coarse radius of wp1
        node._robot_x = 9.0
        node._robot_y = 0.0
        node._tick()
        self.assertEqual(node._state, State.COARSE_ARRIVED)

    def test_coarse_arrived_no_marker_stops(self):
        node = _make_follower(self._wp_file)
        node._state  = State.COARSE_ARRIVED
        node._mode   = 'AUTO'
        node._marker_x = None
        node._tick()
        last_call = node._pub_cmd.publish.call_args[0][0]
        # Zero twist
        self.assertEqual(last_call.linear.x,  0.0)
        self.assertEqual(last_call.angular.z, 0.0)

    def test_coarse_arrived_with_marker_starts_final_approach(self):
        node = _make_follower(self._wp_file)
        node._state    = State.COARSE_ARRIVED
        node._mode     = 'AUTO'
        node._robot_x  = 9.0
        node._robot_y  = 0.0
        node._marker_x = 10.0
        node._marker_y = 0.0
        node._tick()
        self.assertEqual(node._state, State.FINAL_APPROACH)

    def test_bench_mode_coarse_arrival_advances_without_marker(self):
        """Bench mode: no marker required, waypoint completes at coarse radius."""
        node = _make_follower(self._wp_file, require_marker=False)
        node._mode_cb(_String(data='AUTO'))
        node._robot_x = 9.0
        node._robot_y = 0.0
        node._tick()
        self.assertEqual(node._state, State.HOMING)

    def test_final_approach_completes_wp(self):
        """When close enough to pass point → advance waypoint."""
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        node._state    = State.FINAL_APPROACH
        # Put robot at the pass point (within final_r)
        node._robot_x  = node._target_x
        node._robot_y  = node._target_y
        node._tick()
        # Only 1 WP → should now be HOMING
        self.assertEqual(node._state, State.HOMING)

    def test_homing_drives_toward_home(self):
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        node._state  = State.HOMING
        node._home_x = 0.0
        node._home_y = 0.0
        node._target_x = 0.0
        node._target_y = 0.0
        node._robot_x  = 5.0
        node._robot_y  = 0.0
        node._tick()
        node._pub_cmd.publish.assert_called()

    def test_homing_completes_at_home(self):
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        node._state    = State.HOMING
        node._target_x = 0.0
        node._target_y = 0.0
        # Robot is already within home_arrival_radius
        node._robot_x  = 0.3
        node._robot_y  = 0.3
        node._tick()
        self.assertEqual(node._state, State.DONE)

    def test_rear_local_target_falls_back_to_waypoint(self):
        """Rear local targets must not override the forward global waypoint."""
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        node._robot_x = 0.0
        node._robot_y = 0.0
        node._robot_yaw = 0.0
        node._target_x = 10.0
        node._target_y = 0.0
        node._local_target_x = -2.0
        node._local_target_y = 0.0
        node._local_target_time = 0.05
        node.get_clock().now.return_value.nanoseconds = int(0.1 * 1e9)

        node._tick()

        cmd = node._pub_cmd.publish.call_args[0][0]
        self.assertGreater(cmd.linear.x, 0.0)
        self.assertAlmostEqual(cmd.angular.z, 0.0, places=6)

    def test_drive_toward_rear_target_turns_in_place(self):
        """Targets behind the robot should not command positive forward speed."""
        node = _make_follower(self._wp_file)
        node._robot_x = 0.0
        node._robot_y = 0.0
        node._robot_yaw = 0.0

        node._drive_toward(-1.0, 0.0)

        cmd = node._pub_cmd.publish.call_args[0][0]
        self.assertAlmostEqual(cmd.linear.x, 0.0, places=6)
        self.assertAlmostEqual(abs(cmd.angular.z), node._max_ang, places=6)

    def test_large_heading_error_rotates_in_place(self):
        """Large heading errors should stop forward motion until aligned."""
        node = _make_follower(self._wp_file)
        node._robot_x = 0.0
        node._robot_y = 0.0
        node._robot_yaw = 0.0

        node._drive_toward(0.0, 10.0)   # 90° heading error

        cmd = node._pub_cmd.publish.call_args[0][0]
        self.assertAlmostEqual(cmd.linear.x, 0.0, places=6)
        self.assertAlmostEqual(cmd.angular.z, node._max_ang, places=6)

    def test_bench_mode_emits_journey_events(self):
        node = _make_follower(
            self._wp_file,
            require_marker=False,
            emit_journey_events=True,
        )
        node._mode_cb(_String(data='AUTO'))
        start_event = json.loads(node._pub_event.publish.call_args_list[0][0][0].data)
        self.assertEqual(start_event['type'], 'MISSION_STARTED')

        node._robot_x = 9.0
        node._robot_y = 0.0
        node._tick()

        arrived_event = json.loads(node._pub_event.publish.call_args_list[1][0][0].data)
        self.assertEqual(arrived_event['type'], 'WAYPOINT_ARRIVED')
        self.assertEqual(arrived_event['waypoint_name'], 'wp1')
        self.assertEqual(arrived_event['arrival_mode'], 'coarse_only')

        node._robot_x = node._home_x
        node._robot_y = node._home_y
        node._tick()

        event_types = [
            json.loads(call.args[0].data)['type']
            for call in node._pub_event.publish.call_args_list
        ]
        self.assertIn('HOME_REACHED', event_types)
        self.assertIn('MISSION_COMPLETE', event_types)


class TestPathFollowerWaypointAdvance(unittest.TestCase):
    """✓ 切到下一个 waypoint 的逻辑正确"""

    def setUp(self):
        self._tmpdir  = tempfile.mkdtemp()
        self._wp_file = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'wp1', 'type': 'xy', 'x': 8.0, 'y': 0.0},
            {'name': 'wp2', 'type': 'xy', 'x': 8.0, 'y': 8.0},
        ])

    def test_advance_moves_to_next_wp(self):
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        self.assertEqual(node._wp_idx, 0)
        self.assertAlmostEqual(node._target_x, 8.0)
        self.assertAlmostEqual(node._target_y, 0.0)

        node._advance_waypoint()
        self.assertEqual(node._wp_idx, 1)
        self.assertEqual(node._state, State.NAVIGATING)
        self.assertAlmostEqual(node._target_x, 8.0)
        self.assertAlmostEqual(node._target_y, 8.0)

    def test_advance_past_last_wp_starts_homing(self):
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        node._wp_idx = 1
        node._advance_waypoint()
        self.assertEqual(node._state, State.HOMING)

    def test_homing_target_is_recorded_home(self):
        node = _make_follower(self._wp_file)
        node._robot_x = 2.0; node._robot_y = 3.0
        node._mode_cb(_String(data='AUTO'))
        node._wp_idx = 1
        node._advance_waypoint()
        self.assertAlmostEqual(node._target_x, node._home_x)
        self.assertAlmostEqual(node._target_y, node._home_y)

    def test_marker_cleared_after_wp_complete(self):
        """Marker detection from one WP must not bleed into the next."""
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))
        node._marker_x = 8.0
        node._marker_y = 0.5
        # Simulate completing WP[0] final approach
        node._state    = State.FINAL_APPROACH
        node._robot_x  = node._target_x
        node._robot_y  = node._target_y
        node._tick()
        self.assertIsNone(node._marker_x)
        self.assertIsNone(node._marker_y)

    def test_wp_count_with_two_waypoints(self):
        """✓ 能加载 waypoint 列表 — verify count."""
        node = _make_follower(self._wp_file)
        self.assertEqual(len(node._waypoints), 2)
        self.assertEqual(node._waypoints[0].name, 'wp1')
        self.assertEqual(node._waypoints[1].name, 'wp2')


class TestPathFollowerModeSwitching(unittest.TestCase):

    def setUp(self):
        self._tmpdir  = tempfile.mkdtemp()
        self._wp_file = _write_waypoints_yaml(self._tmpdir, [
            {'name': 'wp1', 'type': 'xy', 'x': 10.0, 'y': 0.0},
        ])

    def test_tick_in_manual_does_not_publish(self):
        node = _make_follower(self._wp_file)
        node._mode  = 'MANUAL'
        node._state = State.NAVIGATING
        node._tick()
        node._pub_cmd.publish.assert_not_called()

    def test_leaving_auto_stops_robot(self):
        node = _make_follower(self._wp_file)
        node._mode_cb(_String(data='AUTO'))    # mission start
        node._mode_cb(_String(data='MANUAL'))  # leave AUTO
        # Last published command must be zero (stop)
        last_call = node._pub_cmd.publish.call_args[0][0]
        self.assertEqual(last_call.linear.x, 0.0)


# ---------------------------------------------------------------------------
# Utility function tests
# ---------------------------------------------------------------------------

class TestUtilFunctions(unittest.TestCase):

    def test_quat_to_yaw_identity(self):
        self.assertAlmostEqual(_quat_to_yaw(0, 0, 0, 1), 0.0, places=5)

    def test_quat_to_yaw_180deg(self):
        yaw = _quat_to_yaw(0, 0, 1, 0)
        self.assertAlmostEqual(abs(yaw), math.pi, places=5)

    def test_quat_to_yaw_90deg(self):
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        yaw = _quat_to_yaw(0, 0, s, c)
        self.assertAlmostEqual(yaw, math.pi / 2, places=5)

    def test_angle_wrap_positive(self):
        self.assertAlmostEqual(_angle_wrap(3 * math.pi), math.pi, places=5)

    def test_angle_wrap_negative(self):
        # -3π = -π (both ±π represent the same direction boundary)
        self.assertAlmostEqual(abs(_angle_wrap(-3 * math.pi)), math.pi, places=5)

    def test_angle_wrap_no_change_in_range(self):
        for v in [0.0, 1.0, -1.0, math.pi - 0.001]:
            self.assertAlmostEqual(_angle_wrap(v), v, places=5)


if __name__ == '__main__':
    unittest.main()
