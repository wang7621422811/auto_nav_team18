"""
Unit tests for MissionControllerNode FSM.

All tests run without a live ROS2 daemon — rclpy and message types are
stubbed in the same pattern used by the existing test suite.

Coverage
--------
  1.  IDLE → MANUAL on first MANUAL control_mode
  2.  IDLE → AUTO_READY when AUTO pressed
  3.  AUTO_READY → NAVIGATE_TO_WAYPOINT when path_follower starts
  4.  AUTO_READY → WEAVING_SEGMENT when segment == "1" on startup
  5.  NAVIGATE_TO_WAYPOINT ↔ WEAVING_SEGMENT segment switching
  6.  NAVIGATE_TO_WAYPOINT → FINAL_APPROACH on COARSE_ARRIVED
  7.  FINAL_APPROACH → CAPTURE_MARKER when wp_status == FINAL_APPROACH
  8.  CAPTURE_MARKER → SEARCH_OBJECT after capture_hold_s (no cached object)
  9.  CAPTURE_MARKER → CAPTURE_OBJECT when object cached before hold expires
  10. SEARCH_OBJECT → CAPTURE_OBJECT on fresh object detection
  11. SEARCH_OBJECT → resume nav on timeout
  12. CAPTURE_OBJECT → NAVIGATE_TO_WAYPOINT after capture_hold_s
  13. NAVIGATE_TO_WAYPOINT → RETURN_HOME when segment=="H"
  14. RETURN_HOME → COMPLETE when wp_status == DONE
  15. Dead-man: any AUTO state → AUTO_PAUSED_DEADMAN when PAUSED
  16. AUTO_PAUSED_DEADMAN → resume previous state when AUTO restored
  17. AUTO_PAUSED_DEADMAN → MANUAL when control_mode→MANUAL
  18. Manual override during NAVIGATE_TO_WAYPOINT → MANUAL_OVERRIDE
  19. MANUAL_OVERRIDE → AUTO_READY when AUTO pressed again
  20. ABORTED via emergency_stop (any state)
  21. Sweep direction flips after search_sweep_step_s
  22. /mission/hold set True on CAPTURE_MARKER entry
  23. /mission/hold released (False) after CAPTURE_OBJECT
"""

from __future__ import annotations

import sys
import time
import types
import unittest
from unittest.mock import MagicMock, call


# ---------------------------------------------------------------------------
# ROS2 stubs — no real ROS2 installation needed
# ---------------------------------------------------------------------------

def _build_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.node = types.ModuleType('rclpy.node')
    rclpy.qos  = types.ModuleType('rclpy.qos')

    class _QoS:
        def __init__(self, **kw): pass

    class _Rel:
        RELIABLE    = 'RELIABLE'
        BEST_EFFORT = 'BEST_EFFORT'

    class _Dur:
        TRANSIENT_LOCAL = 'TRANSIENT_LOCAL'

    rclpy.qos.QoSProfile        = _QoS
    rclpy.qos.ReliabilityPolicy = _Rel
    rclpy.qos.DurabilityPolicy  = _Dur
    rclpy.init         = MagicMock()
    rclpy.spin         = MagicMock()
    rclpy.try_shutdown = MagicMock()

    class _Node:
        def __init__(self, name):
            self._params = {}
            self._logger = MagicMock()

        def get_logger(self):              return self._logger
        def create_publisher(self, *a, **kw):    return MagicMock()
        def create_subscription(self, *a, **kw): return MagicMock()
        def create_timer(self, *a, **kw):        return MagicMock()
        def destroy_node(self):           pass

        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            val = self._params.get(name)
            p   = MagicMock()
            if isinstance(val, float):
                p.get_parameter_value.return_value.double_value  = val
            elif isinstance(val, int):
                p.get_parameter_value.return_value.integer_value = val
            else:
                p.get_parameter_value.return_value.string_value  = val or ''
            return p

    rclpy.node.Node = _Node

    # geometry_msgs / std_msgs stubs
    geometry_msgs          = types.ModuleType('geometry_msgs')
    geometry_msgs.msg      = types.ModuleType('geometry_msgs.msg')
    std_msgs               = types.ModuleType('std_msgs')
    std_msgs.msg           = types.ModuleType('std_msgs.msg')

    class _PoseStamped:
        def __init__(self):
            self.pose = MagicMock()
            self.pose.position.x = 0.0
            self.pose.position.y = 0.0

    class _Twist:
        def __init__(self):
            self.linear  = MagicMock(); self.linear.x  = 0.0
            self.angular = MagicMock(); self.angular.z = 0.0

    class _String:
        def __init__(self, data=''):
            self.data = data

    class _Bool:
        def __init__(self, data=False):
            self.data = data

    geometry_msgs.msg.PoseStamped = _PoseStamped
    geometry_msgs.msg.Twist       = _Twist
    std_msgs.msg.String           = _String
    std_msgs.msg.Bool             = _Bool

    for mod_name, mod in [
        ('rclpy',                    rclpy),
        ('rclpy.node',               rclpy.node),
        ('rclpy.qos',                rclpy.qos),
        ('geometry_msgs',            geometry_msgs),
        ('geometry_msgs.msg',        geometry_msgs.msg),
        ('std_msgs',                 std_msgs),
        ('std_msgs.msg',             std_msgs.msg),
    ]:
        sys.modules[mod_name] = mod


_build_stubs()

# Now we can import the module under test
sys.path.insert(0, '/home/parallels/workspace/auto_nav_team18')
from auto_nav.mission.mission_controller import MissionControllerNode, MS  # noqa: E402


# ---------------------------------------------------------------------------
# Helper: construct a node with controlled clock
# ---------------------------------------------------------------------------

class _FakeClock:
    """Monotonic clock that can be stepped forward in tests."""
    def __init__(self):
        self._t = 0.0

    def advance(self, secs: float):
        self._t += secs

    def __call__(self) -> float:
        return self._t


def _make_node(fast_timeouts: bool = True) -> tuple[MissionControllerNode, _FakeClock]:
    """Return (node, clock).  Patches node._now() to use the fake clock."""
    node = MissionControllerNode.__new__(MissionControllerNode)
    # Manually initialise attributes that __init__ would set via ROS
    node._params = {
        'search_trigger_s':        0.0 if fast_timeouts else 2.0,
        'search_sweep_duration_s': 0.01 if fast_timeouts else 6.0,
        'search_sweep_step_s':     0.005 if fast_timeouts else 1.5,
        'search_sweep_yaw':        0.3,
        'capture_hold_s':          0.0 if fast_timeouts else 1.0,
        'object_cache_timeout_s':  10.0,
        'tick_rate_hz':            10.0,
    }

    node._logger = MagicMock()

    def _get_logger():
        return node._logger
    node.get_logger = _get_logger

    # Publishers (mocked)
    node._pub_state  = MagicMock()
    node._pub_event  = MagicMock()
    node._pub_hold   = MagicMock()
    node._pub_search = MagicMock()
    node._pub_cmd    = MagicMock()

    # Internal state (mirror of __init__)
    node._state              = MS.IDLE
    node._prev_state         = MS.IDLE
    node._control_mode       = 'MANUAL'
    node._wp_status          = 'IDLE'
    node._nav_segment        = ''
    node._deadman_ok         = False
    node._emergency          = False
    node._marker_detected    = False
    node._marker_time        = -1.0
    node._last_object_json   = None
    node._last_object_time   = -1.0  # negative sentinel = not yet received
    node._state_entry_time   = 0.0
    node._search_start_time  = 0.0
    node._sweep_step_time    = 0.0
    node._sweep_direction    = 1.0
    node._resume_state       = MS.AUTO_READY
    node._hold_active        = False
    node._search_active_flag = False

    # Step-6 attributes (odometry / per-waypoint tracking)
    node._odom_x           = None
    node._odom_y           = None
    node._total_distance_m = 0.0
    node._current_wp_idx   = 0
    node._wp_marker_photo  = None
    node._wp_object_photo  = None

    # Use configurable params
    p = node._params
    node._search_trigger_s        = p['search_trigger_s']
    node._search_sweep_duration_s = p['search_sweep_duration_s']
    node._search_sweep_step_s     = p['search_sweep_step_s']
    node._search_sweep_yaw        = p['search_sweep_yaw']
    node._capture_hold_s          = p['capture_hold_s']
    node._object_cache_timeout_s  = p['object_cache_timeout_s']

    clock = _FakeClock()
    node._now = clock  # patch _now to fake clock

    return node, clock


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestMissionFSM(unittest.TestCase):

    # ── 1. IDLE → MANUAL ──────────────────────────────────────────────────
    def test_idle_to_manual(self):
        node, _ = _make_node()
        node._control_mode = 'MANUAL'
        node._tick()
        self.assertEqual(node._state, MS.MANUAL)

    # ── 2. IDLE → AUTO_READY ──────────────────────────────────────────────
    def test_idle_to_auto_ready(self):
        node, _ = _make_node()
        node._control_mode = 'AUTO'
        node._tick()
        self.assertEqual(node._state, MS.AUTO_READY)

    # ── 3. AUTO_READY → NAVIGATE_TO_WAYPOINT ─────────────────────────────
    def test_auto_ready_to_navigate(self):
        node, _ = _make_node()
        node._state        = MS.AUTO_READY
        node._control_mode = 'AUTO'
        node._wp_status    = 'NAVIGATING'
        node._nav_segment  = '0'
        node._tick()
        self.assertEqual(node._state, MS.NAVIGATE_TO_WAYPOINT)

    # ── 4. AUTO_READY → WEAVING_SEGMENT ──────────────────────────────────
    def test_auto_ready_to_weaving(self):
        node, _ = _make_node()
        node._state        = MS.AUTO_READY
        node._control_mode = 'AUTO'
        node._wp_status    = 'NAVIGATING'
        node._nav_segment  = '1'
        node._tick()
        self.assertEqual(node._state, MS.WEAVING_SEGMENT)

    # ── 5. NAVIGATE ↔ WEAVING segment switch ─────────────────────────────
    def test_nav_to_weaving_switch(self):
        node, _ = _make_node()
        node._state        = MS.NAVIGATE_TO_WAYPOINT
        node._control_mode = 'AUTO'
        node._wp_status    = 'NAVIGATING'
        node._nav_segment  = '1'
        node._tick()
        self.assertEqual(node._state, MS.WEAVING_SEGMENT)

    def test_weaving_to_nav_switch(self):
        node, _ = _make_node()
        node._state        = MS.WEAVING_SEGMENT
        node._control_mode = 'AUTO'
        node._wp_status    = 'NAVIGATING'
        node._nav_segment  = '2'
        node._tick()
        self.assertEqual(node._state, MS.NAVIGATE_TO_WAYPOINT)

    # ── 6. NAVIGATE → FINAL_APPROACH ─────────────────────────────────────
    def test_navigate_to_final_approach(self):
        node, _ = _make_node()
        node._state        = MS.NAVIGATE_TO_WAYPOINT
        node._control_mode = 'AUTO'
        node._wp_status    = 'COARSE_ARRIVED'
        node._tick()
        self.assertEqual(node._state, MS.FINAL_APPROACH)

    # ── 7. FINAL_APPROACH → CAPTURE_MARKER ───────────────────────────────
    def test_final_approach_to_capture_marker(self):
        node, _ = _make_node()
        node._state        = MS.FINAL_APPROACH
        node._control_mode = 'AUTO'
        node._wp_status    = 'FINAL_APPROACH'
        node._tick()
        self.assertEqual(node._state, MS.CAPTURE_MARKER)
        # hold must be set when entering CAPTURE_MARKER
        node._pub_hold.publish.assert_called()

    # ── 8. CAPTURE_MARKER → SEARCH_OBJECT (no object) ────────────────────
    def test_capture_marker_to_search_object(self):
        node, clock = _make_node(fast_timeouts=True)
        node._state            = MS.CAPTURE_MARKER
        node._control_mode     = 'AUTO'
        node._last_object_time = -1.0  # no cached object (sentinel)
        clock.advance(1.0)             # well past capture_hold_s=0
        node._tick()
        self.assertEqual(node._state, MS.SEARCH_OBJECT)

    # ── 9. CAPTURE_MARKER → CAPTURE_OBJECT (object already cached) ───────
    def test_capture_marker_to_capture_object_fast_path(self):
        node, clock = _make_node(fast_timeouts=True)
        node._state            = MS.CAPTURE_MARKER
        node._control_mode     = 'AUTO'
        node._last_object_time = clock()     # fresh
        node._last_object_json = '[{"color":"red"}]'
        clock.advance(1.0)
        node._tick()
        self.assertEqual(node._state, MS.CAPTURE_OBJECT)

    # ── 10. SEARCH_OBJECT → CAPTURE_OBJECT on detection ──────────────────
    def test_search_object_to_capture_object(self):
        node, clock = _make_node(fast_timeouts=True)
        node._state            = MS.SEARCH_OBJECT
        node._control_mode     = 'AUTO'
        node._last_object_time = clock()
        node._last_object_json = '[{"color":"blue"}]'
        node._tick()
        self.assertEqual(node._state, MS.CAPTURE_OBJECT)

    # ── 11. SEARCH_OBJECT timeout → resume navigation ─────────────────────
    def test_search_object_timeout_resumes_nav(self):
        node, clock = _make_node(fast_timeouts=True)
        node._state             = MS.SEARCH_OBJECT
        node._control_mode      = 'AUTO'
        node._wp_status         = 'NAVIGATING'
        node._nav_segment       = '2'
        node._search_start_time = 0.0
        clock.advance(10.0)  # beyond search_sweep_duration_s=0.01
        node._tick()
        self.assertIn(node._state, (MS.NAVIGATE_TO_WAYPOINT, MS.WEAVING_SEGMENT,
                                    MS.RETURN_HOME, MS.COMPLETE))

    # ── 12. CAPTURE_OBJECT → NAVIGATE_TO_WAYPOINT ─────────────────────────
    def test_capture_object_resumes_nav(self):
        node, clock = _make_node(fast_timeouts=True)
        node._state        = MS.CAPTURE_OBJECT
        node._control_mode = 'AUTO'
        node._wp_status    = 'NAVIGATING'
        node._nav_segment  = '2'
        clock.advance(5.0)
        node._tick()
        self.assertEqual(node._state, MS.NAVIGATE_TO_WAYPOINT)

    # ── 13. NAVIGATE → RETURN_HOME ────────────────────────────────────────
    def test_navigate_to_return_home(self):
        node, _ = _make_node()
        node._state        = MS.NAVIGATE_TO_WAYPOINT
        node._control_mode = 'AUTO'
        node._wp_status    = 'HOMING'
        node._tick()
        self.assertEqual(node._state, MS.RETURN_HOME)

    # ── 14. RETURN_HOME → COMPLETE ────────────────────────────────────────
    def test_return_home_to_complete(self):
        node, _ = _make_node()
        node._state        = MS.RETURN_HOME
        node._control_mode = 'AUTO'
        node._wp_status    = 'DONE'
        node._tick()
        self.assertEqual(node._state, MS.COMPLETE)

    # ── 15. Dead-man: AUTO → AUTO_PAUSED_DEADMAN ─────────────────────────
    def test_deadman_pauses_auto(self):
        for active_state in (MS.NAVIGATE_TO_WAYPOINT, MS.WEAVING_SEGMENT,
                             MS.FINAL_APPROACH, MS.RETURN_HOME):
            with self.subTest(state=active_state):
                node, _ = _make_node()
                node._state        = active_state
                node._control_mode = 'PAUSED'
                node._tick()
                self.assertEqual(node._state, MS.AUTO_PAUSED_DEADMAN)

    # ── 16. AUTO_PAUSED_DEADMAN → resume previous state ───────────────────
    def test_deadman_resume(self):
        node, _ = _make_node()
        node._state         = MS.AUTO_PAUSED_DEADMAN
        node._control_mode  = 'AUTO'
        node._resume_state  = MS.NAVIGATE_TO_WAYPOINT
        node._tick()
        self.assertEqual(node._state, MS.NAVIGATE_TO_WAYPOINT)

    # ── 17. AUTO_PAUSED_DEADMAN → MANUAL ─────────────────────────────────
    def test_deadman_to_manual(self):
        node, _ = _make_node()
        node._state        = MS.AUTO_PAUSED_DEADMAN
        node._control_mode = 'MANUAL'
        node._tick()
        self.assertEqual(node._state, MS.MANUAL)

    # ── 18. Manual override during AUTO ───────────────────────────────────
    def test_manual_override(self):
        node, _ = _make_node()
        node._state        = MS.NAVIGATE_TO_WAYPOINT
        node._control_mode = 'MANUAL'
        node._tick()
        self.assertEqual(node._state, MS.MANUAL_OVERRIDE)

    # ── 19. MANUAL_OVERRIDE → AUTO_READY ─────────────────────────────────
    def test_manual_override_to_auto_ready(self):
        node, _ = _make_node()
        node._state        = MS.MANUAL_OVERRIDE
        node._control_mode = 'AUTO'
        node._tick()
        self.assertEqual(node._state, MS.AUTO_READY)

    # ── 20. Emergency stop from any AUTO state ────────────────────────────
    def test_emergency_stop(self):
        for active_state in (MS.NAVIGATE_TO_WAYPOINT, MS.RETURN_HOME,
                             MS.SEARCH_OBJECT, MS.CAPTURE_MARKER):
            with self.subTest(state=active_state):
                node, _ = _make_node()
                node._state     = active_state
                node._emergency = True
                node._tick()
                self.assertEqual(node._state, MS.ABORTED)

    # ── 21. Sweep direction flips ─────────────────────────────────────────
    def test_sweep_direction_flip(self):
        node, clock = _make_node()
        node._search_sweep_step_s = 0.5
        node._search_sweep_yaw    = 0.3
        node._search_start_time   = 0.0
        node._sweep_step_time     = 0.0
        node._sweep_direction     = 1.0

        # First step — no flip yet
        node._do_sweep_step()
        self.assertAlmostEqual(node._sweep_direction, 1.0)

        # Advance past step interval — direction should flip
        clock.advance(0.6)
        node._do_sweep_step()
        self.assertAlmostEqual(node._sweep_direction, -1.0)

    # ── 22. /mission/hold set True when CAPTURE_MARKER entered ────────────
    def test_hold_set_on_capture_marker(self):
        node, _ = _make_node()
        node._state        = MS.FINAL_APPROACH
        node._control_mode = 'AUTO'
        node._wp_status    = 'FINAL_APPROACH'
        node._tick()
        self.assertEqual(node._state, MS.CAPTURE_MARKER)
        # _set_hold(True) must have been called → pub_hold published True
        calls = [c.args[0].data for c in node._pub_hold.publish.call_args_list
                 if hasattr(c.args[0], 'data')]
        self.assertIn(True, calls)

    # ── 23. /mission/hold released after CAPTURE_OBJECT ──────────────────
    def test_hold_released_after_capture_object(self):
        node, clock = _make_node(fast_timeouts=True)
        node._state        = MS.CAPTURE_OBJECT
        node._control_mode = 'AUTO'
        node._wp_status    = 'NAVIGATING'
        node._nav_segment  = '2'
        node._hold_active  = True   # simulate hold was on
        clock.advance(5.0)
        node._tick()
        # After CAPTURE_OBJECT → resume, hold must be False
        self.assertFalse(node._hold_active)
        calls = [c.args[0].data for c in node._pub_hold.publish.call_args_list
                 if hasattr(c.args[0], 'data')]
        self.assertIn(False, calls)

    # ── No automatic transitions from ABORTED ─────────────────────────────
    def test_aborted_is_terminal(self):
        node, _ = _make_node()
        node._state        = MS.ABORTED
        node._control_mode = 'AUTO'
        node._tick()
        self.assertEqual(node._state, MS.ABORTED)

    # ── No automatic transitions from COMPLETE ────────────────────────────
    def test_complete_is_terminal(self):
        node, _ = _make_node()
        node._state        = MS.COMPLETE
        node._control_mode = 'AUTO'
        node._tick()
        self.assertEqual(node._state, MS.COMPLETE)


if __name__ == '__main__':
    unittest.main()
