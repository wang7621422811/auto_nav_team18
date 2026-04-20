"""
MissionControllerNode — top-level mission state machine (Step 5 / Step 6).

States
------
  IDLE                 system not active (startup)
  MANUAL               manual driving mode (O button / joystick disconnected)
  AUTO_READY           X pressed, waiting for PathFollower to start navigating
  NAVIGATE_TO_WAYPOINT driving toward a waypoint (non-weave segment)
  WEAVING_SEGMENT      driving through slalom segment (navigation/segment == "1")
  FINAL_APPROACH       coarse-arrived at waypoint — path_follower waiting for marker
  CAPTURE_MARKER       robot passing orange cone (path_follower FINAL_APPROACH)
  SEARCH_OBJECT        performing yaw-sweep to find colored object
  CAPTURE_OBJECT       colored object confirmed and logged
  RETURN_HOME          all waypoints visited, path_follower heading home
  AUTO_PAUSED_DEADMAN  dead-man trigger released; ready to resume
  MANUAL_OVERRIDE      O pressed during AUTO; waiting for X to restart
  ABORTED              emergency stop latched
  COMPLETE             home reached — mission done

Key coordination with PathFollowerNode
---------------------------------------
  Publishes /mission/hold (Bool, latched).  When True, PathFollowerNode will NOT
  call _advance_waypoint() after completing FINAL_APPROACH, giving the mission
  controller a window for CAPTURE_MARKER + SEARCH_OBJECT before the robot drives
  on.  The hold is released after CAPTURE_OBJECT (or OBJECT_TIMEOUT).

Subscribes
----------
  /control_mode        (std_msgs/String)           — ModeManagerNode
  /waypoint/status     (std_msgs/String)           — PathFollowerNode
  /navigation/segment  (std_msgs/String)           — PathFollowerNode
  /deadman_ok          (std_msgs/Bool)             — GamepadWatchdogNode
  /emergency_stop      (std_msgs/Bool)             — ModeManagerNode / ObstacleGuard
  /marker/detection    (geometry_msgs/PoseStamped) — ConeDetectorNode
  /object/detection    (std_msgs/String)           — ObjectDetectorNode (JSON)
  /photo/saved         (std_msgs/String)           — PhotoCaptureNode

Publishes
---------
  /mission/state        (std_msgs/String, latched) — current mission state name
  /journey/event        (std_msgs/String)          — JSON event records
  /mission/hold         (std_msgs/Bool,   latched) — pause path_follower advance
  /mission/search_active(std_msgs/Bool,   latched) — hint: search sweep active
  /cmd_vel_auto         (geometry_msgs/Twist)      — search-sweep commands only

Parameters  (config/mission.yaml  →  mission_controller namespace)
----------
  search_trigger_s         delay in FINAL_APPROACH before starting object search
                           after CAPTURE_MARKER  (default 1.0 s)
  search_sweep_duration_s  max duration of one search sweep (default 6.0 s)
  search_sweep_step_s      yaw-direction flip interval (default 1.5 s)
  search_sweep_yaw         yaw angular speed during sweep  (default 0.3 rad/s)
  capture_hold_s           hold time between CAPTURE_MARKER→SEARCH_OBJECT (default 1.0 s)
  object_cache_timeout_s   max age of a cached /object/detection msg (default 5.0 s)
  tick_rate_hz             FSM update frequency (default 10.0)

Step-6 additions
----------------
  Subscribes to /odom to accumulate total_distance_m.
  Emits WAYPOINT_SUMMARY events with per-waypoint photo paths, color, shape,
  range_m and cumulative distance_m.
  Emits EMERGENCY_STOP on rising edge of /emergency_stop.
  Parses "FINAL_APPROACH:<idx>" status strings to track waypoint indices.
"""

from __future__ import annotations

import json
import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from .mission_events import (
    DEADMAN_PAUSED, DEADMAN_RESUMED, EMERGENCY_STOP, HOME_REACHED,
    MANUAL_OVERRIDE as EV_MANUAL_OVERRIDE,
    MARKER_CAPTURED, MISSION_ABORTED, MISSION_COMPLETE, MISSION_STARTED,
    OBJECT_DETECTED, OBJECT_TIMEOUT,
    SEARCH_COMPLETE, SEARCH_START,
    WAYPOINT_ARRIVED, WAYPOINT_SUMMARY, WEAVE_END, WEAVE_START,
)


# ---------------------------------------------------------------------------
# State enum
# ---------------------------------------------------------------------------

class MS(str, Enum):
    """Mission state values — also used as the string published on /mission/state."""
    IDLE                 = 'IDLE'
    MANUAL               = 'MANUAL'
    AUTO_READY           = 'AUTO_READY'
    NAVIGATE_TO_WAYPOINT = 'NAVIGATE_TO_WAYPOINT'
    WEAVING_SEGMENT      = 'WEAVING_SEGMENT'
    FINAL_APPROACH       = 'FINAL_APPROACH'
    CAPTURE_MARKER       = 'CAPTURE_MARKER'
    SEARCH_OBJECT        = 'SEARCH_OBJECT'
    CAPTURE_OBJECT       = 'CAPTURE_OBJECT'
    RETURN_HOME          = 'RETURN_HOME'
    AUTO_PAUSED_DEADMAN  = 'AUTO_PAUSED_DEADMAN'
    MANUAL_OVERRIDE      = 'MANUAL_OVERRIDE'
    ABORTED              = 'ABORTED'
    COMPLETE             = 'COMPLETE'


# States that require AUTO control mode and can be interrupted
_AUTO_ACTIVE = frozenset({
    MS.AUTO_READY,
    MS.NAVIGATE_TO_WAYPOINT,
    MS.WEAVING_SEGMENT,
    MS.FINAL_APPROACH,
    MS.CAPTURE_MARKER,
    MS.SEARCH_OBJECT,
    MS.CAPTURE_OBJECT,
    MS.RETURN_HOME,
    MS.AUTO_PAUSED_DEADMAN,
})

# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MissionControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('mission_controller')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('search_trigger_s',        1.0)
        self.declare_parameter('search_sweep_duration_s', 6.0)
        self.declare_parameter('search_sweep_step_s',     1.5)
        self.declare_parameter('search_sweep_yaw',        0.3)
        self.declare_parameter('capture_hold_s',          1.0)
        self.declare_parameter('object_cache_timeout_s',  5.0)
        self.declare_parameter('tick_rate_hz',           10.0)

        def _dbl(n: str) -> float:
            return self.get_parameter(n).get_parameter_value().double_value

        self._search_trigger_s        = _dbl('search_trigger_s')
        self._search_sweep_duration_s = _dbl('search_sweep_duration_s')
        self._search_sweep_step_s     = _dbl('search_sweep_step_s')
        self._search_sweep_yaw        = _dbl('search_sweep_yaw')
        self._capture_hold_s          = _dbl('capture_hold_s')
        self._object_cache_timeout_s  = _dbl('object_cache_timeout_s')
        tick_rate                     = _dbl('tick_rate_hz')

        # ── QoS ─────────────────────────────────────────────────────────────
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_state  = self.create_publisher(String, '/mission/state',         latched)
        self._pub_event  = self.create_publisher(String, '/journey/event',         10)
        self._pub_hold   = self.create_publisher(Bool,   '/mission/hold',          latched)
        self._pub_search = self.create_publisher(Bool,   '/mission/search_active', latched)
        self._pub_cmd    = self.create_publisher(Twist,  '/cmd_vel_auto',          10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String,      '/control_mode',       self._mode_cb,    latched)
        self.create_subscription(String,      '/waypoint/status',    self._wp_cb,      latched)
        self.create_subscription(String,      '/navigation/segment', self._seg_cb,     latched)
        self.create_subscription(Bool,        '/deadman_ok',         self._dm_cb,      latched)
        self.create_subscription(Bool,        '/emergency_stop',     self._estop_cb,   latched)
        self.create_subscription(PoseStamped, '/marker/detection',   self._marker_cb,  10)
        self.create_subscription(String,      '/object/detection',   self._object_cb,  10)
        self.create_subscription(String,      '/photo/saved',        self._photo_cb,   10)
        # Step 6: subscribe to odometry for cumulative distance tracking
        self.create_subscription(Odometry,    '/odom',               self._odom_cb,    10)

        # ── Internal state ───────────────────────────────────────────────────
        self._state: MS      = MS.IDLE
        self._prev_state: MS = MS.IDLE

        # Inputs from subscriptions
        self._control_mode    = 'MANUAL'
        self._wp_status       = 'IDLE'
        self._nav_segment     = ''
        self._deadman_ok      = False
        self._emergency       = False

        # Perception caches
        self._marker_detected = False
        self._marker_time     = -1.0
        self._last_object_json: str | None = None
        self._last_object_time             = -1.0  # negative = not yet received

        # Timing helpers
        self._state_entry_time  = 0.0   # wall clock when current state was entered
        self._search_start_time = 0.0
        self._sweep_step_time   = 0.0
        self._sweep_direction   = 1.0   # +1.0 = left, -1.0 = right

        # Resume target for dead-man pause
        self._resume_state: MS = MS.AUTO_READY

        # Hold / search flags (track what we have published)
        self._hold_active          = False
        self._search_active_flag   = False

        # Step 6: odometry-based distance tracking
        self._odom_x: float | None = None
        self._odom_y: float | None = None
        self._total_distance_m: float = 0.0

        # Step 6: per-waypoint tracking for WAYPOINT_SUMMARY events
        # waypoint_idx matches the index in PathFollower / photo filenames
        self._current_wp_idx: int = 0
        self._wp_marker_photo: str | None = None   # path set by _photo_cb
        self._wp_object_photo: str | None = None   # path set by _photo_cb

        # ── Tick ─────────────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / tick_rate, self._tick)

        # Publish initial state
        self._publish_state()
        self._set_hold(False)
        self._set_search_active(False)

        self.get_logger().info('MissionController ready — state: IDLE')

    # =========================================================================
    # Subscription callbacks
    # =========================================================================

    def _mode_cb(self, msg: String) -> None:
        self._control_mode = msg.data

    def _wp_cb(self, msg: String) -> None:
        raw = msg.data
        # PathFollower may publish "FINAL_APPROACH:<idx>" — extract index and
        # normalise the status string so existing comparisons still work.
        if ':' in raw:
            base, _, idx_str = raw.partition(':')
            try:
                self._current_wp_idx = int(idx_str)
            except ValueError:
                pass
            self._wp_status = base
        else:
            self._wp_status = raw

    def _seg_cb(self, msg: String) -> None:
        self._nav_segment = msg.data

    def _dm_cb(self, msg: Bool) -> None:
        self._deadman_ok = msg.data

    def _estop_cb(self, msg: Bool) -> None:
        # Emit EMERGENCY_STOP on rising edge (False → True)
        if msg.data and not self._emergency:
            self._emit_event(EMERGENCY_STOP, {
                'distance_m': round(self._total_distance_m, 2),
            })
        self._emergency = msg.data

    def _marker_cb(self, msg: PoseStamped) -> None:
        self._marker_detected = True
        self._marker_time = self._now()

    def _object_cb(self, msg: String) -> None:
        try:
            objs = json.loads(msg.data)
            if objs:  # non-empty list
                self._last_object_json = msg.data
                self._last_object_time = self._now()
        except (json.JSONDecodeError, TypeError):
            pass

    def _photo_cb(self, msg: String) -> None:
        """Track marker/object photo paths for the current waypoint."""
        path = msg.data
        self.get_logger().info(f'Photo saved: {path}')
        # PhotoCaptureNode publishes the annotated composite; derive the
        # individual marker / object filenames from the same prefix.
        if '_annotated' in path:
            base = path[:path.rfind('_annotated')]
            self._wp_marker_photo = base + '_marker.jpg'
            self._wp_object_photo = base + '_object.jpg'
        self._emit_event('PHOTO_SAVED', {
            'path': path,
            'waypoint_idx': self._current_wp_idx,
        })

    def _odom_cb(self, msg: Odometry) -> None:
        """Accumulate Euclidean distance from successive odometry poses."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._odom_x is not None:
            dx = x - self._odom_x
            dy = y - self._odom_y
            self._total_distance_m += math.sqrt(dx * dx + dy * dy)
        self._odom_x = x
        self._odom_y = y

    # =========================================================================
    # Main tick — FSM step
    # =========================================================================

    def _tick(self) -> None:
        # ── Global overrides (highest priority) ─────────────────────────────
        if self._emergency and self._state != MS.ABORTED:
            self._stop_sweep()
            self._set_hold(False)
            self._set_search_active(False)
            self._transition(MS.ABORTED, MISSION_ABORTED,
                             extra={'distance_m': round(self._total_distance_m, 2)})
            return

        # ── Dead-man: any active AUTO state → PAUSED ─────────────────────────
        if (self._state in _AUTO_ACTIVE
                and self._state != MS.AUTO_PAUSED_DEADMAN
                and self._control_mode == 'PAUSED'):
            self._resume_state = self._state
            self._transition(MS.AUTO_PAUSED_DEADMAN, DEADMAN_PAUSED)
            return

        # ── Dispatch on current state ────────────────────────────────────────
        s = self._state
        if s in (MS.IDLE, MS.MANUAL, MS.MANUAL_OVERRIDE):
            self._handle_inactive()
        elif s == MS.AUTO_PAUSED_DEADMAN:
            self._handle_paused()
        elif s == MS.AUTO_READY:
            self._handle_auto_ready()
        elif s in (MS.NAVIGATE_TO_WAYPOINT, MS.WEAVING_SEGMENT):
            self._handle_navigating()
        elif s == MS.FINAL_APPROACH:
            self._handle_final_approach()
        elif s == MS.CAPTURE_MARKER:
            self._handle_capture_marker()
        elif s == MS.SEARCH_OBJECT:
            self._handle_search_object()
        elif s == MS.CAPTURE_OBJECT:
            self._handle_capture_object()
        elif s == MS.RETURN_HOME:
            self._handle_return_home()
        # ABORTED, COMPLETE: terminal — no further transitions

    # =========================================================================
    # Per-state handlers
    # =========================================================================

    def _handle_inactive(self) -> None:
        """IDLE / MANUAL / MANUAL_OVERRIDE: wait for X button (AUTO mode)."""
        cm = self._control_mode
        if cm == 'AUTO':
            # Reset odometry accumulator at mission start
            self._total_distance_m = 0.0
            self._odom_x = None
            self._odom_y = None
            self._transition(MS.AUTO_READY, MISSION_STARTED)
        elif cm == 'MANUAL' and self._state != MS.MANUAL:
            self._transition(MS.MANUAL)
        elif cm == 'ABORTED' and self._state != MS.ABORTED:
            self._transition(MS.ABORTED, MISSION_ABORTED,
                             extra={'distance_m': round(self._total_distance_m, 2)})

    def _handle_paused(self) -> None:
        """AUTO_PAUSED_DEADMAN: wait for deadman or manual."""
        cm = self._control_mode
        if cm == 'AUTO':
            # Deadman restored → resume
            self._emit_event(DEADMAN_RESUMED)
            self._transition(self._resume_state)
        elif cm == 'MANUAL':
            self._transition(MS.MANUAL)

    def _handle_auto_ready(self) -> None:
        """AUTO_READY: wait for PathFollower to start navigating."""
        if self._control_mode != 'AUTO':
            self._transition(MS.MANUAL, EV_MANUAL_OVERRIDE)
            return
        ws = self._wp_status
        if ws == 'NAVIGATING':
            new = MS.WEAVING_SEGMENT if self._nav_segment == '1' else MS.NAVIGATE_TO_WAYPOINT
            self._transition(new)
        elif ws in ('COARSE_ARRIVED', 'FINAL_APPROACH'):
            # Already at a waypoint
            self._on_coarse_arrived()
        elif ws == 'HOMING':
            self._transition(MS.RETURN_HOME)
        elif ws == 'DONE':
            self._transition(MS.COMPLETE, MISSION_COMPLETE,
                             extra={'distance_m': round(self._total_distance_m, 2)})

    def _handle_navigating(self) -> None:
        """NAVIGATE_TO_WAYPOINT / WEAVING_SEGMENT: tracking path_follower."""
        if self._control_mode == 'MANUAL':
            self._set_hold(False)
            self._transition(MS.MANUAL_OVERRIDE, EV_MANUAL_OVERRIDE)
            return

        ws  = self._wp_status
        seg = self._nav_segment

        if ws == 'DONE':
            self._set_hold(False)
            self._transition(MS.COMPLETE, MISSION_COMPLETE,
                             extra={'distance_m': round(self._total_distance_m, 2)})
        elif ws == 'HOMING':
            self._set_hold(False)
            self._transition(MS.RETURN_HOME)
        elif ws in ('COARSE_ARRIVED', 'FINAL_APPROACH'):
            self._on_coarse_arrived()
        elif ws == 'NAVIGATING':
            # Update NAVIGATE vs WEAVE based on segment
            expected = MS.WEAVING_SEGMENT if seg == '1' else MS.NAVIGATE_TO_WAYPOINT
            if expected != self._state:
                event = WEAVE_START if expected == MS.WEAVING_SEGMENT else WEAVE_END
                self._transition(expected, event)

    def _on_coarse_arrived(self) -> None:
        """Called when path_follower enters COARSE_ARRIVED for a new waypoint."""
        # Reset per-waypoint photo state for the new waypoint
        self._wp_marker_photo = None
        self._wp_object_photo = None
        self._emit_event(WAYPOINT_ARRIVED, {
            'segment':      self._nav_segment,
            'waypoint_idx': self._current_wp_idx,
            'distance_m':   round(self._total_distance_m, 2),
        })
        self._transition(MS.FINAL_APPROACH)

    def _handle_final_approach(self) -> None:
        """FINAL_APPROACH: path_follower in COARSE_ARRIVED, waiting for marker."""
        if self._control_mode == 'MANUAL':
            self._set_hold(False)
            self._set_search_active(False)
            self._transition(MS.MANUAL_OVERRIDE, EV_MANUAL_OVERRIDE)
            return

        ws = self._wp_status
        if ws == 'FINAL_APPROACH':
            # Path_follower confirmed marker and is driving through pass point
            self._set_hold(True)   # prevent advance until SEARCH_OBJECT done
            self._transition(MS.CAPTURE_MARKER, MARKER_CAPTURED,
                             extra={'waypoint_idx': self._current_wp_idx})
            return
        if ws == 'HOMING':
            self._set_hold(False)
            self._transition(MS.RETURN_HOME)
            return
        if ws == 'DONE':
            self._set_hold(False)
            self._transition(MS.COMPLETE, MISSION_COMPLETE,
                             extra={'distance_m': round(self._total_distance_m, 2)})
            return
        if ws == 'NAVIGATING':
            # Path_follower advanced without marker — unlikely but handle gracefully
            self._set_hold(False)
            new = MS.WEAVING_SEGMENT if self._nav_segment == '1' else MS.NAVIGATE_TO_WAYPOINT
            self._transition(new)

    def _handle_capture_marker(self) -> None:
        """CAPTURE_MARKER: path_follower is in FINAL_APPROACH (driving to pass pt).
        We hold until the robot stops at the pass point, then start object search."""
        if self._control_mode == 'MANUAL':
            self._set_hold(False)
            self._transition(MS.MANUAL_OVERRIDE, EV_MANUAL_OVERRIDE)
            return

        elapsed = self._now() - self._state_entry_time
        if elapsed < self._capture_hold_s:
            return  # wait briefly for photo_capture to grab the frame

        # Try fast-path: object already detected during approach
        if self._has_recent_object():
            self._transition(MS.CAPTURE_OBJECT, OBJECT_DETECTED)
            return

        # Start yaw-sweep search for colored object
        self._start_search()
        self._transition(MS.SEARCH_OBJECT)

    def _handle_search_object(self) -> None:
        """SEARCH_OBJECT: yaw-sweep to find colored object near waypoint."""
        if self._control_mode == 'MANUAL':
            self._stop_sweep()
            self._set_hold(False)
            self._set_search_active(False)
            self._transition(MS.MANUAL_OVERRIDE, EV_MANUAL_OVERRIDE)
            return

        # Object found during sweep
        if self._has_recent_object():
            self._stop_sweep()
            self._set_search_active(False)
            self._transition(MS.CAPTURE_OBJECT, OBJECT_DETECTED)
            return

        elapsed = self._now() - self._search_start_time
        if elapsed >= self._search_sweep_duration_s:
            # Timeout: log, emit waypoint summary (no object found), resume
            self._stop_sweep()
            self._set_search_active(False)
            self._emit_event(OBJECT_TIMEOUT)
            self._emit_waypoint_summary(object_found=False)
            self._set_hold(False)
            self._resume_from_search()
            return

        # Continue sweep
        self._do_sweep_step()

    def _handle_capture_object(self) -> None:
        """CAPTURE_OBJECT: colored object confirmed — hold briefly, then resume."""
        if self._control_mode == 'MANUAL':
            self._set_hold(False)
            self._transition(MS.MANUAL_OVERRIDE, EV_MANUAL_OVERRIDE)
            return

        elapsed = self._now() - self._state_entry_time
        if elapsed >= self._capture_hold_s:
            self._emit_waypoint_summary(object_found=True)
            self._set_search_active(False)
            self._set_hold(False)
            self._resume_from_search()

    def _handle_return_home(self) -> None:
        """RETURN_HOME: path_follower is homing."""
        if self._control_mode == 'MANUAL':
            self._transition(MS.MANUAL_OVERRIDE, EV_MANUAL_OVERRIDE)
            return

        ws = self._wp_status
        if ws == 'DONE':
            self._emit_event(HOME_REACHED, {'distance_m': round(self._total_distance_m, 2)})
            self._transition(MS.COMPLETE, MISSION_COMPLETE,
                             extra={'distance_m': round(self._total_distance_m, 2)})

    # =========================================================================
    # Step-6 helpers
    # =========================================================================

    def _emit_waypoint_summary(self, *, object_found: bool) -> None:
        """Emit a WAYPOINT_SUMMARY event with consolidated per-waypoint data.

        Called after CAPTURE_OBJECT (object_found=True) or OBJECT_TIMEOUT
        (object_found=False).  Includes photo paths, detected object attributes,
        and the cumulative odometry distance at this moment.
        """
        extra: dict = {
            'waypoint_idx': self._current_wp_idx,
            'object_found': object_found,
            'marker_photo': self._wp_marker_photo,
            'object_photo': self._wp_object_photo,
            'distance_m':   round(self._total_distance_m, 2),
        }

        if object_found and self._last_object_json:
            try:
                objs = json.loads(self._last_object_json)
                if objs:
                    # Pick the largest detected object by bounding-box area
                    best = max(objs, key=lambda o: o.get('w', 0) * o.get('h', 0))
                    extra['color']   = best.get('color')
                    extra['shape']   = best.get('shape')
                    extra['range_m'] = best.get('range_m')
            except (json.JSONDecodeError, TypeError):
                pass

        self._emit_event(WAYPOINT_SUMMARY, extra)

        # Reset per-waypoint accumulators so the next waypoint starts clean
        self._wp_marker_photo = None
        self._wp_object_photo = None

    # =========================================================================
    # Search sweep helpers
    # =========================================================================

    def _start_search(self) -> None:
        self._search_start_time = self._now()
        self._sweep_step_time   = self._now()
        self._sweep_direction   = 1.0
        self._set_search_active(True)
        self._emit_event(SEARCH_START)

    def _do_sweep_step(self) -> None:
        """Publish one frame of alternating yaw sweep to /cmd_vel_auto."""
        elapsed_step = self._now() - self._sweep_step_time
        if elapsed_step >= self._search_sweep_step_s:
            self._sweep_direction *= -1.0
            self._sweep_step_time  = self._now()

        cmd = Twist()
        cmd.angular.z = float(self._sweep_direction * self._search_sweep_yaw)
        self._pub_cmd.publish(cmd)

    def _stop_sweep(self) -> None:
        """Stop any ongoing sweep motion and log completion."""
        self._pub_cmd.publish(Twist())  # zero velocity
        self._emit_event(SEARCH_COMPLETE)

    def _resume_from_search(self) -> None:
        """Transition back to the appropriate navigation state after search."""
        ws  = self._wp_status
        seg = self._nav_segment
        if ws == 'DONE':
            self._transition(MS.COMPLETE, MISSION_COMPLETE,
                             extra={'distance_m': round(self._total_distance_m, 2)})
        elif ws in ('HOMING',):
            self._transition(MS.RETURN_HOME)
        elif ws == 'NAVIGATING':
            new = MS.WEAVING_SEGMENT if seg == '1' else MS.NAVIGATE_TO_WAYPOINT
            self._transition(new)
        else:
            # Fallback: wait for path_follower to report NAVIGATING
            self._transition(MS.NAVIGATE_TO_WAYPOINT)

    # =========================================================================
    # Hold / search-active helpers
    # =========================================================================

    def _set_hold(self, val: bool) -> None:
        if val != self._hold_active:
            self._hold_active = val
            self._pub_hold.publish(Bool(data=val))
            self.get_logger().debug(f'mission/hold → {val}')

    def _set_search_active(self, val: bool) -> None:
        if val != self._search_active_flag:
            self._search_active_flag = val
            self._pub_search.publish(Bool(data=val))

    # =========================================================================
    # State transition helpers
    # =========================================================================

    def _transition(self, new_state: MS, event_type: str | None = None,
                    *, extra: dict | None = None) -> None:
        if new_state == self._state and event_type is None:
            return
        if new_state != self._state:
            self.get_logger().info(f'Mission: {self._state.value} → {new_state.value}')
        self._prev_state       = self._state
        self._state            = new_state
        self._state_entry_time = self._now()
        self._publish_state()
        if event_type:
            self._emit_event(event_type, extra)

    def _publish_state(self) -> None:
        self._pub_state.publish(String(data=self._state.value))

    def _emit_event(self, event_type: str, extra: dict | None = None) -> None:
        payload: dict = {
            'type':  event_type,
            'ts':    time.time(),
            'state': self._state.value,
        }
        if extra:
            payload.update(extra)
        try:
            self._pub_event.publish(String(data=json.dumps(payload)))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Event publish failed: {exc}')

    # =========================================================================
    # Time / perception helpers
    # =========================================================================

    def _now(self) -> float:
        """Monotonic wall-clock seconds (independent of ROS clock)."""
        return time.monotonic()

    def _has_recent_object(self) -> bool:
        if self._last_object_time < 0.0:  # negative sentinel = never received
            return False
        return (self._now() - self._last_object_time) < self._object_cache_timeout_s


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
