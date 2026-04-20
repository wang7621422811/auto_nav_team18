"""
Mission event type strings published on /journey/event (JSON).

Each message is a JSON object:
  {
    "type":  <EVENT_TYPE_STRING>,
    "ts":    <float epoch seconds>,
    "state": <current MissionState name>,
    ...extra key-value pairs specific to the event
  }

Step-6 additions
----------------
EMERGENCY_STOP   — emitted each time /emergency_stop transitions to True
WAYPOINT_SUMMARY — emitted after CAPTURE_OBJECT (or OBJECT_TIMEOUT) with full
                   per-waypoint data: waypoint_idx, marker_photo, object_photo,
                   color, shape, range_m, distance_m (cumulative odometry).
"""

from __future__ import annotations

# ── Mission lifecycle ────────────────────────────────────────────────────────
MISSION_STARTED   = 'MISSION_STARTED'
MISSION_COMPLETE  = 'MISSION_COMPLETE'
MISSION_ABORTED   = 'MISSION_ABORTED'

# ── Navigation ───────────────────────────────────────────────────────────────
WAYPOINT_ARRIVED  = 'WAYPOINT_ARRIVED'   # coarse arrival
WAYPOINT_SKIPPED  = 'WAYPOINT_SKIPPED'   # timed out without marker
WEAVE_START       = 'WEAVE_START'        # entered weaving segment
WEAVE_END         = 'WEAVE_END'          # exited weaving segment

# ── Perception ───────────────────────────────────────────────────────────────
MARKER_CAPTURED   = 'MARKER_CAPTURED'    # robot passing orange cone
SEARCH_START      = 'SEARCH_START'       # starting object search sweep
SEARCH_COMPLETE   = 'SEARCH_COMPLETE'    # search sweep finished
OBJECT_DETECTED   = 'OBJECT_DETECTED'    # colored object confirmed
OBJECT_TIMEOUT    = 'OBJECT_TIMEOUT'     # search timed out, no object
WAYPOINT_SUMMARY  = 'WAYPOINT_SUMMARY'   # per-waypoint consolidated record

# ── Safety / mode ────────────────────────────────────────────────────────────
MANUAL_OVERRIDE   = 'MANUAL_OVERRIDE'    # operator pressed O button mid-auto
DEADMAN_PAUSED    = 'DEADMAN_PAUSED'     # dead-man trigger released
DEADMAN_RESUMED   = 'DEADMAN_RESUMED'    # dead-man trigger re-pressed
EMERGENCY_STOP    = 'EMERGENCY_STOP'     # /emergency_stop fired (rising edge)
HOME_REACHED      = 'HOME_REACHED'       # robot returned to home position
