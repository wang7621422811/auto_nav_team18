"""
SummaryGeneratorNode — reads the journey JSONL log and writes a human-readable
summary at mission end (Step 6).

Watches /mission/state; when it sees "COMPLETE" or "ABORTED" it reads the
current JSONL log and produces two files under summary_dir:

  summary_<timestamp>.json   — machine-readable structured data
  summary_<timestamp>.md     — Markdown report matching the Step-6 template

Subscribes
----------
  /mission/state  (std_msgs/String, latched) — from MissionControllerNode

Publishes
---------
  /mission/summary_path  (std_msgs/String)  — absolute path of the .md file

Parameters  (config/mission.yaml  →  summary_generator namespace)
----------
  log_dir      directory for the JSONL log  (default "artifacts/logs")
  log_filename JSONL log filename           (default "journey.jsonl")
  summary_dir  output directory            (default "artifacts/summaries")

Mission Result logic
--------------------
  SUCCESS  — final_state == COMPLETE  AND  HOME_REACHED event present
  PARTIAL  — at least one WAYPOINT_SUMMARY emitted  (mission incomplete)
  FAIL     — no waypoints visited (or aborted before any waypoint)
"""

from __future__ import annotations

import json
import os
from collections import defaultdict
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class SummaryGeneratorNode(Node):

    def __init__(self) -> None:
        super().__init__('summary_generator')

        self.declare_parameter('log_dir',      'artifacts/logs')
        self.declare_parameter('log_filename', 'journey.jsonl')
        self.declare_parameter('summary_dir',  'artifacts/summaries')

        self._log_dir     = self.get_parameter('log_dir').get_parameter_value().string_value
        self._log_file    = self.get_parameter('log_filename').get_parameter_value().string_value
        self._summary_dir = self.get_parameter('summary_dir').get_parameter_value().string_value
        self._log_path    = os.path.join(self._log_dir, self._log_file)

        os.makedirs(self._summary_dir, exist_ok=True)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_path = self.create_publisher(String, '/mission/summary_path', latched)
        self.create_subscription(String, '/mission/state', self._state_cb, latched)

        self._summary_written = False
        self.get_logger().info('SummaryGenerator ready')

    # ------------------------------------------------------------------

    def _state_cb(self, msg: String) -> None:
        state = msg.data
        if state in ('COMPLETE', 'ABORTED') and not self._summary_written:
            self._summary_written = True
            self._write_summary(state)

    # ------------------------------------------------------------------

    def _write_summary(self, final_state: str) -> None:
        events  = self._load_events()
        data    = self._build_summary(events, final_state)

        ts_str   = datetime.now(tz=timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        json_path = os.path.join(self._summary_dir, f'summary_{ts_str}.json')
        md_path   = os.path.join(self._summary_dir, f'summary_{ts_str}.md')

        # Write JSON
        try:
            with open(json_path, 'w') as f:
                json.dump(data, f, indent=2, default=str)
            self.get_logger().info(f'Journey summary JSON  → {json_path}')
        except OSError as exc:
            self.get_logger().error(f'Failed to write JSON summary: {exc}')

        # Write Markdown
        try:
            md_text = _render_markdown(data)
            with open(md_path, 'w') as f:
                f.write(md_text)
            self.get_logger().info(f'Journey summary MD    → {md_path}')
            self._pub_path.publish(String(data=md_path))
        except OSError as exc:
            self.get_logger().error(f'Failed to write Markdown summary: {exc}')

    # ------------------------------------------------------------------

    def _load_events(self) -> list[dict]:
        events: list[dict] = []
        if not os.path.exists(self._log_path):
            self.get_logger().warn(f'Log file not found: {self._log_path!r}')
            return events
        try:
            with open(self._log_path) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        try:
                            events.append(json.loads(line))
                        except json.JSONDecodeError:
                            pass
        except OSError as exc:
            self.get_logger().warn(f'Cannot read log: {exc}')
        return events

    def _build_summary(self, events: list[dict], final_state: str) -> dict:
        """Aggregate events into a structured summary dict."""
        counters: dict[str, int] = defaultdict(int)
        for ev in events:
            counters[ev.get('type', 'UNKNOWN')] += 1

        start_ts = events[0]['ts']  if events else None
        end_ts   = events[-1]['ts'] if events else None
        duration = (end_ts - start_ts) if (start_ts and end_ts) else None

        # Total distance: last event that carries 'distance_m'
        total_distance_m: float = 0.0
        for ev in reversed(events):
            if 'distance_m' in ev:
                total_distance_m = float(ev['distance_m'])
                break

        deadman_stops   = counters.get('DEADMAN_PAUSED',  0)
        emergency_stops = counters.get('EMERGENCY_STOP',  0)
        home_reached    = any(ev.get('type') == 'HOME_REACHED' for ev in events)

        # Per-waypoint data from WAYPOINT_SUMMARY events (keyed by waypoint_idx)
        waypoints: dict[int, dict] = {}
        for ev in events:
            if ev.get('type') == 'WAYPOINT_SUMMARY':
                idx = ev.get('waypoint_idx', 0)
                waypoints[idx] = {
                    'waypoint_idx':   idx,
                    'object_found':   ev.get('object_found', False),
                    'marker_photo':   ev.get('marker_photo'),
                    'object_photo':   ev.get('object_photo'),
                    'color':          ev.get('color'),
                    'shape':          ev.get('shape'),
                    'range_m':        ev.get('range_m'),
                    'distance_m':     ev.get('distance_m'),
                }

        # Arrival timestamps per waypoint (from WAYPOINT_ARRIVED)
        for ev in events:
            if ev.get('type') == 'WAYPOINT_ARRIVED':
                idx = ev.get('waypoint_idx', 0)
                if idx not in waypoints:
                    waypoints[idx] = {'waypoint_idx': idx}
                waypoints[idx]['arrival_ts'] = ev.get('ts')

        # Mission result
        if final_state == 'COMPLETE' and home_reached:
            result = 'SUCCESS'
        elif len(waypoints) > 0:
            result = 'PARTIAL'
        else:
            result = 'FAIL'

        # Ordered waypoint list (ascending index)
        waypoints_list = [waypoints[k] for k in sorted(waypoints)]

        return {
            'result':           result,
            'final_state':      final_state,
            'start_time':       datetime.fromtimestamp(start_ts, tz=timezone.utc).isoformat()
                                if start_ts else None,
            'end_time':         datetime.fromtimestamp(end_ts, tz=timezone.utc).isoformat()
                                if end_ts else None,
            'duration_s':       round(duration, 1) if duration is not None else None,
            'total_distance_m': round(total_distance_m, 2),
            'deadman_stops':    deadman_stops,
            'emergency_stops':  emergency_stops,
            'home_reached':     home_reached,
            'waypoints':        waypoints_list,
            'event_counts':     dict(counters),
        }


# ---------------------------------------------------------------------------
# Pure function — also used by unit tests
# ---------------------------------------------------------------------------

def _render_markdown(data: dict) -> str:
    """Render a summary dict as a Markdown report matching the Step-6 template."""
    lines: list[str] = []

    result   = data.get('result',           'UNKNOWN')
    duration = data.get('duration_s')
    dist     = data.get('total_distance_m', 0.0)
    dm_stops = data.get('deadman_stops',    0)
    em_stops = data.get('emergency_stops',  0)

    # Format duration as HH:MM:SS when available
    if duration is not None:
        h = int(duration) // 3600
        m = (int(duration) % 3600) // 60
        s = int(duration) % 60
        duration_str = f'{h:02d}:{m:02d}:{s:02d}'
    else:
        duration_str = 'N/A'

    lines.append('# Mission Summary\n')
    lines.append(f'Mission Result: **{result}**  ')
    lines.append(f'Total Time: {duration_str}  ')
    lines.append(f'Total Distance: {dist:.2f} m  ')
    lines.append(f'Dead-man Stops: {dm_stops}  ')
    lines.append(f'Emergency Stops: {em_stops}  ')

    waypoints: list[dict] = data.get('waypoints', [])
    if waypoints:
        lines.append('')
        for wp in waypoints:
            wp_num = int(wp.get('waypoint_idx', 0)) + 1  # display as 1-based
            lines.append(f'\n## Waypoint {wp_num}')

            marker_photo = wp.get('marker_photo') or '(not captured)'
            object_photo = wp.get('object_photo') or '(not captured)'
            color        = wp.get('color')  or '(unknown)'
            shape        = wp.get('shape')  or '(unknown)'
            range_m      = wp.get('range_m')

            lines.append(f'- Marker photo: {marker_photo}')
            lines.append(f'- Object photo: {object_photo}')
            lines.append(f'- Object color: {color}')
            lines.append(f'- Object shape: {shape}')
            if range_m is not None:
                lines.append(f'- Distance to marker: {float(range_m):.2f} m')
            else:
                lines.append('- Distance to marker: (unknown)')
    else:
        lines.append('\n*No waypoints visited.*')

    lines.append('')  # trailing newline
    return '\n'.join(lines) + '\n'


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SummaryGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
