"""
Unit tests for SummaryGeneratorNode (Step 6).

Tests cover:
  1.  _build_summary: SUCCESS when COMPLETE + HOME_REACHED
  2.  _build_summary: PARTIAL when some waypoints visited but not COMPLETE
  3.  _build_summary: FAIL when no waypoints visited
  4.  _build_summary: total_distance_m extracted from last event with distance_m
  5.  _build_summary: deadman_stops and emergency_stops counted correctly
  6.  _build_summary: per-waypoint photo paths, color, shape, range_m parsed
  7.  _build_summary: empty events produces safe defaults (no crash)
  8.  _render_markdown: SUCCESS result uses correct template structure
  9.  _render_markdown: PARTIAL result appears in header
  10. _render_markdown: each waypoint section rendered with correct fields
  11. _render_markdown: missing photo paths replaced with "(not captured)"
  12. _render_markdown: missing range_m replaced with "(unknown)"
  13. _render_markdown: duration formatted as HH:MM:SS
  14. _render_markdown: no waypoints renders fallback message
  15. _write_summary: writes summary.json and summary.md files
  16. _write_summary: works correctly with partial/empty event list
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import MagicMock


# ---------------------------------------------------------------------------
# ROS2 stubs — no real ROS2 installation needed
# ---------------------------------------------------------------------------

def _build_stubs() -> None:
    rclpy      = types.ModuleType('rclpy')
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

        def get_logger(self):                     return self._logger
        def create_publisher(self, *a, **kw):     return MagicMock()
        def create_subscription(self, *a, **kw):  return MagicMock()
        def destroy_node(self):                   pass

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
                p.get_parameter_value.return_value.string_value  = str(val or '')
            return p

    rclpy.node.Node = _Node

    std_msgs     = types.ModuleType('std_msgs')
    std_msgs.msg = types.ModuleType('std_msgs.msg')

    class _String:
        def __init__(self, data=''):
            self.data = data

    std_msgs.msg.String = _String

    for mod_name, mod in [
        ('rclpy',       rclpy),
        ('rclpy.node',  rclpy.node),
        ('rclpy.qos',   rclpy.qos),
        ('std_msgs',    std_msgs),
        ('std_msgs.msg', std_msgs.msg),
    ]:
        sys.modules[mod_name] = mod


_build_stubs()

sys.path.insert(0, '/home/parallels/workspace/auto_nav_team18')
from auto_nav.mission.summary_generator import (  # noqa: E402
    SummaryGeneratorNode, _render_markdown,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

NOW = time.time()


def _ev(event_type: str, ts_offset: float = 0.0, **extra) -> dict:
    return {'type': event_type, 'ts': NOW + ts_offset, 'state': 'AUTO', **extra}


def _wp_summary(idx: int, **kw) -> dict:
    defaults = {
        'waypoint_idx': idx,
        'object_found': True,
        'marker_photo': f'artifacts/photos/wp_{idx:02d}_marker.jpg',
        'object_photo': f'artifacts/photos/wp_{idx:02d}_object.jpg',
        'color':        'red',
        'shape':        'circle',
        'range_m':      1.5,
        'distance_m':   10.0 * (idx + 1),
    }
    defaults.update(kw)
    return _ev('WAYPOINT_SUMMARY', ts_offset=float(idx + 1), **defaults)


def _make_node(tmpdir: str) -> SummaryGeneratorNode:
    """Instantiate a SummaryGeneratorNode with its params pointing to tmpdir."""
    node = SummaryGeneratorNode.__new__(SummaryGeneratorNode)
    node._params = {
        'log_dir':      tmpdir,
        'log_filename': 'journey.jsonl',
        'summary_dir':  tmpdir,
    }
    node._logger        = MagicMock()
    node._log_dir       = tmpdir
    node._log_file      = 'journey.jsonl'
    node._log_path      = os.path.join(tmpdir, 'journey.jsonl')
    node._summary_dir   = tmpdir
    node._pub_path      = MagicMock()
    node._summary_written = False
    node.get_logger     = lambda: node._logger
    return node


# ---------------------------------------------------------------------------
# Tests — _build_summary
# ---------------------------------------------------------------------------

class TestBuildSummary(unittest.TestCase):

    def _build(self, events, final_state='COMPLETE'):
        node = _make_node('/tmp')
        return node._build_summary(events, final_state)

    # 1. SUCCESS
    def test_success_when_complete_and_home_reached(self):
        events = [
            _ev('MISSION_STARTED'),
            _wp_summary(0),
            _ev('HOME_REACHED', ts_offset=5.0, distance_m=15.0),
            _ev('MISSION_COMPLETE', ts_offset=5.1, distance_m=15.0),
        ]
        s = self._build(events, 'COMPLETE')
        self.assertEqual(s['result'], 'SUCCESS')
        self.assertTrue(s['home_reached'])

    # 2. PARTIAL
    def test_partial_when_some_waypoints_visited(self):
        events = [
            _ev('MISSION_STARTED'),
            _wp_summary(0),
            _ev('MISSION_ABORTED', ts_offset=8.0, distance_m=12.0),
        ]
        s = self._build(events, 'ABORTED')
        self.assertEqual(s['result'], 'PARTIAL')

    # 3. FAIL
    def test_fail_when_no_waypoints(self):
        events = [
            _ev('MISSION_STARTED'),
            _ev('MISSION_ABORTED', ts_offset=1.0),
        ]
        s = self._build(events, 'ABORTED')
        self.assertEqual(s['result'], 'FAIL')

    # 4. total_distance_m
    def test_total_distance_from_last_event_with_distance(self):
        events = [
            _ev('MISSION_STARTED'),
            _wp_summary(0, distance_m=10.5),
            _ev('HOME_REACHED', ts_offset=9.0, distance_m=20.3),
            _ev('MISSION_COMPLETE', ts_offset=9.1, distance_m=20.3),
        ]
        s = self._build(events, 'COMPLETE')
        self.assertAlmostEqual(s['total_distance_m'], 20.3, places=1)

    # 5. counters
    def test_deadman_and_emergency_counts(self):
        events = [
            _ev('MISSION_STARTED'),
            _ev('DEADMAN_PAUSED',  ts_offset=1.0),
            _ev('DEADMAN_PAUSED',  ts_offset=2.0),
            _ev('EMERGENCY_STOP',  ts_offset=3.0, distance_m=5.0),
            _ev('MISSION_ABORTED', ts_offset=3.1),
        ]
        s = self._build(events, 'ABORTED')
        self.assertEqual(s['deadman_stops'],   2)
        self.assertEqual(s['emergency_stops'], 1)

    # 6. per-waypoint fields
    def test_waypoint_fields_parsed(self):
        events = [
            _ev('MISSION_STARTED'),
            _wp_summary(0, color='blue', shape='square', range_m=2.1,
                         marker_photo='photos/wp_00_marker.jpg',
                         object_photo='photos/wp_00_object.jpg'),
        ]
        s = self._build(events, 'ABORTED')
        wp = s['waypoints'][0]
        self.assertEqual(wp['color'],        'blue')
        self.assertEqual(wp['shape'],        'square')
        self.assertAlmostEqual(wp['range_m'], 2.1, places=2)
        self.assertEqual(wp['marker_photo'], 'photos/wp_00_marker.jpg')
        self.assertEqual(wp['object_photo'], 'photos/wp_00_object.jpg')

    # 7. empty events
    def test_empty_events_no_crash(self):
        s = self._build([], 'ABORTED')
        self.assertEqual(s['result'],     'FAIL')
        self.assertIsNone(s['start_time'])
        self.assertEqual(s['waypoints'],  [])
        self.assertEqual(s['total_distance_m'], 0.0)

    # Waypoints ordered by index
    def test_waypoints_ordered_ascending(self):
        events = [
            _ev('MISSION_STARTED'),
            _wp_summary(2, distance_m=30.0),
            _wp_summary(0, distance_m=10.0),
            _wp_summary(1, distance_m=20.0),
        ]
        s = self._build(events, 'ABORTED')
        indices = [wp['waypoint_idx'] for wp in s['waypoints']]
        self.assertEqual(indices, [0, 1, 2])


# ---------------------------------------------------------------------------
# Tests — _render_markdown
# ---------------------------------------------------------------------------

class TestRenderMarkdown(unittest.TestCase):

    def _data(self, result='SUCCESS', waypoints=None, duration_s=3661.0,
              dist=25.5, dm=1, em=0) -> dict:
        return {
            'result':           result,
            'final_state':      'COMPLETE',
            'start_time':       '2026-04-20T10:00:00+00:00',
            'end_time':         '2026-04-20T11:01:01+00:00',
            'duration_s':       duration_s,
            'total_distance_m': dist,
            'deadman_stops':    dm,
            'emergency_stops':  em,
            'home_reached':     result == 'SUCCESS',
            'waypoints':        waypoints or [],
        }

    # 8. SUCCESS in header
    def test_success_result_in_header(self):
        md = _render_markdown(self._data('SUCCESS'))
        self.assertIn('SUCCESS', md)

    # 9. PARTIAL result
    def test_partial_result_in_header(self):
        md = _render_markdown(self._data('PARTIAL'))
        self.assertIn('PARTIAL', md)

    # 10. Per-waypoint section rendered
    def test_waypoint_section_rendered(self):
        wps = [{
            'waypoint_idx': 0,
            'marker_photo': 'photos/wp_00_marker.jpg',
            'object_photo': 'photos/wp_00_object.jpg',
            'color':        'green',
            'shape':        'triangle',
            'range_m':      1.8,
        }]
        md = _render_markdown(self._data('SUCCESS', waypoints=wps))
        self.assertIn('Waypoint 1', md)
        self.assertIn('photos/wp_00_marker.jpg', md)
        self.assertIn('photos/wp_00_object.jpg', md)
        self.assertIn('green', md)
        self.assertIn('triangle', md)
        self.assertIn('1.80 m', md)

    # 11. Missing photo → "(not captured)"
    def test_missing_photo_fallback(self):
        wps = [{'waypoint_idx': 0, 'marker_photo': None, 'object_photo': None,
                'color': None, 'shape': None, 'range_m': None}]
        md = _render_markdown(self._data(waypoints=wps))
        self.assertIn('(not captured)', md)

    # 12. Missing range_m → "(unknown)"
    def test_missing_range_m_fallback(self):
        wps = [{'waypoint_idx': 0, 'marker_photo': None, 'object_photo': None,
                'color': 'red', 'shape': 'circle', 'range_m': None}]
        md = _render_markdown(self._data(waypoints=wps))
        self.assertIn('(unknown)', md)

    # 13. Duration formatted as HH:MM:SS (3661 s → 01:01:01)
    def test_duration_formatted_as_hhmmss(self):
        md = _render_markdown(self._data(duration_s=3661.0))
        self.assertIn('01:01:01', md)

    # 14. No waypoints → fallback message
    def test_no_waypoints_fallback(self):
        md = _render_markdown(self._data('FAIL', waypoints=[]))
        self.assertIn('No waypoints visited', md)

    # Distance and stop counts rendered
    def test_distance_and_stops_in_output(self):
        md = _render_markdown(self._data(dist=42.75, dm=3, em=2))
        self.assertIn('42.75 m', md)
        self.assertIn('Dead-man Stops: 3', md)
        self.assertIn('Emergency Stops: 2', md)

    # Two waypoints produce two sections
    def test_two_waypoints_two_sections(self):
        wps = [
            {'waypoint_idx': 0, 'marker_photo': None, 'object_photo': None,
             'color': 'red', 'shape': 'circle', 'range_m': 1.0},
            {'waypoint_idx': 1, 'marker_photo': None, 'object_photo': None,
             'color': 'blue', 'shape': 'square', 'range_m': 2.0},
        ]
        md = _render_markdown(self._data(waypoints=wps))
        self.assertIn('Waypoint 1', md)
        self.assertIn('Waypoint 2', md)

    # N/A when duration is None
    def test_duration_none_shows_na(self):
        md = _render_markdown(self._data(duration_s=None))
        self.assertIn('N/A', md)


# ---------------------------------------------------------------------------
# Tests — _write_summary (integration: actual file I/O)
# ---------------------------------------------------------------------------

class TestWriteSummary(unittest.TestCase):

    # 15. _write_summary writes both JSON and MD files
    def test_writes_json_and_md(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create a minimal JSONL log
            events = [
                _ev('MISSION_STARTED'),
                _wp_summary(0, distance_m=10.0),
                _ev('HOME_REACHED', ts_offset=5.0, distance_m=12.0),
                _ev('MISSION_COMPLETE', ts_offset=5.1, distance_m=12.0),
            ]
            log_path = os.path.join(tmpdir, 'journey.jsonl')
            with open(log_path, 'w') as f:
                for ev in events:
                    f.write(json.dumps(ev) + '\n')

            node = _make_node(tmpdir)
            node._write_summary('COMPLETE')

            files = os.listdir(tmpdir)
            json_files = [fn for fn in files if fn.endswith('.json')]
            md_files   = [fn for fn in files if fn.endswith('.md')]
            self.assertEqual(len(json_files), 1, 'Expected exactly one .json file')
            self.assertEqual(len(md_files),   1, 'Expected exactly one .md file')

            with open(os.path.join(tmpdir, json_files[0])) as f:
                data = json.load(f)
            self.assertEqual(data['result'], 'SUCCESS')
            self.assertAlmostEqual(data['total_distance_m'], 12.0, places=1)

            with open(os.path.join(tmpdir, md_files[0])) as f:
                md = f.read()
            self.assertIn('SUCCESS', md)
            self.assertIn('Waypoint 1', md)

    # 16. _write_summary with empty log produces FAIL result without crashing
    def test_empty_log_produces_fail(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            # Empty log file
            log_path = os.path.join(tmpdir, 'journey.jsonl')
            open(log_path, 'w').close()

            node = _make_node(tmpdir)
            node._write_summary('ABORTED')

            json_files = [fn for fn in os.listdir(tmpdir) if fn.endswith('.json')]
            self.assertEqual(len(json_files), 1)
            with open(os.path.join(tmpdir, json_files[0])) as f:
                data = json.load(f)
            self.assertEqual(data['result'], 'FAIL')

    # Missing log file handled gracefully
    def test_missing_log_file_no_crash(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            node = _make_node(tmpdir)
            # No JSONL file created — _write_summary should not raise
            node._write_summary('ABORTED')
            json_files = [fn for fn in os.listdir(tmpdir) if fn.endswith('.json')]
            self.assertEqual(len(json_files), 1)


if __name__ == '__main__':
    unittest.main()
