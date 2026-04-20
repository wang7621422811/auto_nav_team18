"""
JourneyLoggerNode — persists /journey/event messages to a JSONL log file.

Each line of the log file is a JSON object (one event per line).
The file is flushed after every write so it is safe to read while the
mission is still running.

Subscribes
----------
  /journey/event  (std_msgs/String)  — JSON events from MissionControllerNode

Publishes
---------
  (none)

Parameters  (config/mission.yaml  →  journey_logger namespace)
----------
  log_dir      directory for output files (default "artifacts/logs")
  log_filename filename of the JSONL log (default "journey.jsonl")
"""

from __future__ import annotations

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class JourneyLoggerNode(Node):

    def __init__(self) -> None:
        super().__init__('journey_logger')

        self.declare_parameter('log_dir',      'artifacts/logs')
        self.declare_parameter('log_filename', 'journey.jsonl')

        log_dir  = self.get_parameter('log_dir').get_parameter_value().string_value
        log_file = self.get_parameter('log_filename').get_parameter_value().string_value

        self._log_path = os.path.join(log_dir, log_file)

        try:
            os.makedirs(log_dir, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f'Cannot create log directory {log_dir!r}: {exc}')

        self._file = None
        self._open_log()

        best_effort = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/journey/event', self._event_cb, best_effort)

        self.get_logger().info(f'JourneyLogger writing to {self._log_path!r}')

    # ------------------------------------------------------------------

    def _open_log(self) -> None:
        try:
            # Append mode: each mission run adds to the same file
            self._file = open(self._log_path, 'a', buffering=1)  # line-buffered
        except OSError as exc:
            self.get_logger().error(f'Cannot open log file {self._log_path!r}: {exc}')
            self._file = None

    def _event_cb(self, msg: String) -> None:
        if self._file is None:
            return
        try:
            # Validate JSON; re-dump with consistent formatting
            data = json.loads(msg.data)
            line = json.dumps(data, separators=(',', ':'))
            self._file.write(line + '\n')
            self.get_logger().debug(f"Event: {data.get('type', '?')}")
        except (json.JSONDecodeError, OSError) as exc:
            self.get_logger().warn(f'Logger failed to write event: {exc}')

    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        if self._file is not None:
            try:
                self._file.flush()
                self._file.close()
            except OSError:
                pass
        super().destroy_node()


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = JourneyLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
