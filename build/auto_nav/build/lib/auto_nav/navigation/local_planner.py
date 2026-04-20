"""
LocalPlannerNode — orchestrator that selects the appropriate local target.

During NAVIGATING state path_follower drives toward the current waypoint.
This node intercepts that phase and provides a LiDAR-informed intermediate
target point that path_follower should drive toward instead.

Routing rules
-------------
  /waypoint/status == "NAVIGATING" and /navigation/segment == "1"
      → forward /weave/local_target as /local_target   (weave mode)

  /waypoint/status == "NAVIGATING" and segment != "1"
      → forward /gap/local_target   as /local_target   (normal gap mode)

  Any other state (COARSE_ARRIVED, FINAL_APPROACH, HOMING, DONE, IDLE)
      → publish nothing (path_follower handles these states directly)

A staleness guard ensures that if neither source has published a fresh
target in `stale_timeout_s`, local_planner stops forwarding (path_follower
then falls back to direct waypoint driving).

Subscribes:
  /gap/local_target       (geometry_msgs/PoseStamped)
  /weave/local_target     (geometry_msgs/PoseStamped)
  /waypoint/status        (std_msgs/String)
  /navigation/segment     (std_msgs/String)

Publishes:
  /local_target           (geometry_msgs/PoseStamped)

Parameters:
  stale_timeout_s         seconds before a source is considered stale (default 0.5)
  publish_rate_hz         rate at which to re-forward the latest target (default 10.0)
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

_WEAVE_SEGMENT = '1'        # segment index that activates weave mode
_NAVIGATING    = 'NAVIGATING'


class LocalPlannerNode(Node):

    def __init__(self) -> None:
        super().__init__('local_planner')

        # ---- Parameters --------------------------------------------------
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)

        def _dbl(n: str) -> float:
            return self.get_parameter(n).get_parameter_value().double_value

        self._stale_timeout = _dbl('stale_timeout_s')
        rate_hz             = _dbl('publish_rate_hz')

        # ---- State -------------------------------------------------------
        self._status:  str = ''
        self._segment: str = ''

        self._gap_target:   Optional[PoseStamped] = None
        self._gap_time:     float = 0.0

        self._weave_target: Optional[PoseStamped] = None
        self._weave_time:   float = 0.0

        # ---- QoS ---------------------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- Publishers --------------------------------------------------
        self._pub_target = self.create_publisher(PoseStamped, '/local_target', 10)

        # ---- Subscribers -------------------------------------------------
        self.create_subscription(PoseStamped, '/gap/local_target',   self._gap_cb,     10)
        self.create_subscription(PoseStamped, '/weave/local_target', self._weave_cb,   10)
        self.create_subscription(String,      '/waypoint/status',    self._status_cb,  latched_qos)
        self.create_subscription(String,      '/navigation/segment', self._segment_cb, latched_qos)

        # ---- Timer -------------------------------------------------------
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'LocalPlanner ready  stale_timeout={self._stale_timeout}s  '
            f'rate={rate_hz}Hz'
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _gap_cb(self, msg: PoseStamped) -> None:
        self._gap_target = msg
        self._gap_time   = self.get_clock().now().nanoseconds * 1e-9

    def _weave_cb(self, msg: PoseStamped) -> None:
        self._weave_target = msg
        self._weave_time   = self.get_clock().now().nanoseconds * 1e-9

    def _status_cb(self, msg: String) -> None:
        self._status = msg.data

    def _segment_cb(self, msg: String) -> None:
        self._segment = msg.data

    # ------------------------------------------------------------------
    # Timer — re-publish the selected target
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._status != _NAVIGATING:
            # Do not interfere with other path_follower states
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        if self._segment == _WEAVE_SEGMENT:
            # Weave mode: use weave_planner output
            if self._weave_target is not None:
                age = now - self._weave_time
                if age <= self._stale_timeout:
                    self._pub_target.publish(self._weave_target)
                    return
            # Weave target stale or missing: fall back to gap planner
            self._try_publish_gap(now)
        else:
            # Normal mode: use gap_planner output
            self._try_publish_gap(now)

    def _try_publish_gap(self, now: float) -> None:
        if self._gap_target is not None:
            age = now - self._gap_time
            if age <= self._stale_timeout:
                self._pub_target.publish(self._gap_target)


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
