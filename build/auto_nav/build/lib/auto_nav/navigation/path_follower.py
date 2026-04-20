"""
PathFollowerNode — waypoint navigation state machine.

Visits waypoints in sequence then returns home:
  HOME  →  WP[0]  →  WP[1]  →  …  →  WP[N-1]  →  HOME

Two-phase arrival per waypoint
-------------------------------
  Phase 1 — Coarse arrival:
    distance to nominal WP position ≤ coarse_arrival_radius_m
    → stop and wait for a marker detection message

  Phase 2 — Final approach:
    /marker/detection received while in COARSE_ARRIVED
    → compute lateral pass point (FinalApproachController) so the marker
      is left on the robot's RIGHT side, then drive through that point

Home pose recording
-------------------
  Recorded when AUTO mode is activated for the FIRST TIME in this session
  (i.e. at mission start), NOT at program startup.  This matches the
  requirement: "home pose 由 mission 开始时记录，而不是程序启动时记录".

LiDAR-assisted local navigation (Step 3)
-----------------------------------------
  During NAVIGATING state the node optionally defers to a local planner:
    /local_target   (geometry_msgs/PoseStamped) — intermediate target computed by
                    LocalPlannerNode (gap-follower or weave-planner).
  If a fresh /local_target is available (received within local_target_timeout_s),
  the robot drives toward it instead of the raw waypoint.  When the local target
  goes stale (LiDAR nodes not running), it falls back to direct waypoint driving.

  The node also publishes:
    /navigation/segment (std_msgs/String) — current segment index as a string
      "0" = navigating toward WP[0]
      "1" = navigating toward WP[1]  (triggers weave mode in WeaveplannerNode)
      …
      "H" = homing
      ""  = idle / done

Subscribes:
  /odom               (nav_msgs/Odometry)         — robot pose
  /control_mode       (std_msgs/String)            — MANUAL / AUTO / PAUSED / ABORTED
  /marker/detection   (geometry_msgs/PoseStamped)  — orange-cone marker position
  /mission/home_pose  (geometry_msgs/PoseStamped)  — optional pre-recorded home
                                                     (used only if mission not yet started)
  /local_target       (geometry_msgs/PoseStamped)  — LiDAR-based intermediate target
                                                     (only used during NAVIGATING)

Publishes:
  /cmd_vel_auto         (geometry_msgs/Twist)       — navigation velocity commands
  /waypoint/current     (geometry_msgs/PoseStamped) — current navigation target (latched)
  /waypoint/status      (std_msgs/String)           — state machine label    (latched)
  /navigation/segment   (std_msgs/String)           — current segment index  (latched)

Parameters  (path_follower section of config/waypoints.yaml):
  waypoints_file            path to the YAML file that contains the 'waypoints' list
  coarse_arrival_radius_m   metres inside which Phase 1 triggers      (default 2.5)
  final_arrival_radius_m    metres inside which a WP is marked done    (default 1.0)
  home_arrival_radius_m     metres inside which home is considered reached (default 1.0)
  pass_offset_m             lateral offset of the final-approach pass point (default 0.8)
  max_linear_vel            m/s speed cap                              (default 0.4)
  max_angular_vel           rad/s angular cap                          (default 1.0)
  k_angular                 proportional heading-error gain            (default 1.5)
  k_linear                  proportional distance gain                 (default 0.4)
  control_rate_hz           control-loop frequency                     (default 10.0)
  local_target_timeout_s    seconds before /local_target is considered stale (default 0.5)
"""

from __future__ import annotations

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from .waypoint_provider import WaypointProvider
from .final_approach import FinalApproachController


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class State(Enum):
    IDLE           = auto()  # waiting for AUTO mode — no motion
    NAVIGATING     = auto()  # driving toward current waypoint (coarse)
    COARSE_ARRIVED = auto()  # within coarse radius — waiting for marker detection
    FINAL_APPROACH = auto()  # marker found — driving through lateral pass point
    HOMING         = auto()  # all waypoints visited — returning to home
    DONE           = auto()  # mission complete


# ---------------------------------------------------------------------------
# PathFollowerNode
# ---------------------------------------------------------------------------

class PathFollowerNode(Node):

    def __init__(self) -> None:
        super().__init__('path_follower')

        # ---- Declare parameters ------------------------------------------
        self.declare_parameter('waypoints_file',          '')
        self.declare_parameter('coarse_arrival_radius_m', 2.5)
        self.declare_parameter('final_arrival_radius_m',  1.0)
        self.declare_parameter('home_arrival_radius_m',   1.0)
        self.declare_parameter('pass_offset_m',           0.8)
        self.declare_parameter('max_linear_vel',          0.4)
        self.declare_parameter('max_angular_vel',         1.0)
        self.declare_parameter('k_angular',               1.5)
        self.declare_parameter('k_linear',                0.4)
        self.declare_parameter('control_rate_hz',        10.0)
        self.declare_parameter('local_target_timeout_s',  0.5)

        def _dbl(name: str) -> float:
            return self.get_parameter(name).get_parameter_value().double_value

        self._wp_file     = (
            self.get_parameter('waypoints_file').get_parameter_value().string_value
        )
        self._coarse_r            = _dbl('coarse_arrival_radius_m')
        self._final_r             = _dbl('final_arrival_radius_m')
        self._home_r              = _dbl('home_arrival_radius_m')
        self._pass_offset         = _dbl('pass_offset_m')
        self._max_lin             = _dbl('max_linear_vel')
        self._max_ang             = _dbl('max_angular_vel')
        self._k_ang               = _dbl('k_angular')
        self._k_lin               = _dbl('k_linear')
        rate_hz                   = _dbl('control_rate_hz')
        self._local_target_timeout = _dbl('local_target_timeout_s')

        # ---- Load waypoints ----------------------------------------------
        if not self._wp_file:
            self.get_logger().error(
                'PathFollower: parameter waypoints_file is empty — '
                'pass it via launch or set in waypoints.yaml'
            )
            self._waypoints = []
        else:
            provider = WaypointProvider(self._wp_file)
            self._waypoints = provider.waypoints
            self.get_logger().info(
                f'PathFollower: loaded {len(self._waypoints)} waypoints '
                f'from {self._wp_file!r}'
            )

        # ---- Helper controller -------------------------------------------
        self._fa_ctrl = FinalApproachController(pass_offset_m=self._pass_offset)

        # ---- State -------------------------------------------------------
        self._state           = State.IDLE
        self._mode            = 'MANUAL'
        self._home_recorded   = False
        self._wp_idx          = 0

        # Robot pose (updated by /odom)
        self._robot_x         = 0.0
        self._robot_y         = 0.0
        self._robot_yaw       = 0.0

        # Home pose (set at mission start)
        self._home_x          = 0.0
        self._home_y          = 0.0

        # Latest marker position (cleared after each waypoint completion)
        self._marker_x: float | None = None
        self._marker_y: float | None = None

        # Current navigation target (waypoint / pass point)
        self._target_x        = 0.0
        self._target_y        = 0.0

        # /local_target from LocalPlannerNode (LiDAR gap/weave, NAVIGATING only)
        self._local_target_x:    float = 0.0
        self._local_target_y:    float = 0.0
        self._local_target_time: float = 0.0   # rclpy clock seconds (0 = never)

        # /mission/hold from MissionControllerNode (Step 5)
        # When True: don't call _advance_waypoint() after pass-point; buffer the advance.
        self._mission_hold:     bool = False
        self._pending_advance:  bool = False

        # ---- QoS ---------------------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- Publishers --------------------------------------------------
        self._pub_cmd     = self.create_publisher(Twist,       '/cmd_vel_auto',        10)
        self._pub_wp      = self.create_publisher(PoseStamped, '/waypoint/current',    latched_qos)
        self._pub_status  = self.create_publisher(String,      '/waypoint/status',     latched_qos)
        self._pub_segment = self.create_publisher(String,      '/navigation/segment',  latched_qos)

        # ---- Subscribers -------------------------------------------------
        self.create_subscription(Odometry,    '/odom',              self._odom_cb,        10)
        self.create_subscription(String,      '/control_mode',      self._mode_cb,        latched_qos)
        self.create_subscription(PoseStamped, '/marker/detection',  self._marker_cb,      10)
        self.create_subscription(PoseStamped, '/mission/home_pose', self._home_cb,        latched_qos)
        self.create_subscription(PoseStamped, '/local_target',      self._local_target_cb, 10)
        self.create_subscription(Bool,        '/mission/hold',      self._hold_cb,        latched_qos)

        # ---- Control loop ------------------------------------------------
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self._publish_status()
        self.get_logger().info(
            f'PathFollower ready  coarse_r={self._coarse_r}m  '
            f'pass_offset={self._pass_offset}m  rate={rate_hz}Hz'
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._robot_x   = pose.position.x
        self._robot_y   = pose.position.y
        self._robot_yaw = _quat_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    def _mode_cb(self, msg: String) -> None:
        prev_mode  = self._mode
        self._mode = msg.data

        # Transition INTO AUTO for the first time → start mission
        if prev_mode != 'AUTO' and self._mode == 'AUTO':
            self._on_mission_start()

        # Leaving AUTO while mid-mission → publish zero immediately
        elif self._mode != 'AUTO' and self._state not in (State.IDLE, State.DONE):
            self._stop_robot()

    def _marker_cb(self, msg: PoseStamped) -> None:
        self._marker_x = msg.pose.position.x
        self._marker_y = msg.pose.position.y

    def _home_cb(self, msg: PoseStamped) -> None:
        """Accept a pre-recorded home pose only if mission hasn't started yet."""
        if not self._home_recorded:
            self._home_x = msg.pose.position.x
            self._home_y = msg.pose.position.y
            self.get_logger().info(
                f'PathFollower: received home_pose from topic '
                f'x={self._home_x:.3f} y={self._home_y:.3f}'
            )

    def _local_target_cb(self, msg: PoseStamped) -> None:
        """Cache the LiDAR-based intermediate target from LocalPlannerNode."""
        self._local_target_x    = msg.pose.position.x
        self._local_target_y    = msg.pose.position.y
        self._local_target_time = self.get_clock().now().nanoseconds * 1e-9

    def _hold_cb(self, msg: Bool) -> None:
        """MissionControllerNode requests a hold on waypoint advance (Step 5)."""
        prev = self._mission_hold
        self._mission_hold = msg.data
        if prev and not self._mission_hold and self._pending_advance:
            # Hold released while advance was deferred — execute it now
            self._pending_advance = False
            self.get_logger().info('PathFollower: mission hold released — advancing waypoint')
            self._advance_waypoint()

    # ------------------------------------------------------------------
    # Mission start — called once when AUTO mode is first activated
    # ------------------------------------------------------------------

    def _on_mission_start(self) -> None:
        if self._home_recorded:
            # Already started (mode toggled AUTO→MANUAL→AUTO) — just resume
            if self._state not in (State.DONE,):
                self.get_logger().info('PathFollower: resuming mission')
            return

        # Record home at mission start (odom position right now)
        self._home_x        = self._robot_x
        self._home_y        = self._robot_y
        self._home_recorded = True
        self.get_logger().info(
            f'PathFollower: home recorded at MISSION START '
            f'x={self._home_x:.3f} y={self._home_y:.3f}'
        )

        # Kick off navigation
        self._wp_idx = 0
        if self._waypoints:
            self._set_state(State.NAVIGATING)
            self._set_target_to_wp(self._wp_idx)
        else:
            self.get_logger().warn(
                'PathFollower: waypoints list is empty — mission complete immediately'
            )
            self._set_state(State.DONE)

    # ------------------------------------------------------------------
    # Control loop (runs at control_rate_hz)
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._mode != 'AUTO':
            return

        if self._state in (State.IDLE, State.DONE):
            return

        dist = _dist(self._robot_x, self._robot_y, self._target_x, self._target_y)

        if self._state == State.NAVIGATING:
            self._do_navigating(dist)
        elif self._state == State.COARSE_ARRIVED:
            self._do_coarse_arrived()
        elif self._state == State.FINAL_APPROACH:
            self._do_final_approach(dist)
        elif self._state == State.HOMING:
            self._do_homing(dist)

    # ------------------------------------------------------------------
    # Per-state handlers
    # ------------------------------------------------------------------

    def _do_navigating(self, dist: float) -> None:
        if dist <= self._coarse_r:
            wp = self._waypoints[self._wp_idx]
            self.get_logger().info(
                f'PathFollower: coarse arrived at WP[{self._wp_idx}] '
                f'({wp.name})  dist={dist:.2f}m'
            )
            self._set_state(State.COARSE_ARRIVED)
            self._stop_robot()
            return

        # Use LiDAR-based local target if fresh, else drive directly to waypoint
        now = self.get_clock().now().nanoseconds * 1e-9
        local_fresh = (
            self._local_target_time > 0.0
            and (now - self._local_target_time) <= self._local_target_timeout
        )
        if local_fresh:
            self._drive_toward(self._local_target_x, self._local_target_y)
        else:
            self._drive_toward(self._target_x, self._target_y)

    def _do_coarse_arrived(self) -> None:
        if self._marker_x is not None and self._marker_y is not None:
            # Marker detected — compute lateral pass point
            pass_pt = self._fa_ctrl.compute_pass_point(
                p_robot  = (self._robot_x,  self._robot_y),
                p_marker = (self._marker_x, self._marker_y),
            )
            self._target_x = pass_pt.x
            self._target_y = pass_pt.y
            self.get_logger().info(
                f'PathFollower: marker at ({self._marker_x:.2f}, {self._marker_y:.2f}) '
                f'→ pass_pt=({pass_pt.x:.2f}, {pass_pt.y:.2f})'
            )
            self._set_state(State.FINAL_APPROACH)
        else:
            # No marker yet — hold still, wait for detection
            self._stop_robot()

    def _do_final_approach(self, dist: float) -> None:
        if dist <= self._final_r:
            if not self._pending_advance:
                # First time reaching pass point in this cycle — log once
                wp = self._waypoints[self._wp_idx]
                self.get_logger().info(
                    f'PathFollower: WP[{self._wp_idx}] ({wp.name}) complete — '
                    f'marker passed on right side'
                )
                self._marker_x = None
                self._marker_y = None

            if self._mission_hold:
                # MissionController wants a window for SEARCH_OBJECT before we advance
                self._pending_advance = True
                return  # do not publish cmd_vel — mission controller drives

            self._pending_advance = False
            self._advance_waypoint()
            return

        self._drive_toward(self._target_x, self._target_y)

    def _do_homing(self, dist: float) -> None:
        if dist <= self._home_r:
            self.get_logger().info('PathFollower: home reached — mission DONE')
            self._set_state(State.DONE)
            self._stop_robot()
            return

        self._drive_toward(self._target_x, self._target_y)

    # ------------------------------------------------------------------
    # Waypoint management
    # ------------------------------------------------------------------

    def _set_target_to_wp(self, idx: int) -> None:
        wp = self._waypoints[idx]
        self._target_x = wp.x
        self._target_y = wp.y
        self._publish_current_wp(wp.x, wp.y)
        self._publish_segment(str(idx))   # e.g. "0", "1", "2" …
        self.get_logger().info(
            f'PathFollower: targeting WP[{idx}] {wp.name!r} '
            f'({wp.x:.2f}, {wp.y:.2f})'
        )

    def _advance_waypoint(self) -> None:
        self._wp_idx += 1
        if self._wp_idx < len(self._waypoints):
            self._set_state(State.NAVIGATING)
            self._set_target_to_wp(self._wp_idx)
        else:
            self.get_logger().info(
                'PathFollower: all waypoints visited — heading home'
            )
            self._target_x = self._home_x
            self._target_y = self._home_y
            self._publish_current_wp(self._home_x, self._home_y)
            self._publish_segment('H')   # "H" = homing segment
            self._set_state(State.HOMING)

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------

    def _drive_toward(self, tx: float, ty: float) -> None:
        """Proportional heading + speed controller toward target (tx, ty)."""
        dx = tx - self._robot_x
        dy = ty - self._robot_y
        dist = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_err     = _angle_wrap(desired_heading - self._robot_yaw)

        angular_vel = _clamp(self._k_ang * heading_err, -self._max_ang, self._max_ang)

        # Scale down linear speed when turning sharply (cos² factor)
        forward_factor = math.cos(heading_err) ** 2
        linear_vel = _clamp(
            self._k_lin * dist * forward_factor,
            0.0,
            self._max_lin,
        )

        cmd = Twist()
        cmd.linear.x  = float(linear_vel)
        cmd.angular.z = float(angular_vel)
        self._pub_cmd.publish(cmd)

    def _stop_robot(self) -> None:
        self._pub_cmd.publish(Twist())

    # ------------------------------------------------------------------
    # State / publish helpers
    # ------------------------------------------------------------------

    def _set_state(self, new_state: State) -> None:
        if new_state != self._state:
            self.get_logger().info(
                f'PathFollower: {self._state.name} → {new_state.name}'
            )
            self._state = new_state
            self._publish_status()

    def _publish_status(self) -> None:
        msg      = String()
        msg.data = self._state.name
        self._pub_status.publish(msg)
        # Clear segment when not actively navigating toward a waypoint
        if self._state in (State.IDLE, State.DONE):
            self._publish_segment('')

    def _publish_segment(self, segment: str) -> None:
        """Publish the current navigation segment identifier."""
        msg      = String()
        msg.data = segment
        self._pub_segment.publish(msg)

    def _publish_current_wp(self, x: float, y: float) -> None:
        msg                    = PoseStamped()
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.header.frame_id    = 'odom'
        msg.pose.position.x    = float(x)
        msg.pose.position.y    = float(y)
        msg.pose.orientation.w = 1.0
        self._pub_wp.publish(msg)


# ---------------------------------------------------------------------------
# Pure utility functions
# ---------------------------------------------------------------------------

def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation about Z axis) from a unit quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_wrap(angle: float) -> float:
    """Wrap angle to (−π, +π]."""
    while angle >  math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _dist(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
