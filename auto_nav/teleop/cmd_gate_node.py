"""
CmdGateNode — the single safety arbiter for all motion commands.

This is the ONLY node that writes to /cmd_vel_safe (the chassis input).
All other nodes publish to intermediate topics; this node decides what
actually moves the robot.

Decision logic (in priority order):
  1. Joystick lost          → publish zero
  2. Dead-man released      → publish zero
  3. Emergency stop latched → publish zero
  4. mode == MANUAL         → forward /cmd_vel_manual
  5. mode == AUTO           → forward /cmd_vel_auto
  6. anything else          → publish zero  (includes PAUSED, ABORTED)

A watchdog timer runs at 10 Hz to guarantee a zero command is published
within 100 ms of any safety condition triggering, even if no new cmd arrives.

Subscribes:
  /cmd_vel_manual (geometry_msgs/Twist)
  /cmd_vel_auto   (geometry_msgs/Twist)
  /control_mode   (std_msgs/String)
  /deadman_ok     (std_msgs/Bool)
  /joy_connected  (std_msgs/Bool)
  /emergency_stop (std_msgs/Bool)

Publishes:
  /cmd_vel_safe   (geometry_msgs/Twist)  — unique chassis output
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

_ZERO = Twist()   # module-level constant; all fields default to 0.0


class CmdGateNode(Node):

    MANUAL = 'MANUAL'
    AUTO   = 'AUTO'

    def __init__(self):
        super().__init__('cmd_gate')

        # Publishing rate for watchdog (ensures zero arrives within 100 ms)
        self.declare_parameter('gate_rate_hz', 10.0)
        rate = self.get_parameter('gate_rate_hz').get_parameter_value().double_value

        self._pub_safe = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Twist,  '/cmd_vel_manual', self._manual_cb,    10)
        self.create_subscription(Twist,  '/cmd_vel_auto',   self._auto_cb,      10)
        self.create_subscription(String, '/control_mode',   self._mode_cb,      latched_qos)
        self.create_subscription(Bool,   '/deadman_ok',     self._deadman_cb,   latched_qos)
        self.create_subscription(Bool,   '/joy_connected',  self._connected_cb, latched_qos)
        self.create_subscription(Bool,   '/emergency_stop', self._estop_cb,     latched_qos)

        self._cmd_manual = Twist()
        self._cmd_auto   = Twist()
        self._mode       = self.MANUAL
        self._deadman_ok = False
        self._connected  = False   # conservative until first /joy_connected message
        self._estop      = False

        # Watchdog fires at requested rate to publish zero if conditions change
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(f'CmdGate ready — watchdog rate={rate} Hz')

    # ------------------------------------------------------------------
    # Incoming command callbacks
    # ------------------------------------------------------------------

    def _manual_cb(self, msg: Twist) -> None:
        self._cmd_manual = msg
        self._gate()

    def _auto_cb(self, msg: Twist) -> None:
        self._cmd_auto = msg
        self._gate()

    # ------------------------------------------------------------------
    # State callbacks
    # ------------------------------------------------------------------

    def _mode_cb(self, msg: String) -> None:
        self._mode = msg.data
        self._gate()

    def _deadman_cb(self, msg: Bool) -> None:
        self._deadman_ok = msg.data
        self._gate()

    def _connected_cb(self, msg: Bool) -> None:
        self._connected = msg.data
        self._gate()

    def _estop_cb(self, msg: Bool) -> None:
        self._estop = msg.data
        self._gate()

    # ------------------------------------------------------------------
    # Watchdog tick — guarantees periodic zero when blocked
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        self._gate()

    # ------------------------------------------------------------------
    # Core arbitration logic
    # ------------------------------------------------------------------

    def _gate(self) -> None:
        out = self._select_command()
        self._pub_safe.publish(out)

    def _select_command(self) -> Twist:
        if not self._connected:
            return _ZERO

        # Both manual and autonomous motion require the dead-man to be held.
        if not self._deadman_ok:
            return _ZERO

        if self._estop:
            return _ZERO

        if self._estop:
            return self._reverse_escape_command()

        if self._mode == self.MANUAL:
            return self._cmd_manual

        if self._mode == self.AUTO:
            return self._cmd_auto

        # PAUSED, ABORTED, or any other unsupported mode
        return _ZERO

    def _reverse_escape_command(self) -> Twist:
        """
        While obstacle estop is active, allow the operator to back out manually.

        This keeps the safety stop effective for all forward motion, but avoids
        trapping the robot against a wall where the LiDAR continues to see the
        obstacle inside the front safety sector.  Reverse escape is deliberately
        restricted to MANUAL mode and strips angular velocity to keep the
        behaviour predictable during recovery.
        """
        if self._mode != self.MANUAL:
            return _ZERO

        if self._cmd_manual.linear.x >= 0.0:
            return _ZERO

        out = Twist()
        out.linear.x = self._cmd_manual.linear.x
        return out


def main(args=None):
    rclpy.init(args=args)
    node = CmdGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
